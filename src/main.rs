#![allow(unused)]
mod settings;
mod boolean;
mod stl_op;
mod skeleton;
mod gcode;
mod geo;
mod blender;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod data;

use geo::{Polygon,Enclosed};
use settings::Settings;
use boolean::{ss_offset, tagged_boolean};
use i_overlay::core::overlay_rule::OverlayRule;
use blender::Blender;

use log::Level;
use settings::Feedrates;
use std::io::{Write, BufReader};
use std::fs::{self,File};
use std::io;
use std::time::Instant;


/// WARNING: all content in the TMP_DIR is deleted each time the project is run
const TEMP_DIR: &str = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/tmp/";

const t_min:f32 = 20.0;

fn main(){
    let start_time = Instant::now();
    // let mut args = env::args();
    // let path = args.next().expect("first arg should be the path");
    // let mesh_layer_dir = args.next().expect("missing argument: stl-layers directory");
 
    init_logger();
    let mut blender = Blender::new();

    let settings = Settings{
        layer_height:0.4,
        perimeter_line_width: 1.,
        infill_percentage: 20,
        nr_of_perimeters: 2,
        brim: 0,
        outer_wall_first: false,
        translate_xy: [50., 50.],
        // feedrates:Feedrates{
        //     initial_layer:800,
        //     initial_layer_infill:1200,
        //     travel: 2000,
        //     ..Default::default()
        // },
        ..Default::default()
    };
    dbg!(&settings);

    let stl_path = "../mesh/simple_overhang.stl";
    // let stl_path = "../mesh/bunny.stl";

    // let stl_path = "../mesh/internal_external.stl";
    // let stl_path = "../mesh/circular overhang.stl";
    // let stl_path = "../mesh/2.stl";
    // let stl_path = "../mesh/rapport demo.stl";
    // let stl_path = "../mesh/curved overhang.stl";
    // let stl_path = "../mesh/pipe.stl";
    // let stl_path = "../mesh/curved overhang.stl";
    // let stl_path = "../mesh/wine_glass3.stl";

    blender.load_mesh(stl_path,"input mesh");
    let layer_perimeters = extract_planar_layers_from_mesh(stl_path,&settings);
    // let d_x2 = (settings.overhang_angle*0.5).tan()*settings.layer_height;
    // let p0_offset = ss_offset(layer_perimeters[0].clone(),-0.5*d_x2);
    // blender.display2d(&p0_offset[0], 0.0, "c0-offset2", "er jeg dum?");
    // blender.display2d(&layer_perimeters[0][0], 1.0, "c0", "er jeg dum?");
        // .enumerate()
        // .flat_map(|(i,layer)|layer.iter().map(move |p|(i,p)))
        // .for_each(|(i,polygon)|blender.display2d(polygon, i as f32, "layer contour", "layer contours"));
    // panic!();

    mesh_gen(&mut blender,&layer_perimeters,&settings);
    // get_user_confimation();
    crate_or_clear_dir(TEMP_DIR);
    let mesh_gen_time = start_time.elapsed();

    blender.export_layers(TEMP_DIR);
    blender.ping(); //make sure blender has processed all messages
    let boolean_time = start_time.elapsed();

    // let start_time = Instant::now();
    gcode::main(&mut blender,TEMP_DIR,&settings);
    let gcode_gen_time = start_time.elapsed();

    
    println!("mesh generation time  {} sec",mesh_gen_time.as_secs_f64());
    println!("mesh booleon time     {} sec",(boolean_time-mesh_gen_time).as_secs_f64());
    println!("gcode generation time {} sec",(gcode_gen_time-boolean_time).as_secs_f64());
    println!("\x1b[034mExported G-code file to {TEMP_DIR}gcode.gcode\x1b[0m");
}

fn crate_or_clear_dir<T:AsRef<std::path::Path>>(tmp:T){
    let _ = fs::remove_dir_all(&tmp); // <- dont care if dir does not exist
    fs::create_dir(&tmp).unwrap();
}

fn get_user_confimation(){
    println!("\x1b[033mpress enter to generate gcode from the planes in blender's results folder\x1b[0m");
    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .expect("Failed to read line");
    println!("nice");
}


fn extract_planar_layers_from_mesh<T:AsRef<std::path::Path>>(stl_path:T,s:&Settings) -> Vec<Vec<Polygon>> {

    let file = File::open(&stl_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");

    let min_a = 0.05;   // min area for contour simplification
    let mut layers:Vec<Vec<Polygon>> = stl_op::extract_planar_layers(&mesh, s.layer_height).into_iter()
        .map(|layer| 
            layer.into_iter().filter_map(|p|p.simplify(min_a)).collect()
        ).collect();
    while layers.last().unwrap().len() == 0 {
        layers.pop();
    }
    return layers
}

fn generate_full_layer(
    layer:Vec<Polygon>,
    face_mask:Vec<Vec<Vec<bool>>>,
    layer_height:f32,
    theta:f32,
    ) -> Option<(Vec<[f32;3]>,Vec<Vec<usize>>)>{
    let (vertices,mut np_faces,mut planar_faces) = generate_layer(layer, face_mask, layer_height, theta )?;
    np_faces.append(&mut planar_faces);
    return Some((vertices,np_faces))
}

fn generate_partial_layer(
    layer:Vec<Polygon>,
    face_mask:Vec<Vec<Vec<bool>>>,
    layer_height:f32,
    theta:f32,
    ) -> Option<(Vec<[f32;3]>,Vec<Vec<usize>>)>{
    let (vertices,np_faces,planar_faces) = generate_layer(layer, face_mask, layer_height, theta )?;
    return Some((vertices,np_faces))
}

fn generate_layer(
    layer:Vec<Polygon>,
    face_mask:Vec<Vec<Vec<bool>>>,
    layer_height:f32,
    theta:f32,
    )
    ->Option<(Vec<[f32;3]>,Vec<Vec<usize>>,Vec<Vec<usize>>)>{
            let polygons:Vec<Polygon> = layer.into_iter()
                .map(|p|{let mut p = p.clone(); p.invert(); p})
                .collect();

            let skeleton = match skeleton::skeleton_from_polygons_with_limit(polygons.clone(),t_min){
                Ok(skeleton) => skeleton,
                Err(err) => {
                    println!("\x1b[031m{err}\x1b[0m");
                    dbg!(&polygons); 
                    return None;
                }
            };
            let mut planar_faces = skeleton.mesh_input_polygon();

            let mut np_faces = skeleton.skeleton_mesh_with_mask(face_mask.clone());
            let edges = skeleton.edges;
            let points:Vec<_> = skeleton.vertices.into_iter()
                .map(|x| [x[0],x[1],layer_height-x[2]*theta.tan()])
                .collect();

         return   Some((points, np_faces, planar_faces))
}

fn mesh_gen(blender:&mut Blender, layers:&Vec<Vec<Polygon>>, s:&Settings){
    let mut avr_support_ratio = 1.;

    let theta = s.overhang_angle;
    let layer_h = s.layer_height;

    let d_x1 = layer_h/theta.sin(); // offset for partial layers
    let d_x2 = (theta*0.5).tan()*layer_h; // offset for full layers


    for (i,layer) in layers.iter().enumerate(){
        let z = (i+1) as f32 * layer_h;
        for polygon in layer{
            blender.display2d(polygon,z,"outer perimeter","part perimeter");
        }
    }
    let last_layer = layers.len() - 1;
    let boundary_clone = layers.clone();

    let mut layers = layers.into_iter();
    let mut prev_support:Vec<Polygon> = layers.next().unwrap().clone();
    blender.solid_polygon(&prev_support, layer_h, "000-000-layer", "result");

    for (i, layer) in layers.enumerate(){
        let layer_nr = i + 1;
        let z_height = ((layer_nr+1) as f32)*layer_h;
        println!("layer {layer_nr}");

        let offset_sup = ss_offset(prev_support.clone(),-d_x2);

        let (mut support,tags) = tagged_boolean(layer.clone(),offset_sup,OverlayRule::Intersect);

        let mut support_a:f32 = support.iter().map(|polygon|polygon.area()).sum();
        let perimeter_a:f32 = layer.iter().map(|polygon|polygon.area()).sum();

        support.iter().for_each(|polygon| 
            blender.display2d(
                polygon,
                z_height,
                format!("planar region {layer_nr:03}-000"),
                "debug".into()
                )
            );

        if let Some( (vertices,faces) ) = generate_full_layer(support.clone(), tags, z_height, theta){
            blender.n_gon_result(vertices,vec![],faces,format!("{layer_nr:03}-000-layer"));
        } else {
            println!("Error: failed to generate layer {layer_nr:03}-000-layer");
        }

        let min_support_ratio = if layer_nr == last_layer {0.95} else {0.5};
        let mut i = 0;
        while {
            match boundary_clone.get(layer_nr+1){
                Some(b_next) => {
                    let (overlap,_) = tagged_boolean(b_next.clone(), support.clone(), OverlayRule::Intersect);
                    let overlap_a:f32 = overlap.iter().map(|polygon|polygon.area()).sum();
                    let b_next_a:f32 = b_next.iter().map(|polygon|polygon.area()).sum();
                    (overlap_a/b_next_a)<0.5
                },
                None => (support_a / perimeter_a) < 0.95,
            } 
        }{
        // while ( support_a / perimeter_a ) < min_support_ratio {
        
            i += 1;
            // if (layer_nr != last_layer)&&(i>3){break}
            if i > 15 {break}
            let offset_sup = ss_offset(support.clone(),-d_x1);

            let (support2,tags) = tagged_boolean(layer.clone() ,offset_sup, OverlayRule::Intersect);
            support = support2;
            support_a = support.iter().map(|polygon| polygon.area() ).sum();

            if let Some( (vertices,faces) ) = generate_partial_layer(support.clone(), tags, z_height, theta){
                blender.n_gon_result(vertices,vec![],faces,format!("{layer_nr:03}-{i:03}-layer"));
            }else{
                println!("Error: failed to generate partial layer {layer_nr:03}-{i:03}-layer");
                break
            };

            support.iter().for_each(|polygon| 
                blender.display2d(
                    polygon,
                    z_height,
                    format!("planar region {layer_nr:03}-{i:03}"),
                    "partial layers".into()
                    ) 
                );
        };
        prev_support = support.into_iter().filter_map(|polygon|polygon.simplify(0.01)).collect();
        // prev_support = support;

    }
}



fn init_logger(){
    env_logger::builder()
        .format(|buf, record|{
            match record.level() {
                Level::Error => write!(buf,"\x1b[031mError\x1b[0m")?,
                Level::Warn  => write!(buf,"\x1b[033mWarn \x1b[0m")?,
                Level::Info  => write!(buf,"\x1b[032mInfo \x1b[0m")?,
                Level::Debug => write!(buf,"\x1b[034mDebug\x1b[0m")?,
                Level::Trace => write!(buf,"\x1b[035mTrace\x1b[0m")?,
            };
            writeln!(buf,": {}",record.args())
        })
    .init();
}

