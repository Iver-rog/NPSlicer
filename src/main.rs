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

use geo::Polygon;
use boolean::{ss_offset, tagged_boolean};
use i_overlay::core::overlay_rule::OverlayRule;
use blender::Blender;

use log::Level;
use settings::Feedrates;
use std::io::{Write, BufReader};
use std::fs::{self,File};
use std::io;


/// WARNING: all content in the TMP_DIR is deleted each time the project is run
const TEMP_DIR: &str = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/tmp/";

fn main(){
    // let mut args = env::args();
    // let path = args.next().expect("first arg should be the path");
    // let mesh_layer_dir = args.next().expect("missing argument: stl-layers directory");
 
    init_logger();
    let mut blender = Blender::new();

    let settings = settings::Settings{
        overhang_angle: 0.2,
        infill_percentage: 20,
        nr_of_perimeters: 3,
        brim: 3,
        outer_wall_first: false,
        translate_xy: [50., 50.],
        feedrates:Feedrates{
            initial_layer:800,
            initial_layer_infill:1200,
            travel: 2000,
            ..Default::default()
        },
        ..Default::default()
    };
    dbg!(&settings);

    // let stl_path = "../mesh/bunny2.stl";
    // let stl_path = "../mesh/stanford-armadillo.stl";
    // let stl_path = "../mesh/curved overhang.stl";
    // let stl_path = "../mesh/simple_overhang.stl";
    // let stl_path = "../mesh/wine_glass3.stl";
    let stl_path = "../mesh/internal_external_simplified.stl";
    // blender.load_mesh(stl_path, "input_mesh");
    // let stl_path = "../mesh/bunny2.stl";

    // mesh_gen(&mut blender,stl_path,&settings);
    // get_user_confimation();
    crate_or_clear_dir(TEMP_DIR);
    blender.export_layers(TEMP_DIR);
    get_user_confimation();

    gcode::main(&mut blender,TEMP_DIR,&settings);
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




fn mesh_gen<T:AsRef<std::path::Path>>(blender:&mut Blender, stl_path:T, s:&settings::Settings){

    let file = File::open(&stl_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.load_mesh(stl_path,"input mesh");

    let theta = s.overhang_angle;
    let layer_h = s.layer_height;

    let min_a = 0.05;   // min area for contour simplification

    let d_x1 = layer_h/theta.tan(); // <- one intermediate layer & constant thicness in vertical direction
    let d_x2 = (theta*0.5).tan()*layer_h; // <- no intermediate layers & constant thicness perpendicular to layer
    // let d_x = layer_h/theta.tan() + (theta*0.5).tan()*layer_h;
    let d_x = d_x1+d_x2;

    let mut layers = stl_op::extract_planar_layers(&mesh, layer_h);
    layers.iter_mut().for_each(|layer| layer.iter_mut().for_each(|polygon|polygon.simplify(min_a)));

    // let layers:Vec<Vec<_>> = layers.into_iter()
    //     .for_each(|layer|{
    //         layer.into_iter()
    //         .filter_map(|mut polygon|{
    //             polygon.simplify(min_a);
    //             if polygon.0.len() == 0 {None} else {Some(polygon)}
    //         })
    //         .collect()
    //         })
    //     .collect();

    for (i,layer) in layers.iter().enumerate(){
        let z = (i+1) as f32 * layer_h;
        for polygon in layer{
            blender.display2d(polygon,z,"outer perimeter","part perimeter");
        }
    }

    for polygon in &layers[0] {
        polygon.validate();
    }

    let mut layers = layers.into_iter();
    let mut support_regions:Vec<Vec<Polygon>> = vec![layers.next().unwrap()];
    let mut face_masks: Vec<_> = vec![Vec::new()];

    for (i, layer) in layers.enumerate(){
        println!("layer {i}");
        let offset_sup = ss_offset(support_regions.last().unwrap().clone(),-d_x);

        // let support:Vec<_> = offset_sup.into_iter()
        //     .map(|polygon|{
        //     let result = polygon.intersect(layer.clone());
        //     if result.len() == 0 { vec![polygon] }else{result}
        //     })
        // .flatten()
        // .collect();

        // let support = boolean(layer,offset_sup,OverlayRule::Intersect);

        let (support,tags) = tagged_boolean(layer,offset_sup,OverlayRule::Intersect);
        // let nr_of_edges = support.iter().map(|polygon|polygon.0.clone()).flatten().map(|contour|contour.points).flatten().count();
        // let nr_of_clip_edges:usize = tags.iter().flatten().flatten().map(|is_clip|usize::from(*is_clip)).sum();
        // println!("{nr_of_clip_edges}/{nr_of_edges} are clip {}",
        //     if nr_of_edges == nr_of_clip_edges{"\x1b[033mno-need\x1b[0m"}else{"\x1b[032myes-need\x1b[0m"});

        // let support_identical_to_perimeter = tags.iter().flatten().flatten().map(|is_clip|).all(|is_clip| *is_clip);
        // println!("{}",if support_identical_to_perimeter {"\x1b[033mno-need\x1b[0m"}else{"\x1b[032myes-need\x1b[0m"});

        // support.iter_mut().for_each(|layer| layer.0.iter_mut().for_each(|polygon|polygon.simplify(min_a)));

        support.iter().for_each(|polygon|blender.display2d(
                polygon,
                ((i+2) as f32)*layer_h,
                "support_polygon",
                "debug")
            );
        support_regions.push(support);
        face_masks.push(tags);
    }

    println!("created {} layers",support_regions.len());

    let skeletons = support_regions.iter()
        .enumerate()
        .map(|(i,layer)|{
            let polygons:Vec<Polygon> = layer.into_iter()
                .map(|p|{let mut p = p.clone(); p.invert(); p})
                .collect();
            (i,polygons)
        })
        .filter_map(|(i,polygons)|{
            match skeleton::skeleton_from_polygons_with_limit(polygons.clone(),10.0){
                Ok(skeleton) => Some((i,skeleton)),
                Err(err) => {
                    println!("\x1b[031m{err}\x1b[0m");
                    dbg!(&polygons); 
                    None
                }
            }
        })
        .for_each(|(i,skeleton)|{ 
            let layer_height = (i+1) as f32 * layer_h;
            // let mut input_polygon_mesh = skeleton.input_polygons_mesh_outer_loop();
            let mut input_polygon_mesh = skeleton.mesh_input_polygon();

            // let mut mesh = skeleton.skeleton_mesh();
            let mut mesh = skeleton.skeleton_mesh_with_mask(face_masks[i].clone());
            let edges = skeleton.edges;
            let points:Vec<_> = skeleton.vertices.into_iter()
                .map(|x| [x[0],x[1],layer_height-x[2]*theta.tan()])
                .collect();

            let d_h = layer_h + d_x2*theta.tan();
            let p2 = points.iter().map(|p|[ p[0], p[1],p[2]+d_h ]).collect();
            let m2 = mesh.clone();
            mesh.append(&mut input_polygon_mesh);
            blender.n_gon(points, vec![], mesh);

            blender.n_gon(p2, vec![], m2);
         });
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

