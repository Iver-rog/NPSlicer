#![allow(unused)]
mod contours;
mod stl_op;
mod skeleton;
mod utils;
use contours::{Contour,Polygon};
use contours::boolean::{boolean, clip_poly, i_simplify, offset, offset_line};
use i_overlay::core::overlay_rule::OverlayRule;
use utils::Blender;
mod data;

use std::f32::consts::PI;
use nalgebra::{Point2, Point3,Vector2};
use log::{error, Level};
use std::io::{Write, BufReader};
use std::fs::File;


fn main(){
    init_logger();

    let mut blender = Blender::new();
    //pipe_line(&mut blender);

    //straight_skeleton(&mut blender);
    //offset_polygon(&mut blender);
    //polygon_boolean(&mut blender);
    //straight_skeleton_with_bounds(&mut blender);
 
    //offset_layers(&mut blender);
    skeleton_layers(&mut blender);
    //boolean_layers2(&mut blender);

    blender.show();
}
#[allow(dead_code)]
fn polygon_boolean(blender:&mut Blender) {
    let poly = data::test_poly7();
    let clip = data::test_poly7_5();
    blender.polygon(&poly,0.0);
    blender.polygon(&clip,0.0);

    let boolean = poly.subtract(clip).into_iter().next().unwrap();
    blender.polygon(&boolean,0.0);
}
fn straight_skeleton_with_bounds(blender:&mut Blender) {
    let mut polygon = data::test_poly8();
    let bounds = Contour::new(vec![
        Point2::new( 4.5, 11.0),
        Point2::new( 4.5,-38.0),
        Point2::new(54.0,-38.0),
        Point2::new(54.0, 11.0),
    ]);
    let bounds = Contour::new(vec![
        Point2::new(54.0, 61.0),
        Point2::new( 4.5, 61.0),
        Point2::new( 4.5,-38.0),
        Point2::new(54.0,-38.0),
        // not
        Point2::new(54.0, 4.5),
        Point2::new(30.0, 4.5),
        Point2::new(30.0,9.8),
        Point2::new(54.0,9.8),
    ]);
    let p = Contour::new(vec![
        Point2::new(30.0, -4.0),
        Point2::new(20.0,-13.5),
        Point2::new(30.0,-23.0),
        Point2::new(38.0,-13.4),
    ]);
    //let mut polygon = Polygon::new(p,vec![]);
    polygon.invert();
    blender.polygon(&polygon, 0.0);
    blender.contour(&bounds, 0.0);
    let secound_polygon = polygon.outer_loop.points.iter().map(|p|p+Vector2::new(1.0,50.0)).collect();

    match skeleton::SkeletonBuilder::from_polygon(polygon){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(mut builder) => {
            builder.bounding_contour(bounds);
            builder.add_loop(secound_polygon).unwrap();
            match builder.compute_skeleton() {
                Err(error) => error!("{error}"),
                Ok((skeleton,debug_contours)) => {
                    blender.line_body3d(skeleton.vertices.into_iter().map(|v| [v[0],v[1],-0.3*v[2]]).collect(), skeleton.edges);
                    for polygon in debug_contours.iter().flatten() {
                        blender.polygon(polygon,0.0);
                    }
                } 
            }
        }
    }
}
#[allow(dead_code)]
fn straight_skeleton(blender:&mut Blender) {
    //let vertices = data::test_poly();
    //let vertices = data::test_poly2();
    //let vertices:Vec<Point2<f32>> = data::test_poly3().into_iter().rev().collect();
    //let vertices = data::test_poly5();
    let vertices = data::test_poly8();

    blender.polygon(&vertices, 0.0);

    //match skeleton::SkeletonBuilder::new(vertices){
    match skeleton::SkeletonBuilder::from_polygon(vertices){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(builder) => match builder.compute_skeleton() {
            Err(error) => error!("{error}"),
            Ok((skeleton,debug_contours)) => {
                let mesh = skeleton.generate_mesh();
                //dbg!(&mesh);
                blender.n_gon(
                    skeleton.vertices.into_iter().map(|v| [v[0],v[1],v[2]]).collect(), 
                    skeleton.edges,
                    mesh
                    );
                for polygon in debug_contours.iter().flatten() {
                    blender.polygon(polygon,0.0);
                }
            } 
        }
    }
    //blender.edge_loop_points(&vertices_as_f32);
}
#[allow(dead_code)]
fn offset_polygon(blender:&mut Blender){
    let polygon = data::test_poly6();

    blender.polygon(&polygon, 0.0);
    let i_overlay_poly = polygon.clone().offset(-2.0);

    i_overlay_poly.iter().for_each(|p|blender.polygon(p, 1.0));

    let skeleton = match skeleton::SkeletonBuilder::from_polygon(polygon.clone()){
        Ok(skeleton_builder) => skeleton_builder,
        Err(err) =>{ println!("\x1b[032m{err}\x1b[0m");
            return }
        };
    let offset_poly = match skeleton.offset_polygon(2.0){
        Ok(polygons) => polygons,
        Err(err) => {println!("{err}"); return;},
        };

    offset_poly.iter().for_each(|p|blender.polygon(p, 0.0));
}
#[allow(dead_code)]
fn boolean_layers2(blender:&mut Blender){
    //let file_path = "../mesh/bunny2.stl";
    //let file_path = "../mesh/stanford-armadillo.stl";
    let file_path = "../mesh/curved overhang.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, format!("input mesh"));

    let min_a = 0.1;   // min area for contour simplification
    let layer_h = 0.2; // layer height in mm
    let theta = PI/6.; // overhang angle
    let d_x = layer_h/theta.tan();

    let mut layers = stl_op::extract_planar_layers(&mesh, layer_h ,blender);

    layers.iter_mut().for_each(|layer| layer.iter_mut().for_each(|polygon|polygon.simplify(min_a)));
    println!("model has {} layers",layers.iter().filter(|n|n.len()!=0).count());
    //layers.iter().enumerate()
    //    .flat_map(|(i,layer)| layer.iter().map(move|l|(i,l) ) )
    //    .for_each(|(i,l)|{ blender.polygon(&l,i as f32 * 1.0) });

    let mut layers = layers.into_iter();
    let mut prev_layer = layers.next().unwrap();
    let mut new_overhangs = Vec::new();
    for (i, layer) in layers.enumerate(){//.skip(5){
        println!("layer {i}");

        let offset_sup = offset(prev_layer.clone(),d_x);
        let clip_lines = clip_poly(offset_sup.clone(), layer.clone());
        let additional_sup:Vec<Polygon> = clip_lines.clone()
            .into_iter()
            .map(|line| offset_line(line,d_x*3.))
            .flatten()
            .collect();

        let new_support1 = boolean(additional_sup.clone(), prev_layer.clone(), OverlayRule::Union);
        let new_support2 = boolean(new_support1.clone(), offset_sup.clone(), OverlayRule::Intersect);
        let mut new_support3 = boolean(new_support2.clone(), layer.clone(), OverlayRule::Intersect);
        new_support3.iter_mut().for_each(|p|p.simplify(min_a/2.0));
        let new_support4:Vec<Polygon> = new_support3.clone().into_iter().filter(|p|p.area().abs() > min_a).collect();
        let new_support5 = i_simplify(new_support4.clone(), min_a);

        //if new_support5.len() == 0 {
        //    println!("Error");
        //    dbg!{&prev_layer};
        //    prev_layer.iter().for_each(|p|blender.polygon(p, 0.0));
        //    offset_sup.iter().for_each(|p|blender.polygon(p, 0.0));
        //    layer.iter().for_each(|p|blender.polygon(p, 1.0));
        //    clip_lines.iter().for_each(|line|blender.line(line,2.0));
        //    additional_sup.iter().for_each(|p|blender.polygon(p, 3.0));
        //    new_support1.iter().for_each(|p|blender.polygon(p, 4.0));
        //    new_support2.iter().for_each(|p|blender.polygon(p, 5.0));
        //    new_support3.iter().for_each(|p|blender.polygon(p, 6.0));
        //    new_support4.iter().for_each(|p|blender.polygon(p, 7.0));
        //    new_support5.iter().for_each(|p|blender.polygon(p, 7.0));
        //    break
        //}

        new_overhangs.push(new_support5.clone());
        prev_layer = new_support5;
    }

    println!("created {} layers",new_overhangs.len());

    for (i,merged_overhang) in new_overhangs.iter().enumerate() {
        let layer_height = layer_h*((i+1) as f32);
        for polygon in merged_overhang {
            blender.polygon(&polygon, layer_height);
        }
    }
    let bounding_box = Contour::new(
            vec![
            Point2::new( 32.4, 36.1),
            Point2::new(-39.1, 36.2),
            Point2::new(-39.0,-31.3),
            Point2::new( 31.6,-31.4),
            ]);
    let skeletons = new_overhangs.iter()
        .enumerate()
        .flat_map(|(i,layer)| layer.iter().map(move|polygon|(i,polygon)))
        .map(|(i,p)|{
            let polygon = Polygon::new(
                bounding_box.clone(),
                vec![p.outer_loop.clone()],
            );
            (i, polygon)
        })
        .filter_map(|(i,polygon)|{
            println!("skeleton for layer {i}");
            match skeleton::skeleton_from_polygon(polygon.clone()){
                Ok(skeleton) => Some((i,skeleton)),
                Err(err) => {
                    println!("\x1b[031m{err}\x1b[0m");
                    dbg!(&polygon); 
                    None
                }
            }
        })
        .for_each(|(i,skeleton)|{ 
            let layer_height = (i+1) as f32 * layer_h;
            blender.line_body3d(
                skeleton.vertices.iter().map(|x| [x[0],x[1],layer_height-x[2]*theta.tan()]).collect::<Vec<[f32;3]>>(),
                skeleton.edges
                );
         });
}
#[allow(dead_code)]
fn boolean_layers(blender:&mut Blender){
    //let file_path = "../mesh/bunny2.stl";
    let file_path = "../mesh/stanford-armadillo.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, format!("input mesh"));

    let layers = stl_op::extract_planar_layers(&mesh, 1.0 ,blender);
    //layers.iter()
    //    .enumerate()
    //    .flat_map(|(i,layer)| 
    //        layer.iter()
    //        .map(move|l|(i,l) ) )
    //    .for_each(|(i,l)|{ blender.polygon(&l,i as f32 * 1.0) });

    let masks = layers.clone().into_iter();
    let overhang_profiles = layers.clone().into_iter()
        .skip(1)
        .zip(masks)
        //.map(|(n,n_minus_1)| filtered_boolean(n,n_minus_1,OverlayRule::Difference,0.1) );
        .map(|(n,n_minus_1)| boolean(n,n_minus_1,OverlayRule::Difference) );

    let l_n = 10; // how many layers down to merge overhangs
    let overhang_profiles:Vec<Vec<Polygon>> = overhang_profiles.collect();
    let merged_overhangs = (0..overhang_profiles.len())
        .map(|i|( i, i.saturating_sub(l_n)..i ) )
        .map(|(i, mask_ndxs)|{ 
            let mut layer = layers[i].clone();
            //let mut layer = overhang_profiles[i].clone();
            let mut masks = mask_ndxs.map(|n|overhang_profiles[n].clone());
            match masks.next(){
                Some(mut mask) => {
                    for next_mask in masks {
                        //layer = filtered_boolean(layer,mask,OverlayRule::Union);
                        mask = boolean(mask,next_mask,OverlayRule::Union);
                        }
                    layer = boolean(layer,mask,OverlayRule::Difference);
                }
                None => ()
            }
            (i,layer)
        });

    for (i,merged_overhang) in merged_overhangs {
        let layer_height = 1.0*(i as f32);
        for polygon in merged_overhang {
            blender.polygon(&polygon, layer_height);
        }
    }
}
#[allow(dead_code)]
fn offset_layers(blender:&mut Blender){
    //let file_path = "../mesh/bunny2.stl";
    let file_path = "../mesh/stanford-armadillo.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, format!("input mesh"));

    let layers = stl_op::extract_planar_layers(&mesh, 0.2 ,blender);
    let nr_layers = layers.len();
    for (i, layer) in layers.into_iter().enumerate() {
        for mut polygon in layer {
            polygon.simplify(0.08);
            let layer_height = i as f32 * 0.2;
            print!("layer {i} of {} ",nr_layers);
            println!("contour {}",blender.line_objects.len());

            blender.edge_loop_points(
                &polygon.outer_loop.points.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>(),
                );
            for hole in polygon.holes.iter(){
                blender.edge_loop_points(
                    &hole.points.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>()
                    );
                }

            //let skeleton = match skeleton::SkeletonBuilder::from_polygon(polygon.clone()){
            //    Ok(skeleton_builder) => skeleton_builder,
            //    Err(err) =>{ println!("\x1b[032m{err}\x1b[0m");
            //        return }
            //    };
            //let offset_polygon = match skeleton.polygon_at_time(2.0){
            //    Ok(polygons) => polygons,
            //    Err(err) => {println!("{err}"); return;},
            //    };
            //for polygon in offset_polygon {
            //blender.polygon(&polygon,layer_height);
            //}
        }
    }
}
#[allow(dead_code)]
fn skeleton_layers(blender:&mut Blender){
    //let file_path = "../mesh/bunny2.stl";
    let file_path = "../mesh/stanford-armadillo.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, format!("input mesh"));

    let layers = stl_op::extract_planar_layers(&mesh, 2.0 ,blender);
    let nr_layers = layers.len();
    for (i, layer) in layers.into_iter().enumerate() {
        println!("layer {i} of {nr_layers}");
        for polygon in layer {
            let layer_height = i as f32 * 2.0;
            //blender.edge_loop_points(
            //    &polygon.outer_loop.points.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>(),
            //    );
            //for hole in polygon.holes.iter(){
            //    blender.edge_loop_points(
            //        &hole.points.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>()
            //        );
            //    }

            let skeleton = match skeleton::skeleton_from_polygon(polygon.clone()){
                Ok(skeleton) => skeleton,
                Err(err) =>{ println!("\x1b[031m{err}\x1b[0m");
                    dbg!(&polygon);continue }
            };
            //blender.line_body3d(
            //    skeleton.vertices.iter().map(|p| [p[0],p[1],p[2]*0.5+layer_height]).collect(),
            //    skeleton.edges.clone()
            //    );
            let mesh = skeleton.generate_mesh();
            let points = skeleton.vertices.into_iter().map(|p|[p.x,p.y,p.z*0.5+layer_height]).collect();
            let edges = skeleton.edges;
            blender.n_gon(points, edges, mesh);

        }
    }
}
#[allow(dead_code)]
fn pipe_line(blender:&mut Blender){
    let file_path = "../mesh/bunny2.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);
    let mesh = stl_io::read_stl(& mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, "input file".to_string());

    let overhangs = stl_op::extract_overhangs(&mesh,-PI/6.0);
    for (i,mesh_region) in overhangs.iter().enumerate() { blender.save_mesh(mesh_region, &mesh.vertices,format!("overhang {i}")) };

    let contours = stl_op::extract_contours_larger_than(overhangs, &mesh, 20.0);
    for contour in contours.iter() { blender.edge_loop(&contour,&mesh); }

    let edge_loops_points:Vec<Vec<Point3<f32>>> = contours.into_iter()
        .map(|contour| contour.into_iter()
            .map(|index| mesh.vertices[index])
            .map(|point| Point3::new(point[0],point[1],point[0]))
            .collect() 
            )
        .collect();

    for (i,contour) in edge_loops_points.into_iter()
        .map(|loop_of_points| 
            Polygon::new(
                Contour::new( loop_of_points.into_iter().map(|p|p.xy()).collect() ),
                    vec![]
                    )
            )
            .enumerate() {
        println!("\x1b[032mcreating skeleton for contour {i}\x1b[0m");

        let skeleton = match skeleton::skeleton_from_polygon( contour ){
            Ok(skeleton) => skeleton,
            Err(err) =>{ println!("\x1b[032m{err}\x1b[0m"); continue }
        };

        blender.line_body2d(
            &skeleton.vertices.iter().map(|x| [x[0],x[1]]).collect(),
            skeleton.edges
            );
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

