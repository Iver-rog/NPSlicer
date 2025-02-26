#![allow(unused)]
mod contours;
mod stl_op;
mod skeleton;
mod utils;
use contours::{Contour,Polygon};
use utils::Blender;
mod data;

use std::f32::consts::PI;
use nalgebra::Point3;
use log::{error, Level};
use std::io::{Write, BufReader};
use std::fs::File;


fn main(){
    init_logger();

    let mut blender = Blender::new();
    //pipe_line(&mut blender);
    //straight_skeleton(&mut blender);
    //straight_skeleton_copy(&mut blender);
    //skeleton_layers(&mut blender);
    offset_layers(&mut blender);
    //offset_polygon(&mut blender);

    blender.show();
}
//#[allow(unused)]
fn straight_skeleton_copy(blender:&mut Blender) {
    //let vertices = test_poly();
    //let vertices = test_poly2();
    //let vertices:Vec<Point2<f32>> = test_poly3().into_iter().rev().collect();
    let vertices = data::test_poly4();

    let vertices_as_f32:Vec<[f32;3]> = vertices.iter().map(|p|[ p[0],p[1], 0.0 ]).collect();

    match skeleton::SkeletonBuilder::new(vertices){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(builder) => match builder.compute_skeleton() {
            Err(error) => error!("{error}"),
            Ok((skeleton,debug_contours)) => {
                blender.line_body2d(&skeleton.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), skeleton.edges);
                for contour in debug_contours {
                    blender.line_body2d(&contour.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), contour.edges);
                }
            } 
        }
    }
    blender.edge_loop_points(&vertices_as_f32);
}
fn straight_skeleton(blender:&mut Blender) {
    //let vertices = data::test_poly();
    //let vertices = data::test_poly2();
    //let vertices:Vec<Point2<f32>> = data::test_poly3().into_iter().rev().collect();
    //let vertices = data::test_poly5();
    let vertices = data::test_poly7();

    blender.polygon(&vertices, 0.0);

    //match skeleton::SkeletonBuilder::new(vertices){
    match skeleton::SkeletonBuilder::from_polygon(vertices){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(builder) => match builder.compute_skeleton() {
            Err(error) => error!("{error}"),
            Ok((skeleton,debug_contours)) => {
                blender.line_body2d(&skeleton.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), skeleton.edges);
                for contour in debug_contours {
                    blender.line_body2d(&contour.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), contour.edges);
                }
            } 
        }
    }
    //blender.edge_loop_points(&vertices_as_f32);
}
fn offset_polygon(blender:&mut Blender){
    let polygon = data::test_poly6();

    blender.polygon(&polygon, 0.0);

    let skeleton = match skeleton::SkeletonBuilder::from_polygon(polygon.clone()){
        Ok(skeleton_builder) => skeleton_builder,
        Err(err) =>{ println!("\x1b[032m{err}\x1b[0m");
            return }
        };
    let offset_polygon = match skeleton.polygon_at_time(2.0){
        Ok(polygons) => polygons,
        Err(err) => {println!("{err}"); return;},
        };
    for polygon in offset_polygon {
        blender.polygon(&polygon, 0.0);
    }
}
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
#[allow(unused)]
fn skeleton_layers(blender:&mut Blender){
    //let file_path = "../mesh/bunny2.stl";
    let file_path = "../mesh/stanford-armadillo.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let mesh = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    blender.save_mesh(&mesh.faces, &mesh.vertices, format!("input mesh"));

    let layers = stl_op::extract_planar_layers(&mesh, 0.2 ,blender);
    let nr_layers = layers.len();
    for (i,mut layer) in layers.into_iter().enumerate() {
        for mut polygon in layer {
            let layer_height = i as f32 * 0.2;
            println!("contour {i} of {}",nr_layers);

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

            blender.line_body3d(
                skeleton.vertices.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>(),
                skeleton.edges
                );
        }
    }
}
#[allow(unused)]
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

