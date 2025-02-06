#![allow(unused)]
mod contours;
mod stl_op;
mod skeleton;
mod utils;
use contours::{polygons_from_contours,Contour};
use utils::Blender;

use std::f32::consts::PI;
use nalgebra::{Point2,Point3};
use log::{error, Level};
use core::f32;
use std::io::{Write, BufReader};
use std::fs::File;


fn main(){
    init_logger();

    let mut blender = Blender::new();
    //pipe_line(&mut blender);
    //straight_skeleton(&mut blender);
    //skeleton_layers(&mut blender);
    offset_polygon(&mut blender);

    blender.show();
}
//#[allow(unused)]
fn straight_skeleton(blender:&mut Blender) {
    //let vertices = test_poly();
    //let vertices = test_poly2();
    let vertices:Vec<Point2<f32>> = test_poly3().into_iter().rev().collect();

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
fn offset_polygon(blender:&mut Blender){
    let contour = Contour::new(test_poly2());
    let polygon = polygons_from_contours(vec![contour]).into_iter().next().unwrap();

    blender.edge_loop_points(
        &polygon.outer_loop.points.iter().map(|x| [x[0],x[1],0.0]).collect::<Vec<[f32;3]>>(),
        );

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
    blender.edge_loop_points(
        &polygon.outer_loop.points.iter().map(|x| [x[0],x[1],0.0]).collect::<Vec<[f32;3]>>(),
        );
    for hole in polygon.holes.iter(){
        blender.edge_loop_points(
            &hole.points.iter().map(|x| [x[0],x[1],0.0]).collect::<Vec<[f32;3]>>()
            );
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

    let layers = stl_op::extract_planar_layers(&mesh, 0.3 ,blender);
    let nr_layers = layers.len();
    for (i,mut layer) in layers.into_iter().enumerate() {
        for mut polygon in layer {
        let layer_height = i as f32 * 0.3;
        println!("contour {i} of {}",nr_layers);

        blender.edge_loop_points(
            &polygon.outer_loop.points.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>(),
            );

        //let skeleton = match skeleton::create_skeleton(contour){
        //    Ok(skeleton) => skeleton,
        //    Err(err) =>{ println!("\x1b[032m{err}\x1b[0m"); continue }
        //};
        //
        //blender.line_body3d(
        //    skeleton.vertices.iter().map(|x| [x[0],x[1],layer_height]).collect::<Vec<[f32;3]>>(),
        //    skeleton.edges
        //    );
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

    for (i,contour) in edge_loops_points.into_iter().enumerate() {
        println!("\x1b[032mcreating skeleton for contour {i}\x1b[0m");

        let skeleton = match skeleton::create_skeleton(
            contour.into_iter().map(|point| point.xy() ).collect()
            ){
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

fn test_poly() -> Vec<Point2<f32>>{
    vec![
        Point2::new(-17.75, 14.74),
        Point2::new(-17.92, 13.70),
        Point2::new(-18.61, 13.02),
        Point2::new(-18.75, 11.93),
        Point2::new(-18.78, 10.78),
        Point2::new(-18.69, 9.76),
        Point2::new(-19.26, 8.92),
        Point2::new(-18.98, 7.93),
        Point2::new(-19.37, 7.12),
        Point2::new(-19.68, 6.06),
        Point2::new(-19.63, 4.98),
        Point2::new(-19.35, 4.17),
        Point2::new(-19.62, 3.09),
        Point2::new(-19.99, 2.17),
        Point2::new(-19.81, 1.54),
        Point2::new(-19.31, 2.28),
        Point2::new(-18.55, 2.39),
        Point2::new(-17.76, 2.63),
        Point2::new(-18.50, 1.99),
        Point2::new(-17.35, 2.18),
        Point2::new(-16.38, 2.20),
        Point2::new(-16.76, 2.80),
        Point2::new(-15.82, 2.32),
        Point2::new(-15.86, 3.12),
        Point2::new(-15.80, 4.09),
        Point2::new(-15.74, 5.12),
        Point2::new(-15.70, 6.19),
        Point2::new(-15.63, 7.19),
        Point2::new(-15.58, 8.24),
        Point2::new(-15.50, 9.32),
        Point2::new(-15.89, 10.14),
        Point2::new(-15.84, 11.29),
        Point2::new(-15.84, 12.26),
        Point2::new(-15.92, 13.16),
        Point2::new(-16.28, 14.24),
        Point2::new(-16.29, 15.28),
        Point2::new(-15.99, 16.13),
        Point2::new(-15.77, 16.89),
        Point2::new(-15.58, 17.83),
        Point2::new(-15.37, 18.70),
        Point2::new(-16.18, 18.82),
        Point2::new(-16.96, 18.22),
        Point2::new(-17.18, 17.58),
        Point2::new(-17.41, 16.73),
        Point2::new(-17.59, 15.77)
        ]
}
fn test_poly2()->Vec<Point2<f32>>{
    vec![
        Point2::new(-21.0656,-0.0768684),
        Point2::new(-21.4896,-0.92803997),
        Point2::new(-21.9739,-1.8365599),
        Point2::new(-21.9711,-2.66609),
        Point2::new(-22.3991,-3.4879),
        Point2::new(-22.7312,-4.42141),
        Point2::new(-22.2332,-5.37018),
        Point2::new(-22.2147,-6.32923),
        Point2::new(-21.7527,-7.22097),
        Point2::new(-21.7995,-8.09682),
        Point2::new(-21.9296,-9.02785),
        Point2::new(-21.4486,-9.85272),
        Point2::new(-20.7735,-10.0156),
        Point2::new(-20.1416,-10.9677),
        Point2::new(-19.2548,-11.7128),
        Point2::new(-18.6434,-11.6845),
        Point2::new(-18.3952,-12.5417),
        Point2::new(-17.6743,-12.4455),
        Point2::new(-17.5128,-13.160599),
        Point2::new(-16.6486,-13.7217),
        Point2::new(-15.7758,-13.685101),
        Point2::new(-14.8194,-14.247401),
        Point2::new(-14.033999,-14.0092),
        Point2::new(-13.024299,-14.261299),
        Point2::new(-11.9878,-14.4597),
        Point2::new(-10.9029,-14.5645),
        Point2::new(-10.2292,-14.228),
        Point2::new(-11.3614,-13.9362),
        Point2::new(-11.801,-13.327399),
        Point2::new(-10.7544,-13.6423),
        Point2::new(-11.1204,-12.983599),
        Point2::new(-11.483,-12.2903),
        Point2::new(-11.5707,-11.5342),
        Point2::new(-12.1643,-10.9404),
        Point2::new(-12.5252,-10.1277),
        Point2::new(-11.671,-10.714),
        Point2::new(-12.0737,-9.60298),
        Point2::new(-12.5398,-8.53232),
        Point2::new(-11.8202,-9.00018),
        Point2::new(-11.9923,-8.11622),
        Point2::new(-11.9114,-7.22032),
        Point2::new(-11.6362,-6.24724),
        Point2::new(-11.6214,-5.1736),
        Point2::new(-12.4357,-4.3233),
        Point2::new(-13.038501,-3.3339198),
        Point2::new(-13.615901,-2.28475),
        Point2::new(-13.697501,-1.06593),
        Point2::new(-13.6741,0.0061502997),
        Point2::new(-13.477899,0.98038894),
        Point2::new(-12.7512,1.69467),
        Point2::new(-12.3436,2.5648),
        Point2::new(-12.8386,2.84869),
        Point2::new(-11.9447,3.31573),
        Point2::new(-11.047,3.84368),
        Point2::new(-11.8241,4.07359),
        Point2::new(-10.9157,4.43313),
        Point2::new(-11.8968,4.69435),
        Point2::new(-11.4538,5.24428),
        Point2::new(-12.4968,5.22142),
        Point2::new(-13.3802,5.62227),
        Point2::new(-14.2164,5.13938),
        Point2::new(-15.2896,5.08885),
        Point2::new(-16.4623,4.77244),
        Point2::new(-17.5989,4.22375),
        Point2::new(-17.9771,3.4608898),
        Point2::new(-18.8642,2.89482),
        Point2::new(-19.1423,2.00642),
        Point2::new(-19.9244,1.60691),
        Point2::new(-20.5316,0.8208281)
    ]
}
fn test_poly3() -> Vec<Point2<f32>>{
    vec![
        Point2::new(-3.9939733,-56.926804),
        Point2::new(-4.23258,-56.471874),
        Point2::new(-4.3377643,-56.002693),
        Point2::new(-4.2944617,-55.611492),
        Point2::new(-4.17033,-55.35061),
        Point2::new(-3.7567666,-55.10629),
        Point2::new(-3.0308802,-54.187786),
        Point2::new(-1.5760118,-54.118313),
        Point2::new(-1.3239056,-54.13446),
        Point2::new(-1.1533076,-54.30105),
        Point2::new(-0.29002458,-55.205486),
        Point2::new(-0.2559766,-55.55263),
        Point2::new(-0.31183338,-56.032677),
        Point2::new(-0.43847734,-56.253956),
        Point2::new(-1.0216255,-56.8932),
        Point2::new(-1.5334679,-57.443157),
        Point2::new(-2.095561,-57.562836),
        Point2::new(-2.3588085,-57.64717),
        Point2::new(-3.142513,-57.581615),
        Point2::new(-3.2289486,-57.549732),
    ]
}
