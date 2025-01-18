#![allow(unused)]
mod stl_op;
mod utils;
use utils::Blender;
mod straight_skeleton;
use nalgebra::Point2;
use log::{error};

#[cfg(test)]
mod test;

fn main(){
    env_logger::init();

    let mut blender = Blender::new();
    //pipe_line(&mut blender);
    straight_skeleton(&mut blender);

    blender.show();
}
#[allow(unused)]
fn straight_skeleton(blender:&mut Blender) {
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(2.0,0.0),
    //    Point2::new(2.0,1.0),
    //    Point2::new(0.0,1.0),
    //];
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(2.0,0.1),
    //    Point2::new(2.1,1.0),
    //    Point2::new(1.3,0.9),
    //    Point2::new(1.1,0.5),
    //    Point2::new(0.7,1.0),
    //    Point2::new(0.0,1.1),
    //];
    let vertices:Vec<Point2<f32>> = vec![
        Point2::new(10.0,50.),
        Point2::new(20.0,20.0),
        Point2::new(25.0,40.),
        Point2::new(63.5,10.),
        Point2::new(63.5,25.0),
        Point2::new(50.0,32.5),
        Point2::new(62.5,42.5),
        Point2::new(40., 52.0)
    ];

    let weights:Vec<f32> = vertices.iter().map(|_| 1.0 ).collect();
    let vertices_as_f32:Vec<[f32;3]> = vertices.clone().into_iter().map(|p|[ p[0],p[1], 0.0 ]).collect();

    match straight_skeleton::SkeletonBuilder::new(vertices.clone()){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(builder) => match builder.compute_skeleton() {
            Err(error) => error!("{error}"),
            Ok((skeleton,debug_contours)) => {
                blender.line_body(&skeleton.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), skeleton.edges);
                for contour in debug_contours {
                    blender.line_body(&contour.vertices.into_iter().map(|v| [v[0],v[1]]).collect(), contour.edges);
                }
            } 
        }
    }
    blender.edge_loop_points(&vertices_as_f32);
}
#[allow(unused)]
fn pipe_line(blender:&mut Blender){
    let contours = stl_op::main(blender);
    let first_contour = contours.iter().next().unwrap();

    let contour:Vec<Point2<f32>> = first_contour.to_owned().iter()
            .map(|vec| Point2::new(vec[0],vec[1]))
            .collect();

    let skeleton = match straight_skeleton::create_skeleton(contour.clone()){
        Ok(skeleton) => skeleton,
        Err(err) =>{ println!("{err}"); panic!() }
    };

    blender.line_body(
        &skeleton.vertices.iter().map(|x| [x[0],x[1]]).collect::<Vec<[f32;2]>>(),
        skeleton.edges.clone()
        );
}
