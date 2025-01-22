#![allow(unused)]
mod stl_op;
mod utils;
use utils::Blender;
mod straight_skeleton;
use nalgebra::Point2;
use log::{error, Level};
use std::io::Write;

#[cfg(test)]
mod test;

fn main(){
    env_logger::builder()
        .format(|buf, record|{
            match record.level() {
                Level::Error => write!(buf,"\x1b[031mError\x1b[0m"),
                Level::Warn  => write!(buf,"\x1b[033mWarn \x1b[0m"),
                Level::Info  => write!(buf,"\x1b[032mInfo \x1b[0m"),
                Level::Debug => write!(buf,"\x1b[034mDebug\x1b[0m"),
                Level::Trace => write!(buf,"\x1b[035mTrace\x1b[0m"),
            };
            writeln!(buf,": {}",record.args())
        })
    .init();

    let mut blender = Blender::new();
    //pipe_line(&mut blender);
    straight_skeleton(&mut blender);

    blender.show();
}
#[allow(unused)]
fn straight_skeleton(blender:&mut Blender) {
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(2.0,0.1),
    //    Point2::new(2.1,1.0),
    //    Point2::new(1.3,0.9),
    //    Point2::new(1.1,0.5),
    //    Point2::new(0.7,1.0),
    //    Point2::new(0.0,1.1),
    //];
    //let vertices:Vec<Point2<f32>> = vec![
    //    Point2::new(10.0,50.),
    //    Point2::new(20.0,20.0),
    //    Point2::new(25.0,40.),
    //    Point2::new(63.5,10.),
    //    Point2::new(63.5,25.0),
    //    Point2::new(50.0,32.5),
    //    Point2::new(62.5,42.5),
    //    Point2::new(40., 52.0)
    //];
    let vertices = test_poly();

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
