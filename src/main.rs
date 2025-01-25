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
    init_logger();

    let mut blender = Blender::new();
    pipe_line(&mut blender);
    //straight_skeleton(&mut blender);

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
    let vertices = test_poly2();

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
    for (i,contour) in contours.into_iter().enumerate() {
        println!("\x1b[032mcreating skeleton for contour {i}\x1b[0m");
        //for point in contour.iter() {
        //    println!("Point2:new({},{}),",point[0],point[1]);
        //}
        let skeleton = match straight_skeleton::create_skeleton(
            contour.into_iter()
                .map(|vec| Point2::new(vec[0],vec[1]))
                .collect()
            ){
            Ok(skeleton) => skeleton,
            Err(err) =>{ println!("\x1b[032m{err}\x1b[0m"); panic!() }
        };

        blender.line_body(
            &skeleton.vertices.iter().map(|x| [x[0],x[1]]).collect::<Vec<[f32;2]>>(),
            skeleton.edges.clone()
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
