#![allow(unused)]
mod stl_op;
mod utils;
use utils::Blender;
mod straight_skeleton;
use nalgebra::Point2;
use std::cmp::max;

#[cfg(test)]
mod test;

fn main(){
    let mut blender = Blender::new();
    pipe_line(&mut blender);
    //straight_skeleton(&mut blender);

    blender.show();
}

fn straight_skeleton(blender:&mut Blender) {
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(2.0,0.0),
    //    Point2::new(2.0,1.0),
    //    Point2::new(0.0,1.0),
    //];
    let vertices = vec![
        Point2::new(0.0,0.0),
        Point2::new(2.0,0.1),
        Point2::new(2.1,1.0),
        Point2::new(1.3,0.9),
        Point2::new(1.1,0.5),
        Point2::new(0.7,1.0),
        Point2::new(0.0,1.1),
    ];
    //let vertices = vec![
    //    Point2::new(40., 52.0),
    //    Point2::new(62.5,42.5),
    //    Point2::new(50.0,32.5),
    //    Point2::new(63.5,25.0),
    //    Point2::new(63.5,10.),
    //    Point2::new(25.0,40.),
    //    Point2::new(20.0,20.0),
    //    Point2::new(10.0,50.)
    //];
    let weights:Vec<f32> = vertices.iter().map(|x| 1.0 ).collect();
    let vertices_as_f32:Vec<[f32;3]> = vertices.clone().into_iter().map(|p|[ p[0],p[1], 0.0 ]).collect();

    match straight_skeleton::create_weighted_skeleton(vertices.clone(), &weights){
        Ok(skeleton) =>{
            let mut skel_points:Vec<[f32;2]> = skeleton.vertices.iter()
                .map(|x| [x[0],x[1]])
                .collect::<Vec<[f32;2]>>();
            println!("skel_points: {skel_points:?}");

            let mut skel_edges:Vec<[usize;2]> = skeleton.edges.clone()
                .into_iter()
                .filter(|[p1,p2]| p1 != p2)
                .collect();
            println!("skel_edges: {skel_edges:?}");

            blender.line_body(
                &skel_points,
                skel_edges
                );
        }
        Err(error) => println!("\x1b[031m{error}\x1b[0m")
    }



    blender.edge_loop_points(&vertices_as_f32);
    
}


fn pipe_line(blender:&mut Blender){
    let profiles = stl_op::main(blender);
    let first_profile = profiles.iter().next().unwrap();

    let profile:Vec<Point2<f32>> = first_profile.to_owned().iter()
            .map(|vec| Point2::new(vec[0],vec[1]))
            .collect();

    let weights: Vec<f32> = profile.iter().map(|_| 1.0 ).collect();

    println!("profile points: {}",
        profile.len(),
        );
    let skeleton = straight_skeleton::create_weighted_skeleton(profile.clone(),&weights).unwrap();

    println!("Points: {} | edges: {} | larges point index:{}",
        skeleton.vertices.len(),
        skeleton.edges.len(),
        skeleton.edges.iter().map(|[p1,p2]| max(p1,p2)).max().unwrap()
        );
    //dbg!(&skeleton);

    blender.line_body(
        &skeleton.vertices.iter().map(|x| [x[0],x[1]]).collect::<Vec<[f32;2]>>(),
        skeleton.edges.clone()
        );

    let points = profile.iter()
        .chain( skeleton.vertices.iter() )
        .map(|x| [x[0],x[1]] )
        .collect::<Vec<[f32;2]>>();

    //let points = skeleton.vertices.iter()
    //    .chain( profile.iter() )
    //    .map(|x| [x[0],x[1]])
    //    .collect::<Vec<[f32;2]>>();

    let mut edges: Vec<[usize;2]>= (0..profile.len())
        .map(|i| [i,i+1] )
        .collect();

    //let mut edges = Vec::new();
    edges.push([profile.len(),0]);
    edges.append( &mut skeleton.edges.clone() );

   println!("Points: {} | edges: {} | larges point index:{}",
        points.len(),
        edges.len(),
        edges.iter().map(|[p1,p2]| max(p1,p2)).max().unwrap()
        );

    blender.line_body(&points, edges);

}

