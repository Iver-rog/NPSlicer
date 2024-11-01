#![allow(unused)]
mod stl_op;
mod utils;
use utils::Blender;
mod straight_skeleton;
use nalgebra::Point2;
use std::cmp::max;

fn main(){
    let mut blender = Blender::new();
    let profiles = stl_op::main(&mut blender);
    let first_profile = profiles.iter().next().unwrap();

    let profile:Vec<Point2<f32>> = first_profile.to_owned().iter()
            .map(|vec| Point2::new(vec[0],vec[1]))
            .collect();

    let weights: Vec<f32> = profile.iter().map(|_| 1.0 ).collect();

    let skeleton = straight_skeleton::create_weighted_skeleton(&profile,&weights).unwrap();

    println!("Points: {} | edges: {} | larges point index:{}",
        skeleton.vertices.len(),
        skeleton.edges.len(),
        skeleton.edges.iter().map(|[p1,p2]| max(p1,p2)).max().unwrap()
        );

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

    blender.show();
}

