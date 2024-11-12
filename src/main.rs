#![allow(unused)]
mod stl_op;
mod utils;
use utils::Blender;
mod straight_skeleton;
mod medial_axis;
use nalgebra::Point2;
use std::cmp::max;

fn main(){
    let mut blender = Blender::new();
     pipe_line(&mut blender);
    //straight_skeleton(&mut blender);
    //medial_axis(&mut blender);

    blender.show();
}

fn straight_skeleton(blender:&mut Blender) {
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(0.0,1.0),
    //    Point2::new(1.0,1.0),
    //    Point2::new(1.0,0.0),
    //];
    //let vertices = vec![
    //    Point2::new(0.0,0.0),
    //    Point2::new(2.0,0.0),
    //    Point2::new(2.0,1.0),
    //    Point2::new(0.0,1.0),
    //];
    let vertices = vec![
        Point2::new(40., 52.0),
        Point2::new(62.5,42.5),
        Point2::new(50.0,32.5),
        Point2::new(63.5,25.0),
        Point2::new(63.5,10.),
        Point2::new(25.0,40.),
        Point2::new(20.0,20.0),
        Point2::new(10.0,50.)
    ];
    let weights:Vec<f32> = vertices.iter().map(|x| 1.0 ).collect();
    let skeleton = straight_skeleton::create_weighted_skeleton(&vertices, &weights).unwrap();
    let vertices_as_f32:Vec<[f32;3]> = vertices.clone().into_iter().map(|p|[ p[0],p[1], 0.0 ]).collect();

    let mut skel_points:Vec<[f32;2]> = skeleton.vertices.iter()
        .map(|x| [x[0],x[1]])
        //.filter(|[p1,p2]| p1 != p2)
        .collect::<Vec<[f32;2]>>();
    println!("skel_points: {skel_points:?}");

    let mut skel_edges:Vec<[usize;2]> = skeleton.edges.clone()
        .into_iter()
        .filter(|[p1,p2]| p1 != p2)
        .collect();
    println!("skel_edges: {skel_edges:?}");

    blender.edge_loop_points(&vertices_as_f32);
    
    blender.line_body(
        //&skeleton.vertices.iter().map(|x| [x[0],x[1]]).collect::<Vec<[f32;2]>>(),
        &skel_points,
        skel_edges
        );
}

fn medial_axis(blender:&mut Blender) {
    let vertices = vec![
        Point2::new(40.0,52.0),
        Point2::new(62.5,42.5),
        Point2::new(50.0,32.5),
        Point2::new(63.5,25.0),
        Point2::new(63.5,10.0),
        Point2::new(25.0,40.0),
        Point2::new(20.0,20.0),
        Point2::new(10.0,50.0)
    ];
    let mut mat = medial_axis::MedialAxisTransform::new(vertices.clone(), 0.01);
    mat.compute();
    let skeleton = mat.get_skeleton();

    println!("Skeleton Edges: {:?}", skeleton);

    blender.edge_loop_points(&vertices.into_iter().map(|p| [p[0] as f32,p[1] as f32,0.0]).collect());
    blender.line_body_points(&skeleton);
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
    let skeleton = straight_skeleton::create_weighted_skeleton(&profile,&weights).unwrap();

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

