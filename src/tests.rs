// use crate;
#![cfg(test)]
use log::error;
use crate::skeleton;
use crate::Blender;
use crate::data;
use crate::geo::Contour;
use nalgebra::{Point2,Vector2};

#[test]
#[ignore = "requires blender"]
fn straight_skeleton() {
    let mut blender = Blender::new();
    //let vertices = data::test_poly();
    //let vertices = data::test_poly2();
    //let vertices:Vec<Point2<f32>> = data::test_poly3().into_iter().rev().collect();
    //let vertices = data::test_poly5();
    let mut vertices = data::test_poly8();
    vertices.invert();

    blender.polygon(&vertices, 0.0);

    //match skeleton::SkeletonBuilder::new(vertices){
    match skeleton::SkeletonBuilder::from_polygon(vertices){
        Err(error) => error!("\x1b[031m{error}\x1b[0m"),
        Ok(builder) => match builder.compute_skeleton_with_limit(40.) {
            Err(error) => error!("{error}"),
            //Ok((skeleton,debug_contours)) => {
            Ok(skeleton) => {
                //blender.line_body3d(
                //    skeleton.vertices.into_iter().map(|p|[p.x,p.y,p.z]).collect(), 
                //    skeleton.edges
                //    );
                let mesh = skeleton.skeleton_mesh();
                blender.n_gon(
                    skeleton.vertices.into_iter().map(|v| [v[0],v[1],v[2]]).collect(), 
                    skeleton.edges,
                    mesh
                    );
            } 
        }
    }
}


#[test]
#[ignore = "requires blender"]
fn straight_skeleton_with_bounds() {
    let mut blender = Blender::new();

    let mut polygon = data::test_poly8();
    let bounds = Contour::from(vec![
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

    polygon.invert();
    blender.polygon(&polygon, 0.0);
    blender.contour(&bounds, 0.0);
    let secound_polygon = polygon.outer_loop().points.iter().map(|p|p+Vector2::new(1.0,50.0)).collect();

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
