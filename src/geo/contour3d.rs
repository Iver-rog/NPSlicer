#[cfg(test)]
use std::f32::EPSILON;

use nalgebra::{Point2,Point3};
use super::Contour;

#[derive(Debug,Clone,PartialEq)]
pub struct Contour3d(pub Vec<Point3<f32>>);

impl From<Vec<Point3<f32>>> for Contour3d{
    fn from(points:Vec<Point3<f32>>) -> Self {
        Self(points)
    }
}
impl Contour3d {
    pub fn from_unchecked(raw_parts:Vec<Point3<f32>>) -> Self{
        Self(raw_parts)
    }
    pub fn points<'a>(&'a self) -> core::slice::Iter<'a,Point3<f32>>{
        self.0.iter()
    }
}

impl Contour3d {
    pub fn from_contour(contour:Contour,height:f32) -> Self {
        Self(contour.into_iter().map(|p| Point3::new(p.x,p.y,height) ).collect())
    }
    pub fn edges<'a>(&'a self) -> impl Iterator<Item = (&'a Point3<f32>,&'a Point3<f32>)>{
        self.points().zip( self.points().cycle().skip(1) )
    }
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        let (new_points,points_removed) = super::merge_by_distance(&self.0[..], distance);
        self.0 = new_points;
        return points_removed
    }

    /// returns true if the point is on or inside the projection of the contour onto the xy-plane
    fn x_distance_to_contour(&self,point:&Point2<f32>)->Option<f32>{

        let points_offset_by_one = self.0.iter()
            .skip(1)
            .chain( self.0.iter() );

        let intersections: Vec<f32> = self.0.iter()
            .zip(points_offset_by_one)
            // cast a ray from the test point towards the right direction allong 
            // the x-axis and check if the ray intersects the edge
            .filter_map(|(p1,p2)|{
                // check if the two points of the edge are on opposite sides of 
                // the horizontal line at test point's x value
                if (p1.y < point.y) == (p2.y <= point.y) { return None }
                // find where the edge intersects the horizontal line at test point's x value 
                // and check if the crossing point lies on the right side of the test point
                else {
                    let d_x = (point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x;
                    if point.x <= d_x { return Some(d_x-point.x)}
                    else {return None;}
                }
            })
            .collect();
        if intersections.len() % 2 == 1 { 
            return Some(intersections.into_iter()
                .fold(f32::INFINITY, |a, b| a.min(b))
                )
        }
        else { return None }
    }
}
#[test]
fn merge_by_distance_test(){
    let mut contour3d = contour3d!(
        [0.0,0.0,0.0], // 1
        [0.0,0.0,0.0], // 1
        [1.0,0.0,0.0], // 2
        [1.1,0.0,0.0], // 2
        [1.2,0.0,0.0], // 2
        [4.0,0.0,0.0], // 3
        [4.2,0.0,0.0], // 3
        [0.2,0.0,0.0]  // 1
    );
    let removed_vertices = contour3d.merge_by_distance(0.3);
    assert_eq!(
        contour3d,
        contour3d!(
            [0.1,        0.0,  0.0], // 1
            [1.1-EPSILON,0.0,  0.0], // 2
            [4.1,        0.0,  0.0]  // 3
        )
    );
    assert_eq!(removed_vertices,5);
}

use crate::contour3d;

#[macro_export]
macro_rules! contour3d {
    ( $( [$x:expr, $y:expr, $z:expr] ),* ) => {
        Contour3d::from(vec![
            $(
                Point3::new($x,$y,$z),
            )*
        ])
    };
}

#[test]
fn contour3d_macro_test(){
    assert_eq!(
        contour3d!([1.+2.,3.,4.],[1.,2.,3.],[1.,2.,3.]),
        Contour3d::from(vec![
            Point3::new(1.+2.,3.,4.),
            Point3::new(1.,2.,3.),
            Point3::new(1.,2.,3.),
        ])
    )
}
