#[cfg(test)]
use std::f32::EPSILON;

use nalgebra::{Point2, Point3, Rotation3, Similarity3, Vector3};
use super::{ContorTrait, Contour, AABB};

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
}

impl ContorTrait<3> for Contour3d{

    fn points<'a>(&'a self) -> core::slice::Iter<'a,Point3<f32>>{
        self.0.iter()
    }

    fn reverse_order(&mut self) {
        self.0.reverse();
    }

    /// returns true if the point is on or inside the projection of the contour onto the xy-plane
    fn x_distance_to_contour(&self,point:&Point3<f32>)->Option<f32>{

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

impl Contour3d {
    pub fn aabb(&self) -> AABB {
        let first_p = self.0[0];
        let mut aabb = AABB{
            x_min: first_p.x,
            x_max: first_p.x,
            y_min: first_p.y,
            y_max: first_p.y,
        };
        for point in self.points().skip(1){
            aabb.x_max = aabb.x_max.max(point.x);
            aabb.x_min = aabb.x_min.min(point.x);
            aabb.y_max = aabb.y_max.max(point.y);
            aabb.y_min = aabb.y_min.min(point.y);
        }
        return aabb
    }
    pub fn from_contour(contour:Contour,height:f32) -> Self {
        Self(contour.points.into_iter().map(|p| Point3::new(p.x,p.y,height) ).collect())
    }
    pub fn edges<'a>(&'a self) -> impl Iterator<Item = (&'a Point3<f32>,&'a Point3<f32>)>{
        self.points().zip( self.points().cycle().skip(1) )
    }
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        let (new_points,points_removed) = super::merge_by_distance(&self.0[..], distance);
        self.0 = new_points;
        return points_removed
    }
    pub fn rotate_scale(&mut self, angle:f32, scale:f32){
        let translation = Vector3::zeros();
        let axisangle = Vector3::z() * angle;
        let transform = Similarity3::new(translation,axisangle,scale);
        // let transform = Similarity3::from_parts(translation,axisangle,scale);
        let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), angle);

        let tanformed_points:Vec<_> = self.0.iter()
            // .map(|p| transform.transform_point(&p) )
            // .map(|p| transform * p )
            .map(|p| rot * p * scale )
            .collect();
        *self = Contour3d::from(tanformed_points);
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
