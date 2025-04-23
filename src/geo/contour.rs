use nalgebra::Vector2;
use nalgebra::{Point2,Matrix2};
use nalgebra_glm::cross2d;
use super::ContorTrait;
use super::Enclosed;
use super::AABB;
use super::Contour3d;

#[cfg(test)]
use std::f32::consts::PI;

#[derive(Debug,Clone,PartialEq)]
pub struct Contour{
    pub area:f32,
    pub aabb:AABB,
    pub points: Vec<Point2<f32>>
}
impl From<Contour3d> for Contour{
    fn from(contour3d:Contour3d) -> Self {
        Self::from(
            contour3d.0.into_iter()
            .map(|point3| point3.xy()) 
            .collect::<Vec<Point2<f32>>>()
        )
    }
}
impl Enclosed for Contour{
    fn area(&self) -> f32 {
        self.area
    }
    fn point_is_inside(&self,point:&Point2<f32>)->bool{
        // returns true if a point is on or inside the contour
        // TODO: points on a line on the left side of polygons are
        // counted as outside even thoug they should not be.
        //if !self.aabb.point_is_inside(&point) { return false }

        let points_offset_by_one = self.points.iter()
            .cycle()
            .skip(1);

        //#[cfg(test)]
        //println!("is {point} inside {:?}?",self.points);
        let intersections: usize = self.points.iter()
            .zip(points_offset_by_one)
            .map(|(p1,p2)|{
                // special case: check if the ray intersects both vertices of the edge
                let ray_is_parallel_to_the_line = (point.y == p1.y) && (point.y == p2.y);
                // check if the ray will intersect the line
                let ray_crosses_the_edge = (p1.y <= point.y) != (p2.y <= point.y);
                // check if the intersection point is on the correct side of the point
                let intersection_is_infront_of_ray = point.x <= ((point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x);
                //#[cfg(test)]
                //{
                //    println!("edge ({p1}-{p2}) ");
                //    println!("{}ray_is_parallel_to_the_line\x1b[0m",
                //        if ray_is_parallel_to_the_line {"\x1b[032m"}else{""});
                //    println!("{}ray_crosses_the_edge\x1b[0m",
                //        if ray_crosses_the_edge {"\x1b[032m"}else{""});
                //    println!("{}intersection_is_infront_of_ray {}\x1b[0m",
                //        if intersection_is_infront_of_ray {"\x1b[032m"}else{""},
                //        ((point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x) );
                //    println!("");
                //}

                if ray_is_parallel_to_the_line {false}//(point.x <= p1.x) || (point.x <= p2.x) }
                else{ ray_crosses_the_edge && intersection_is_infront_of_ray }
            })
            .map(|bool| match bool { true => 1, false => 0, } )
            .sum();
            //#[cfg(test)]
            //println!("\x1b[033mintersections: {intersections}\x1b[0m");

            return intersections % 2 == 1
    }
}
impl From<Vec<Point2<f32>>> for Contour {
    fn from(points:Vec<Point2<f32>>) -> Self {
        debug_assert!(3 <= points.len());
        let first_point = points[0];
        let last_point = points[points.len()-1];

        let mut aabb = AABB{
            x_max: first_point.x,
            x_min: first_point.x,
            y_max: first_point.y,
            y_min: first_point.y,
        };
        let mut area = last_point.x*first_point.y-first_point.x*last_point.y;

        let mut prev_point = first_point.clone();
        for point in points.iter().skip(1) {
            aabb.x_max = aabb.x_max.max(point.x);
            aabb.x_min = aabb.x_min.min(point.x);
            aabb.y_max = aabb.y_max.max(point.y);
            aabb.y_min = aabb.y_min.min(point.y);

            area += prev_point.x*point.y-point.x*prev_point.y;
            prev_point = *point;
        }

        return Self{
            area: area/2.0,
            aabb,
            points,
        }
    }
}

impl ContorTrait<2> for Contour {

    fn points<'a>(&'a self) -> core::slice::Iter<'a,Point2<f32>>{
        self.points.iter()
    }

    fn x_distance_to_contour(&self,point:&Point2<f32>)->Option<f32>{
        // returns true if a point is on or inside the contour
        if !self.aabb.point_is_inside(&point) { return None }

        let intersections: Vec<f32> = self.edges()
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

    fn reverse_order(&mut self) {
        self.points.reverse();
        self.area = self.area * -1.0;
    }
}

#[test]
fn contour_rotate_test(){
    let mut contour = contour!([0.,0.],[1.,0.],[1.,1.],[0.,1.]);
    contour.transform(PI/4.,Vector2::new(0.2,0.2));
    dbg!(&contour);
    assert_eq!(contour,
        contour!([0.5,0.5],[0.5,0.5],[0.5,0.5],[0.5,0.5])
        );

}

impl Contour {
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        let (new_points, points_removed) = super::merge_by_distance(&self.points[..], distance);
        self.points = new_points;
        return points_removed
    }
    pub fn scale(&mut self, scale:f32){
        for point in self.points.iter_mut(){ *point *= scale; };
        self.area *= scale;
        self.aabb.scale(scale);
    }
    pub fn transform(&mut self, angle:f32, displacement:Vector2<f32>){
        let rotation_matrix = Matrix2::new(
            angle.cos(),-angle.sin(),
            angle.sin(), angle.cos(),
            );
        let tanformed_points:Vec<_> = self.points.iter()
            .map(|p|{ (rotation_matrix*p) + displacement})
            .collect();
        *self = Contour::from(tanformed_points);
    }
    pub fn rotate(&mut self, angle:f32){
        let rotation_matrix = Matrix2::new(
            angle.cos(),-angle.sin(),
            angle.sin(), angle.cos(),
            );
        let rotated_points:Vec<_> = self.points.iter()
            .map(|p|{ rotation_matrix*p})
            .collect();
        *self = Contour::from(rotated_points);
    }
    pub fn edges(&self) -> impl Iterator<Item = (&Point2<f32>,&Point2<f32>)>{
        let points = self.points.iter();
        let points_offset_by_one = points.clone().cycle().skip(1);
        points.zip(points_offset_by_one)
    }
    // returns a iterator over pairs of edges (pairs of points)
    pub fn into_edges(self) -> impl Iterator<Item = (Point2<f32>,Point2<f32>)> {
        let points = self.points.into_iter();
        let points_offset_by_one = points.clone().cycle().skip(1);
        points.zip(points_offset_by_one)
    }
    pub fn simplify(&mut self, min_a:f32){
        let len = self.points.len();
        for i in (0..len).rev(){
            let i_pluss = (i+1)%self.points.len();
            let i_minus = if i == 0 {self.points.len().saturating_sub(1)}else{i.saturating_sub(1)};

            let p = self.points[i];
            let p_p = self.points[i_pluss];
            let p_m = self.points[i_minus];

            let v1 = p_p - p;
            let v2 = p_m - p;

            let area_x2 = cross2d(&v1, &v2);
            if area_x2.abs() < min_a*2. {self.points.remove(i);}
        }
        if self.points.len() < 3 {self.points.clear(); self.area = 0.0;}
    }
    pub fn from_points(points:Vec<Point2<f32>>) -> Self {
        Self::from(points)
    }
}

use crate::contour;

#[macro_export]
macro_rules! contour {
    ( $( [$x:expr, $y:expr] ),* ) => {
        Contour::from(vec![
            $(
                Point2::new($x,$y),
            )*
        ])
    };
}

#[test]
fn contour_macro_test(){
    assert_eq!(
        contour!([2.+3.,4.],[3.,4.],[3.,4.]),
        Contour::from(vec![
            Point2::new(2.+3.,4.),
            Point2::new(3.,4.),
            Point2::new(3.,4.),
        ])
    )
}
