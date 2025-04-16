use core::todo;

use nalgebra::{Point, Point3};
use crate::geo::Contour3d;

pub enum PathType{
    Perimeter,
    Infill,
}

#[derive(Debug)]
pub struct Path{
    pub points:Vec<Point3<f32>>,
}
impl From<Contour3d> for Path {
    fn from(mut contour3d:Contour3d) -> Self{
        contour3d.set_start();
        let mut points = contour3d.0;
        points.push(points[0].clone());
        Self{ points }
    }
}
impl Path{
    pub fn edges(&self) -> impl Iterator<Item = (&Point3<f32>,&Point3<f32>)> {
        let points = self.points.iter();
        let points_offset_by_one = points.clone().skip(1);
        points.zip(points_offset_by_one)
    }

    pub fn shorten_ends(&mut self, offset:f32) {
        self.shorten_end(offset/2.0);
        self.shorten_start(offset/2.0);
    }

    pub fn shorten_start(&mut self, offset:f32) {
        let mut removed_length = 0.;
        let mut first_edge_length;
        let mut remove_ndx = None;

        for (i,(first,next)) in self.edges().enumerate() {
            first_edge_length = (next - first).magnitude();
            if removed_length + first_edge_length > offset {
                remove_ndx = Some(i);
                break;
            }
            removed_length += first_edge_length;
        }

        self.points.drain(0..remove_ndx.expect("path is shorter than the desired offset"));

        let end_vec = (self.points[1] - self.points[0]).normalize();
        *self.points.first_mut().expect("path is shorter than the desired offset") 
            += end_vec * (offset-removed_length);
    }

    pub fn shorten_end(&mut self, offset:f32) {
        // shorten end
        let mut removed_length = 0.;
        let mut last_edge_length;

        while {
            let last = self.points[self.points.len()-1];
            let next_last = self.points[self.points.len()-2];
            last_edge_length = (next_last - last).magnitude();
            removed_length + last_edge_length
        } < offset {
            self.points.pop().expect("path is shorter than the desired offset");
            removed_length += last_edge_length;
        }
        let end_vec = (self.points[self.points.len()-2] - self.points[self.points.len()-1]).normalize();
        
        *self.points.last_mut().expect("path is shorter than the desired offset") 
            += end_vec * (offset-removed_length);
    }
}
