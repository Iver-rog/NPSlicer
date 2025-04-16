use nalgebra::Point3;
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
    // Shortens the start and end of the path by the offset amout.
    pub fn shorten_ends(&mut self, offset:f32) {
        let half_offset = offset/2.0;

        // if  (self.points[1] - self.points[0]).magnitude() < half_offset  || 
        //     (self.points[last_ndx-1] - self.points[last_ndx]).magnitude() < half_offset
        //     {panic!("end vertices to close together for shorten ends to work correctly")}

        // shorten end
        let mut removed_length = 0.;
        let mut last_edge_length;

        while {
            let last = self.points[self.points.len()-1];
            let next_last = self.points[self.points.len()-2];
            last_edge_length = (next_last - last).magnitude();
            removed_length + last_edge_length
        } < half_offset {
            self.points.pop().expect("path is shorter than the desired offset");
            removed_length += last_edge_length;
        }
        let end_vec = (self.points[self.points.len()-2] - self.points[self.points.len()-1]).normalize();
        
        *self.points.last_mut().expect("path is shorter than the desired offset") += end_vec * (half_offset-removed_length);

        // shorten start
        let mut removed_length = 0.;
        let mut first_edge_length;

        while {
            let first = self.points[0];
            let next = self.points[1];
            first_edge_length = (next - first).magnitude();
            removed_length + first_edge_length
        } < half_offset {
            // NOTE: Very inefficient
            self.points.remove(0);
            removed_length += first_edge_length;
        }
        let end_vec = (self.points[1] - self.points[0]).normalize();
        
        *self.points.first_mut().expect("path is shorter than the desired offset") += end_vec * (half_offset-removed_length);
    }
}
