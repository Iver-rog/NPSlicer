use nalgebra::{Point2,Point3};

#[derive(Debug,Clone)]
pub struct Contour3d(pub Vec<Point3<f32>>);

impl From<Vec<Point3<f32>>> for Contour3d{
    fn from(points:Vec<Point3<f32>>) -> Self {
        Self(points)
    }
}

impl Contour3d{
    pub fn from_unchecked(raw_parts:Vec<Point3<f32>>) -> Self{
        Self(raw_parts)
    }
    /// returns true if the point is on or inside the projection of the contour onto the xy-plane
    pub fn x_distance_to_contour(&self,point:&Point2<f32>)->Option<f32>{

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
