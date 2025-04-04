use nalgebra::Point3;

// pub struct Contour3d{
//     pub points: Vec<Point3<f32>>
// }
#[derive(Debug,Clone)]
pub struct Contour3d(pub Vec<Point3<f32>>);

impl From<Vec<Point3<f32>>> for Contour3d{
    fn from(points:Vec<Point3<f32>>) -> Self {
        Self(points)
    }
}
