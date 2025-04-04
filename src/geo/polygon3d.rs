use nalgebra::Point3;
use super::Contour3d;

pub struct Polygon3d(pub Vec<Contour3d>);

impl Polygon3d{
    pub fn from_contours() -> Self {
        todo!()
    }
    pub fn invert(&mut self) {
        todo!()
    }
    pub fn area(&self) -> f32 {
        todo!()
    }
    pub fn contours<'a>(&'a self) -> core::slice::Iter<'a,Contour3d>{
        self.0.iter()
    }
    pub fn into_contours(self) -> std::vec::IntoIter<Contour3d>{//core::slice::Iter<'_,Contour3d>{
        self.0.into_iter()
    }
}
