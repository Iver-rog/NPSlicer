use super::{Contour3d,FromUnChecked};

pub struct Polygon3d(pub Vec<Contour3d>);

impl FromUnChecked<Vec<Contour3d>> for Polygon3d {
    fn from_unchecked(contours:Vec<Contour3d>) -> Self {
        Self(contours)
    }
}

impl Polygon3d{
    pub fn invert(&mut self) {
        todo!()
    }
    pub fn area(&self) -> f32 {
        todo!()
    }
    pub fn contours<'a>(&'a self) -> core::slice::Iter<'a,Contour3d>{
        self.0.iter()
    }
    pub fn into_contours(self) -> std::vec::IntoIter<Contour3d>{
        self.0.into_iter()
    }
}
