use super::{Contour3d,FromUnChecked};
use nalgebra::Point3;
use std::f32::EPSILON;

#[derive(Debug,PartialEq)]
pub struct Polygon3d(pub Vec<Contour3d>);

impl FromUnChecked<Vec<Contour3d>> for Polygon3d {
    fn from_unchecked(contours:Vec<Contour3d>) -> Self {
        Self(contours)
    }
}

impl Polygon3d{
    pub fn all_edges<'a>(&'a self) -> impl Iterator<Item = (&'a Point3<f32>,&'a Point3<f32>)>{
        self.0.iter().flat_map(|contour|contour.edges())
    }
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        self.0.iter_mut()
            .map(|contour| contour.merge_by_distance(distance))
            .sum()
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
    pub fn into_contours(self) -> std::vec::IntoIter<Contour3d>{
        self.0.into_iter()
    }
}


#[test]
fn merge_by_distance_test(){
    let mut contour3d = Polygon3d::from_unchecked(vec![Contour3d::from(vec![
        Point3::new(0.0,0.0,0.0), // 1
        Point3::new(0.0,0.0,0.0), // 1
        Point3::new(1.0,0.0,0.0), // 2
        Point3::new(1.1,0.0,0.0), // 2
        Point3::new(1.2,0.0,0.0), // 2
        Point3::new(4.0,0.0,0.0), // 3
        Point3::new(4.2,0.0,0.0), // 3
        Point3::new(0.2,0.0,0.0), // 1
    ])]
    );
    let removed_vertices = contour3d.merge_by_distance(0.3);
    assert_eq!(contour3d,
    Polygon3d::from_unchecked(vec![Contour3d::from(vec![
        Point3::new(0.1,        0.0,  0.0), // 1
        //                 v-- due to rounding error
        Point3::new(1.1-EPSILON,0.0,  0.0), // 2
        Point3::new(4.1,        0.0,  0.0), // 3
        ])]
    )
    );
    assert_eq!(removed_vertices,5);
}
