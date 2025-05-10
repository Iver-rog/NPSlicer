use super::{Contour3d,FromUnChecked};
use nalgebra::Point3;
use super::AABB;

#[cfg(test)]
use std::f32::{ EPSILON, consts::PI };

#[derive(Debug,PartialEq,Clone)]
pub struct Polygon3d(pub Vec<Contour3d>);

impl FromUnChecked<Vec<Contour3d>> for Polygon3d {
    fn from_unchecked(contours:Vec<Contour3d>) -> Self {
        Self(contours)
    }
}

#[allow(unused)]
use crate::contour3d;

#[test]
fn polygon3d_rotation(){
    let mut polygon = Polygon3d::from_unchecked(vec![
        contour3d!(
            [0.0,0.0,0.0],
            [1.0,2.0,0.0],
            [3.0,1.0,0.0]
            ),
    ]);
    let mut rot = polygon.clone();
    let mut scale = polygon.clone();


    scale.rotate_scale(0.,2.);
    assert_eq!(
        scale,
        Polygon3d::from_unchecked(vec![
        contour3d!(
            [0.0, 0.0, 0.0],
            [2.0, 4.0, 0.0],
            [6.0, 2.0, 0.0]
            ),
        ]),
        "pure scaleing"
    );
    assert_ne!(polygon.aabb(),scale.aabb())

}

impl Polygon3d{
    pub fn aabb(&self) -> AABB{
        self.outer_loop().aabb()
    }
    pub fn outer_loop(&self) -> &Contour3d{
        &self.0[0]
    }
    pub fn all_edges<'a>(&'a self) -> impl Iterator<Item = (&'a Point3<f32>,&'a Point3<f32>)>{
        self.0.iter().flat_map(|contour|contour.edges())
    }
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        self.0.iter_mut()
            .map(|contour| contour.merge_by_distance(distance))
            .sum()
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
