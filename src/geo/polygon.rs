use nalgebra::Point2;
use stl_io::Vector;
use super::{contour, Contour};
pub use super::Enclosed;

use std::ops::{Deref,DerefMut};

#[derive(Debug,Clone,PartialEq)]
pub struct Polygon(pub Vec<Contour>);

impl Deref for Polygon {
    type Target = Vec<Contour>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl Polygon {
    pub fn contours<'a>(&'a self) -> core::slice::Iter<'a,Contour> {
        self.0.iter()
    }
    pub fn outer_loop(&self) -> &Contour{
        &self.0[0]
    }
    pub fn outer_loop_mut(&mut self) -> &mut Contour{
        &mut self.0[0]
    }
    pub fn holes(&self) -> &[Contour]{
        self.0.get(1..).unwrap_or(&[])
    }
    pub fn holes_mut(&mut self) -> &mut[Contour]{
        self.0.get_mut(1..).unwrap_or(&mut[])
    }
}
#[test]
fn ølaskdjfølksadjf(){
    let contour = Contour::from(vec![
            Point2::new(0.1,0.1),
            Point2::new(0.1,0.1),
            Point2::new(0.1,0.1),
    ]);
    let polygon = Polygon(vec![
        contour.clone(),
        contour.clone(),
        contour.clone(),
    ]);
    assert_eq!(polygon.clone(),polygon);
}
#[test]
fn polygon_holes_test(){
    let contour = Contour::from(vec![
            Point2::new(0.1,0.1),
            Point2::new(0.1,0.1),
            Point2::new(0.1,0.1),
    ]);
    let polygon = Polygon(vec![
        contour.clone(),
        contour.clone(),
        contour.clone(),
    ]);
    assert!(polygon.holes().len() == 2);

    let mut p_iter = polygon.holes().iter();
    assert_eq!(p_iter.next(),Some(&contour));
    assert_eq!(p_iter.next(),Some(&contour));
    assert_eq!(p_iter.next(),None);
}

// #[derive(Debug,Clone,PartialEq)]
// pub struct Polygon{
//     pub outer_loop: Contour,
//     pub holes: Vec<Contour>,
// }
impl Enclosed for Polygon {
    fn area(&self) -> f32 {
        self.iter().map(|contour| contour.area).sum()
        // let area = self.outer_loop().area();
        // let hole_area: f32 = self.holes.iter().map(|contour| contour.area() ).sum();
        // return area - hole_area
    }
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        if !self.outer_loop().point_is_inside(point) {println!("outside outer_loop");return false}
        for hole in self.holes(){
            if hole.point_is_inside(point) {println!("inside hole");return false}
        }
        return true;
    }
}

impl Polygon {
    pub fn invert(&mut self){
        self.0.iter_mut().for_each(|contour|contour.reverse_order());
        // for contour in &mut self {
        //     contour.reverse_order()
        // }
        // self.outer_loop.reverse_order();
        // self.holes.iter_mut().for_each(|c|c.reverse_order());
    }
    pub fn new(mut outer_loop:Contour,mut holes:Vec<Contour>)->Self{
        if outer_loop.area.is_sign_negative() {
            outer_loop.reverse_order();
        }
        for hole in holes.iter_mut() {
            if hole.area.is_sign_positive(){
                hole.reverse_order();
            }
        }
        holes.insert(0,outer_loop);

        Self(holes)
    }
    pub fn simplify(&mut self, min_a:f32){
        self.0[0].simplify(min_a);
        if (self.outer_loop().area < min_a) | (self.outer_loop().points.len() < 3){
            self.0[0].points.clear();
            self.0[0].area = 0.0;
            self.0.truncate(1);
        };
        self.holes_mut().iter_mut().for_each(|hole|hole.simplify(min_a));
        self.0.retain(|c|c.area.abs() > min_a);
    }
    pub fn flatten(self) -> Vec<Contour>{
        self.0
    }
}

pub fn polygons_from_contours(mut contours:Vec<Contour>)->Vec<Polygon>{
    let mut contour_inside_ref = vec![None;contours.len()];
    for (i,contour) in contours.iter().enumerate() {
        let test_point = contour.points[0];

        let contour_i_is_inside = contours.iter()
            .enumerate()
            .filter(|(n,_)|*n!=i)
            .filter_map(|(n,intersection_contour)|
                match intersection_contour.x_distance_to_contour(&test_point){
                    Some(distance) => Some((n,distance)),
                    None => None,
            })
            .enumerate() // count how many intersections occur
            .reduce(|(_,(n_min,min_distance)),(n_intersect,(n_new,new_distance))| {
                if new_distance < min_distance {(n_intersect,(n_new,new_distance))} else {(n_intersect,(n_min,min_distance))}
            });

        match contour_i_is_inside{
            Some((n_intersect,(n,_))) => { 
                // n_intersect is zero based eg n_intersect = 0 => 1 intersection
                if n_intersect % 2 == 0 { 
                    assert!(contour_inside_ref[i]==None);
                    contour_inside_ref[i]=Some(n);
                } 
            },
            None => (),
        }
    }
    let mut polygons = vec![(None,Vec::new());contours.len()];
    for (i,is_inside) in contour_inside_ref.into_iter().enumerate().rev(){
        match is_inside {
            Some(i) => { polygons[i].1.push( contours.pop().unwrap() ) },
            None => polygons[i].0 = Some(contours.pop().unwrap()),
        }
    }

    polygons.into_iter()
        .filter_map(|(option_outer_loop,holes)| match option_outer_loop {
            Some(outer_loop) => Some((outer_loop,holes)),
            None => None,
        })
        .map(|(outer_loop,holes)| Polygon::new(outer_loop,holes) )
        .collect()
}
