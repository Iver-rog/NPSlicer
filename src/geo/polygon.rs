use nalgebra::Point2;
use super::{ContorTrait, Contour, FromUnChecked, Polygon3d};
pub use super::Enclosed;

#[derive(Debug,Clone,PartialEq)]
pub struct Polygon(pub Vec<Contour>);

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

    /// assert that the polygon has valid data
    pub fn validate(&self) {
        assert![self.0.len()!=0];
        for contour in self.0.iter() {
            assert!(contour.points.len()!=0);
            for point in &contour.points{
                assert!(!point.x.is_nan());
                assert!(!point.y.is_nan());
            }
        }
    }
    pub fn merge_by_distance(&mut self, distance:f32) -> usize {
        self.0.iter_mut()
            .map(|contour| contour.merge_by_distance(distance) )
            .sum()
    }
    pub fn all_edges<'a>(&'a self) -> impl Iterator<Item = (&'a Point2<f32>,&'a Point2<f32>)>{
        self.0.iter().flat_map(|contour|contour.edges())
    }
    pub fn invert(&mut self){
        self.0.iter_mut().for_each(|contour|contour.reverse_order());
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
    pub fn simplify(self, min_a:f32)-> Option<Self>{
        let mut contour_iter = self.0.into_iter();
        let outer_contour = contour_iter.next()?.simplify(min_a)?;
        let mut new_poly = Polygon(vec![outer_contour]);

        for hole in contour_iter.filter_map(|c|c.simplify(min_a)){
            new_poly.0.push(hole)
        }
        return Some(new_poly);
    }
    pub fn flatten(self) -> Vec<Contour>{
        self.0
    }
}

impl FromUnChecked<Vec<Contour>> for Polygon {
    fn from_unchecked(contours:Vec<Contour>) -> Self {
        Self(contours.into_iter()
            .enumerate()
            .map(|(i,mut contour)|{
                match i {
                    0 => if contour.area().is_sign_negative() { contour.reverse_order() },
                    _ => if contour.area().is_sign_positive() { contour.reverse_order() },
                }
                contour
            })
            .collect()
        )
    }
}

impl From<Polygon3d> for Polygon {
    fn from(polygon3d:Polygon3d) -> Self {
        Self(polygon3d.into_contours()
            .map(|contour| contour.into())
            .collect()
        )
    }
}

impl Enclosed for Polygon {
    fn area(&self) -> f32 {
        self.0.iter().map(|contour| contour.area).sum()
    }
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        if !self.outer_loop().point_is_inside(point) {println!("outside outer_loop");return false}
        for hole in self.holes(){
            if hole.point_is_inside(point) {println!("inside hole");return false}
        }
        return true;
    }
}
