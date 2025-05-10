#![allow(unused)]
use super::{Polygon, Polygon3d};
use crate::stl_op::Edge;

pub struct Layer(pub Vec<Polygon>);
pub struct Layer3d(pub Vec<Polygon3d>);

impl Layer{
    fn project_onto(self,plane:Vec<Edge>) -> Layer3d {
        todo!()
    }
}


