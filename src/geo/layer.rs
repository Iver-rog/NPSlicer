use super::{Polygon, Polygon3d};
use crate::stl_op::Edge;

pub struct Layer(Vec<Polygon>);
pub struct Layer3d(Vec<Polygon3d>);

impl Layer{
    fn project_onto(self,plane:Vec<Edge>) -> Layer3d {
        todo!()
    }
}


