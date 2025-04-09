use nalgebra::Point2;

use super::Enclosed;

/// Axis Aligned Bounding Box
#[derive(Debug,Clone,PartialEq)]
pub struct AABB{
    pub x_max:f32,
    pub x_min:f32,
    pub y_max:f32,
    pub y_min:f32,
}
impl Enclosed for AABB {
    fn area(&self) -> f32 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        self.x_min <= point.x && point.x <= self.x_max &&
        self.y_min <= point.y && point.y <= self.y_max
    }
}
