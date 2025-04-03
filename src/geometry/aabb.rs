use nalgebra::Point2;

#[derive(Debug,Clone,PartialEq)]
pub (super) struct AABB{
    pub (super) x_max:f32,
    pub (super) x_min:f32,
    pub (super) y_max:f32,
    pub (super) y_min:f32,
}
impl AABB {
    pub fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        self.x_min <= point.x && point.x <= self.x_max &&
        self.y_min <= point.y && point.y <= self.y_max
    }
}
