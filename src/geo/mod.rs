use nalgebra::Point2;

mod aabb;
pub use aabb::*;

mod contour;
pub use contour::*;

mod contour3d;
pub use contour3d::*;

mod polygon;
pub use polygon::*;

mod polygon3d;
pub use polygon3d::*;

mod layer;
pub use layer::*;

/// The Enclosed trait defines that the struct encloses an area.
/// structs that implement Enclosed therefore have an interior and an exterior.
/// The enclosed trait is most sutable for 2d shapes, however 3d shapes can also 
/// implement the enclosed trait, in this case it is the 2d projection of the 3d shape
/// onto the xy-plane that is used to implement the Enclosed-methods.
pub trait Enclosed {
    /// computes the area of the shape projected onto the xy-plane
    fn area(&self) -> f32;
    /// returns true if the point is inside the enclosed area
    fn point_is_inside(&self,point:&Point2<f32>) -> bool;
}
