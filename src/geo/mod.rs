use nalgebra::{Point2, Vector};

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

/// The ContourTrait defines that the struct is made up of a sirular loop of points
pub trait ContorTrait {
    type PointType: Into<Point2<f32>> + Clone;
    /// Method used for ray casting that returns Oprion(distance).
    /// A some value means the point is inside the contour and 
    /// the distance value is the distance allong the x axis
    /// to the closest intersection with the contur.
    fn x_distance_to_contour(&self,point:&Point2<f32>) -> Option<f32>;
    /// Method used to iterate over the points of the contour.
    fn points<'a>(&'a self) -> core::slice::Iter<'a,Self::PointType>;
        // where T: ;
}

/// Used To Create Polygon/Polygon3d from vectors without checking hierarchy of input Contours
pub trait FromUnChecked<T> {
    fn from_uncheced(raw_parts:T) -> Self;
}

pub fn polygons_from_contours<C,T>(mut contours:Vec<C>) -> Vec<T> 
where
    C: ContorTrait + Clone,
    T: FromUnChecked<Vec<C>>
{
    let mut contour_inside_ref = vec![None;contours.len()];
    for (i,contour) in contours.iter().enumerate() {
        let test_point:Point2<f32> = contour.points().cloned().next().unwrap().into();

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
        .map(|(outer_loop, mut holes)|{
            holes.insert(0,outer_loop);
            T::from_uncheced(holes) 
        })
        .collect()
}
