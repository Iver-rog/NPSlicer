use nalgebra::{Point2,Vector2};
use crate::straight_skeleton;

#[test]
fn bisector_test(){
    let current_point = Point2::new(0.,0.);
    let next_point = Point2::new( 1.,1.);
    let prev_point = Point2::new(-1.,1.);
    let correct_bisector = Vector2::new(0.,2.0_f32.sqrt());
    assert_eq!(correct_bisector,straight_skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn bisector_test_right_to_left(){
    let current_point = Point2::new(0.,0.);
    let next_point = Point2::new(-1.,1.);
    let prev_point = Point2::new( 1.,1.);
    let correct_bisector = Vector2::new(0.,-2.0_f32.sqrt());
    assert_eq!(correct_bisector,straight_skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn bisector_test_convex_edge(){
    let current_point = Point2::new(0.,1.);
    let next_point = Point2::new( 1.,0.);
    let prev_point = Point2::new(-1.,0.);
    let correct_bisector = Vector2::new(0.,-2.0_f32.sqrt());
    assert_eq!(correct_bisector,straight_skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
