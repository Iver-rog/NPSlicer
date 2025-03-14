use nalgebra::{ Point2, Vector2};
use ordered_float::OrderedFloat;
use crate::*;
use crate::skeleton::split_event::is_point_on_edge;
use super::*;

#[test]
fn ccw_angle_test() {
    let v1 = Vector2::new(4.0, 0.0);
    let v2 = Vector2::new(0.0, 1.0);
    
    let angle1: f32 = ccw_angle(&v1, &v2);
    let angle2: f32 = ccw_angle(&v2, &v1);
    assert!(angle1 != angle2);
    let pi = std::f32::consts::PI;
    assert_eq!(angle1 + angle2, 2.0*pi );
}

#[test]
fn bisector_test(){
    let current_point = Point2::new(0.,0.);
    let next_point = Point2::new( 1.,1.);
    let prev_point = Point2::new(-1.,1.);
    let correct_bisector = Vector2::new(0.,2.0_f32.sqrt());
    assert_eq!(correct_bisector,skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn bisector_test_right_to_left(){
    let current_point = Point2::new(0.,1.);
    let next_point = Point2::new(-1.,0.);
    let prev_point = Point2::new( 1.,0.);
    let correct_bisector = Vector2::new(0.,-2.0_f32.sqrt());
    assert_eq!(correct_bisector,skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn bisector_test_reflex_angle(){
    let current_point = Point2::new(0.,1.);
    let next_point = Point2::new( 1.,0.);
    let prev_point = Point2::new(-1.,0.);
    let correct_bisector = Vector2::new(0.,2.0_f32.sqrt());
    assert_eq!(correct_bisector,skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn intersect_test(){
    let p1 = Point2::new(0.,0.);
    let v1 = Vector2::new(1.,1.);
    let p2 = Point2::new(2.,0.);
    let v2 = Vector2::new(0.,1.);
    assert_eq!(Point2::new(2.,2.), skeleton::intersect(p1, v1, p2, v2).unwrap());
}
#[test]
fn is_reflex_test(){
    let prev_point = Point2::new(-1.0, 0.0);
    let current_point = Point2::new(0.0, 1.0);
    let next_point = Point2::new(1.0, 0.0);
    assert!(skeleton::is_reflex(current_point, next_point, prev_point));

    let current_point = Point2::new(0.0, -1.0);
    assert!( ! skeleton::is_reflex(current_point, next_point, prev_point) );

    assert!( skeleton::is_reflex(current_point, prev_point, next_point) );
}

#[test]
fn nodes_forwards_iterator_test(){
    let node0 = skeleton::Node{
            ndx:0,
            next_ndx:1,
            prev_ndx:2,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node1 = skeleton::Node{
            ndx:1,
            next_ndx:2,
            prev_ndx:0,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node2 = skeleton::Node{
            ndx:2,
            next_ndx:0,
            prev_ndx:1,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let nodes = skeleton::Nodes::from_closed_curve(vec![node0,node1,node2]);
    println!("{nodes}");
        
    let forward_iter:Vec<skeleton::Node> = nodes.iter_from(&node0)
        .into_iter()
        .collect();
    assert_eq!(vec![node1,node2,node0],forward_iter);
}
#[test]
fn nodes_backwards_iterator_test(){
    let node0 = skeleton::Node{
            ndx:0,
            next_ndx:1,
            prev_ndx:2,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node1 = skeleton::Node{
            ndx:1,
            next_ndx:2,
            prev_ndx:0,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node2 = skeleton::Node{
            ndx:2,
            next_ndx:0,
            prev_ndx:1,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let nodes = skeleton::Nodes::from_closed_curve(vec![node0,node1,node2]);
    println!("{nodes}");
        
    let backwards_iter:Vec<skeleton::Node> = nodes.back_iter_from(&node0)
        .into_iter()
        .collect();
    assert_eq!(vec![node2,node1,node0],backwards_iter);
}
#[test]
fn is_point_on_edge_test(){
    let p1 = Point2::new(1.0,1.0);
    let e0 = Point2::new(0.0,0.0);
    let e1 = Point2::new(4.0,4.0);
    assert!(is_point_on_edge(&p1,&e0,&e1).unwrap())
}
