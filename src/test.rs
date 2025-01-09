use nalgebra::{ Point2, Vector2};
use ordered_float::OrderedFloat;
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
    let current_point = Point2::new(0.,1.);
    let next_point = Point2::new(-1.,0.);
    let prev_point = Point2::new( 1.,0.);
    let correct_bisector = Vector2::new(0.,-2.0_f32.sqrt());
    assert_eq!(correct_bisector,straight_skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn bisector_test_reflex_angle(){
    let current_point = Point2::new(0.,1.);
    let next_point = Point2::new( 1.,0.);
    let prev_point = Point2::new(-1.,0.);
    let correct_bisector = Vector2::new(0.,2.0_f32.sqrt());
    assert_eq!(correct_bisector,straight_skeleton::bisector(current_point, next_point, prev_point).unwrap());
}
#[test]
fn intersect_test(){
    let p1 = Point2::new(0.,0.);
    let v1 = Vector2::new(1.,1.);
    let p2 = Point2::new(2.,0.);
    let v2 = Vector2::new(0.,1.);
    assert_eq!(Point2::new(2.,2.), straight_skeleton::intersect(p1, v1, p2, v2));
}
#[test]
fn is_reflex_test(){
    let prev_point = Point2::new(-1.0, 0.0);
    let current_point = Point2::new(0.0, 1.0);
    let next_point = Point2::new(1.0, 0.0);
    assert!(straight_skeleton::is_reflex(current_point, next_point, prev_point));

    let current_point = Point2::new(0.0, -1.0);
    assert!( ! straight_skeleton::is_reflex(current_point, next_point, prev_point) );

    assert!( straight_skeleton::is_reflex(current_point, prev_point, next_point) );
}

#[test]
fn test_invalid_polygon() {
    let points = vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
    ];
    let weights = vec![1.0, 1.0];
    let result = straight_skeleton::create_skeleton(points);
    assert!(result.is_err());
}
#[test]
fn nodes_forwards_iterator_test(){
    let node0 = straight_skeleton::Node{
            ndx:0,
            next_ndx:1,
            prev_ndx:2,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node1 = straight_skeleton::Node{
            ndx:1,
            next_ndx:2,
            prev_ndx:0,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node2 = straight_skeleton::Node{
            ndx:2,
            next_ndx:0,
            prev_ndx:1,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let nodes = straight_skeleton::Nodes::from_closed_curve(vec![node0,node1,node2]);
    println!("{nodes}");
        
    let forward_iter:Vec<straight_skeleton::Node> = nodes.iter(&node0)
        .into_iter()
        .collect();
    assert_eq!(vec![node1,node2,node0],forward_iter);
}
#[test]
fn nodes_backwards_iterator_test(){
    let node0 = straight_skeleton::Node{
            ndx:0,
            next_ndx:1,
            prev_ndx:2,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node1 = straight_skeleton::Node{
            ndx:1,
            next_ndx:2,
            prev_ndx:0,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let node2 = straight_skeleton::Node{
            ndx:2,
            next_ndx:0,
            prev_ndx:1,
            bisector:[OrderedFloat::default(),OrderedFloat::default()],
            vertex_ndx:0,
        };
    let nodes = straight_skeleton::Nodes::from_closed_curve(vec![node0,node1,node2]);
    println!("{nodes}");
        
    let backwards_iter:Vec<straight_skeleton::Node> = nodes.back_iter(&node0)
        .into_iter()
        .collect();
    assert_eq!(vec![node2,node1,node0],backwards_iter);
}
#[test]
fn is_point_on_edge_test(){
    let p1 = Point2::new(1.0,1.0);
    let e0 = Point2::new(0.0,0.0);
    let e1 = Point2::new(4.0,4.0);
    assert!(straight_skeleton::is_point_on_edge(&p1,&e0,&e1).unwrap())
}
