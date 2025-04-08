use i_overlay::vector::edge;
use nalgebra::{Point2,Point3};

/// Check if point q lies on line segment 'pr'.
pub fn on_segment(p:&Point2<f32>, q:&Point2<f32>, r:&Point2<f32>) -> bool {
    return (
        (p[0].min(r[0]) <= q[0] )&&( q[0] <= p[0].max(r[0]) )&&
        (p[1].min(r[1]) <= q[1] )&&( q[1] <= p[1].max(r[1]) )
    )
}

/// Find the orientation of the ordered triplet (p, q, r).
/// Returns:
///     0 --> Colinear
///     1 --> Clockwise
///     2 --> Counterclockwise
pub fn orientation(p:Point2<f32>, q:Point2<f32>, r:Point2<f32>) -> usize {
    let val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    if val == 0.0 { return 0 }
    return if val > 0.0 {1}else {2}
}

/// Check if line segments p1q1 and p2q2 intersect.
pub fn do_intersect([p1,q1]:&[Point2<f32>;2], [p2,q2]:&[Point2<f32>;2]) -> bool {
    let o1 = orientation(*p1, *q1, *p2);
    let o2 = orientation(*p1, *q1, *q2);
    let o3 = orientation(*p2, *q2, *p1);
    let o4 = orientation(*p2, *q2, *q1);

    // General case
    if ( o1 != o2 )&&( o3 != o4 ){ return true }

    // Special Cases
    if (o1 == 0) && on_segment(p1, p2, q1){ return true }
    if (o2 == 0) && on_segment(p1, q2, q1){ return true }
    if (o3 == 0) && on_segment(p2, p1, q2){ return true }
    if (o4 == 0) && on_segment(p2, q1, q2){ return true }

    return false
}

pub fn edge_edge_intersection3d(edge1:[&Point2<f32>;2],edge2:&[Point3<f32>;2]) -> Option<(f32,f32)> {

    let (p1, q1, p2, q2) = (edge1[0],edge1[1], edge2[0],edge2[1]);
    // Returns the scalar 't' such that:
    // intersection = p1 + t * (q1 - p1)
    // if the segments intersect, otherwise returns None.

    // Convert to vector form
    let dx1 = q1[0] - p1[0];
    let dy1 = q1[1] - p1[1];
    let dx2 = q2[0] - p2[0];
    let dy2 = q2[1] - p2[1];

    let denominator = dx1 * dy2 - dy1 * dx2;

    if denominator == 0.0 {return None} // Parallel lines

    // Solve for t and u such that:
    // p1 + t*(q1 - p1) = p2 + u*(q2 - p2)
    let dx3 = p2[0] - p1[0];
    let dy3 = p2[1] - p1[1];

    let t = (dx3 * dy2 - dy3 * dx2) / denominator;
    let u = (dx3 * dy1 - dy3 * dx1) / denominator;
    let z = (edge2[1].z-edge2[0].z)*u+edge2[0].z;

    if (0. <= t)&&(t <= 1.) && (0. <= u)&&(u <= 1.) { return Some((t,z)) } // Scalar on segment p1→q1 where the intersection occurs
    else{ return None } // The lines intersect but not within the segments
}

#[test]
fn edge_edge_intersection_test(){
    let start = Point2::new(0., 0.);
    let end = Point2::new(2., 2.);
    let edge1 = [&start,&end];
    let edge2 = [
        Point3::new(0., 2., 0.),
        Point3::new(2., 0., 0.),
    ];
    let edge3 = [
        Point3::new(2., 4., 0.),
        Point3::new(4., 2., 0.),
    ];
    let edge4 = [
        Point3::new(1.5, 1.5, 0.),
        Point3::new(0.5, 0.5, 0.),
    ];

    assert_eq!(edge_edge_intersection3d(edge1, &edge2),Some((0.5,0.0)));
    assert_eq!(edge_edge_intersection3d(edge1, &edge3),None);
    assert_eq!(edge_edge_intersection3d(edge1, &edge4),None);// <-edges are parallel
}
