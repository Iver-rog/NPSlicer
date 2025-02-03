use crate::contours::*;
use crate::stl_op::area_contour2d;

#[test]
fn equivalent_area_calculations_test(){
    let points = vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(0.0,1.0),
    ];
    let contour = Contour::new(points);
    assert_eq!(area_contour2d(&contour.points),contour.area);
}
#[test]
fn contour_reverse_order_test(){
    let mut contour = Contour::new(vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(0.0,1.0),
    ]);
    assert!(contour.area.is_sign_positive());
    contour.reverse_order();
    assert_eq!(contour.points,
        vec![
            Point2::new(0.0,1.0),
            Point2::new(1.0,0.0),
            Point2::new(0.0,0.0),
        ] );
    assert!(contour.area.is_sign_negative());
}
#[test]
pub fn point_is_inside_contour_test(){
    let contour = Contour::new( vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(0.0,1.0),
    ]);
    assert!(   contour.point_is_inside(&Point2::new( 0.2,0.2 )),"test 1");
    assert!(   contour.point_is_inside(&Point2::new( 0.5,0.5 )),"test 2");
    assert!(   contour.point_is_inside(&Point2::new( 0.0,0.0 )),"test 3");
    assert!( ! contour.point_is_inside(&Point2::new( 0.9,0.9 )),"test 4");
    assert!( ! contour.point_is_inside(&Point2::new( 0.5,1.0 )),"test 5");
    assert!( ! contour.point_is_inside(&Point2::new( 2.0,0.6 )),"test 6");
    assert!( ! contour.point_is_inside(&Point2::new(-1.0,0.5 )),"test 7");
    assert!( ! contour.point_is_inside(&Point2::new( 1.0,1.0 )),"test 8");
}
#[test]
pub fn polygons_from_contours_test(){
    let loop1 = vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(0.0,1.0),
    ];
    let loop2 = vec![
        Point2::new(-1.0,-1.0),
        Point2::new( 3.0,-1.0),
        Point2::new(-1.0, 3.0),
    ];
    let mut contour1 = Contour::new(loop1);
    let contour2 = Contour::new(loop2);

    let contours = vec![
        contour1.clone(),
        contour2.clone()
    ];
    contour1.reverse_order();
    assert_eq!(
    polygons_from_contours(contours)[0],
    Polygon{
        outer_loop:contour2,
        holes:vec![contour1],
        }
    );
}
