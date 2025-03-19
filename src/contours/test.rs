use crate::contours::*;
use crate::stl_op::area_contour2d;
use crate::data;
#[test]
fn simplify_contour(){
    //let mut blender = Blender::new();

    let mut c = Contour::new(data::test_poly());
    //blender.polygon(&Polygon::new(c.clone(),vec![]), 0.0);

    let len = c.points.len();
    println!("original len {len}");
    c.simplify(0.05);
    println!("len after simplify {}",c.points.len());

    //blender.polygon(&Polygon::new(c.clone(),vec![]), 0.0);
    //blender.show();
    assert!(c.points.len()<len);
}
pub fn i_offset_test(){
    let contours = vec![
        Contour::new(vec![
            Point2::new(1.0, 2.0),
            //Point2::new(1.0, 3.0),
            Point2::new(1.0, 4.0),
            Point2::new(2.0, 5.0),
            Point2::new(4.0, 5.0),
            Point2::new(5.0, 4.0),
            Point2::new(5.0, 3.0),
            Point2::new(8.0, 3.0),
            Point2::new(8.0, 4.0),
            Point2::new(9.0, 4.0),
            Point2::new(10.0, 3.0),
            Point2::new(11.0, 3.0),
            Point2::new(11.0, 4.0),
            Point2::new(12.0, 4.0),
            Point2::new(12.0, 3.0),
            Point2::new(13.0, 3.0),
            Point2::new(13.0, 2.0),
            Point2::new(5.0, 2.0),
            Point2::new(4.0, 1.0),
            Point2::new(2.0, 1.0),
        ]),
    ];
    let shape = polygons_from_contours(contours).into_iter().next().unwrap();
    let p = shape.offset(0.2);
    assert!(p.len()!=0)
}
#[test]
fn reverse_order(){
    let mut contour = Contour::new(vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(1.0,1.0),
        Point2::new(0.0,1.0),
    ]);
    assert!(contour.area > 0.0);
    contour.reverse_order();
    assert!(contour.area < 0.0);
    assert_eq!(
        contour,
        Contour::new(vec![
            Point2::new(0.0,1.0),
            Point2::new(1.0,1.0),
            Point2::new(1.0,0.0),
            Point2::new(0.0,0.0),
            ])
        )
}
#[test]
fn polygon_from_contours_2(){
    let contours = vec![
            Contour::new(vec![
                Point2::new(0.0,0.0),
                Point2::new(1.0,0.0),
                Point2::new(1.0,1.0),
                Point2::new(0.0,1.0),
            ]),
            Contour::new(vec![
                Point2::new(0.1,0.1),
                Point2::new(0.4,0.0),
                Point2::new(0.4,0.9),
                Point2::new(0.1,0.9),
            ]),
            Contour::new(vec![
                Point2::new(0.5,0.1),
                Point2::new(0.9,0.1),
                Point2::new(0.9,0.9),
                Point2::new(0.5,0.9),
            ]),
            Contour::new(vec![
                Point2::new(2.0,0.0),
                Point2::new(3.0,0.0),
                Point2::new(3.0,1.9),
                Point2::new(2.0,1.9),
            ]),
        ];
    {
        let mut polygons = polygons_from_contours(contours.clone());
        assert_eq!(polygons.len(),2);

        let polygon1 = polygons[0].clone();
        assert_eq!(polygon1.holes.len(),2);

        assert_eq!(polygon1.outer_loop,contours[0]);
        assert!(polygon1.outer_loop.area > 0.0);
        assert!(polygon1.holes[0].area < 0.0);
        assert!(polygon1.holes[1].area < 0.0);

        let polygon2 = polygons[1].clone();
        assert_eq!(polygon2.outer_loop,contours[3]);
        assert!(polygon2.outer_loop.area > 0.0);
    }

    let mut contours_random_orientation = contours.clone();
    contours_random_orientation[0].reverse_order();
    contours_random_orientation[2].reverse_order();

    assert!(contours_random_orientation != contours);

    {
        let mut polygons = polygons_from_contours(contours_random_orientation.clone());
        assert_eq!(polygons.len(),2);

        let polygon1 = polygons[0].clone();
        assert_eq!(polygon1.holes.len(),2);

        assert_eq!(polygon1.outer_loop,contours[0]);
        assert!(polygon1.outer_loop.area > 0.0);
        assert!(polygon1.holes[0].area < 0.0);
        assert!(polygon1.holes[1].area < 0.0);

        let polygon2 = polygons[1].clone();
        assert_eq!(polygon2.outer_loop,contours[3]);
        assert!(polygon2.outer_loop.area > 0.0);
    }
}

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
        Point2::new(0.5,-0.2),
        Point2::new(1.0,0.0),
        Point2::new(1.0,0.5),
        Point2::new(0.5,0.5),
        Point2::new(0.4,0.7),
        Point2::new(0.0,1.0),
    ]);
    //let mut b = crate::Blender::new(); b.contour(&contour, 0.0); b.show();
    //for point in &contour.points {
    //    assert!( contour.point_is_inside(point),"point: {point} should be inside");
    //}
    assert!(   contour.point_is_inside(&Point2::new( 0.4,0.5 )),"test 1");
    assert!(   contour.point_is_inside(&Point2::new( 0.2,0.7 )),"test 2");
    assert!( ! contour.point_is_inside(&Point2::new(-0.1,1.0 )),"test 3");
    assert!( ! contour.point_is_inside(&Point2::new(-0.1,-0.2)),"test 4");
    assert!( ! contour.point_is_inside(&Point2::new( 1.1,0.5 )),"test 5");
    assert!( ! contour.point_is_inside(&Point2::new( 1.1,0.4 )),"test 6");
    assert!( ! contour.point_is_inside(&Point2::new( 0.5,1.1 )),"test 7");
    assert!( ! contour.point_is_inside(&Point2::new( 0.1,1.0 )),"test 8");
    assert!( ! contour.point_is_inside(&Point2::new( 0.0,1.1 )),"test 9");
}
#[test]
fn point_is_inside_polygon_test(){
    let p = Polygon::new(
        Contour::new(vec![
                Point2::new(0.0,0.0),
                Point2::new(3.6,0.3),
                Point2::new(3.8,1.6),
                Point2::new(0.4,2.4),
            ]),
            vec![
                Contour::new(vec![
                    Point2::new(0.5,0.4),
                    Point2::new(1.5,0.6),
                    Point2::new(1.3,1.7),
                    ]),
                Contour::new(vec![
                    Point2::new(2.7,0.5),
                    Point2::new(3.3,1.0),
                    Point2::new(2.4,1.2),
                    ]),
            ]
        );
    //let mut b = crate::Blender::new(); b.polygon(&p, 0.0); b.show();
    //for point in &p.outer_loop.points {
    //    assert!(p.point_is_inside(point),"point {point} should be inside")
    //}

    assert!(!p.point_is_inside(&Point2::new(-1.0,0.0)),"test 1");
    assert!(!p.point_is_inside(&Point2::new( 1.0,1.0)),"test 2");
    assert!(!p.point_is_inside(&Point2::new( 2.7,0.8)),"test 3");
    assert!( p.point_is_inside(&Point2::new( 0.2,0.2)),"test 4");
    assert!(!p.point_is_inside(&Point2::new( 3.8,0.6)),"test 5");
    assert!(!p.point_is_inside(&Point2::new( 3.8,1.1)),"test 6");
    assert!( p.point_is_inside(&Point2::new( 3.1,1.6)),"test 7");
    assert!( p.point_is_inside(&Point2::new( 1.9,1.6)),"test 8");
    assert!(!p.point_is_inside(&Point2::new( 0.1,1.6)),"test 9");

}
#[test]
pub fn polygons_from_contours_test(){
    let mut contour0 = Contour::new(vec![
        Point2::new(0.0,0.0),
        Point2::new(1.0,0.0),
        Point2::new(0.0,1.0),
    ]);
    let contour1 = Contour::new(vec![
        Point2::new(-1.0,-1.0),
        Point2::new( 3.0,-1.0),
        Point2::new(-1.0, 3.0),
    ]);
    let contour2 = Contour::new(vec![
        Point2::new(0.1,0.1),
        Point2::new(0.9,0.1),
        Point2::new(0.1,0.9),
    ]);

    let contours = vec![
        contour0.clone(),
        contour1.clone(),
        contour2.clone()
    ];
    dbg!(polygons_from_contours(contours.clone()));
    contour0.reverse_order();
    assert_eq!(polygons_from_contours(contours.clone()).len(),2);
    assert_eq!(
    polygons_from_contours(contours),vec![
    Polygon{
        outer_loop:contour1,
        holes:vec![contour0],
        },
    Polygon{
        outer_loop:contour2,
        holes:vec![],
        },
    ]
    );
}
