use crate::stl_op::{IndexedEdge};
use stl_io::{IndexedTriangle,Vector};

use super::projection::*;
use super::*;

/// This is not a real test but is used to debugg infill generation
// #[test]
// fn infill_visualization(){
//     let mut blender = Blender::new();
//
//     let vertices = vec![
//         Point3::new( 0.0,  0.0,  0.0),
//         Point3::new(-2.5, -2.5, -2.5),
//         Point3::new( 15.0,  9.0,  2.5),
//     ];
//     let edges = vec![
//         IndexedEdge(0,1),
//         IndexedEdge(1,2),
//     ];
//     blender.line_body3d(
//         vertices.iter().map(|p|[p.x,p.y,p.z]).collect(),
//         edges.iter().map(|e|[e.0,e.1]).collect()
//         );
//     let face = IndexedTriangle{
//         normal: Vector::new([0.4244,-0.8163,0.3918]),
//         vertices: [0,1,2],
//     };
//
//     let mesh = MeshCollider { faces:vec![face], edges, vertices, };
//
//     let bounds = Polygon::from_unchecked(vec![Contour::from(vec![
//         Point2::new(1.0, 0.0),
//         Point2::new(11.0, 0.0),
//         Point2::new(11.0, 10.0),
//         Point2::new(1.0, 11.0),
//     ])]);
//     blender.polygon(&bounds, 0.0);
//
//     let paths = generate_3d_infill(bounds, &mesh, 0.2, InfillDirection::X);
//     for path in paths {
//         blender.path(&path);
//     }
//     // blender.show();
//     assert!(true);
// }

#[test]
fn point_is_inside_slice_test(){
    let slice = [
        Point3::new(1.0,0.0,0.0),
        Point3::new(0.0,0.0,0.0),
        Point3::new(0.0,1.0,0.0),
    ];

    assert!((&slice).point_is_inside(&Point2::new(0.2,0.2)));

    assert!((&slice).point_is_inside(&Point2::new(0.0,0.0)));
    assert!((&slice).point_is_inside(&Point2::new(1.0,0.0)));
    assert!((&slice).point_is_inside(&Point2::new(0.0,1.0)));

    assert!((&slice).point_is_inside(&Point2::new(0.5,0.5)));
    assert!((&slice).point_is_inside(&Point2::new(0.5,0.0)));
    assert!((&slice).point_is_inside(&Point2::new(0.0,0.5)));

    assert!(!(&slice).point_is_inside(&Point2::new(1.0,1.0)));
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

#[test]
fn path_shorten_ends(){
    let mut path = Path{ 
        points: vec![
            Point3::new(-0.100, 0.0, 0.0),
            Point3::new( 0.000, 0.0, 0.0),
            Point3::new( 1.000, 0.0, 0.0),
            Point3::new( 1.001, 0.0, 0.0),
        ],
        path_type: PathType::Infill 
    };

    path.shorten_ends(0.2);

    assert_eq!( path,
        Path{
            points: vec![
                Point3::new(0.000, 0.0, 0.0),
                Point3::new(0.901, 0.0, 0.0),
            ],
            path_type: PathType::Infill,
        }
    )
}

