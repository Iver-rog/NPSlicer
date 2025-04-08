use nalgebra::{Point2,Point3,Vector3};

use stl_io::IndexedTriangle;
use crate::stl_op;
use crate::geo::{Contour,Contour3d,Enclosed,Polygon,Polygon3d,FromUnChecked};
use core::f32::EPSILON;

impl Polygon{
    pub fn project_onto(&self,mesh:&MeshCollider) -> Polygon3d {
        let contours:Vec<Contour3d> = self.contours()
            .map(|contour| contour.project_onto(mesh))
            .collect();
        Polygon3d::from_unchecked(contours)
    }
}
impl Contour{
    pub fn project_onto(&self,mesh:&MeshCollider) -> Contour3d {
        project_contour_onto(self, mesh)
    }
}


#[derive(Debug)]
pub struct MeshCollider{
    pub faces:Vec<IndexedTriangle>,
    pub edges:Vec<stl_op::IndexedEdge>,
    pub vertices:Vec<Point3<f32>>,
}


pub fn project_point_onto(point:&Point2<f32>,mesh:&MeshCollider) -> Point3<f32> {
    let x = mesh.faces.iter()
        .map(|face|(face.vertices,face.normal))
        .map(|(v,norm)|([mesh.vertices[v[0]],mesh.vertices[v[1]],mesh.vertices[v[2]]],norm))
        .filter(|(tri,norm)| tri.point_is_inside(&point))
        // .inspect(|x|{println!("x [{},{},{}] norm:{:?}",x.0[0],x.0[1],x.0[2],x.1)})
        // .inspect(|x|{println!("x filtered {:?}",x);})
        .map(|(tri,norm)|{
            let x = ((tri[1]-tri[0]).normalize().cross(&(tri[2]-tri[1]).normalize())).normalize();
            assert!((x.x-norm[0]).abs() < 0.00001);
            assert!((x.y-norm[1]).abs() < 0.00001);
            assert!((x.z-norm[2]).abs() < 0.00001);
            project_point_down(&point, &tri[0], norm)
        })
        .next();
        // .unwrap_or(Point3::new(point.x,point.y,-1.0))
        // .expect("could not project point onto mesh_collider");
    if let Some(point) = x { return point } else {
        // dbg!(mesh);
        println!("found no triangles intersection point: {point}");
        Point3::new(point.x,point.y,-1.0)
        // panic!("found no triangles intersection point: {point}")
    }
}
pub fn project_contour_onto(contour:&Contour,mesh:&MeshCollider) -> Contour3d {

    Contour3d::from_unchecked( 
        contour.edges()
        .flat_map(|(e_start,e_end)|{
            let ed_vec = e_end-e_start;

            let mut intersections:Vec<(f32,f32)> = mesh.edges.iter()
                .filter_map(move |edge|{
                    let target_edge:[Point3<f32>;2] = [mesh.vertices[edge.0],mesh.vertices[edge.1]];

                    edge_edge_intersection3d([e_start,e_end],&target_edge)
                })
                .collect();

            intersections.sort_unstable_by(|(t1,_),(t2,_)| t1.partial_cmp(t2).unwrap());

            let intersection_points:Vec<Point3<f32>> = intersections.into_iter()
                .map(move |(t,z)|{
                    let point2d = e_start + t*ed_vec;
                    Point3::new(point2d.x,point2d.y,z)
                }).collect();

            let first_point = project_point_onto(e_start, mesh);
            // intersection_points.into_iter()
            std::iter::once(first_point).chain(intersection_points.into_iter())
        })
    .collect()
    )
}
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
impl Enclosed for &[Point3<f32>;3]{
    fn area(&self) -> f32 {
        todo!()
    }
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        let a = self[0].xy();
        let b = self[1].xy();
        let c = self[2].xy();

        let v0 = c - a;
        let v1 = b - a;
        let v2 = point - a;

        let dot00 = v0.dot(&v0);
        let dot01 = v0.dot(&v1);
        let dot02 = v0.dot(&v2);
        let dot11 = v1.dot(&v1);
        let dot12 = v1.dot(&v2);

        let denom = dot00 * dot11 - dot01 * dot01;
        if denom == 0.0 { return false } // Degenerate triangle

        let inv_denom = 1.0 / denom;
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= -EPSILON) && (v >= -EPSILON) && (u + v <= (1.0+EPSILON))
    }
}
#[test]
fn project_point_down_test(){
    let tri_pnt = Point3::new(12.468662, -11.523754, 0.20000005);
    let point = Point2::new( 11.968662,-11.023754 );
    let norm = stl_io::Vector::new([-0.0, 0.0, 1.0]);

    let s = project_point_down(&point, &tri_pnt, norm);
    assert!(s.is_some());
}

fn project_point_down(point:&Point2<f32>, tri_pnt:&Point3<f32>, norm:stl_io::Vector<f32>) -> Point3<f32>{
    let normal = Vector3::new(norm[0],norm[1],norm[2]);
    if normal.z == 0.0 {return Point3::new(point.x, point.y, tri_pnt.z)}
    // debug_assert!(normal.z > 1e-5); // <- face should not be parallell to the z-axis
    let a_point = Vector3::new(tri_pnt.x,tri_pnt.y,tri_pnt.z);

    // Solve for Z using plane equation:
    // normal · (P - v0) = 0 → normal_x * (x - x0) + normal_y * (y - y0) + normal_z * (z - z0) = 0
    let d = -normal.dot(&a_point);
    let z = -(normal[0] * point.x + normal[1] * point.y + d) / normal[2];
    Point3::new(point.x, point.y, z)
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
