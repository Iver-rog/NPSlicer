use core::{option::Option, unreachable};
use std::os::unix::process;
use crate::geo::Enclosed;

use crate::skeleton::*;

#[derive(Debug,Clone)]
pub struct Node{coords:Point3<f32>,next:[Option<usize>;2]}
pub struct VertexTree(Vec<Node>);

impl SkeletonBuilder {
    pub fn offset_inside_bounds(&self,bounds:Polygon,time:f32) -> Vec<Contour> {
        let vertex_positions:HashMap<usize,Option<Point2<f32>>> = self.shrining_polygon.active_nodes_iter()
            .map(|ndx|{
                let node = self.shrining_polygon.nodes[ndx];
                let node_v = &self.vertices[node.vertex_ndx];
                let current_pos = node_v.coords + node.bisector() * (time-node_v.time);
                ( node.vertex_ndx, if bounds.point_is_inside(&current_pos){Some(current_pos)}else{None} )
            }).collect();
        let vertex_tree = self.vertex_tree();
        let polygon = self.input_polygon_refs.iter();

        //let (vertex,is_new) = self.input_polygon_refs.iter()
        let offset_contours:Vec<Contour> = self.input_polygon_refs.iter()
            .cloned()
            .flat_map(|polygon_iter|iter::once(polygon_iter.outer_loop).chain(polygon_iter.holes.into_iter()))
            .inspect(|contour| {dbg!(contour);})
            .filter_map(|contour_iter|{
                let mut vertex_loop = Vec::new();
                for vertex in contour_iter {
                    let mut end_vertex = vertex;
                    loop{
                        match vertex_tree.0[end_vertex].next {
                            [Some(next_ndx),None] => {end_vertex = next_ndx;break},
                            [Some(next_ndx1),Some(next_ndx2)] => break,
                            [None,None] => break,
                            [_,_] => unreachable!(),
                        }
                    }
                    let vert = match vertex_positions.get(&end_vertex) {
                        Some(is_inside_bouds) => is_inside_bouds.unwrap_or(
                            self.vertices[vertex].coords.clone()
                            ),
                        None => return None, //self.vertices[vertex].coords.clone(),
                    };
                    match vertex_loop.last(){
                        Some(previous_vertex) => {if *previous_vertex == vert {continue}},
                        None => (),
                        }
                    vertex_loop.push(vert);
                }
                return Some(vertex_loop)
            })
            .filter(|contour| contour.len() >= 3)
            .map(|conotour|Contour::from(conotour))
            .collect();
        println!("there are {} contours", offset_contours.len());

        return offset_contours
    }
    pub fn vertex_tree(&self) -> VertexTree  {
        let mut vertex_tree:Vec<Node> = self.vertices.iter()
            .map(|p|Node{
                coords:Point3::new(p.coords.x,p.coords.y,p.time),
                next:[None,None]})
            .collect();
        for edge in self.edges.iter(){
            vertex_tree[edge.start].next = match vertex_tree[edge.start].next {
                [None,None] => [Some(edge.end),None],
                [Some(prev),None] => [Some(prev),Some(edge.end)],
                _ => unreachable!()
            }
        }
        return VertexTree(vertex_tree)
    }
}
#[test]
fn vertex_tree_test(){
    //let bounds = Polygon::new(Contour::new(vec![
    //    Point2::new( 0.2,-0.2),
    //    Point2::new( 1.2,-0.2),
    //    Point2::new( 1.2, 1.2),
    //    Point2::new( 0.2, 1.2),
    //    ]),
    let bounds = Polygon::new(Contour::new(vec![
        Point2::new(-0.2,-0.2),
        Point2::new( 1.2,-0.2),
        Point2::new( 1.2, 0.91),
        Point2::new(-0.2, 0.91),
        ]),
        vec![]
        );
    let outer_loop = Contour::new(vec![
        Point2::new(0.0,0.0),
        Point2::new(0.9,0.1),
        Point2::new(0.95,1.05),
        Point2::new(0.1,0.9),
    ]);
    let hole1 = Contour::from(vec![
        Point2::new(0.3,0.2),
        Point2::new(0.6,0.2),
        Point2::new(0.6,0.55),
        Point2::new(0.3,0.5),
    ]);
    let hole2 = Contour::from(vec![
        Point2::new(0.1,0.1),
        Point2::new(0.2,0.1),
        Point2::new(0.2,0.15),
        Point2::new(0.1,0.2),
    ]);
    let mut polygon = Polygon::new( outer_loop, vec![hole1,hole2] );
    polygon.invert();
    dbg!(&polygon);
    let time = 0.1;
    let mut skeleton = SkeletonBuilder::new();
    let mut blender = crate::Blender::new();
    blender.polygon(&polygon,0.0);
    blender.polygon(&bounds,0.1);
    skeleton.add_polygon(polygon);
    skeleton.compute_untill(time).unwrap();
    //blender.line_body2d(
    //    &skeleton.vertices.iter().map(|v|[v.coords.x,v.coords.y]).collect(),
    //    skeleton.edges.iter().map(|e|[e.start,e.end]).collect()
    //    );
    let offset_contours = skeleton.offset_inside_bounds(bounds, time);
    offset_contours.iter().for_each(|contour| blender.contour(contour,0.2));
    blender.show();
    assert!(false)
}
