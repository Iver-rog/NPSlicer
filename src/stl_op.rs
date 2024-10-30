#![allow(unused)]
use crate::*;
use std::fs::{File, OpenOptions};
use std::io::BufReader;
use std::f32::consts::PI;
use std::collections::{HashMap, HashSet, LinkedList};
use nalgebra_glm::equal_eps_vec;
use stl_io::{self, Triangle, IndexedTriangle, Vector};

pub fn main() {
    let file_path = "../mesh/bunny2.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let stl_data = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    let edge_to_tri = edges_to_triangles_map(&stl_data);

    let overhangs: Vec<IndexedTriangle> = stl_data.faces.clone().into_iter()
        .filter(|tri| tri.normal[2] < -PI/6.0 )
        .filter(|tri|{ !(
            stl_data.vertices[tri.vertices[0]][2] < 0.0 &&
            stl_data.vertices[tri.vertices[1]][2] < 0.0 &&
            stl_data.vertices[tri.vertices[2]][2] < 0.0 
            )})
        .collect();

    let edge_perimeters = extract_perimeters(overhangs.clone());
    //utils::write_loops_to_file(&edge_perimeters,&stl_data);

    let large_edge_loops:Vec<Vec<usize>> = edge_perimeters.into_iter()
        .filter(|edge_loop| area(edge_loop,&stl_data.vertices) > 20.0 )
        .collect();
    utils::write_loops_to_file(&large_edge_loops,&stl_data);

    let out:Vec<Triangle> = overhangs.into_iter()
        .map(|tri|
          Triangle {
            normal: tri.normal,
            vertices: [
              stl_data.vertices[tri.vertices[0]],
              stl_data.vertices[tri.vertices[1]],
              stl_data.vertices[tri.vertices[2]],
            ]
          }
        )
        .collect();

    let mut file = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open("../mesh/output2.stl")
        .expect("Failed to create output file");
    stl_io::write_stl(&mut file, out.iter()).unwrap();
    println!("wrote stl file to disk");

}
fn edges_to_triangles_map<'a>( stl_data:&'a stl_io::IndexedMesh ) -> HashMap<Edge,[&'a IndexedTriangle;2]>{
    let mut edge_to_tri_ndx: HashMap<Edge,(&IndexedTriangle,Option<&IndexedTriangle>)>
        = HashMap::with_capacity(stl_data.faces.len()*2);

    for tri in stl_data.faces.iter() {
        let [v1, v2, v3] = tri.vertices;
        let edges = [
            Edge::new(v1,v2),
            Edge::new(v2,v3),
            Edge::new(v3,v1)
        ];
        for edge in edges { 
            let entry = edge_to_tri_ndx.entry(edge)
                .or_insert((&tri,None));
            entry.1 = Some(&tri);
        }
    }
    let mut edge_to_tri = HashMap::with_capacity(stl_data.faces.len());
    for (key,val) in edge_to_tri_ndx.into_iter() {
        edge_to_tri.insert( key, [val.0, val.1.expect("Non manifold mesh")] );
    }
    return edge_to_tri
}
#[derive(Debug, Clone, Hash, Eq, PartialEq)]
struct Edge(usize,usize);
impl Edge {
    fn new(v1:usize, v2:usize) -> Self {
        if v1 < v2 {
            Edge(v1, v2)
        } else {
            Edge(v2, v1) 
        }
    }
}


fn extract_perimeters(triangles: Vec<IndexedTriangle>) -> Vec<Vec<usize>> {
    // note: default hashmap is not optimized for integers, a different hashmap will likley preforme better
    let mut edge_count: HashMap<Edge, bool> = HashMap::new(); 

    for tri in triangles{
        let [v1, v2, v3] = tri.vertices;
        let edges = [
            Edge::new(v1,v2),
            Edge::new(v2,v3),
            Edge::new(v3,v1)
        ];
        for edge in edges { 
            edge_count.entry(edge)
                .and_modify(|only_1_ref| *only_1_ref = false)
                .or_insert(true);
        }
    }

    let mut boundary_edges: LinkedList<Edge> = edge_count.into_iter()
        .filter(|(_,only_1_ref)| *only_1_ref )
        .map(|(edge,_)| edge)
        .collect();

    let mut edge_loops = Vec::new();

    while ! boundary_edges.is_empty() {
        let first_edge = boundary_edges.pop_front().expect("boundary edges is empty");
        let starting_vertex = first_edge.0;
        let mut next_vertex = first_edge.1;
        let mut edge_loop = vec![starting_vertex];

        while next_vertex != starting_vertex {
            edge_loop.push(next_vertex);
            let (i,edge) = boundary_edges.iter()
                .enumerate()
                .find(|(_,edge)|{
                    edge.0 == next_vertex || edge.1 == next_vertex
                })
                .expect("next vertex could not be found");
            next_vertex = if next_vertex != edge.0 {edge.0} else {edge.1};
            let mut list_remainder = boundary_edges.split_off(i);
            list_remainder.pop_front();
            boundary_edges.append(&mut list_remainder);
        }
        edge_loops.push(edge_loop);
    }
    return edge_loops
}
fn find_connected_tris <'a>(
    triangles: &[IndexedTriangle], 
    edge_to_tris: HashMap<Edge,[&'a IndexedTriangle;2]>, 
    ) -> Vec<Vec<&'a IndexedTriangle>> {

    let mut islands: Vec<Vec<Triangle>> = Vec::new();
    let mut unvisited: HashSet<usize> = (0..triangles.len()).collect();

    let mut triangle_adjacency: Vec<Vec<usize>> = vec![Vec::new();triangles.len()];

    for tri in triangles{
    }

    todo!()
}
fn neighbouring_tris<'a>(
    tri:&IndexedTriangle, 
    edge_to_tris: HashMap<Edge,[&'a IndexedTriangle;3]>
    ) -> Result<[&'a IndexedTriangle;3],()>
    {
    let [v1, v2, v3] = tri.vertices;
    let edges = [
        Edge::new(v1,v2),
        Edge::new(v2,v3),
        Edge::new(v3,v1)
    ];
    let mut neighbours = edges.into_iter()
        .map(|edge|{
            let tris = *edge_to_tris.get(&edge).unwrap();
            return if tris[0] == tri {tris[1]} else { tris[0] }
        });
    return Ok([
        neighbours.next().unwrap(),
        neighbours.next().unwrap(),
        neighbours.next().unwrap()
    ])
}

fn area(point_index:&Vec<usize>, points: &Vec<Vector<f32>>) -> f32 {
    // computes the area of a polygon projected onto the xy-plane using Green's theorem
    let point_index_offset_by_one = point_index.iter()
        .skip(1)
        .chain( point_index.iter() );

    return 0.5 * point_index.iter()
        .zip(point_index_offset_by_one)
        .map(|(p1,p2)|{
            let x1 = points[*p1][0];
            let y1 = points[*p1][1];
            let x2 = points[*p2][0];
            let y2 = points[*p2][1];
            return x1*y2 - x2*y1
        })
        .sum::<f32>()
        .abs()
}
#[test]
fn test_area_function(){
    // computes the area of a square centered on the origin with edges = 2
    let point_index = vec![0,1,2,3];
    let points = vec![
        Vector::new([ 1.0,  1.0, 0.0]),
        Vector::new([-1.0,  1.0, 0.0]),
        Vector::new([-1.0, -1.0, 0.0]),
        Vector::new([ 1.0, -1.0, 0.0])
    ];
    assert_eq!(area(&point_index, &points),4.0)
}
