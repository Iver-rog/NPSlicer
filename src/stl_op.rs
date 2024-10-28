#![allow(unused)]
use crate::*;
use std::fs::{File, OpenOptions};
use std::io::BufReader;
use std::f32::consts::PI;
use std::collections::{HashMap, LinkedList};
use nalgebra_glm::equal_eps_vec;
use stl_io::{self, Normal, Triangle, IndexedTriangle};

pub fn main() {
    let file_path = "../mesh/bunny2.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let stl_data = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");

    let overhangs: Vec<IndexedTriangle> = stl_data.faces.clone().into_iter()
        .filter(|tri| tri.normal[2] < -PI/6.0 )
        .filter(|tri| stl_data.vertices[tri.vertices[0]][2] > 0.0 )
        .filter(|tri| stl_data.vertices[tri.vertices[1]][2] > 0.0 )
        .filter(|tri| stl_data.vertices[tri.vertices[2]][2] > 0.0 )
        .collect();

    let edge_perimeters = extract_perimeters(overhangs.clone());
    utils::write_loops_to_file(edge_perimeters,&stl_data);

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
