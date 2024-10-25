#![allow(unused)]
use std::fs::{File, OpenOptions};
use std::io::BufReader;
use stl_io::{self, Normal, Triangle, IndexedTriangle};
use core::f32::consts::PI;
use std::collections::{HashMap, HashSet};


pub fn main() {
    let file_path = "../mesh/bunny2.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let stl_data = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");

    let overhangs = stl_data.faces.into_iter()
        .filter(|tri| tri.normal[2] < -PI/6. )
        .filter(|tri| stl_data.vertices[tri.vertices[0]][2] > 0.0 )
        .filter(|tri| stl_data.vertices[tri.vertices[1]][2] > 0.0 )
        .filter(|tri| stl_data.vertices[tri.vertices[2]][2] > 0.0 );

    // note: default hashmap is not optimized for integers, a different hashmap will likley preforme better

    let mut edge_count: HashMap<Edge, u32> = HashMap::new(); 

    for tri in overhangs.clone(){
        let [v1, v2, v3] = tri.vertices;
        let edges = [
            Edge::new(v1,v2),
            Edge::new(v2,v3),
            Edge::new(v3,v1),
        ];
        for edge in edges {
            *edge_count.entry(edge).or_insert( 0 ) +=1;
        }
    }
    let boundary_edges: Vec<Edge> = edge_count
        .into_iter()
        .filter(|(_,count)| *count == 1 )
        .map(|(edge,_)| edge)
        .collect();

    let result = Vec::new();
    for next_edge in boundary_edges.iter() {
      match next_edge {
        None => break,
        Some(edge) => {
          if result.len() <= boundary_edges.len() + 1 {break}

        }
      }
    }
        

    let out:Vec<Triangle> = overhangs.map(|tri|
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

}
#[derive(Clone, Hash, Eq, PartialEq)]
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


fn extract_perimeter(triangles: &[IndexedTriangle]) -> Vec<usize> {
    // Count how many times each edge appears
    let mut edge_count: HashMap<Edge, u32> = HashMap::new();
    
    // For each triangle, count its edges
    for triangle in triangles {
        let [v1, v2, v3] = triangle.vertices;
        let edges = [
            Edge::new(v1, v2),
            Edge::new(v2, v3),
            Edge::new(v3, v1),
        ];
        
        for edge in edges {
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }
    
    // Collect boundary edges (those that appear only once)
    let boundary_edges: Vec<Edge> = edge_count
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(edge, _)| edge)
        .collect();
    
    // Sort boundary edges into a continuous chain
    let mut result = Vec::new();
    if boundary_edges.is_empty() {
        return result;
    }
    
    let mut current_vertex = boundary_edges[0].0;
    let mut used_edges: HashSet<Edge> = HashSet::new();
    result.push(current_vertex);
    
    while !boundary_edges.is_empty() && result.len() <= boundary_edges.len() + 1 {
        let next_edge = boundary_edges.iter()
            .find(|edge| {
                !used_edges.contains(edge) && 
                (edge.0 == current_vertex || edge.1 == current_vertex)
            });
        
        if let Some(edge) = next_edge {
            used_edges.insert(edge.clone());
            current_vertex = if edge.0 == current_vertex { edge.1 } else { edge.0 };
            result.push(current_vertex);
        } else {
            break;
        }
    }
    
    result
}

// Helper function to convert vertex indices back to actual 3D points
fn get_perimeter_points(perimeter_indices: &[usize], vertices: &[[f32; 3]]) -> Vec<[f32; 3]> {
    perimeter_indices.iter()
        .map(|&idx| vertices[idx])
        .collect()
}
