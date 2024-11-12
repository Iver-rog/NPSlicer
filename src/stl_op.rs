#![allow(unused)]
use crate::*;
use utils::Blender;
use std::fs::File;
use std::io::BufReader;
use std::f32::consts::PI;
use std::collections::{HashMap, HashSet, LinkedList, VecDeque};
use stl_io::{self, Triangle, IndexedTriangle, Vector};

pub fn main(blender:&mut Blender) -> Vec<Vec<Vector<f32>>>{
    let file_path = "../mesh/bunny2.stl";

    let file = File::open(file_path).expect("Failed to open STL file");
    let mut reader = BufReader::new(file);

    let stl_data = stl_io::read_stl(&mut reader).expect("Failed to parse STL file");
    //let edge_to_tri = edges_to_triangles_map(&stl_data);

    let overhangs: Vec<IndexedTriangle> = stl_data.faces.iter()
        .filter(|tri| tri.normal[2] < -PI/6.0 )
        .filter(|tri|{ !(
            stl_data.vertices[tri.vertices[0]][2] < 0.0 &&
            stl_data.vertices[tri.vertices[1]][2] < 0.0 &&
            stl_data.vertices[tri.vertices[2]][2] < 0.0 
            )})
        .map(|tri| tri.clone() )
        .collect();
    blender.save_mesh(&overhangs, &stl_data.vertices,"overhangs".to_string());

    let overhang_regions = find_connected_components(&overhangs);
    utils::print_component_info(&overhang_regions);
    for (i,islands) in overhang_regions.iter().enumerate(){
        let filename = format!("overhang_island{i}");
        blender.save_mesh(&islands, &stl_data.vertices,filename);
    }

    let mut edge_perimeters:Vec<Vec<usize>> = Vec::new();
    for region in overhang_regions {
        edge_perimeters.push( extract_perimeter(region) );
    }

    let large_edge_loops:Vec<Vec<usize>> = edge_perimeters.into_iter()
        .filter(|edge_loop| area(edge_loop,&stl_data.vertices) > 20.0 )
        .collect();
    for edge_loop in large_edge_loops.clone() {
        blender.edge_loop(&edge_loop,&stl_data);
    }

    let mut edge_loops_points = Vec::new();
    for edge_loop in large_edge_loops {
        let edge_loop_points:Vec<Vector<f32>> = edge_loop.into_iter()
            .map(|ndx| stl_data.vertices[ndx])
            .collect();
        edge_loops_points.push(edge_loop_points);
    }

    return edge_loops_points
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


fn extract_perimeter(triangles: Vec<IndexedTriangle>) -> Vec<usize> {
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
    return edge_loop
}

fn area(point_index:&Vec<usize>, points: &Vec<Vector<f32>>) -> f32 {
    // computes the area of a edge perimeter projected onto the xy-plane using Green's theorem
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
fn find_connected_components(triangles: &[IndexedTriangle]) -> Vec<Vec<IndexedTriangle>> {
    let mut components: Vec<Vec<IndexedTriangle>> = Vec::new();
    let mut unvisited: HashSet<usize> = (0..triangles.len()).collect();
    
    // Build adjacency information
    let mut edge_to_triangles: HashMap<Edge, Vec<usize>> = HashMap::new();
    
    // Map edges to the triangles that contain them
    for (triangle_idx, triangle) in triangles.iter().enumerate() {
        let [v1, v2, v3] = triangle.vertices;
        let edges = [
            Edge::new(v1, v2),
            Edge::new(v2, v3),
            Edge::new(v3, v1),
        ];
        for edge in edges {
            edge_to_triangles
                .entry(edge)
                .or_insert_with(Vec::new)
                .push(triangle_idx);
        }
    }
    // Build triangle adjacency list
    let mut triangle_adjacency: Vec<Vec<usize>> = vec![Vec::new(); triangles.len()];
    
    for triangle_indices in edge_to_triangles.values() {
        if triangle_indices.len() == 2 {
            triangle_adjacency[triangle_indices[0]].push(triangle_indices[1]);
            triangle_adjacency[triangle_indices[1]].push(triangle_indices[0]);
        }
    }
    // Find connected components using BFS
    while let Some(&start) = unvisited.iter().next() {
        let mut component = Vec::new();
        let mut queue = VecDeque::new();
        
        queue.push_back(start);
        unvisited.remove(&start);
        
        while let Some(current) = queue.pop_front() {
            component.push(triangles[current].clone());
            
            // Add unvisited neighbors to queue
            for &neighbor in &triangle_adjacency[current] {
                if unvisited.remove(&neighbor) {
                    queue.push_back(neighbor);
                }
            }
        }
        components.push(component);
    }
    return components
}
