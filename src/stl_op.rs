use crate::utils::Blender;
use core::{assert, f32};
use std::iter::{IntoIterator, Iterator};
use std::collections::{HashMap, HashSet, LinkedList, VecDeque};
use stl_io::{self, IndexedMesh, IndexedTriangle, Vector};
use nalgebra::{min, point, wrap, Point2, Point3, Vector2};

#[derive(Debug,Clone,PartialEq)]
pub struct Polygon{
    outer_loop: Contour,
    holes: Vec<Contour>,
}
impl Polygon {
    fn new(mut outer_loop:Contour,mut holes:Vec<Contour>)->Self{
        if outer_loop.area.is_sign_negative() {
            outer_loop.reverse_order();
        }
        for hole in holes.iter_mut() {
            if hole.area.is_sign_positive(){
                hole.reverse_order();
            }
        }
        Self{
            outer_loop,
            holes,
        }
    }
}
#[derive(Debug,Clone,PartialEq)]
pub struct Contour{
    area:f32,
    aabb:AABB,
    points: Vec<Point2<f32>>
}
#[derive(Debug,Clone,PartialEq)]
pub struct AABB{
    x_max:f32,
    x_min:f32,
    y_max:f32,
    y_min:f32,
}
impl AABB {
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        self.x_min <= point.x && point.x <= self.x_max &&
        self.y_min <= point.y && point.y <= self.y_max
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
impl Contour {
    pub fn reverse_order(&mut self) {
        self.points.reverse();
        self.area = self.area * -1.0;
    }
    pub fn new(points:Vec<Point2<f32>>) -> Self {
        let first_point = points[0];

        let mut aabb = AABB{
            x_max: first_point.x,
            x_min: first_point.x,
            y_max: first_point.y,
            y_min: first_point.y,
        };
        let mut area = 0.0;
        let mut prev_point = first_point.clone();
        for point in points.iter().skip(1) {
            aabb.x_max = aabb.x_max.max(point.x);
            aabb.x_min = aabb.x_min.min(point.x);
            aabb.y_max = aabb.y_max.max(point.y);
            aabb.y_min = aabb.y_min.min(point.y);
            
            area += prev_point.x*point.y-point.x*prev_point.y;
            prev_point = *point;
        }

        return Contour{
            area: area/2.0,
            aabb,
            points,
        }
    }
    pub fn point_is_inside(&self,point:&Point2<f32>)->bool{
        // returns true if a point is on or inside the contour
        if !self.aabb.point_is_inside(&point) { return false }

        let points_offset_by_one = self.points.iter()
            .skip(1)
            .chain( self.points.iter() );

        let intersections: usize = self.points.iter()
            .zip(points_offset_by_one)
            // cast a ray from the test point towards the right direction allong 
            // the x-axis and check if the ray intersects the edge
            .map(|(p1,p2)|{
                // check if the two points of the edge are on opposite sides of 
                // the horizontal line at test point's x value
                ((p1.y < point.y) != (p2.y <= point.y)) && 
                // find where the edge intersects the horizontal line at test point's x value 
                // and check if the crossing point lies on the right side of the test point
                point.x <= ((point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x)
            })
            .map(|bool| match bool { true => 1, false => 0, } )
            .sum();
        return intersections % 2 == 1
    }
    pub fn x_distance_to_contour(&self,point:&Point2<f32>)->Option<f32>{
        // returns true if a point is on or inside the contour
        if !self.aabb.point_is_inside(&point) { return None }

        let points_offset_by_one = self.points.iter()
            .skip(1)
            .chain( self.points.iter() );

        let intersections: Vec<f32> = self.points.iter()
            .zip(points_offset_by_one)
            // cast a ray from the test point towards the right direction allong 
            // the x-axis and check if the ray intersects the edge
            .filter_map(|(p1,p2)|{
                // check if the two points of the edge are on opposite sides of 
                // the horizontal line at test point's x value
                if (p1.y < point.y) == (p2.y <= point.y) { return None }
                // find where the edge intersects the horizontal line at test point's x value 
                // and check if the crossing point lies on the right side of the test point
                else {
                    let d_x = (point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x;
                    if point.x <= d_x { return Some(d_x-point.x)}
                    else {return None;}
                }
            })
            .collect();
        if intersections.len() % 2 == 1 { 
            //let min = f32::INFINITY;
            return Some(intersections.into_iter()
                .fold(f32::INFINITY, |a, b| a.min(b))
                )
        }
        else { return None }
    }
    pub fn is_positively_oriented(&self) -> bool { self.area.is_sign_positive() }
}
//pub fn ray_intersect(point:Point2<f32>,vector:Vector2<f32>,edge:[Point2<f32>;2])-> bool {
//    let a = edge[0] - point;
//    let b = edge[1] - point;
//    vector.dot(a);
//    true
//}
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

pub fn polygons_from_contours(contours:Vec<Contour>)->Vec<Polygon>{
    let mut polygons = vec![Some(Vec::new());contours.len()];
    for (i,contour) in contours.iter().enumerate() {
        let test_point = contour.points[0];

        let mut contour_i_is_inside = None;
        let mut contour_i_is_exterior = true;
        for (n,intersection_contour) in contours.iter().enumerate(){
            if i == n {continue}
            match intersection_contour.x_distance_to_contour(&test_point) {
                Some(new_distance) => {
                    match contour_i_is_inside {
                        Some((_contour,old_distance)) => {
                            if new_distance < old_distance { 
                            contour_i_is_inside = Some((n,new_distance))
                            }
                        }
                        None => {
                            contour_i_is_inside = Some((n,new_distance));
                            contour_i_is_exterior = false;
                        },
                    }
                }
                None => (),
            }
            println!("{contour_i_is_inside:?}");
        }
        //dbg!( &contour_i_is_inside );
        //dbg!( &contour_i_is_exterior );
        if !contour_i_is_exterior {
            match contour_i_is_inside{
                Some((n,d_x)) => { 
                    match &mut polygons[n]{
                        Some(vector) => {
                            vector.push(i);
                            polygons[i] = None;
                        },
                        none => panic!(),
                    }
                },
                None => (),
            }
        }
    }
    //dbg!(&polygons);
    polygons.iter()
        .enumerate()
        .filter_map(|(i,p)| match p {
            Some(x)=> Some((i,x)),
            None => None,
        } )
        .map(|(contour_ndx,holes_ndxes)|{
            let contour = contours[contour_ndx].clone();
            let holes = holes_ndxes.into_iter().map(|ndx|contours[*ndx].clone()).collect();
            Polygon::new(contour,holes)
        }).collect()
}

pub fn extract_planar_layers( mesh:&IndexedMesh, layer_height:f32 , blender:&mut Blender) -> Vec<Vec<Vec<Point2<f32>>>> {
    let z_max = mesh.vertices.iter().map(|vert| (vert[2]/layer_height).ceil() as usize).max().unwrap();

    // Identify which faces intersect which z-planes and save result in look_up_table
    let mut look_up_table:Vec<HashSet<usize>> = vec![HashSet::new();z_max];
    for (face_ndx,min,max) in mesh.faces.iter().enumerate()
        .map(|(i,tri)| {
            let min = tri.vertices.iter()
                .map(|vert| (mesh.vertices[*vert][2] / layer_height ).ceil() as usize)
                .min().unwrap();
            let max: usize = tri.vertices.iter()
                .map(|vert| (mesh.vertices[*vert][2] / layer_height ).floor() as usize)
                .max().unwrap();
            (i,min,max)
        }){
            for layer_nr in min..=max {
                look_up_table[layer_nr].insert(face_ndx);
            }
        }

    // Debug: export faces to blender
    //for (layer_nr,face_ndxes) in look_up_table.iter().enumerate() {
    //assert_eq!(look_up_table[layer_nr],face_ndxes.clone());
    //    let owned_faces:Vec<IndexedTriangle> = face_ndxes.into_iter()
    //        .map(|face_ndx| mesh.faces[*face_ndx].clone())
    //        .collect();
    //    blender.save_mesh(&owned_faces,&mesh.vertices,format!("layer {layer_nr}"));
    //}

    let edges_to_tri_map = edges_to_triangles_map( mesh );
    let mut contours:Vec<Vec<Vec<Point2<f32>>>> = vec![Vec::new();z_max];

    for (layer_nr,face_ndxes) in look_up_table.iter().enumerate() {
        let z_plane = layer_height*(layer_nr as f32);
        //println!("layer number: {} z_plane: {}",&layer_nr,&z_plane);
        let contour = extract_layer(
            z_plane,
            face_ndxes,
            mesh,
            &edges_to_tri_map,
            );
        contours[layer_nr] = contour;
        //for acontour in contour{
        //contours[layer_nr].push(acontour);
        //    //blender.edge_loop_points(&acontour.iter().map(|p|[p.x,p.y,p.z]).collect());
        //    contours.push(acontour);
        //}
    }
    assert_eq!(contours.len(),z_max);
    contours
}
#[allow(non_snake_case)]
fn extract_layer(
    z_plane:f32,
    face_ndxes:&HashSet<usize>,
    mesh:&IndexedMesh,
    edges_to_tri_map:&HashMap<Edge,[usize;2]>
    ) ->Vec<Vec<Point2<f32>>> {

    let mut handled_faces:HashSet<usize> = HashSet::new();
    let mut handled_edges:HashSet<Edge> = HashSet::new();
    let mut contours = Vec::new();
    while handled_faces.len() < face_ndxes.len() {

        // Find a unprosessed face and edge to start the contour
        for face_ndx in face_ndxes.iter(){
            if handled_faces.contains(face_ndx) {continue}
            let face = &mesh.faces[*face_ndx];
            handled_faces.insert(*face_ndx);
            let edges = get_edges(face);

            let (intersection_p,first_edge) = match edges.iter()
                .filter_map(|edge|{
                    let p1 = mesh.vertices[edge.0];
                    let edge_start = Point3::new(p1[0],p1[1],p1[2]);
                    let p2 = mesh.vertices[edge.1];
                    let edge_end = Point3::new(p2[0],p2[1],p2[2]);
                    match edge_zplane_intersection(edge_start, edge_end, z_plane){
                        Some(intersection_p) => {
                            Some((intersection_p.xy(),edge))
                        },
                        None => {
                            None
                        },
                    }
                }).next() {
                    Some(result)=>{ result },
                    None => {
                        continue
                    }
                };
            handled_edges.insert(first_edge.clone());

            // Inner loop
            let mut prev_edge = first_edge.clone();
            let mut contour = vec![intersection_p];
            loop{
                let [face1,face2] = edges_to_tri_map.get(&prev_edge).expect("prev edge exists in edges to tri map");

                let next_tri = if handled_faces.contains(face1) { face2 }
                    else { 
                        if handled_faces.contains(face1){ 
                            println!("Loop completed");
                            break
                        }
                        else { face1}
                    };

                handled_faces.insert(*next_tri);
                let face = &mesh.faces[*next_tri];
                let edges = get_edges(face);

                match edges.iter()
                    .filter_map(|edge|{
                        if handled_edges.contains(edge){ return None;}
                        let p1 = mesh.vertices[edge.0];
                        let edge_start = Point3::new(p1[0],p1[1],p1[2]);
                        let p2 = mesh.vertices[edge.1];
                        let edge_end = Point3::new(p2[0],p2[1],p2[2]);
                        match edge_zplane_intersection(edge_start, edge_end, z_plane){
                            Some(intersection_p) => Some((intersection_p,edge)),
                            None => None,
                        }
                    }).next() {
                        Some((intersection_p,edge))=>{
                            contour.push(intersection_p.xy());
                            handled_edges.insert(edge.clone());
                            prev_edge = edge.clone();
                            continue;
                        },
                        None => {
                            contours.push(contour);
                            break
                        }
                    };
                }
            }
        }
    contours
}
fn get_edges(face:&IndexedTriangle)->[Edge;3]{
    let [v1, v2, v3] = face.vertices;
    [
        Edge::new(v1,v2),
        Edge::new(v2,v3),
        Edge::new(v3,v1)
    ]
}
#[test]
fn edge_zplane_intersection_test(){
    let edge_start = Point3::new(0.,0.,0.);
    let edge_end = Point3::new(1.,1.,1.);
    let result = edge_zplane_intersection(edge_start, edge_end, 0.5).expect("valid result");
    assert_eq!(Point3::new(0.5,0.5,0.5),result);
    let result = edge_zplane_intersection(edge_end, edge_start, 0.5).expect("valid result");
    assert_eq!(Point3::new(0.5,0.5,0.5),result);
    let result = edge_zplane_intersection(edge_start, edge_end, 1.5);
    assert_eq!(None,result);
    let result = edge_zplane_intersection(edge_end, edge_start, 1.5);
    assert_eq!(None,result);
}
fn edge_zplane_intersection(edge_start:Point3<f32>,edge_end:Point3<f32>,z_plane:f32)-> Option<Point3<f32>>{
    let edge_vec = edge_end - edge_start;
    let scale = (z_plane - edge_start.z) / edge_vec.z;
    if scale.is_sign_negative() {return None;}
    if scale > 1.0 {return None;}
    return Some(edge_start + edge_vec*scale)
}

pub fn extract_overhangs(mesh:&IndexedMesh,angle:f32) -> Vec<Vec<IndexedTriangle>>{

    let overhangs: Vec<&IndexedTriangle> = mesh.faces.iter()
        .filter(|tri| tri.normal[2] < angle )
        .filter(|tri|{ !(
            mesh.vertices[tri.vertices[0]][2] < 0.0 &&
            mesh.vertices[tri.vertices[1]][2] < 0.0 &&
            mesh.vertices[tri.vertices[2]][2] < 0.0 
            )})
        .collect();

    let overhang_regions = find_connected_components(&overhangs);
    return overhang_regions
}

pub fn extract_contours_larger_than(overhang_regions:Vec<Vec<IndexedTriangle>>,mesh:&IndexedMesh,min_area:f32)->Vec<Vec<usize>>{

    let mut overhang_contours:Vec<Vec<usize>> = overhang_regions.into_iter()
        .map(|region| extract_perimeter(region))
        .filter(|edge_loop| area_indexed_contour(edge_loop,&mesh.vertices).abs() > min_area )
        .collect();

    for edge_loop in &mut overhang_contours{
        if area_indexed_contour(&edge_loop,&mesh.vertices) < 0.0{
            edge_loop.reverse();
        }
    }

    return overhang_contours

    //let mut edge_loops_points = overhang_contours.into_iter()
    //    .map(|contour| contour.into_iter().map(|ndx| mesh.vertices[ndx]).collect() )
    //    .collect();
    //
    //return edge_loops_points
}

fn edges_to_triangles_map<'a>( stl_data:&'a stl_io::IndexedMesh ) -> HashMap<Edge,[usize;2]>{
    let mut edge_to_tri_ndx: HashMap<Edge,(usize,Option<usize>)>
        = HashMap::with_capacity(stl_data.faces.len()*2);

    for (i,tri) in stl_data.faces.iter().enumerate() {
        let [v1, v2, v3] = tri.vertices;
        let edges = [
            Edge::new(v1,v2),
            Edge::new(v2,v3),
            Edge::new(v3,v1)
        ];
        for edge in edges { 
            let entry = edge_to_tri_ndx.entry(edge)
                .or_insert((i,None));
            entry.1 = Some(i);
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

pub fn area_contour2d(point_index:&Vec<Point2<f32>>) -> f32 {
    // computes the area of a edge perimeter using Green's theorem
    let point_index_offset_by_one = point_index.iter()
        .skip(1)
        .chain( point_index.iter() );

    return 0.5 * point_index.iter()
        .zip(point_index_offset_by_one)
        .map(|(p1,p2)|{
            let x1 = p1[0];
            let y1 = p1[1];
            let x2 = p2[0];
            let y2 = p2[1];
            return x1*y2 - x2*y1
        })
        .sum::<f32>();
}
pub fn area_indexed_contour(point_index:&Vec<usize>, points: &Vec<Vector<f32>>) -> f32 {
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
        .sum::<f32>();
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
    assert_eq!(area_indexed_contour(&point_index, &points),4.0)
}
fn find_connected_components(triangles: &[&IndexedTriangle]) -> Vec<Vec<IndexedTriangle>> {
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
