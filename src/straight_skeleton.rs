use nalgebra::{Point2, Vector2};
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::collections::HashSet;
use thiserror::Error;
use std::hash::Hash;

#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("Computation error: {0}")]
    ComputationError(String),
}
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Vertex {
    pub ndx: usize,
    pub next_ndx: usize,
    pub prev_ndx: usize,
    //pub bisector: Vector2<f32>
}
#[derive(Debug, Clone)]
pub struct VertexData {
    pub cooridnates: Point2<f32>,
    pub bisector: Vector2<f32>,
}
#[derive(Debug, Clone)]
pub struct Edge {
    pub start: usize,
    pub end: usize,
}
#[derive(Debug)]
pub struct StraightSkeleton {
    pub vertices: Vec<Point2<f32>>,
    pub edges: Vec<[usize;2]>,
}
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct Event {
    time: OrderedFloat<f32>,
    vertex: Vertex,
    event_type: EventType,
}
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum EventType {
    Edge,  // A edge shrinks to zero lenght
    Split, // A region is split into two parts
    Vertex,// A region disapears/colapses into a vertex
}

#[derive(Debug)]
pub struct SkeletonBuilder {
    vert_refs: Vec<Vertex>,
    vert_data: Vec<VertexData>,
    active_vertices: HashSet<usize>,
    edges: Vec<Edge>,
    events: PriorityQueue<Event, OrderedFloat<f32>>,
}
impl SkeletonBuilder {
    pub fn new(points: Vec<Point2<f32>>, weights: &[f32]) -> Result<Self, SkeletonError> {
        if points.len() < 3 {
            return Err(SkeletonError::InvalidPolygon(
                "Polygon must have at least 3 vertices".to_string(),
            ));
        }
        if points.len() != weights.len() {
            return Err(SkeletonError::InvalidPolygon(
                "Number of weights must match number of vertices".to_string(),
            ));
        }
        let mut vert_refs = Vec::new();
        let mut edges = Vec::new();
        let mut active_vertices = HashSet::new();
        let mut vert_data = Vec::new();

        //builder.initialize_polygon(weights)?;
        for i in 0..points.len() {
            let next_ndx = (i + 1) % points.len();
            let prev_ndx = if i == 0 {points.len()-1} else { i-1 };

            vert_refs.push(Vertex {
                ndx: i,
                next_ndx,
                prev_ndx,
            });
            edges.push(Edge {
                start: i,
                end: next_ndx,
                //weight: weights[i],
            });
            active_vertices.insert(i);

            let p_current = points[i];
            let p_next = points[next_ndx];
            let p_prev = points[prev_ndx];
            vert_data.push( VertexData{
                cooridnates: p_current,
                bisector: bisector(p_current, p_next, p_prev).unwrap(),
            }
            )
        }

        let mut builder = SkeletonBuilder {
            vert_refs,
            edges,
            vert_data,
            events: PriorityQueue::new(),
            active_vertices,
        };


        //initialize events 
        let mut event_que: PriorityQueue<Event, OrderedFloat<f32>> = PriorityQueue::new();
        for vertex in builder.vert_refs.iter() { 
            let next_idx = vertex.next_ndx;

            // Edge events
            let edge_event = compute_edge_event(&vertex, builder.vert_data[vertex.ndx].clone(),builder.vert_data[next_idx].clone())?;
            if let Some(event) = edge_event {
                event_que.push(event.clone(), -event.time);
            }
            // Split events
            let split_events = builder.compute_split_events(vertex)?;
            for event in split_events {
                event_que.push(event.clone(), -event.time);
            }
        }
        builder.events = event_que;

        Ok(builder)
    }
    fn position(&self, vertex:&Vertex) -> Point2<f32> {
        self.vert_data[vertex.ndx].cooridnates
    }

    fn find_events(&mut self, vertex:Vertex, current_time: OrderedFloat<f32>) -> Result<(), SkeletonError> {

        // Edge events
        let edge_event = compute_edge_event(&vertex, self.vert_data[vertex.ndx].clone(), self.vert_data[vertex.next_ndx].clone())?;
        if let Some(event) = edge_event {
            self.events.push(event.clone(), -(current_time+event.time));
        }
        // Split events
        let split_events = self.compute_split_events(&vertex)?;
        for event in split_events {
            self.events.push(event.clone(), -(current_time+event.time));
        }
        Ok(())
    }

    fn compute_split_events<'a>(&'a self, vertex: &'a Vertex) -> Result<Vec<Event>, SkeletonError> {
        let mut events = Vec::new();
        //let vertex = &self.vert_data[vertex.ndx];

        for edge in self.edges.iter() {
            if edge.start != vertex.ndx && edge.end != vertex.ndx {
                let v1 = &self.vert_data[edge.start].cooridnates;
                let v2 = &self.vert_data[edge.end].cooridnates;

                if let Some(t) = self.compute_split_time(&vertex, &v1, &v2)? {
                    events.push(Event {
                        time: OrderedFloat(t),
                        vertex: *vertex,
                        event_type: EventType::Split,
                    });
                }
            }
        }
        Ok(events)
    }


    fn compute_split_time(
        &self,
        vertex: &Vertex,
        v1: &Point2<f32>,
        v2: &Point2<f32>,
    ) -> Result<Option<f32>, SkeletonError> {
        // Implementation of split event time computation
        // This is a simplified version - you might want to add more robust geometric computations
        let edge_vec = v2 - v1;
        let vertex_vec = self.position(vertex) - v1;
        
        let t = edge_vec.dot(&vertex_vec) / edge_vec.norm_squared();
        
        if t > 0.0 && t < 1.0 {
            Ok(Some(t))
        } else {
            Ok(None)
        }
    }
    pub fn compute_skeleton(mut self) -> Result<StraightSkeleton, SkeletonError> {
        while let Some((event, _)) = self.events.pop() {
            match event.event_type {
                EventType::Edge => self.handle_edge_event(event)?,
                EventType::Split => {self.handle_split_event(event)?; },
                EventType::Vertex => todo!(),//self.handle_vertex_event(event)?,
            }
            if self.active_vertices.len() < 3 {
                break;
            }
        }
        let vertices = self.vert_data.into_iter()
            .map(|vert_data| vert_data.cooridnates )
            .collect();
        let edges = self.edges.into_iter()
            .map(|edge| [edge.start,edge.end] )
            .collect();
        let result = StraightSkeleton {
            vertices,
            edges,
        };
        Ok(result)
    }
    fn handle_edge_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        let vertex_ndx = event.vertex.ndx;
        let next_ndx = event.vertex.next_ndx;
        let prev_ndx = event.vertex.prev_ndx;

        if !self.active_vertices.contains(&vertex_ndx) || !self.active_vertices.contains(&next_ndx) {
            return Ok(());
        }

        // Calculate new vertex position
        let vertex = &self.vert_data[vertex_ndx];
        let time = event.time.0;
        let new_position = vertex.cooridnates + vertex.bisector * time;
        // Add new skeleton vertex and edges
        let new_vertex_ndx = self.vert_data.len();
        self.edges.push(Edge{start:vertex_ndx, end:new_vertex_ndx});
        self.edges.push(Edge{start:next_ndx, end:new_vertex_ndx});
        // Update active vertices
        self.active_vertices.remove(&vertex_ndx);
        self.active_vertices.remove(&next_ndx);
        // Update next and previous vertices refrences to the new vertex
        let next_ndx = self.vert_refs[next_ndx].next_ndx;
        self.vert_refs[next_ndx].next_ndx = new_vertex_ndx;
        self.vert_refs[prev_ndx].prev_ndx = new_vertex_ndx;
        // Calculate bisecotr for newly created vertex
        let p1 = self.vert_data[next_ndx].cooridnates; 
        let p2 = self.vert_data[prev_ndx].cooridnates;
        let bisector = bisector(new_position, p1, p2)?;
        let new_vertex = VertexData{cooridnates:new_position,bisector};
        self.vert_data.push(new_vertex);
        let new_vertex = Vertex{ndx: new_vertex_ndx,next_ndx,prev_ndx };
        self.vert_refs.push(new_vertex);

        //find events for the new vertex
        self.find_events(new_vertex,event.time);

        println!("\x1b[033mInfo:\x1b[0m Edge Event for vertex: {vertex_ndx} at: {new_position}");
        Ok(())
    }

    fn handle_split_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        // This involves splitting an edge and creating new events
        let vertex = event.vertex;
        let time = event.time.0;

        // Check if the vertex is still active
        if !self.active_vertices.contains(&vertex.ndx) {
            return Ok(());
        }

        // Calculate the position where the split occurs
        let vertex_data = &self.vert_data[vertex.ndx];
        let split_position = vertex_data.cooridnates + vertex_data.bisector * time;

        // ======== Find the edge that is being split ========
        let mut split_edge_idx = None;

       for vert in self.vert_refs.iter()
           .filter(|vert| !self.active_vertices.contains(&vert.ndx) ){
                let v1 = &self.vert_data[vert.ndx].cooridnates;
                let v2 = &self.vert_data[vert.next_ndx].cooridnates;
                // Check if the split position lies on this edge
                if let Some(t) = self.point_on_edge(&split_position, &v1, &v2)? {
                    if t > 0.0 && t < 1.0 {
                        split_edge_idx = Some(vert.clone());
                        break;
                    }
                }
            }

        match split_edge_idx {
            None => (),
            Some(vert) => {
                self.active_vertices.remove(&vertex.ndx);

                let splitting_vert = &self.vert_data[vertex.ndx];
                let new_point_coordinates = splitting_vert.cooridnates + splitting_vert.bisector * time;
                println!("\x1b[033mInfo:\x1b[0m Split Event for vertex: {} at: {}",vertex.ndx,new_point_coordinates);
                println!("time: {time}");
                println!("bisector: {:?}",splitting_vert.bisector);

                // ============= First edge loop ================
                let edge_start = self.vert_refs[vert.ndx];
                let edge_start_data = &self.vert_data[edge_start.ndx];
                let edge_start_possition = edge_start_data.cooridnates + edge_start_data.bisector * time;

                // splitting vertex's neighbour forming a close loop with edge_start vertex:
                let s_vert_start = &self.vert_data[vertex.next_ndx];
                let s_vert_start_possition = s_vert_start.cooridnates + s_vert_start.bisector * time;

                //assert!(edge_start != edge_end_neighbour); // make sure the remaining region is not a triangle (3 verts)

                dbg!(s_vert_start.cooridnates);
                dbg!(edge_start_data.cooridnates);
                dbg!(new_point_coordinates);
                // add new vertex to vertex list
                self.vert_data.push( VertexData{
                    cooridnates: new_point_coordinates,
                    bisector: bisector(new_point_coordinates, s_vert_start_possition, edge_start_possition)?
                    });

                // add new vertex to vert_ref list
                let new_vertex_index = self.vert_refs.len()-1;
                self.vert_refs.push( Vertex{
                    ndx: new_vertex_index,
                    next_ndx: edge_start.ndx,
                    prev_ndx: vertex.next_ndx,
                    });
                // update the neigbouring vertices refrences
                self.vert_refs[edge_start.ndx].next_ndx = new_vertex_index;
                self.vert_refs[vertex.next_ndx].prev_ndx = new_vertex_index;

                let edge_end = self.vert_refs[vert.next_ndx];
                let edge_end_data = &self.vert_data[vert.next_ndx];
                let edge_end_possition = edge_end_data.cooridnates + edge_end_data.bisector * time;

                // ============= Secound edge loop ================
                // splitting vertex's neighbour forming a close loop with edge_start vertex:
                let s_vert_end = &self.vert_data[vertex.prev_ndx];
                let s_vert_end_possition = s_vert_end.cooridnates + s_vert_end.bisector * time;

                self.vert_data.push( VertexData{
                    cooridnates: new_point_coordinates,
                    bisector: bisector(new_point_coordinates, edge_end_possition, s_vert_end_possition )?
                    });
                // add new vertex to vert_ref list
                let new_vertex_index = self.vert_refs.len()-1;
                self.vert_refs.push( Vertex{
                    ndx: new_vertex_index,
                    next_ndx: edge_end.ndx,
                    prev_ndx: vertex.prev_ndx,
                    });
                // update the neigbouring vertices refrences
                self.vert_refs[edge_end.ndx].prev_ndx = new_vertex_index;
                self.vert_refs[vertex.prev_ndx].next_ndx = new_vertex_index;

                // Find new events for the new verices
                // ToDo: it might also be nessesary to find events for the previous vertices
                let first_new_vertex = self.vert_refs.len()-2;
                self.edges.push(Edge{start:vertex.ndx,end:first_new_vertex});
                let secound_new_vertex = self.vert_refs.len()-1;

                self.find_events(self.vert_refs[first_new_vertex], event.time)?;
                self.find_events(self.vert_refs[secound_new_vertex], event.time)?;
                // Debug printing
                dbg!(self.vert_refs[first_new_vertex]);
                dbg!(self.vert_refs[secound_new_vertex]);
                },
            }


        Ok(())
    }
    //
    //fn handle_vertex_event(&mut self, event: Event) -> Result<(), SkeletonError> {
    //    println!("vertex_event");
    //    // Implementation of vertex event handling
    //    // This would handle cases where multiple vertices collapse to a single point
    //    let vertex_idx = event.vertex_idx;
    //    let time = event.time.0;
    //
    //    // Check if the vertex is still active
    //    if !self.active_vertices.contains(&vertex_idx) {
    //        return Ok(());
    //    }
    //
    //    // Find all vertices that collapse to this point
    //    let vertex = &self.vertices[vertex_idx];
    //    let collapse_position = vertex.position + vertex.bisector * time;
    //
    //    let mut collapsing_vertices = HashSet::new();
    //    collapsing_vertices.insert(vertex_idx);
    //
    //    // Find other vertices that collapse to the same point
    //    for (idx, other_vertex) in self.vertices.iter().enumerate() {
    //        if idx != vertex_idx && self.active_vertices.contains(&idx) {
    //            let other_position = other_vertex.position + other_vertex.bisector * time;
    //            if (other_position - collapse_position).norm() < 1e-10 {
    //                collapsing_vertices.insert(idx);
    //            }
    //        }
    //    }
    //
    //    // Create new vertex at collapse point
    //    let new_vertex_idx = self.result.vertices.len();
    //    self.result.vertices.push(collapse_position);
    //
    //    // Add skeleton edges from all collapsing vertices
    //    for &old_idx in &collapsing_vertices {
    //        self.result.edges.push([old_idx, new_vertex_idx]);
    //        self.active_vertices.remove(&old_idx);
    //    }
    //
    //    // Update edge list
    //    self.edges.retain(|edge| {
    //        !collapsing_vertices.contains(&edge.start) && 
    //        !collapsing_vertices.contains(&edge.end)
    //    });
    //
    //    // Create new edges connecting to non-collapsing neighbors
    //    let mut new_edges = Vec::new();
    //    let mut processed_pairs = HashSet::new();
    //
    //    for &collapsing_idx in &collapsing_vertices {
    //        for edge in &self.edges {
    //            if edge.start == collapsing_idx || edge.end == collapsing_idx {
    //                let other_idx = if edge.start == collapsing_idx {
    //                    edge.end
    //                } else {
    //                    edge.start
    //                };
    //
    //                if !collapsing_vertices.contains(&other_idx) {
    //                    let pair = if other_idx < new_vertex_idx {
    //                        (other_idx, new_vertex_idx)
    //                    } else {
    //                        (new_vertex_idx, other_idx)
    //                    };
    //
    //                    if !processed_pairs.contains(&pair) {
    //                        new_edges.push(Edge {
    //                            start: pair.0,
    //                            end: pair.1,
    //                            weight: edge.weight,
    //                        });
    //                        processed_pairs.insert(pair);
    //                    }
    //                }
    //            }
    //        }
    //    }
    //
    //    // Add the new edges
    //    self.edges.extend(new_edges);
    //
    //    // Add new vertex to active set
    //    self.active_vertices.insert(new_vertex_idx);
    //
    //    // Compute new events for the collapsed vertex
    //    if let Ok(new_events) = self.compute_new_events_after_collapse(new_vertex_idx) {
    //        for event in new_events {
    //            self.events.push(event.clone(), -event.time);
    //        }
    //    }
    //    Ok(())
    //}
    //   // Helper methods for the new implementations
    //
    fn point_on_edge(
        &self,
        point: &Point2<f32>,
        edge_start: &Point2<f32>,
        edge_end: &Point2<f32>,
    ) -> Result<Option<f32>, SkeletonError> {
        let edge_vec = edge_end - edge_start;
        let point_vec = point - edge_start;

        let edge_length_sq = edge_vec.norm_squared();
        if edge_length_sq < 1e-5 {
            return Err(SkeletonError::ComputationError(
                "Edge length too small".to_string(),
            ));
        }

        let t = edge_vec.dot(&point_vec) / edge_length_sq;
        Ok(Some(t))
    }
    //
    //fn compute_new_events_after_split(
    //    &self,
    //    vertex_idx: usize,
    //) -> Result<Vec<Event>, SkeletonError> {
    //    let mut new_events = Vec::new();
    //
    //    // Compute new edge events
    //    for edge in &self.edges {
    //        if edge.start == vertex_idx || edge.end == vertex_idx {
    //            if let Ok(Some(event)) = self.compute_edge_event(edge.start, edge.end) {
    //                new_events.push(event);
    //            }
    //        }
    //    }
    //
    //    // Compute new split events
    //    if let Ok(mut split_events) = self.compute_split_events(vertex_idx) {
    //        new_events.extend(split_events);
    //    }
    //
    //    Ok(new_events)
    //}
    //
    //fn compute_new_events_after_collapse(
    //    &self,
    //    vertex_idx: usize,
    //) -> Result<Vec<Event>, SkeletonError> {
    //    let mut new_events = Vec::new();
    //
    //    // Similar to compute_new_events_after_split, but also check for potential vertex events
    //    // Compute edge events
    //    for edge in &self.edges {
    //        if edge.start == vertex_idx || edge.end == vertex_idx {
    //            if let Ok(Some(event)) = self.compute_edge_event(edge.start, edge.end) {
    //                new_events.push(event);
    //            }
    //        }
    //    }
    //
    //    // Compute split events
    //    if let Ok(mut split_events) = self.compute_split_events(vertex_idx) {
    //        new_events.extend(split_events);
    //    }
    //
    //    // Check for potential vertex events
    //    let vertex_pos = &self.vertices[vertex_idx].position;
    //    let vertex_bisector = &self.vertices[vertex_idx].bisector;
    //
    //    for (other_idx, other_vertex) in self.vertices.iter().enumerate() {
    //        if other_idx != vertex_idx && self.active_vertices.contains(&other_idx) {
    //            let time = self.compute_vertex_event_time(vertex_pos, vertex_bisector, 
    //                &other_vertex.position, &other_vertex.bisector)?;
    //
    //            if let Some(t) = time {
    //                new_events.push(Event {
    //                    time: OrderedFloat(t),
    //                    vertex_idx,
    //                    event_type: EventType::Vertex,
    //                });
    //            }
    //        }
    //    }
    //
    //    Ok(new_events)
    //}

    fn compute_vertex_event_time(
        &self,
        pos1: &Point2<f32>,
        bisector1: &Vector2<f32>,
        pos2: &Point2<f32>,
        bisector2: &Vector2<f32>,
    ) -> Result<Option<f32>, SkeletonError> {
        // Compute intersection time of two vertex trajectories
        let relative_pos = pos2 - pos1;
        let relative_vel = bisector2 - bisector1;

        if relative_vel.norm_squared() < 1e-10 {
            return Ok(None);
        }

        let t = -relative_pos.dot(&relative_vel) / relative_vel.norm_squared();
        if t > 0.0 {
            Ok(Some(t))
        } else {
            Ok(None)
        }
    }

}
fn compute_edge_event(vert:&Vertex , v1:VertexData, v2:VertexData) -> Result<Option<Event>, SkeletonError> {
    //let v1_position = &self.vert_data[v1_idx].cooridnates;
    //let v1_bisector = &self.vert_data[v1_idx].bisector.expect("bisector not calculated yet");
    //let v2_position = &self.vert_data[v2_idx].cooridnates;
    //let v2_bisector = &self.vert_data[v2_idx].bisector.expect("bisector not calculated yet");

    // Calculate intersection of bisectors
    let t = compute_intersection_time(&v1.cooridnates, &v1.bisector, &v2.cooridnates, &v2.bisector)?;

    if t > 0.0 {
        Ok(Some(Event {
            time: OrderedFloat(t),
            vertex: *vert,
            event_type: EventType::Edge,
        }))
    } else {
        Ok(None)
    }
}
fn compute_intersection_time(
    p1: &Point2<f32>,
    v1: &Vector2<f32>,
    p2: &Point2<f32>,
    v2: &Vector2<f32>,
) -> Result<f32, SkeletonError> {
    // Solve the system of equations:
    // p1 + t*v1 = p2 + t*v2
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let det = v1.x * (-v2.y) - (-v2.x) * v1.y;

    if det.abs() < 1e-5 {
        return Err(SkeletonError::ComputationError(
            "Parallel bisectors detected".to_string(),
        ));
    }
    let t = (dx * (-v2.y) - (-v2.x) * dy) / det;
    Ok(t)
}
pub fn bisector(
    current_point: Point2<f32>,
    next_point: Point2<f32>,
    prev_point: Point2<f32>,
) -> Result<Vector2<f32>, SkeletonError> {
    let v1 = Vector2::new(
        prev_point.x - current_point.x,
        prev_point.y - current_point.y,
    ).normalize();
    let v2 = Vector2::new(
        next_point.x - current_point.x,
        next_point.y - current_point.y,
    ).normalize();

    //let angle = v1.angle(&v2);
    //if angle.abs() < 1e-5 || (std::f32::consts::PI - angle).abs() < 1e-5 {
    //    return Err(SkeletonError::ComputationError(
    //        "Cannot compute bisector for parallel or antiparallel edges".to_string(),
    //    ));
    //}
    let mut bisector = v1 + v2;
    if (next_point.x - prev_point.x) < 0.0 || (next_point.y - prev_point.y) < 0.0 {
        bisector = - bisector
    }
    Ok(bisector)
}

// Example usage
pub fn create_weighted_skeleton(
    points: Vec<Point2<f32>>,
    weights: &[f32],
) -> Result<StraightSkeleton, SkeletonError> {
    let builder = SkeletonBuilder::new(points, weights)?;
    return builder.compute_skeleton();
}

// Tests module
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_polygon() -> Result<(), SkeletonError> {
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];
        let weights = vec![1.0, 1.0, 1.0, 1.0];

        let skeleton = create_weighted_skeleton(points, &weights)?;
        assert!(skeleton.vertices.len() > 0);
        assert!(skeleton.edges.len() > 0);
        Ok(())
    }

    #[test]
    fn test_invalid_polygon() {
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ];
        let weights = vec![1.0, 1.0];
        let result = create_weighted_skeleton(points, &weights);
        assert!(result.is_err());
    }
}
