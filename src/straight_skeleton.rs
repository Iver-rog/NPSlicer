#![allow(unused)]
use nalgebra::{Point2, Vector2};
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::collections::HashSet;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("Computation error: {0}")]
    ComputationError(String),
}
#[derive(Debug, Clone)]
pub struct Vertex {
    pub position: Point2<f32>,
    pub bisector: Vector2<f32>,
}
#[derive(Debug, Clone)]
pub struct Edge {
    pub start: usize,
    pub end: usize,
    pub weight: f32,
}
#[derive(Debug)]
pub struct StraightSkeleton {
    pub vertices: Vec<Point2<f32>>,
    pub edges: Vec<[usize;2]>,
}
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct Event {
    time: OrderedFloat<f32>,
    vertex_idx: usize,
    next_vert_indx: usize,
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
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    events: PriorityQueue<Event, OrderedFloat<f32>>,
    active_vertices: HashSet<usize>,
    result: StraightSkeleton,
}

impl SkeletonBuilder {
    pub fn new(points: &[Point2<f32>], weights: &[f32]) -> Result<Self, SkeletonError> {
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
        let mut builder = SkeletonBuilder {
            vertices: Vec::new(),
            edges: Vec::new(),
            events: PriorityQueue::new(),
            active_vertices: HashSet::new(),
            result: StraightSkeleton {
                vertices: points.iter().map(|p| p.clone()).collect(),
                edges: Vec::new(),
            },
        };
        builder.initialize_polygon(points, weights)?;
        Ok(builder)
    }

    fn initialize_polygon(&mut self, points: &[Point2<f32>], weights: &[f32]) -> Result<(), SkeletonError> {
        // Initialize vertices and edges
        for i in 0..points.len() {
            let prev = (i + points.len() - 1) % points.len();
            let next = (i + 1) % points.len();

            let v1 = Vector2::new(
                points[prev].x - points[i].x,
                points[prev].y - points[i].y,
            ).normalize();
            let v2 = Vector2::new(
                points[next].x - points[i].x,
                points[next].y - points[i].y,
            ).normalize();

            // Calculate bisector
            let bisector = self.calculate_weighted_bisector(v1, v2, weights[i])?;

            self.vertices.push(Vertex {
                position: points[i],
                bisector,
            });
            self.edges.push(Edge {
                start: i,
                end: (i + 1) % points.len(),
                weight: weights[i],
            });
            self.active_vertices.insert(i);
        }
        // Initialize events
        self.initialize_events()?;
        Ok(())
    }

    fn calculate_weighted_bisector(
        &self,
        v1: Vector2<f32>,
        v2: Vector2<f32>,
        weight: f32,
    ) -> Result<Vector2<f32>, SkeletonError> {
        let angle = v1.angle(&v2);
        if angle.abs() < 1e-5 || (std::f32::consts::PI - angle).abs() < 1e-5 {
            return Err(SkeletonError::ComputationError(
                "Cannot compute bisector for parallel or antiparallel edges".to_string(),
            ));
        }
        let bisector = (v1 + v2).normalize();
        Ok(bisector * weight)
    }

    fn initialize_events(&mut self) -> Result<(), SkeletonError> {
        dbg!(&self);
        for i in 0..self.vertices.len() {
            let vertex = &self.vertices[i];
            
            // Edge events
            let next_idx = (i + 1) % self.vertices.len();
            let edge_event = self.compute_edge_event(i, next_idx)?;
            dbg!(&edge_event);
            if let Some(event) = edge_event {
                self.events.push(event.clone(), -event.time);
            }
            // Split events
            let split_events = self.compute_split_events(i)?;
            dbg!(&split_events);
            for event in split_events {
                self.events.push(event.clone(), -event.time);
            }
        }
        dbg!(&self.events);
        Ok(())
    }

    fn compute_edge_event(&self, v1_idx: usize, v2_idx: usize) -> Result<Option<Event>, SkeletonError> {
        let v1 = &self.vertices[v1_idx];
        let v2 = &self.vertices[v2_idx];

        // Calculate intersection of bisectors
        let t = self.compute_intersection_time(&v1.position, &v1.bisector, &v2.position, &v2.bisector)?;

        if t > 0.0 {
            Ok(Some(Event {
                time: OrderedFloat(t),
                vertex_idx: v1_idx,
                event_type: EventType::Edge,
            }))
        } else {
            Ok(None)
        }
    }

    fn compute_split_events(&self, vertex_idx: usize) -> Result<Vec<Event>, SkeletonError> {
        let mut events = Vec::new();
        let vertex = &self.vertices[vertex_idx];

        for (edge_idx, edge) in self.edges.iter().enumerate() {
            if edge.start != vertex_idx && edge.end != vertex_idx {
                let v1 = &self.vertices[edge.start];
                let v2 = &self.vertices[edge.end];

                if let Some(t) = self.compute_split_time(vertex, v1, v2)? {
                    events.push(Event {
                        time: OrderedFloat(t),
                        vertex_idx,
                        event_type: EventType::Split,
                    });
                }
            }
        }
        Ok(events)
    }

    fn compute_intersection_time(
        &self,
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

    fn compute_split_time(
        &self,
        vertex: &Vertex,
        v1: &Vertex,
        v2: &Vertex,
    ) -> Result<Option<f32>, SkeletonError> {
        // Implementation of split event time computation
        // This is a simplified version - you might want to add more robust geometric computations
        let edge_vec = v2.position - v1.position;
        let vertex_vec = vertex.position - v1.position;
        
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
                EventType::Split => self.handle_split_event(event)?,
                EventType::Vertex => self.handle_vertex_event(event)?,
            }
            if self.active_vertices.len() < 3 {
                break;
            }
        }
        Ok(self.result)
    }
    fn handle_edge_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        let vertex_idx = event.vertex_idx;
        let next_idx = (vertex_idx + 1) % self.vertices.len();

        if !self.active_vertices.contains(&vertex_idx) || !self.active_vertices.contains(&next_idx) {
            return Ok(());
        }
        // Calculate new vertex position
        let vertex = &self.vertices[vertex_idx];
        let time = event.time.0;
        let new_position = vertex.position + vertex.bisector * time;
        // Add new skeleton vertex and edges
        let new_vertex_idx = self.result.vertices.len();
        self.result.vertices.push(new_position);
        self.result.edges.push([vertex_idx, new_vertex_idx]);
        self.result.edges.push([next_idx, new_vertex_idx]);
        // Update active vertices
        self.active_vertices.remove(&vertex_idx);
        self.active_vertices.remove(&next_idx);

        Ok(())
    }

    fn handle_split_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        // Implementation of split event handling
        // This would involve splitting an edge and creating new events
        let vertex_idx = event.vertex_idx;
        let time = event.time.0;

        // Check if the vertex is still active
        if !self.active_vertices.contains(&vertex_idx) {
            return Ok(());
        }

        // Calculate the position where the split occurs
        let vertex = &self.vertices[vertex_idx];
        let split_position = vertex.position + vertex.bisector * time;

        // Find the edge that is being split
        let mut split_edge_idx = None;
        let mut split_point = None;
        
        for (idx, edge) in self.edges.iter().enumerate() {
            if edge.start != vertex_idx && edge.end != vertex_idx {
                let v1 = &self.vertices[edge.start];
                let v2 = &self.vertices[edge.end];
                
                // Check if the split position lies on this edge
                if let Some(t) = self.point_on_edge(&split_position, &v1.position, &v2.position)? {
                    if t > 0.0 && t < 1.0 {
                        split_edge_idx = Some(idx);
                        split_point = Some(t);
                        break;
                    }
                }
            }
        }
        let vertex_idx = event.vertex_idx;
        let time = event.time.0;

        // Check if the vertex is still active
        if !self.active_vertices.contains(&vertex_idx) {
            return Ok(());
        }

        // Calculate the position where the split occurs
        let vertex = &self.vertices[vertex_idx];
        let split_position = vertex.position + vertex.bisector * time;

        // Find the edge that is being split
        let mut split_edge_idx = None;
        let mut split_point = None;
        
        for (idx, edge) in self.edges.iter().enumerate() {
            if edge.start != vertex_idx && edge.end != vertex_idx {
                let v1 = &self.vertices[edge.start];
                let v2 = &self.vertices[edge.end];
                
                // Check if the split position lies on this edge
                if let Some(t) = self.point_on_edge(&split_position, &v1.position, &v2.position)? {
                    if t > 0.0 && t < 1.0 {
                        split_edge_idx = Some(idx);
                        split_point = Some(t);
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    fn handle_vertex_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        println!("vertex_event");
        // Implementation of vertex event handling
        // This would handle cases where multiple vertices collapse to a single point
        let vertex_idx = event.vertex_idx;
        let time = event.time.0;

        // Check if the vertex is still active
        if !self.active_vertices.contains(&vertex_idx) {
            return Ok(());
        }

        // Find all vertices that collapse to this point
        let vertex = &self.vertices[vertex_idx];
        let collapse_position = vertex.position + vertex.bisector * time;
        
        let mut collapsing_vertices = HashSet::new();
        collapsing_vertices.insert(vertex_idx);

        // Find other vertices that collapse to the same point
        for (idx, other_vertex) in self.vertices.iter().enumerate() {
            if idx != vertex_idx && self.active_vertices.contains(&idx) {
                let other_position = other_vertex.position + other_vertex.bisector * time;
                if (other_position - collapse_position).norm() < 1e-10 {
                    collapsing_vertices.insert(idx);
                }
            }
        }

        // Create new vertex at collapse point
        let new_vertex_idx = self.result.vertices.len();
        self.result.vertices.push(collapse_position);

        // Add skeleton edges from all collapsing vertices
        for &old_idx in &collapsing_vertices {
            self.result.edges.push([old_idx, new_vertex_idx]);
            self.active_vertices.remove(&old_idx);
        }

        // Update edge list
        self.edges.retain(|edge| {
            !collapsing_vertices.contains(&edge.start) && 
            !collapsing_vertices.contains(&edge.end)
        });

        // Create new edges connecting to non-collapsing neighbors
        let mut new_edges = Vec::new();
        let mut processed_pairs = HashSet::new();

        for &collapsing_idx in &collapsing_vertices {
            for edge in &self.edges {
                if edge.start == collapsing_idx || edge.end == collapsing_idx {
                    let other_idx = if edge.start == collapsing_idx {
                        edge.end
                    } else {
                        edge.start
                    };

                    if !collapsing_vertices.contains(&other_idx) {
                        let pair = if other_idx < new_vertex_idx {
                            (other_idx, new_vertex_idx)
                        } else {
                            (new_vertex_idx, other_idx)
                        };

                        if !processed_pairs.contains(&pair) {
                            new_edges.push(Edge {
                                start: pair.0,
                                end: pair.1,
                                weight: edge.weight,
                            });
                            processed_pairs.insert(pair);
                        }
                    }
                }
            }
        }

        // Add the new edges
        self.edges.extend(new_edges);

        // Add new vertex to active set
        self.active_vertices.insert(new_vertex_idx);

        // Compute new events for the collapsed vertex
        if let Ok(new_events) = self.compute_new_events_after_collapse(new_vertex_idx) {
            for event in new_events {
                self.events.push(event.clone(), -event.time);
            }
        }
        Ok(())
    }
       // Helper methods for the new implementations

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

    fn compute_new_events_after_split(
        &self,
        vertex_idx: usize,
    ) -> Result<Vec<Event>, SkeletonError> {
        let mut new_events = Vec::new();

        // Compute new edge events
        for edge in &self.edges {
            if edge.start == vertex_idx || edge.end == vertex_idx {
                if let Ok(Some(event)) = self.compute_edge_event(edge.start, edge.end) {
                    new_events.push(event);
                }
            }
        }

        // Compute new split events
        if let Ok(mut split_events) = self.compute_split_events(vertex_idx) {
            new_events.extend(split_events);
        }

        Ok(new_events)
    }

    fn compute_new_events_after_collapse(
        &self,
        vertex_idx: usize,
    ) -> Result<Vec<Event>, SkeletonError> {
        let mut new_events = Vec::new();

        // Similar to compute_new_events_after_split, but also check for potential vertex events
        // Compute edge events
        for edge in &self.edges {
            if edge.start == vertex_idx || edge.end == vertex_idx {
                if let Ok(Some(event)) = self.compute_edge_event(edge.start, edge.end) {
                    new_events.push(event);
                }
            }
        }

        // Compute split events
        if let Ok(mut split_events) = self.compute_split_events(vertex_idx) {
            new_events.extend(split_events);
        }

        // Check for potential vertex events
        let vertex_pos = &self.vertices[vertex_idx].position;
        let vertex_bisector = &self.vertices[vertex_idx].bisector;

        for (other_idx, other_vertex) in self.vertices.iter().enumerate() {
            if other_idx != vertex_idx && self.active_vertices.contains(&other_idx) {
                let time = self.compute_vertex_event_time(vertex_pos, vertex_bisector, 
                    &other_vertex.position, &other_vertex.bisector)?;
                
                if let Some(t) = time {
                    new_events.push(Event {
                        time: OrderedFloat(t),
                        vertex_idx,
                        event_type: EventType::Vertex,
                    });
                }
            }
        }

        Ok(new_events)
    }

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

// Example usage
pub fn create_weighted_skeleton(
    points: &[Point2<f32>],
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

        let skeleton = create_weighted_skeleton(&points, &weights)?;
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
        let result = create_weighted_skeleton(&points, &weights);
        assert!(result.is_err());
    }
}
