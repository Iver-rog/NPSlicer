use nalgebra::{Point2, Vector2, Matrix2};
use nalgebra_glm::cross2d;
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::{collections::HashSet, f32::EPSILON, fmt::Formatter, io::Split};
use thiserror::Error;
use std::hash::Hash;
use std::fmt::{self, Display};
use std::cmp::max;

#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("Computation error: {0}")]
    ComputationError(String),
    }
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Node {
    pub ndx: usize,
    pub next_ndx: usize,
    pub prev_ndx: usize,
    pub bisector: [OrderedFloat<f32>;2],
    pub vertex_ndx: usize,
}
impl Node {
    fn bisector(&self)-> Vector2<f32>{
        Vector2::new(*self.bisector[0],*self.bisector[1])
    }
}
#[derive(Debug,Default)]
pub struct Nodes {
    pub nodes: Vec<Node>,
    active_nodes: HashSet<usize>,
}
impl Nodes {
    pub fn new(nodes:Vec<Node>) -> Self {
        Nodes {
        active_nodes: HashSet::from_iter(0..nodes.len()),
        nodes,
        }
    }
    fn next(&mut self,strat_node:Node) -> Node{
        self.nodes[strat_node.next_ndx]
    }
    fn prev(&mut self,strat_node:Node) -> Node{
        self.nodes[strat_node.prev_ndx]
    }
    fn len(& self) -> usize {
        return self.nodes.len()-self.active_nodes.len();
    }
}
pub struct NodesIntoIterator<'a>{
    starting:usize,
    nodes:&'a Nodes,
    next_node:usize,
    stop: bool,
}
pub struct NodesIntoBackwardsIterator<'a>{
    starting:usize,
    nodes:&'a Nodes,
    prev_node:usize,
    stop: bool,
}
impl <'a> Iterator for NodesIntoIterator<'a>{
    type Item = Node;
    fn next(&mut self) -> Option<Self::Item> {
        if !self.stop {
            let node = self.nodes.nodes[self.next_node];
            if node.ndx == self.starting{
                self.stop = true
            }
            self.next_node = node.next_ndx;
            return Some(node)
        }
        None
    }
}
impl <'a> Iterator for NodesIntoBackwardsIterator<'a>{
    type Item = Node;
    fn next(&mut self) -> Option<Self::Item> {
        if !self.stop {
            let node = self.nodes.nodes[self.prev_node];
            if node.ndx == self.starting{
                self.stop = true
            }
            self.prev_node = node.prev_ndx;
            return Some(node)
        }
        None
    }
}
impl Nodes {
    pub fn iter(&self,starting_node:&Node) -> NodesIntoIterator {
        NodesIntoIterator{
            starting:starting_node.ndx,
            nodes:&self,
            next_node:starting_node.next_ndx,
            stop: false,
        }
    }
    pub fn back_iter(&self,starting_node:&Node) -> NodesIntoBackwardsIterator {
        NodesIntoBackwardsIterator{
            starting:starting_node.ndx,
            nodes:&self,
            prev_node:starting_node.prev_ndx,
            stop: false,
        }
    }
}

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
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
    node: Node,
    event_type: EventType,
}
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum EventType {
    Edge,  // A edge shrinks to zero lenght
    Split(SplitEvent), // A region is split into two parts
    Vertex,// A region disapears/colapses into a vertex
}
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct SplitEvent{
    b: [OrderedFloat<f32>;2],
    edge: Edge, 
}

#[derive(Debug)]
pub struct SkeletonBuilder {
    nodes: Vec<Node>,
    vertices: Vec<Point2<f32>>,
    active_nodes: HashSet<usize>,
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
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        let mut active_nodes = HashSet::new();

        //builder.initialize_polygon(weights)?;
        for i in 0..points.len() {
            let next_ndx = (i + 1) % points.len();
            let prev_ndx = if i == 0 {points.len()-1} else { i-1 };

            let p_current = points[i];
            let p_next = points[next_ndx];
            let p_prev = points[prev_ndx];

            let bisector = bisector(p_current, p_next, p_prev).unwrap();

            nodes.push(Node {
                ndx: i,
                next_ndx,
                prev_ndx,
                bisector:[OrderedFloat(bisector[0]), OrderedFloat(bisector[1])],
                vertex_ndx: i
            });
            edges.push(Edge {
                start: i,
                end: next_ndx,
            });
            active_nodes.insert(i);


        }

        let mut builder = SkeletonBuilder {
            nodes,
            edges,
            vertices:points,
            events: PriorityQueue::new(),
            active_nodes,
        };


        //initialize events 
        let mut event_que: PriorityQueue<Event, OrderedFloat<f32>> = PriorityQueue::new();
        for node in builder.nodes.iter() { 
            let next_idx = node.next_ndx;

            // Edge events
            let edge_event = compute_edge_event(
                &node, 
                builder.vertices[node.vertex_ndx],
                node.bisector(),
                builder.vertices[builder.nodes[next_idx].vertex_ndx],
                builder.nodes[next_idx].bisector()
                )?;
            if let Some(event) = edge_event {
                event_que.push(event.clone(), -event.time);
            }
            // Split events
            let split_events = builder.compute_split_events(node)?;
            for event in split_events {
                event_que.push(event.clone(), -event.time);
            }
        }
        builder.events = event_que;

        Ok(builder)
    }

    fn find_events(&mut self, vertex:Node, current_time: OrderedFloat<f32>) -> Result<(), SkeletonError> {

        // Edge events
        let edge_event = compute_edge_event(
            &vertex, 
            self.vertices[vertex.vertex_ndx], 
            vertex.bisector(),
            self.vertices[self.nodes[vertex.next_ndx].vertex_ndx],
            self.nodes[vertex.next_ndx].bisector()
            )?;
        if let Some(event) = edge_event {
            let time = current_time+event.time;
            self.events.push(event, -time);
        }
        // Split events

        let split_events = self.compute_split_events(&vertex)?;
        for event in split_events {
            let time = current_time+event.time;
            self.events.push(event, -time);
        }
        Ok(())
    }

    fn compute_split_events<'a>(&'a self, node: &'a Node) -> Result<Vec<Event>, SkeletonError> {
        let mut events = Vec::new();
        // Chech if edge is a reflex angle

        if !is_reflex(self.vertices[node.vertex_ndx],
            self.vertices[self.nodes[node.next_ndx].vertex_ndx],
            self.vertices[self.nodes[node.prev_ndx].vertex_ndx])
        {
            return Ok(events)
        }
        println!("\x1b[033mFinding split events for node: {}  {}\x1b[0m",node.ndx, self.vertices[node.vertex_ndx]);
        let node_p = self.vertices[node.vertex_ndx];

        // Looking for splitt candidates
        for edge_start in self.nodes.iter()
            .filter(|e| e.ndx != node.ndx && e.next_ndx != node.ndx)
            .filter(|n| self.active_nodes.contains(&n.ndx)) 
            {
                let edge_end = self.nodes[edge_start.next_ndx];
                print!("  considering edge from node {}-{} ",edge_start.ndx,edge_end.ndx);

                // coordinates (Points)
                let edge_start_p = self.vertices[edge_start.vertex_ndx];
                let edge_end_p = self.vertices[edge_end.vertex_ndx];
                println!("  with cooridnates {} - {}",edge_start_p,edge_end_p);

                // vector pointing in the direction of the tested edge
                let edge_vec = edge_start_p + edge_start.bisector() 
                           - ( edge_end_p + edge_end.bisector() );

                // vector pointing form the splitting vertex to its next vertex
                let edge_left = node_p
                    - self.vertices[self.nodes[node.next_ndx].vertex_ndx];
                // vector pointing from the splitting vertex to its previous vertex
                let edge_right = node_p 
                    - self.vertices[self.nodes[node.prev_ndx].vertex_ndx];

                // a potential b is at the intersection of between our own bisector and the 
		            // bisector of the angle between the tested edge and any one of our own edges.

				        // we choose the "less parallel" edge (in order to exclude a potentially parallel edge)
                let leftdot = (edge_left.normalize().dot(&edge_vec.normalize())).abs();
                let rightdot = (edge_right.normalize().dot(&edge_vec.normalize())).abs();
                let self_edge =  if leftdot < rightdot { edge_left }else{ edge_right };
                let other_edge = if leftdot > rightdot { edge_left }else{ edge_right };

                let i = intersect( edge_start_p, edge_vec, node_p, self_edge );

                if (i-node_p).magnitude() < 1e-5{
                    println!("  skiping node: {}. value = {}", node.ndx,(i-node_p).magnitude());
                    continue;
                }
                // Locate candidate b
                let line_vec = (node_p - i).normalize();
                let mut ed_vec = (edge_vec).normalize();
                if line_vec.dot(&ed_vec) < 0.0 {
                    ed_vec = - ed_vec
                }

                let bisector = ed_vec + line_vec;
                if bisector.magnitude() == 0.0 { continue; };

                let b = intersect(i, bisector, node_p, node.bisector() );

                // Check eligebility of b
                // a valid b should lie within the area limited by the edge and the bisectors of its two vertices:
                let x_start = cross2d(&edge_end.bisector().normalize(),
                    &(b-edge_end_p).normalize()) > -EPSILON;
                let x_end = cross2d(&edge_start.bisector().normalize(),
                    &(b-edge_start_p).normalize()) < EPSILON;
                let x_edge = cross2d(&edge_vec.normalize(),
                    &(b-edge_start_p).normalize()) < EPSILON;

                if !(x_start && x_end && x_edge) {
                    println!("  - discarding candidate b: {b}");
                    continue;
                };
                println!("\x1b[032m  - found candidate b: {b}\x1b[0m");


                let time = (node_p - b).magnitude() / node.bisector().magnitude();
                let split = SplitEvent{
                    b:[OrderedFloat(b[0]),OrderedFloat(b[1])],
                    edge:Edge{
                        start:edge_start.ndx,
                        end:edge_end.ndx
                    },
                };

                events.push(Event {
                    time: OrderedFloat(time),
                    node: *node,
                    event_type: EventType::Split(split),
                });
            }
        Ok(events)
    }


    fn compute_split_time(
        &self,
        vertex: &Node,
        v1: &Point2<f32>,
        v2: &Point2<f32>,
    ) -> Result<Option<f32>, SkeletonError> {
        // Implementation of split event time computation
        // This is a simplified version - you might want to add more robust geometric computations
        let edge_vec = v2 - v1;
        let vertex_vec = self.vertices[vertex.vertex_ndx] - v1;
        
        let t = edge_vec.dot(&vertex_vec) / edge_vec.norm_squared();
        
        if t > 0.0 && t < 1.0 {
            Ok(Some(t))
        } else {
            Ok(None)
        }
    }
    pub fn compute_skeleton(mut self) -> Result<StraightSkeleton, SkeletonError> {
        while let Some((event, _)) = self.events.pop() {
            println!("{self}");
            match event.event_type {
                EventType::Edge => self.handle_edge_event(event)?,
                EventType::Split(_) => {self.handle_split_event(event)?; },
                EventType::Vertex => todo!(),//self.handle_vertex_event(event)?,
            }
            if self.active_nodes.len() < 3 {
                break;
            }
        }
        let vertices = self.vertices;
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
        let node_ndx = event.node.ndx;
        let next_ndx = event.node.next_ndx;
        let prev_ndx = event.node.prev_ndx;

        if !self.active_nodes.contains(&node_ndx) || !self.active_nodes.contains(&next_ndx) {
            return Ok(());
        }

        // Calculate new vertex position
        let vertex = &self.vertices[event.node.vertex_ndx];
        let time = event.time.0;
        let new_position = vertex + event.node.bisector() * event.time.0;
        // Add new vertex to list
        let new_vertex_ndx = self.vertices.len();
        self.vertices.push(new_position);
        // Add new skeleton vertex and edges
        let new_node_ndx = self.nodes.len();
        self.edges.push(Edge{start:node_ndx, end:new_vertex_ndx});
        self.edges.push(Edge{start:next_ndx, end:new_vertex_ndx});
        // Update active vertices
        self.active_nodes.remove(&node_ndx);
        self.active_nodes.remove(&next_ndx);
        self.active_nodes.insert(new_node_ndx);
        // Update next and previous vertices refrences to the new vertex
        let next_ndx = self.nodes[next_ndx].next_ndx;
        self.nodes[next_ndx].next_ndx = new_node_ndx;
        self.nodes[prev_ndx].prev_ndx = new_node_ndx;
        // Calculate bisecotr for newly created vertex
        let p1 = self.vertices[self.nodes[next_ndx].vertex_ndx]; 
        let p2 = self.vertices[self.nodes[prev_ndx].vertex_ndx];
        let bisector = bisector(new_position, p1, p2)?;
        let new_vertex = Node{
            ndx: new_node_ndx,
            next_ndx,
            prev_ndx, 
            bisector: [OrderedFloat(bisector[0]),OrderedFloat(bisector[1])],
            vertex_ndx: new_vertex_ndx,
        };
        self.nodes.push(new_vertex);

        //find events for the new vertex
        self.find_events(new_vertex,event.time);

        println!("\x1b[033mEdge Event for node:{node_ndx} at: {new_position}\x1b[0m");
        Ok(())
    }
    fn handle_split_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        let node = event.node;
        let split = match event.event_type { 
            EventType::Split(split) => (split),
            _ => panic!("wrong event type sendt to handle split event funcion")
        };

        if !(self.active_nodes.contains(&node.ndx) 
            && self.active_nodes.contains(&split.edge.start) 
            && self.active_nodes.contains(&split.edge.end))
        {
            return Ok(());
        }
        
        let time = event.time.0;
        let b = Point2::new(split.b[0].0, split.b[1].0);

        self.vertices.push(b);
        self.active_nodes.remove(&node.ndx);

        let splitting_vert = &self.vertices[node.vertex_ndx];
        println!("\x1b[033mSplit Event for node: {} at: {}\x1b[0m",node.ndx,b);

        // ============= First edge loop ================
        let edge_start = self.nodes[split.edge.start];
        let edge_start_p = &self.vertices[edge_start.vertex_ndx];
        let edge_start_possition = edge_start_p + edge_start.bisector() * time;

        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_start = &self.nodes[node.next_ndx];
        let s_vert_start_p = &self.vertices[node.vertex_ndx];
        let s_vert_start_possition = s_vert_start_p + s_vert_start.bisector() * time;

        //assert!(edge_start != edge_end_neighbour); // make sure the remaining region is not a triangle (3 verts)

        dbg!(s_vert_start);
        dbg!(edge_start_p);
        // add new vertex to vertex lisShort-circuiting logical ANDt

        // add new vertex to vert_ref list
        let new_node_index = self.nodes.len();
        let bisect = bisector(b, s_vert_start_possition, edge_start_possition)?;
        self.nodes.push( Node{
            ndx: new_node_index,
            next_ndx: node.next_ndx,
            prev_ndx: edge_start.ndx,
            bisector: [OrderedFloat(bisect[0]),OrderedFloat(bisect[1])],
            vertex_ndx: self.vertices.len()-1
            });
        // update the neigbouring vertices refrences
        self.nodes[edge_start.ndx].next_ndx = new_node_index;
        self.nodes[node.next_ndx].prev_ndx = new_node_index;
        self.active_nodes.insert(new_node_index);

        let edge_end = self.nodes[split.edge.end];
        let edge_end_p = &self.vertices[edge_end.vertex_ndx];
        let edge_end_possition = edge_end_p + edge_end.bisector() * time;

        // ============= Secound edge loop ================
        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_end = &self.nodes[node.prev_ndx];
        let s_vert_end_p = &self.vertices[s_vert_end.vertex_ndx];
        let s_vert_end_possition = s_vert_end_p + s_vert_end.bisector() * time;

        // add new vertex to vert_ref list
        let new_node_index = self.nodes.len();
        let bisector = bisector(b, edge_end_possition, s_vert_end_possition )?;

        self.nodes.push( Node{
            ndx: new_node_index,
            next_ndx: edge_end.ndx,
            prev_ndx: node.prev_ndx,
            bisector: [OrderedFloat(bisector[0]) , OrderedFloat(bisector[1])],
            vertex_ndx: self.vertices.len()-1
            });
        // update the neigbouring vertices refrences
        self.nodes[edge_end.ndx].prev_ndx = new_node_index;
        self.nodes[node.prev_ndx].next_ndx = new_node_index;
        self.active_nodes.insert(new_node_index);

        // Find new events for the new verices
        // ToDo: it might also be nessesary to find events for the previous vertices
        let first_new_vertex = self.nodes.len()-2;
        self.edges.push(Edge{start:node.ndx,end:first_new_vertex});
        let secound_new_vertex = self.nodes.len()-1;

        dbg!(self.nodes[first_new_vertex]);
        dbg!(self.nodes[secound_new_vertex]);

        self.find_events(self.nodes[first_new_vertex], event.time)?;
        self.find_events(self.nodes[secound_new_vertex], event.time)?;
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
fn compute_edge_event(vert:&Node, 
    v1_coord:Point2<f32>, 
    v1_bisect:Vector2<f32>, 
    v2_coord:Point2<f32>, 
    v2_bisect:Vector2<f32>
    ) -> Result<Option<Event>, SkeletonError> {

    // Calculate intersection of bisectors
    let t = compute_intersection_time(&v1_coord, &v1_bisect, &v2_coord, &v2_bisect)?;

    if t > 0.0 {
        Ok(Some(Event {
            time: OrderedFloat(t),
            node: *vert,
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
    let v1 = (next_point - current_point).normalize();
    let v2 = (prev_point - current_point).normalize();

    let d = v1 + v2;
    let mut a = Matrix2::from_columns(&[d,-v1]);
    let t = Vector2::new(-v1[1], v1[0]);
    assert!(a.try_inverse_mut());

    let s = a * t;

    let bisector = d * s[0];

    Ok(bisector)
}
pub fn is_reflex(
    current_point: Point2<f32>,
    next_point: Point2<f32>,
    prev_point: Point2<f32>,
) -> bool {
    let v1 = Vector2::new(
        prev_point.x - current_point.x,
        prev_point.y - current_point.y,
    ).normalize();
    let v2 = Vector2::new(
        next_point.x - current_point.x,
        next_point.y - current_point.y,
    ).normalize();

    let mut bisector = v1 + v2;
    return (-v1).perp(&v2) < 0.0;
}
pub fn intersect(p1:Point2<f32>, v1:Vector2<f32>, p2:Point2<f32>, v2:Vector2<f32>) -> Point2<f32> {
    // calculate the intersection of two lines represented as a point(p) and a vector(v)
    let u = p2 - p1;
    let mut v = Matrix2::from_columns(&[v1,-v2]);

    assert!(v.try_inverse_mut()); // panic if v is not invertable
    let s = v*u;
    return p1 + v1 * s[0];
}

pub fn create_skeleton(
    points: Vec<Point2<f32>>,
    weights: &[f32],
) -> Result<StraightSkeleton, SkeletonError> {
    let builder = SkeletonBuilder::new(points, weights)?;
    return builder.compute_skeleton();
}


impl Display for EventType{
    fn fmt(&self, b:&mut fmt::Formatter) -> Result<(),fmt::Error> {
        match self {
            EventType::Edge => write!(b,"Edge")?,
            EventType::Split(_) => write!(b,"Split")?,
            EventType::Vertex => write!(b,"Vertex")?,
        }
        Ok(())
    }
}

impl Display for SkeletonBuilder{
    fn fmt(&self, b: &mut Formatter)->Result<(),fmt::Error> {

        writeln!(b,"\x1b[1m|             Nodes            | Bisector  |  | Vertices  |");
        writeln!(b,"\x1b[1;4m| ndx | next | prev | vert_ndx |  x  |  y  |\x1b[0m  \x1b[1;4m|  x  |  y  |\x1b[0m");
        for (i, node) in self.nodes.iter().enumerate() {
            // Nodes
            if self.active_nodes.contains(&node.ndx){
                write!(b,"\x1b[036m")?;
            }
            write!(b,"| {:<4}| {:<4} | {:<4} | {:<4}     |{:+.2}|{:+.2}|",
                node.ndx,
                node.next_ndx,
                node.prev_ndx,
                node.vertex_ndx,
                node.bisector[0],
                node.bisector[1],
                )?;
            write!(b,"\x1b[0m  ")?;

            // Vertices
            match self.vertices.get(i) {
                None => write!(b,"     --")?,
                Some(vert) => write!(b, "|{:+.2}|{:+.2}|",vert[0],vert[1])?,
            };
            writeln!(b,"\x1b[0m  ")?;
        }
        // Events
        match self.events.peek() {
            None => (),
            Some(event) => writeln!(b, "Next Event: {} event at {:.2} for node {}",
                event.0.event_type,
                event.0.time,
                event.0.node.ndx
                )?,
        };
        Ok(())
    }
} 

impl Display for Nodes{
    fn fmt(&self, b: &mut Formatter)->Result<(),fmt::Error> {

        writeln!(b,"\x1b[1m|             Nodes            | Bisector  |");
        writeln!(b,"\x1b[1;4m| ndx | next | prev | vert_ndx |  x  |  y  |\x1b[0m");
        for node in self.nodes.iter() {
            // Nodes
            if self.active_nodes.contains(&node.ndx){
                write!(b,"\x1b[036m")?;
            }
            write!(b,"| {:<4}| {:<4} | {:<4} | {:<4}     |{:+.2}|{:+.2}|",
                node.ndx,
                node.next_ndx,
                node.prev_ndx,
                node.vertex_ndx,
                node.bisector[0],
                node.bisector[1],
                )?;
            writeln!(b,"\x1b[0m  ")?;
        }
        Ok(())
    }
} 
