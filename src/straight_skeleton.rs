use nalgebra::{Matrix2, Point2, Vector2};
use nalgebra_glm::cross2d;
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::{collections::HashSet, f32::EPSILON, fmt::Formatter};
use thiserror::Error;
use std::hash::Hash;
use std::fmt::{self, Display};

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
    pub fn new() -> NodeBuilder{
        NodeBuilder::default()
    }
}
#[derive(Default,Debug)]
pub struct NodeBuilder {
    pub ndx: Option<usize>,
    pub next_ndx: Option<usize>,
    pub prev_ndx: Option<usize>,
    pub bisector: Option<Vector2<f32>>,
    pub vertex_ndx: Option<usize>,
}
impl NodeBuilder {
    fn ndx(&mut self,ndx:usize) { self.ndx = Some(ndx) }
    fn next_ndx(mut self,ndx:usize) -> NodeBuilder { self.next_ndx = Some(ndx); return self }
    fn prev_ndx(mut self,ndx:usize) -> NodeBuilder { self.prev_ndx = Some(ndx); return self }
    fn bisector(mut self,bisector:Vector2<f32>) -> NodeBuilder { self.bisector = Some(bisector); return self }
    fn vertex_ndx(mut self,vertex_ndx:usize) -> NodeBuilder { self.vertex_ndx = Some(vertex_ndx); return self }
}
impl From<NodeBuilder> for Node {
    fn from(builder: NodeBuilder) -> Self {
        let bisector = builder.bisector.unwrap();
        Node{
            ndx: builder.ndx.unwrap(),
            next_ndx: builder.next_ndx.unwrap(),
            prev_ndx: builder.prev_ndx.unwrap(),
            bisector: [OrderedFloat::from(bisector[0]),OrderedFloat::from(bisector[1])],
            vertex_ndx: builder.vertex_ndx.unwrap(),
        }
    }
}
#[derive(Debug,Default)]
pub struct Nodes {
    pub nodes: Vec<Node>,
    active_nodes: HashSet<usize>,
}
impl Nodes {
    pub fn from_closed_curve(nodes:Vec<Node>) -> Self {
        Nodes {
        active_nodes: HashSet::from_iter(0..nodes.len()),
        nodes,
        }
    }
    pub fn merge(&mut self, mut node: NodeBuilder) -> Node {
        // Insert new node
        node.ndx(self.nodes.len());
        let node = Node::from(node);
        self.nodes.push(node);
        self.active_nodes.insert(node.ndx);
        // remove old vertices
        let unused1 = self.nodes[node.prev_ndx].next_ndx;
        let unused2 = self.nodes[node.next_ndx].prev_ndx;
        self.active_nodes.remove(&unused1);
        self.active_nodes.remove(&unused2);
        // update node refrences to point to the new node
        self.nodes[node.prev_ndx].next_ndx = node.ndx;
        self.nodes[node.next_ndx].prev_ndx = node.ndx;
        return node
    }
    pub fn split(&mut self, mut left_node:NodeBuilder, mut right_node:NodeBuilder)->[Node;2]{
        left_node.ndx( self.nodes.len() );
        let left_node = Node::from(left_node);
        self.nodes.push(left_node);
        let splitting_vert = self.nodes[left_node.next_ndx].prev_ndx;
        self.nodes[left_node.next_ndx].prev_ndx = left_node.ndx;
        self.nodes[left_node.prev_ndx].next_ndx = left_node.ndx;

        right_node.ndx( self.nodes.len() );
        let right_node = Node::from( right_node );
        self.nodes.push(right_node);
        self.nodes[right_node.next_ndx].prev_ndx = right_node.ndx;
        self.nodes[right_node.prev_ndx].next_ndx = right_node.ndx;

        self.active_nodes.insert(right_node.ndx);
        self.active_nodes.insert(left_node.ndx);
        self.active_nodes.remove(&splitting_vert);
        return [left_node,right_node]
    }

    fn next(&self,strat_node:Node) -> Node{
        self.nodes[strat_node.next_ndx]
    }
    fn prev(&self,strat_node:Node) -> Node{
        self.nodes[strat_node.prev_ndx]
    }
    fn len(&self) -> usize {
        self.active_nodes.len()
    }
    fn contains(&self, index:&usize) -> bool {
        self.active_nodes.contains(index)
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
#[allow(unused)]
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
    Split([OrderedFloat<f32>;2]), // A region is split into two parts
    Vertex,// A region disapears/colapses into a vertex
}
#[derive(Debug)]
pub struct SkeletonBuilder {
    shrining_polygon: Nodes,
    original_polygon: Nodes,
    vertices: Vec<Point2<f32>>,
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
            shrining_polygon: Nodes::from_closed_curve(nodes.clone()),
            original_polygon: Nodes::from_closed_curve(nodes),
            edges,
            vertices:points,
            events: PriorityQueue::new(),
        };


        //initialize events 
        let mut event_que: PriorityQueue<Event, OrderedFloat<f32>> = PriorityQueue::new();
        for node in builder.shrining_polygon.nodes.iter() { 

            // Edge events
            let edge_event = compute_edge_event(
                &node, 
                builder.vertices[node.vertex_ndx],
                node.bisector(),
                builder.vertices[builder.shrining_polygon.next(*node).vertex_ndx],
                builder.shrining_polygon.next(*node).bisector()
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
            self.vertices[self.shrining_polygon.next(vertex).vertex_ndx],
            self.shrining_polygon.next(vertex).bisector()
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
    fn find_edge_event(&mut self, vertex:Node, current_time: OrderedFloat<f32>) -> Result<(), SkeletonError> {
        // Edge events
        let edge_event = compute_edge_event(
            &vertex, 
            self.vertices[vertex.vertex_ndx], 
            vertex.bisector(),
            self.vertices[self.shrining_polygon.next(vertex).vertex_ndx],
            self.shrining_polygon.next(vertex).bisector()
            )?;
        if let Some(event) = edge_event {
            let time = current_time+event.time;
            self.events.push(event, -time);
        }
        Ok(())
    }

    fn compute_split_events<'a>(&'a self, node: &'a Node) -> Result<Vec<Event>, SkeletonError> {
        let mut events = Vec::new();
        // Check if edge is a reflex angle

        if !is_reflex(self.vertices[node.vertex_ndx],
            self.vertices[self.shrining_polygon.next(*node).vertex_ndx],
            self.vertices[self.shrining_polygon.prev(*node).vertex_ndx])
        {
            return Ok(events)
        }
        println!("\x1b[033mFinding split events for node: {} at {}\x1b[0m",node.ndx, self.vertices[node.vertex_ndx]);
        let node_p = self.vertices[node.vertex_ndx];

        // Looking for splitt candidates
        for edge_start in self.original_polygon.nodes.iter()
            .filter(|e| e.vertex_ndx != node.vertex_ndx && 
                self.shrining_polygon.next(**e).vertex_ndx != node.vertex_ndx)
            {
                let edge_end = self.shrining_polygon.next(*edge_start);
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
                    - self.vertices[self.shrining_polygon.next(*node).vertex_ndx];
                // vector pointing from the splitting vertex to its previous vertex
                let edge_right = node_p 
                    - self.vertices[self.shrining_polygon.prev(*node).vertex_ndx];

                // a potential b is at the intersection of between our own bisector and the 
		            // bisector of the angle between the tested edge and any one of our own edges.

				        // we choose the "less parallel" edge (in order to exclude a potentially parallel edge)
                let leftdot = (edge_left.normalize().dot(&edge_vec.normalize())).abs();
                let rightdot = (edge_right.normalize().dot(&edge_vec.normalize())).abs();
                let self_edge =  if leftdot < rightdot { edge_left }else{ edge_right };

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
                let split = [OrderedFloat(b[0]),OrderedFloat(b[1])];

                events.push(Event {
                    time: OrderedFloat(time),
                    node: *node,
                    event_type: EventType::Split(split),
                });
            }
        Ok(events)
    }


    pub fn compute_skeleton(mut self) -> Result<StraightSkeleton, SkeletonError> {
        println!("{self}");
        while let Some((event, _)) = self.events.pop() {
            match event.event_type {
                EventType::Edge => self.handle_edge_event(event)?,
                EventType::Split(_) => self.handle_split_event(event)?,
                EventType::Vertex => todo!(),//self.handle_vertex_event(event)?,
            }
            println!("{self}");
            if self.shrining_polygon.len() < 3 {
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
        let edge_start = event.node;
        let edge_end = self.shrining_polygon.next(edge_start);

        if !self.shrining_polygon.contains(&edge_start.ndx) || !self.shrining_polygon.contains(&edge_end.ndx) {
            return Ok(());
        }
        // Calculate new vertex position
        let edge_start_p = &self.vertices[edge_start.vertex_ndx];
        let new_vertex = edge_start_p + edge_start.bisector() * event.time.0;
        // Add new vertex to list
        let new_vertex_ndx = self.vertices.len();
        self.vertices.push(new_vertex);
        // Add new skeleton vertex and edges
        self.edges.push(Edge{start:edge_start.vertex_ndx, end:new_vertex_ndx});
        self.edges.push(Edge{start:edge_end.vertex_ndx, end:new_vertex_ndx});
        // Calculate bisecotr for newly created vertex
        let bisector = 0.5*(edge_start.bisector() + edge_end.bisector());
        let new_node = Node::new()
            .next_ndx(edge_end.next_ndx)
            .prev_ndx(edge_start.prev_ndx) 
            .bisector(bisector)
            .vertex_ndx(new_vertex_ndx);
        let new_node = self.shrining_polygon.merge(new_node);

        //find events for the new vertex
        self.find_events(new_node,event.time)?;

        println!("\x1b[033mEdge Event for node:{} at t={:.3} p={}\x1b[0m",edge_start.ndx,event.time,new_vertex);
        Ok(())
    }
    fn handle_split_event(&mut self, event: Event) -> Result<(), SkeletonError> {
        let node = event.node;
        let time = event.time.0;

        if !self.shrining_polygon.contains(&node.ndx) { return Ok(()) }

        let b = match event.event_type { 
            EventType::Split(split) => Point2::new(split[0].0,split[1].0),
            _ => panic!("wrong event type sendt to handle split event funcion")
        };
        // find edge beeing split
        let mut edge = None;
        for [edge_start, edge_end] in self.shrining_polygon.nodes.iter()
            .filter(|n| n.ndx != node.ndx)
            .filter(|n| self.shrining_polygon.contains(&n.ndx))
            .map(|n| [n, &self.shrining_polygon.nodes[n.next_ndx]] )
            .filter(|[_,edge_end]| edge_end.ndx != node.ndx ){
                let start = self.vertices[edge_start.vertex_ndx] + edge_start.bisector()*time;
                let end = self.vertices[edge_end.vertex_ndx] + edge_end.bisector()*time;
                if is_point_on_edge(&b, &start, &end)? {
                    edge = Some([edge_start,edge_end])
                    }
                }
        if edge.is_none() { println!("No split event found"); return Ok(()) }
        let edge_start = edge.unwrap()[0];
        let edge_end = edge.unwrap()[1];
        println!("edge start {}", edge_start.ndx);
        println!("edge end {}", edge_end.ndx);

        self.vertices.push(b);
        self.edges.push(Edge{start:node.vertex_ndx,end:self.vertices.len()-1});
        //self.active_nodes.remove(&node.ndx);

        println!("\x1b[033mSplit Event between node: {} and edge: ({}-{}) at: {}\x1b[0m",
            node.ndx,edge_start.ndx,edge_end.ndx,b);

        // ============= First edge loop ================
        let edge_start_p = &self.vertices[edge_start.vertex_ndx];
        let edge_start_possition = edge_start_p + edge_start.bisector() * time;

        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_start = &self.shrining_polygon.nodes[node.next_ndx];
        let s_vert_start_p = &self.vertices[node.vertex_ndx];
        let s_vert_start_possition = s_vert_start_p + s_vert_start.bisector() * time;

        //assert!(edge_start != edge_end_neighbour); // make sure the remaining region is not a triangle (3 verts)

        // add new vertex to vertex lisShort-circuiting logical ANDt

        // add new vertex to vert_ref list
        let bisect = bisector(b, s_vert_start_possition, edge_start_possition)?;
        let left_node = Node::new()
            .next_ndx( node.next_ndx)
            .prev_ndx( edge_start.ndx)
            .bisector( bisect )
            .vertex_ndx( self.vertices.len()-1 );

        let edge_end_p = &self.vertices[edge_end.vertex_ndx];
        let edge_end_possition = edge_end_p + edge_end.bisector() * time;

        // ============= Secound edge loop ================
        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_end = &self.shrining_polygon.nodes[node.prev_ndx];
        let s_vert_end_p = &self.vertices[s_vert_end.vertex_ndx];
        let s_vert_end_possition = s_vert_end_p + s_vert_end.bisector() * time;

        // add new vertex to vert_ref list
        let bisector = bisector(b, edge_end_possition, s_vert_end_possition )?;

        let right_node = Node::new()
            .next_ndx( edge_end.ndx)
            .prev_ndx( node.prev_ndx)
            .bisector( bisector)
            .vertex_ndx( self.vertices.len()-1);

        let [left_node,right_node] = self.shrining_polygon.split(left_node, right_node);

        // Find new events for the new verices
        self.find_events(left_node, event.time)?;
        self.find_events(right_node, event.time)?;
        // ToDo: it might also be nessesary to find events for the previous vertices
        self.find_edge_event(
            self.shrining_polygon.nodes[left_node.prev_ndx],
            event.time)?;
        self.find_edge_event(
            self.shrining_polygon.nodes[right_node.prev_ndx],
            event.time)?;
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
        let nodes_display = format!("{}",self.shrining_polygon);
        let mut nodes_lines = nodes_display.split('\n');
        writeln!(b,"{}  | Vertices  |",
            nodes_lines.next()
            .unwrap_or("\x1b[1m|             Nodes            | Bisector  |"))?;
        writeln!(b,"{}  \x1b[1;4m|  x  |  y  |\x1b[0m",
            nodes_lines.next()
            .unwrap_or("\x1b[1;4m| ndx | next | prev | vert_ndx |  x  |  y  |\x1b[0m"))?;
        for (i, node) in nodes_lines.enumerate() {
            // Nodes
            write!(b,"{node}")?;

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
            Some(event) => writeln!(b, "Next Event: {} event at {:.3} for node {}",
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

        writeln!(b,"\x1b[1m|             Nodes            | Bisector  |")?;
        writeln!(b,"\x1b[1;4m| ndx | next | prev | vert_ndx |  x  |  y  |\x1b[0m")?;
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
pub fn is_point_on_edge(
    point: &Point2<f32>,
    edge_start: &Point2<f32>,
    edge_end: &Point2<f32>,
) -> Result<bool, SkeletonError> {
    let edge_vec = edge_end - edge_start;
    let point_vec = point - edge_start;

    let edge_length_sq = edge_vec.norm_squared();
    if edge_length_sq < 1e-5 {
        return Err(SkeletonError::ComputationError(
            "Edge length too small".to_string(),
        ));
    }

    let t = edge_vec.dot(&point_vec) / edge_length_sq;
    let point_on_edge = edge_start + t*edge_vec;
    Ok((point_on_edge - point).magnitude() < 1e-6 )
}
