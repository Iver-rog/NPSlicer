use nalgebra::{Matrix2, Point2, Vector2};
use nalgebra_glm::cross2d;
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::{
    collections::HashSet, 
    f32::EPSILON, 
    fmt::{self, Display, Formatter}
};
use thiserror::Error;
use log::{debug, error, info, trace};


#[derive(Debug, Error)]
pub enum SkeletonError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("Computation error: {0}")]
    ComputationError(String),
    #[error("Bisector Calculation Erorr: {0}")]
    BisectorError(String),
    #[error("Initialization Error could not create SkeletonBuilder {0}")]
    InitializationError(String),
    #[error("Split event Error {0}")]
    EdgeEventError(String),
    #[error("Split event Error {0}")]
    SplitEventError(String),
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
    pub fn deactivate(&mut self, node_ndx:&usize){
        self.active_nodes.remove(node_ndx);
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
}
#[derive(Debug)]
pub struct Vertex {
    coords: Point2<f32>,
    time: f32,
}
#[derive(Debug)]
pub struct SkeletonBuilder {
    shrining_polygon: Nodes,
    original_polygon: Nodes,
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    events: PriorityQueue<Event, OrderedFloat<f32>>,
}
impl SkeletonBuilder {
    pub fn new(points: Vec<Point2<f32>>) -> Result<Self, SkeletonError> {
        if points.len() < 3 {
            return Err(SkeletonError::InvalidPolygon(
                "Polygon must have at least 3 vertices".to_string(),
            ));
        }
        info!("\x1b[034m========================== Initializing Polygon ==========================\x1b[0m");
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

            let bisector = match bisector(p_current, p_next, p_prev){
                Ok(bisector) => bisector,
                Err(error) => return Err(SkeletonError::InitializationError("Could not construct bisector for node {i}\n{error}".to_string()))
            };

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
            vertices:points.into_iter().map(|p| Vertex{coords:p,time:0.0}).collect(),
            events: PriorityQueue::new(),
        };

        //initialize events 
        for node in builder.shrining_polygon.nodes.clone().iter() {
            builder.find_events(*node, OrderedFloat(0.0))?;
        }
        Ok(builder)
    }

    fn find_events(&mut self, node:Node, current_time: OrderedFloat<f32>) -> Result<(), SkeletonError> {
        // Edge event between node and next_node
        let edge_event = self.compute_edge_event(
            node.ndx, 
            )?;
        if let Some(event) = edge_event {
            let time = event.time;
            self.events.push(event, -time);
        }
        // Split events
        let split_events = self.compute_split_events(&node)?;
        for mut event in split_events {
            let time = event.time;
            event.time = time;
            self.events.push(event, -time);
        }
        Ok(())
    }
    fn find_edge_event(&mut self, node_ndx:usize,time:f32) -> Result<(), SkeletonError> {
        // Edge events
        let edge_event = self.compute_edge_event(
            node_ndx,
            )?;
        if let Some(event) = edge_event {
            let time = event.time;
            self.events.push(event, -time);
        }
        Ok(())
    }
    fn compute_edge_event(&self, vert_ndx:usize) -> Result<Option<Event>, SkeletonError> {

        let v1 = self.shrining_polygon.nodes[vert_ndx];
        let v1_vert = &self.vertices[v1.vertex_ndx];
        let v2 = self.shrining_polygon.nodes[v1.next_ndx];
        let v2_vert = &self.vertices[v2.vertex_ndx];

        let max_time = v1_vert.time.max(v2_vert.time);

        let v1_coord = v1_vert.coords + v1.bisector() * (max_time-v1_vert.time);
        let v2_coord = v2_vert.coords + v2.bisector() * (max_time-v2_vert.time);
        // Calculate intersection of bisectors
        let t = compute_intersection_time(&v1_coord, &v1.bisector(), &v2_coord, &v2.bisector())?;

        if t > 0.0 {
            Ok(Some(Event {
                time: OrderedFloat(t+max_time),
                node: self.shrining_polygon.nodes[vert_ndx],
                event_type: EventType::Edge,
            }))
        } else {
            Ok(None)
        }
    }
    fn compute_split_events<'a>(&'a self, node: &'a Node) -> Result<Vec<Event>, SkeletonError> {
        let mut events = Vec::new();
        // Check if edge is a reflex angle

        let node_v = &self.vertices[node.vertex_ndx];
        let next_node = self.shrining_polygon.next(*node);
        let next_node_v = &self.vertices[next_node.vertex_ndx];
        let prev_node = self.shrining_polygon.prev(*node);
        let prev_node_v = &self.vertices[prev_node.vertex_ndx];

        if !is_reflex(node_v.coords,
            next_node_v.coords + (node_v.time - next_node_v.time)*next_node.bisector(),
            prev_node_v.coords + (node_v.time - prev_node_v.time)*prev_node.bisector())
        {
            return Ok(events)
        }
        info!("\x1b[033mFinding split events for node: {} at {}\x1b[0m",node.ndx, self.vertices[node.vertex_ndx].coords);
        let node_p = self.vertices[node.vertex_ndx].coords;

        // Looking for splitt candidates
        for edge_start in self.original_polygon.nodes.iter()
            .filter(|e| e.vertex_ndx != node.vertex_ndx && 
                self.shrining_polygon.next(**e).vertex_ndx != node.vertex_ndx)
            {
                let edge_end = self.shrining_polygon.next(*edge_start);

                // coordinates (Points)
                let edge_start_p = self.vertices[edge_start.vertex_ndx].coords;
                let edge_end_p = self.vertices[edge_end.vertex_ndx].coords;
                trace!("considering edge from node {}-{}  with cooridnates {} - {}"
                    ,edge_start.ndx, edge_end.ndx, edge_start_p,edge_end_p );

                // vector pointing in the direction of the tested edge
                let edge_vec = edge_start_p + edge_start.bisector() 
                           - ( edge_end_p + edge_end.bisector() );

                // vector pointing form the splitting vertex to its next vertex
                let edge_left = node_p
                    - self.vertices[self.shrining_polygon.next(*node).vertex_ndx].coords;
                // vector pointing from the splitting vertex to its previous vertex
                let edge_right = node_p 
                    - self.vertices[self.shrining_polygon.prev(*node).vertex_ndx].coords;

                // a potential b is at the intersection of between our own bisector and the 
		            // bisector of the angle between the tested edge and any one of our own edges.

				        // we choose the "less parallel" edge (in order to exclude a potentially parallel edge)
                let leftdot = (edge_left.normalize().dot(&edge_vec.normalize())).abs();
                let rightdot = (edge_right.normalize().dot(&edge_vec.normalize())).abs();
                let self_edge =  if leftdot < rightdot { edge_left }else{ edge_right };

                let i = intersect( edge_start_p, edge_vec, node_p, self_edge );

                if (i-node_p).magnitude() < 1e-5{
                    info!("skiping node: {}. value = {}", node.ndx,(i-node_p).magnitude());
                    continue;
                }
                // Locate candidate b
                let line_vec = (node_p - i).normalize();
                let mut ed_vec = (edge_vec).normalize();

                if leftdot > rightdot {
                    ed_vec = - ed_vec
                }

                let bisector = ed_vec + line_vec;
                if bisector.magnitude() == 0.0 { continue; };

                let b = intersect(i, bisector, node_p, node.bisector() );

                // Check eligebility of b
                // a valid b should lie within the area limited by the edge and the bisectors of its two vertices:
                let x_start = cross2d(&edge_start.bisector().normalize(),
                    &(b-edge_start_p).normalize()) < EPSILON;
                let x_end = cross2d(&edge_end.bisector().normalize(),
                    &(b-edge_end_p).normalize()) > -EPSILON;
                let x_edge = cross2d(&edge_vec.normalize(),
                    &(b-edge_start_p).normalize()) < EPSILON;

                if !(x_start && x_end && x_edge) {
                    //debug!("- discarding candidate for edge:({}-{}) b: {b}",edge_start.ndx,edge_end.ndx);
                    continue;
                };
                info!("  - found candidate for edge:({}-{}) b: {b}",edge_start.ndx,edge_end.ndx);


                let t = (node_p - b).magnitude() / node.bisector().magnitude();
                let split = [OrderedFloat(b[0]),OrderedFloat(b[1])];

                events.push(Event {
                    time: OrderedFloat(t+node_v.time),
                    node: *node,
                    event_type: EventType::Split(split),
                });
            }
        Ok(events)
    }
    pub fn shrinking_polygon_at_time(&self, time:f32) -> StraightSkeleton {
        let mut computed_vertecies:HashSet<usize> = HashSet::new();
        let mut vertices = Vec::new();
        let mut edges:Vec<[usize;2]> = Vec::new();
        for node_ndx in self.shrining_polygon.active_nodes.iter()
            .filter(|n|self.shrining_polygon.contains(n)) {
                if computed_vertecies.contains(&node_ndx){continue};
                let start_node = self.shrining_polygon.nodes[*node_ndx];
                let first_index = edges.len();
                for node in self.shrining_polygon.iter(&start_node){
                    let vertex = self.vertices[node.vertex_ndx].coords + node.bisector()*(time - self.vertices[node.vertex_ndx].time);
                    vertices.push(vertex);
                    edges.push([vertices.len()-1,vertices.len()]);
                    computed_vertecies.insert(node.ndx);
                }
                let last_index = edges.len()-1;
                edges[last_index] = [last_index,first_index];
            }

        let contour = StraightSkeleton{
            vertices,
            edges,
        };
        return contour;
    }

    pub fn compute_skeleton(mut self) -> Result<(StraightSkeleton,Vec<StraightSkeleton>), SkeletonError> {
        info!("\x1b[034m========================== Computing Skeleton ==========================\x1b[0m");
        let mut debug_contours: Vec<StraightSkeleton> = Vec::new();
        let mut handled_events = 0;
        while let Some((event, _)) = self.events.pop() {
            let current_time = *event.time;
            let result = match event.event_type {
                EventType::Edge => self.handle_edge_event(event),
                EventType::Split(_) => self.handle_split_event(event),
            };
            match result {
                Ok(print_debug) => {
                    handled_events += 1;
                    if print_debug {
                        debug!("\n{self}");
                        debug_contours.push(self.shrinking_polygon_at_time(current_time))
                    }
                },
                Err(error) => {
                    println!("\x1b[031mevent number: {handled_events} {error}\x1b[0m");
                    println!("{self}");
                }
            }
            if self.shrining_polygon.len() < 3 {
                break;
            }
        }
        let vertices = self.vertices.into_iter().map(|n| n.coords).collect();
        let edges = self.edges.into_iter()
            .map(|edge| [edge.start,edge.end] )
            .collect();
        let skeleton = StraightSkeleton {
            vertices,
            edges,
        };
        Ok((skeleton,debug_contours))
    }
    fn handle_edge_event(&mut self, event:Event ) -> Result<bool, SkeletonError> {
        if !self.shrining_polygon.contains(&event.node.ndx) || 
           !self.shrining_polygon.contains(&event.node.next_ndx) {
            info!("t:{:.3} skipping Edge  Event node: {} \x1b[031minactive node in edge\x1b[0m",
                event.time,event.node.ndx);
            return Ok(false);
        }
        let edge_start = self.shrining_polygon.nodes[event.node.ndx];
        let edge_end = self.shrining_polygon.next(edge_start);

        // Calculate new vertex position
        let edge_start_v = &self.vertices[edge_start.vertex_ndx];
        let new_vertex = edge_start_v.coords + edge_start.bisector() * (event.time.0-edge_start_v.time);
        // Add new vertex to list
        let new_vertex_ndx = self.vertices.len();
        self.vertices.push(Vertex{coords:new_vertex,time:*event.time});
        // Add new skeleton vertex and edges
        self.edges.push(Edge{start:edge_start.vertex_ndx, end:new_vertex_ndx});
        self.edges.push(Edge{start:edge_end.vertex_ndx, end:new_vertex_ndx});

        if edge_start.prev_ndx == edge_end.next_ndx {
            // triangle detected vertex event
            let remaining_vertex = self.shrining_polygon.nodes[edge_start.prev_ndx];
            self.edges.push(Edge{start:remaining_vertex.vertex_ndx, end:new_vertex_ndx});
            self.shrining_polygon.deactivate(&edge_start.ndx);
            self.shrining_polygon.deactivate(&edge_end.ndx);
            self.shrining_polygon.deactivate(&remaining_vertex.ndx);
            info!("\x1b[032mt:{:.3} Vertex Event for node: {} & ({},{}) \x1b[0m",
                event.time,edge_start.ndx,edge_end.ndx,remaining_vertex.ndx);
            return Ok(true);
        }

        // Calculate bisecotr for newly created vertex
        let edge_end_next = self.shrining_polygon.next(edge_end);
        let edge_end_next_v = &self.vertices[edge_end_next.vertex_ndx];
        let edge_end_next_p = edge_end_next_v.coords + (edge_end_next.bisector()*(event.time.0-edge_end_next_v.time)) ;

        let edge_start_prev = self.shrining_polygon.prev(edge_start);
        let edge_start_prev_v = &self.vertices[edge_start_prev.vertex_ndx];
        let edge_start_prev_p = edge_start_prev_v.coords + (edge_start_prev.bisector()*(event.time.0-edge_start_prev_v.time)) ;

        let bisector = match bisector(new_vertex,edge_end_next_p,edge_start_prev_p){
            Ok(bisector) => bisector,
            Err(error)   => return Err(SkeletonError::EdgeEventError("could not calculate error for newly created vertex: {error}".to_string()))
        };
        //let bisector = 0.5*(edge_start.bisector() + edge_end.bisector());
        let new_node = Node::new()
            .next_ndx(edge_end.next_ndx)
            .prev_ndx(edge_start.prev_ndx) 
            .bisector(bisector)
            .vertex_ndx(new_vertex_ndx);
        let new_node = self.shrining_polygon.merge(new_node);

        //find events for the new vertex
        self.find_events(new_node,event.time)?;
        let prev_node = self.shrining_polygon.nodes[new_node.prev_ndx];
        // find edge event for previous node
        let edge_event = self.compute_edge_event( prev_node.ndx)?;
        if let Some(event) = edge_event {
            let time = event.time;
            self.events.push(event, -time);
        }

        info!("\x1b[032mt:{:.3} Edge Event for node:{} at p={}\x1b[0m",event.time,edge_start.ndx,new_vertex);
        Ok(true)
    }
    fn handle_split_event(&mut self, event: Event) -> Result<bool, SkeletonError> {
        let b = match event.event_type { 
            EventType::Split(split) => Point2::new(split[0].0,split[1].0),
            _ => panic!("wrong event type sendt to handle split event funcion")
        };

        let node = self.shrining_polygon.nodes[event.node.ndx];
        let time = event.time.0;

        if !self.shrining_polygon.contains(&event.node.ndx) { 
            info!("t:{:.3} skipping Split Event node: {} split point: {} \x1b[031minactive node\x1b[0m",
                event.time,node.ndx,b);
            return Ok(false) 
        }

        // find edge beeing split
        let mut edge = None;
        // TODO: this loop redundantly checks each edge twise
        for [edge_start, edge_end] in self.shrining_polygon.nodes.iter()
            .filter(|n| self.shrining_polygon.contains(&n.ndx))
            .map(|n| [n, &self.shrining_polygon.nodes[n.next_ndx]] )
            // split event has become a vertex event since split event calculation, a vertex event is expected to follow
            .filter(|[edge_start,edge_end]| edge_start.ndx != node.next_ndx && edge_end.ndx != node.prev_ndx )
            .filter(|[edge_start,edge_end]| edge_start.ndx != node.ndx && edge_end.ndx != node.ndx ){
                let start_v = &self.vertices[edge_start.vertex_ndx];
                let end_v = &self.vertices[edge_end.vertex_ndx];

                // the point b should lie inside the area bounded by the bisectors of the nodes of
                // the edge beeing split:
                let b_start = b - start_v.coords;
                if cross2d(&edge_start.bisector(), &b_start) > EPSILON {continue}

                let b_end = b - end_v.coords;
                if cross2d(&edge_end.bisector(), &b_end) < EPSILON {continue}

                // the point b should lie aproximatly on the edge at the time where the split event ocures
                let start = start_v.coords + edge_start.bisector()*(time-start_v.time);
                let end = end_v.coords + edge_end.bisector()*(time-end_v.time);
                if is_point_on_edge(&b, &start, &end)? {
                    if edge.is_some() {println!("multiple edges found")};
                    edge = Some([edge_start,edge_end])
                    }
                }
        if edge.is_none() { 
            info!("t:{:.3} skipping Split Event node: {} split point: {} \x1b[031mcould not find edge\x1b[0m",
                event.time,node.ndx,b);
            return Ok(false) 
        }
        let edge_start = edge.unwrap()[0];
        let edge_end = edge.unwrap()[1];

        self.vertices.push(Vertex{coords:b,time});
        self.edges.push(Edge{start:node.vertex_ndx,end:self.vertices.len()-1});

        info!("\x1b[032mt:{:.3} Split Event between node: {} and edge: ({}-{}) at point: {} \x1b[0m",
            event.time,node.ndx,edge_start.ndx,edge_end.ndx,b);

        // ============= First edge loop ================
        let edge_start_p = &self.vertices[edge_start.vertex_ndx].coords;
        let edge_start_possition = edge_start_p + edge_start.bisector() * time;

        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_start = &self.shrining_polygon.nodes[node.next_ndx];
        let s_vert_start_p = &self.vertices[s_vert_start.vertex_ndx].coords;
        let s_vert_start_possition = s_vert_start_p + s_vert_start.bisector() * time;

        //assert!(edge_start != edge_end_neighbour); // make sure the remaining region is not a triangle (3 verts)

        // add new vertex to vertex lisShort-circuiting logical ANDt

        // add new vertex to vert_ref list
        let bisect = match bisector(b, s_vert_start_possition, edge_start_possition){
            Ok(bisector) => bisector,
            Err(error)   => return Err(SkeletonError::EdgeEventError(format!("s_vert_start:{} edge_start:{} {error}",s_vert_start.ndx,edge_start.ndx)))
        };
        let left_node = Node::new()
            .next_ndx( node.next_ndx)
            .prev_ndx( edge_start.ndx)
            .bisector( bisect )
            .vertex_ndx( self.vertices.len()-1 );

        let edge_end_p = &self.vertices[edge_end.vertex_ndx].coords;
        let edge_end_possition = edge_end_p + edge_end.bisector() * time;

        // ============= Secound edge loop ================
        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_end = &self.shrining_polygon.nodes[node.prev_ndx];
        let s_vert_end_p = &self.vertices[s_vert_end.vertex_ndx].coords;
        let s_vert_end_possition = s_vert_end_p + s_vert_end.bisector() * time;

        // add new vertex to vert_ref list
        let bisector = match bisector(b, edge_end_possition, s_vert_end_possition ){
            Ok(bisector) => bisector,
            Err(error)   => return Err(SkeletonError::EdgeEventError(format!("edge_end:{} s_vert_end:{} {error}",edge_end.ndx, s_vert_end.ndx)))
        };

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
        self.find_edge_event(left_node.prev_ndx,time)?;
        self.find_edge_event(right_node.prev_ndx,time)?;
        Ok(true)
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
    match a.try_inverse_mut() {
        true => (),
        false => Err(SkeletonError::BisectorError( 
            format!("invertable matrix {a}  current_point: {current_point}
  next_point: {next_point}
  prev_point: {prev_point}"
                    ) ) )?
    }

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
) -> Result<StraightSkeleton, SkeletonError> {
    let builder = SkeletonBuilder::new(points)?;
    let (skeleton, _debug_contours) = builder.compute_skeleton()?;
    return Ok(skeleton);
}


impl Display for EventType{
    fn fmt(&self, b:&mut fmt::Formatter) -> Result<(),fmt::Error> {
        match self {
            EventType::Edge => write!(b,"Edge")?,
            EventType::Split(_) => write!(b,"Split")?,
        }
        Ok(())
    }
}

impl Display for SkeletonBuilder{
    fn fmt(&self, b: &mut Formatter)->Result<(),fmt::Error> {
        let nodes_display = format!("{}",self.shrining_polygon);
        let mut nodes_lines = nodes_display.split('\n');
        writeln!(b,"{}  |     Vertices     |",
            nodes_lines.next()
            .unwrap_or("\x1b[1m|             Nodes            | Bisector  |"))?;
        writeln!(b,"{}  \x1b[1;4m|  x  |  y  | time |\x1b[0m",
            nodes_lines.next()
            .unwrap_or("\x1b[1;4m| ndx | next | prev | vert_ndx |  x  |  y  |\x1b[0m"))?;
        for (i, node) in nodes_lines.enumerate() {
            // Nodes
            write!(b,"{node}")?;

            // Vertices
            match self.vertices.get(i) {
                None => write!(b,"     --")?,
                Some(vert) => write!(b, "|{:+.2}|{:+.2}| {:.2} |",vert.coords[0],vert.coords[1],vert.time)?,
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
    let distance_from_edge = (point_vec - t*edge_vec).magnitude();
    Ok(distance_from_edge < 1e-3 )
}
