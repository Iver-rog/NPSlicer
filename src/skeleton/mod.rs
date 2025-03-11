use nalgebra::{Matrix2, Point2, Point3, Vector2};
use nalgebra_glm::cross2d;
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::{
    collections::HashSet, 
    f32::EPSILON, 
    fmt::{self, Display, Formatter}, 
    iter
};
use log::{debug, error, info, trace};

#[cfg(test)]
mod test;

mod error;
use error::{SkeletonError,BisectorError};

mod nodes;
use nodes::{Nodes,Node};

mod bound_event;
mod split_event;
mod edge_event;

use crate::contours::{ polygons_from_contours, Polygon, Contour};


#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct Edge {
    pub start: usize,
    pub end: usize,
}
#[derive(Debug,Default)]
pub struct StraightSkeleton {
    pub vertices: Vec<Point3<f32>>,
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
    Split{ split_point:[OrderedFloat<f32>;2] }, // A region is split into two parts
    Bound{ edge_ndx:usize, intersect_p:[OrderedFloat<f32>;2] }, // node travels outside the bounding shape
}
impl Display for Event {
    fn fmt(&self, b:&mut std::fmt::Formatter<'_>) -> Result<(),std::fmt::Error>{
        write!(b, "t: {} {} Event:\n{} ",self.time,self.event_type,self.node)?;
        Ok(())
    }
}
#[derive(Debug)]
pub struct Vertex {
    coords: Point2<f32>,
    time: f32,
}
#[derive(Debug,Default)]
pub struct SkeletonBuilder {
    shrining_polygon: Nodes,
    original_polygon: Nodes,
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    bounding_contour: Option<Contour>,
}
impl SkeletonBuilder {
    pub fn new() -> Self { Self::default() }
    pub fn from_polygon(polygon:Polygon) -> Result<Self,SkeletonError> {
        let mut builder = SkeletonBuilder::new();
        builder.add_loop(polygon.outer_loop.points)?;
        for hole in polygon.holes.into_iter(){
            builder.add_loop(hole.points)?;
        }
        return Ok(builder)
    }
    pub fn add_loop(&mut self, points: Vec<Point2<f32>>) -> Result<(), SkeletonError> {
        if points.len() < 3 { return Err(SkeletonError::InvalidInput) }
        info!("\x1b[034m========================== Adding Contour ==========================\x1b[0m");

        let offset = self.shrining_polygon.len();
        for i in 0..points.len() {
            let next_ndx = (i + 1) % points.len();
            let prev_ndx = if i == 0 {points.len()-1} else { i-1 };

            let p_current = points[i];
            let p_next = points[next_ndx];
            let p_prev = points[prev_ndx];

            let bisector = match bisector(p_current, p_next, p_prev){
                Ok(bisector) => bisector,
                Err(error) => return Err(SkeletonError::InitializationError{node:i,error})
            };

            self.shrining_polygon.insert(Node {
                ndx: i + offset,
                next_ndx: next_ndx + offset,
                prev_ndx: prev_ndx + offset,
                bisector:[OrderedFloat(bisector[0]), OrderedFloat(bisector[1])],
                vertex_ndx: i + offset
            });
            self.original_polygon.insert(Node {
                ndx: i + offset,
                next_ndx: next_ndx + offset,
                prev_ndx: prev_ndx + offset,
                bisector:[OrderedFloat(bisector[0]), OrderedFloat(bisector[1])],
                vertex_ndx: i + offset
            });
            self.edges.push(Edge {
                start: i + offset,
                end: next_ndx + offset,
            });
        }
        self.vertices.extend(points.into_iter().map(|p| Vertex{coords:p,time:0.0}));

        return Ok(())
    }
    pub fn bounding_contour(&mut self, mut contour:Contour){
        if contour.area.is_sign_positive() { contour.reverse_order() }
        self.bounding_contour = Some(contour)
    }
    fn handle_event(&mut self,events:&mut PriorityQueue<Event,OrderedFloat<f32>>,event:Event) -> Result<bool,SkeletonError>{
        match event.event_type {
            EventType::Edge => self.handle_edge_event(events,event),
            EventType::Split{..} => self.handle_split_event(events,event),
            EventType::Bound{..} => self.handle_bound_event(events,event),
        }
    }
    pub fn polygon_at_time(mut self,stop_time:f32) -> Result<Vec<Polygon>, SkeletonError> {
        let mut events: PriorityQueue<Event, OrderedFloat<f32>> = PriorityQueue::new();
        //initialize events 
        for node in self.shrining_polygon.nodes.iter() {
            self.find_events(&mut events,*node)?;
        }
        while let Some((event, priority)) = events.pop() {
            let time = -priority.0;
            if stop_time < time { break }
            if self.shrining_polygon.len() == 0 { break }
            match self.handle_event(&mut events, event){
                Ok(_valid_event) => (),
                Err(error) => { 
                    println!("\x1b[031m{error}\x1b[0m"); 
                    println!("{self}");}
            }
        }
        return Ok(self.offset_active_vertices(stop_time))
    }
    pub fn compute_skeleton(mut self) -> Result<(StraightSkeleton,Vec<Vec<Polygon>>), SkeletonError> {

        let mut events: PriorityQueue<Event, OrderedFloat<f32>> = PriorityQueue::new();
        //initialize events 
        for node in self.shrining_polygon.nodes.iter() {
            self.find_events(&mut events,*node)?;
        }
        info!("\x1b[034m========================== Computing Skeleton ==========================\x1b[0m");
        let mut debug_contours: Vec<Vec<Polygon>> = Vec::new();
        let mut handled_events = 0;
        while let Some((event, priority)) = events.pop() {
            let time = -priority.0;
            if self.shrining_polygon.len() == 0 { break }
            let current_time = time;
            match self.handle_event(&mut events, event) {
                Ok(valid_event) => {
                    if valid_event { 
                        debug_contours.push(self.offset_active_vertices(current_time));
                        debug!("\n{self}");
                        handled_events += 1;
                    };
                },
                Err(error) => {
                    println!("\x1b[031mevent number: {handled_events} {error}\x1b[0m");
                    debug_contours.push(self.offset_active_vertices(current_time));
                    break 
                }
            }
        }
        let vertices = self.vertices.into_iter()
            .map(|n| Point3::new(n.coords.x,n.coords.y,n.time))
            .collect();
        let edges = self.edges.into_iter()
            .map(|edge| [edge.start,edge.end] )
            .collect();
        let skeleton = StraightSkeleton {
            vertices,
            edges,
        };
        return Ok((skeleton,debug_contours))
    }

    fn find_events(&self, events:&mut PriorityQueue<Event, OrderedFloat<f32>>, node:Node) -> Result<(), SkeletonError> {
        // Edge event between node and next_node
        let edge_event = self.compute_edge_event(
            node.ndx, 
            )?;
        if let Some(event) = edge_event {
            let time = event.time;
            events.push(event, -time);
        }
        // Split events
        let split_events = self.compute_split_events(&node)?;
        for mut event in split_events {
            let time = event.time;
            event.time = time;
            events.push(event, -time);
        }
        // Bounds event
        if let Some(event) = self.compute_bounds_event(&node){
            let time = event.time;
            events.push(event, -time);
        }
        Ok(())
    }

    pub fn offset_active_vertices(&self, time:f32) -> Vec<Polygon> {
        let mut computed_vertecies:HashSet<usize> = HashSet::new();
        let mut contours = Vec::new();
        for node_ndx in self.shrining_polygon.active_nodes_iter() {
                if computed_vertecies.contains(&node_ndx){continue};
                let mut contour = Vec::new();
                let start_node = self.shrining_polygon.nodes[*node_ndx];
                for node in self.shrining_polygon.iter(&start_node){
                    let vertex = self.vertices[node.vertex_ndx].coords + node.bisector()*(time - self.vertices[node.vertex_ndx].time);
                    contour.push(vertex);
                    computed_vertecies.insert(node.ndx);
                }
                contours.push(Contour::new(contour));
            }
        return polygons_from_contours(contours)
    }
}
fn compute_intersection_time(
    p1: &Point2<f32>,
    v1: &Vector2<f32>,
    p2: &Point2<f32>,
    v2: &Vector2<f32>,
) -> Result<Option<f32>, SkeletonError> {
    // Solve the system of equations:
    // p1 + t*v1 = p2 + t*v2
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let det = v1.x * (-v2.y) - (-v2.x) * v1.y;

    // check if the bisectors are parallel
    if det.abs() < 1e-5 {
        // check if p2 lies on the line defined by p1 and v1
        let s_x = ( p2.x - p1.x )/v1.x;
        let s_y = ( p2.y - p1.y )/v1.y;
        if (s_x - s_y).abs() > 1e-5 {return Ok(None)};
        return Err(SkeletonError::ComputationError(
            format!("Parallel bisectors detected p1:{p1} v1:{v1} p2:{p2} v2:{v2}"),
        ));
    }
    let t = (dx * (-v2.y) - (-v2.x) * dy) / det;
    Ok(Some(t))
}
pub fn bisector(
    current_point: Point2<f32>,
    next_point: Point2<f32>,
    prev_point: Point2<f32>,
) -> Result<Vector2<f32>, BisectorError> {
    let v1 = (next_point - current_point).normalize();
    let v2 = (prev_point - current_point).normalize();

    let d = v1 + v2;
    let mut a = Matrix2::from_columns(&[d,-v1]);
    let t = Vector2::new(-v1[1], v1[0]);
    match a.try_inverse_mut() {
        true => (),
        false => return Err(BisectorError{
            matrix:a,
            current_p:current_point,
            next_p:next_point,
            prev_p:prev_point,
         })
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
pub fn intersect(p1:Point2<f32>, v1:Vector2<f32>, p2:Point2<f32>, v2:Vector2<f32>) -> Option<Point2<f32>> {
    // calculate the intersection of two lines represented as a point(p) and a vector(v)

    let det = v1.x*v2.y - v1.y * v2.x; // determinant
    if det.abs() < f32::EPSILON {
        // vectors are parallel cant compute intersection
        return None
    }

    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    let t = (dx * v2.y - dy * v2.x) / det;

    let xi = p1.x + t * v1.x;
    let yi = p1.y + t * v1.y;

    return Some(Point2::new(xi,yi))
}

pub fn skeleton_from_polygon( polygon:Polygon ) -> Result<StraightSkeleton, SkeletonError> {
    let builder = SkeletonBuilder::from_polygon(polygon)?;
    let (skeleton, _debug_contours) = builder.compute_skeleton()?;
    return Ok(skeleton);
}

impl Display for EventType{
    fn fmt(&self, b:&mut fmt::Formatter) -> Result<(),fmt::Error> {
        match self {
            EventType::Edge => write!(b,"Edge")?,
            EventType::Split{split_point:split_p} => write!(b,"Split Event at {}{}",split_p[0],split_p[1])?,
            EventType::Bound{edge_ndx, intersect_p} => write!(b,"Bound Event node hit edge {edge_ndx} at {}{}",intersect_p[0],intersect_p[1])?,
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
        Ok(())
    }
} 

