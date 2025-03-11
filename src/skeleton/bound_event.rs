use crate::skeleton::*;

impl SkeletonBuilder{
    pub (super) fn handle_bound_event(&mut self,events:&mut PriorityQueue<Event,OrderedFloat<f32>>,event:Event)->Result<bool,SkeletonError>{
        if !self.shrining_polygon.contains(&event.node.ndx) {return Ok(false);}
        let (edge_ndx,intersect_p) = match event.event_type {
            EventType::Bound { edge_ndx, intersect_p } => (edge_ndx,intersect_p),
            _ => panic!("wrong event sendt to handle_bound_event"),
        };
        self.shrining_polygon.deactivate(&event.node.ndx);
        self.edges.push(Edge{
            start: event.node.vertex_ndx,
            end: self.vertices.len()
        });
        self.vertices.push(Vertex{
            coords: Point2::new(intersect_p[0].0,intersect_p[1].0),
            time:*event.time,
        });
        Ok(true)
    }
    pub (super) fn find_bound_event(&self, events:&mut PriorityQueue<Event, OrderedFloat<f32>>, node_ndx:usize){

    }
    pub (super) fn compute_bounds_event(&self, node:&Node) -> Option<Event>{
        let node_v = &self.vertices[node.vertex_ndx];
        let node_p = node_v.coords;
        let bounds = match &self.bounding_contour{
            Some(contour) => &contour.points,
            None => return None,
        };
        let edge_start = iter::once(&bounds[bounds.len()-1])
            .chain(bounds.iter());
        let edge_end = bounds.iter();
        let edges = edge_start.zip(edge_end);
        let intersection = edges.enumerate()
            .filter_map(|(i,(edge_start,edge_end))|{
                let edge_vec = edge_end - edge_start;
                match intersect(node_p, node.bisector(), *edge_start, edge_vec){
                    Some(intersection) => {
                        let p_vec = intersection - edge_start;
                        let s = if edge_vec.x.abs() > edge_vec.y.abs() {p_vec.x/edge_vec.x}
                            else {p_vec.y/edge_vec.y};
                         
                    if ((0.0<s) && (s<=1.0)) {Some((i, intersection))} else {None}},
                    None => None,
                }
            })
            .filter(|(i,intersection)|{
                ((intersection - node_p).normalize() - node.bisector().normalize()).magnitude() < 0.01
            })
            .reduce(|(prev_i,prev_intersection),(i,intersection)|{
                let prev_d = (prev_intersection - node_p).magnitude();
                let new_d = (intersection - node_p).magnitude();
                if prev_d > new_d {(i,intersection)} else {(prev_i,prev_intersection)}
            });
        match intersection {
            None => None,
            Some((edge_ndx,i_p)) => {
                let intersection_time = (i_p-node_p).magnitude()/node.bisector().magnitude();
                let event = Event{
                    time: OrderedFloat(intersection_time + node_v.time),
                    node: *node,
                    event_type:EventType::Bound { 
                        edge_ndx, 
                        intersect_p:[OrderedFloat(i_p.x),OrderedFloat(i_p.y)] 
                    }
                };
                Some(event)
            }
        }
    }
}
