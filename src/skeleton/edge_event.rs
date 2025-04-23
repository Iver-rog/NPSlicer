use crate::skeleton::*;

impl SkeletonBuilder {
    pub(super) fn find_edge_event(&self, events:&mut PriorityQueue<Event, OrderedFloat<f32>> ,node_ndx:usize) -> Result<(), SkeletonError> {
        // Edge events
        let edge_event = self.compute_edge_event(
            node_ndx,
            )?;
        if let Some(event) = edge_event {
            let time = event.time;
            events.push(event, -time);
        }
        Ok(())
    }
    pub (super) fn compute_edge_event(&self, vert_ndx:usize) -> Result<Option<Event>, SkeletonError> {

        let v1 = self.shrinking_polygon.nodes[vert_ndx];
        let v1_vert = &self.vertices[v1.vertex_ndx];
        let v2 = self.shrinking_polygon.nodes[v1.next_ndx];
        let v2_vert = &self.vertices[v2.vertex_ndx];

        let max_time = v1_vert.time.max(v2_vert.time);

        let v1_coord = v1_vert.coords + v1.bisector() * (max_time-v1_vert.time);
        let v2_coord = v2_vert.coords + v2.bisector() * (max_time-v2_vert.time);
        // Calculate intersection of bisectors
        let t = match compute_intersection_time(&v1_coord, &v1.bisector(), &v2_coord, &v2.bisector())?{
            Some(t) => t,
            None => return Ok(None),
        };

        if t > 0.0 {
            Ok(Some(Event {
                time: OrderedFloat(t+max_time),
                node: self.shrinking_polygon.nodes[vert_ndx],
                event_type: EventType::Edge,
            }))
        } else {
            Ok(None)
        }
    }

    pub (super) fn handle_edge_event(&mut self,events:&mut PriorityQueue<Event, OrderedFloat<f32>>, event:Event ) -> Result<bool, SkeletonError> {
        if !self.shrinking_polygon.contains(&event.node.ndx) || 
           !self.shrinking_polygon.contains(&event.node.next_ndx) {
            info!("t:{:.3} skipping Edge  Event node: {} \x1b[031minactive node in edge\x1b[0m",
                event.time,event.node.ndx);
            return Ok(false);
        }
        let edge_start = self.shrinking_polygon.nodes[event.node.ndx];
        let edge_end = self.shrinking_polygon.next(edge_start);

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
            let remaining_vertex = self.shrinking_polygon.nodes[edge_start.prev_ndx];
            self.edges.push(Edge{start:remaining_vertex.vertex_ndx, end:new_vertex_ndx});
            self.shrinking_polygon.deactivate(&edge_start.ndx);
            self.shrinking_polygon.deactivate(&edge_end.ndx);
            self.shrinking_polygon.deactivate(&remaining_vertex.ndx);
            info!("\x1b[032mt:{:.3} Vertex Event for node: {} & ({},{}) \x1b[0m",
                event.time,edge_start.ndx,edge_end.ndx,remaining_vertex.ndx);
            return Ok(true);
        }

        // Calculate bisecotr for newly created vertex
        let edge_end_next = self.shrinking_polygon.next(edge_end);
        let edge_end_next_v = &self.vertices[edge_end_next.vertex_ndx];
        let edge_end_next_p = edge_end_next_v.coords + (edge_end_next.bisector()*(event.time.0-edge_end_next_v.time)) ;

        let edge_start_prev = self.shrinking_polygon.prev(edge_start);
        let edge_start_prev_v = &self.vertices[edge_start_prev.vertex_ndx];
        let edge_start_prev_p = edge_start_prev_v.coords + (edge_start_prev.bisector()*(event.time.0-edge_start_prev_v.time)) ;

        let bisector = match bisector(new_vertex,edge_end_next_p,edge_start_prev_p){
            Ok(bisector) => bisector,
            Err(error) => return Err(SkeletonError::EdgeEventError(error))
        };
        //let bisector = 0.5*(edge_start.bisector() + edge_end.bisector());
        let new_node = Node::new()
            .next_ndx(edge_end.next_ndx)
            .prev_ndx(edge_start.prev_ndx) 
            .bisector(bisector)
            .vertex_ndx(new_vertex_ndx);
        let new_node = self.shrinking_polygon.merge(new_node);

        //find events for the new vertex
        self.find_events(events,new_node)?;
        let prev_node = self.shrinking_polygon.nodes[new_node.prev_ndx];
        // find edge event for previous node
        let edge_event = self.compute_edge_event( prev_node.ndx)?;
        if let Some(event) = edge_event {
            let time = event.time;
            events.push(event, -time);
        }

        info!("\x1b[032mt:{:.3} Edge Event for node:{} at p={}\x1b[0m",event.time,edge_start.ndx,new_vertex);
        Ok(true)
    }
}
