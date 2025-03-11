use crate::skeleton::*;

impl SkeletonBuilder{
    pub (super) fn compute_split_events(&self, node: &Node) -> Result<Vec<Event>, SkeletonError> {
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
                let edge_vec = if edge_start.vertex_ndx == edge_end.vertex_ndx {
                edge_end_p + edge_end.bisector() - ( edge_start_p + edge_start.bisector() )
                } else { edge_end_p - edge_start_p };

                // vector pointing form the splitting vertex to its next vertex
                let edge_left = (node_p
                    - self.vertices[self.shrining_polygon.next(*node).vertex_ndx].coords).normalize();
                // vector pointing from the splitting vertex to its previous vertex
                let edge_right = (node_p 
                    - self.vertices[self.shrining_polygon.prev(*node).vertex_ndx].coords).normalize();

                // a potential b is at the intersection of between our own bisector and the 
		            // bisector of the angle between the tested edge and any one of our own edges.

				        // we choose the "less parallel" edge (in order to exclude a potentially parallel edge)
                let leftdot = (edge_left.normalize().dot(&edge_vec.normalize())).abs();
                let rightdot = (edge_right.normalize().dot(&edge_vec.normalize())).abs();
                let self_edge =  if leftdot < rightdot { edge_left }else{ edge_right };

                let i = match intersect( edge_start_p, edge_vec, node_p, self_edge ){
                    Some(i) => i,
                    None => {error!("cant compute i: parallel bisectors"); continue }
                };

                if (i-node_p).magnitude() < 1e-5{
                    info!("skiping node: {}. value = {}", node.ndx,(i-node_p).magnitude());
                    continue;
                }
                // Locate candidate b
                let line_vec = (node_p - i).normalize();
                let mut ed_vec = (edge_vec).normalize();

                if leftdot < rightdot { ed_vec = - ed_vec }

                let bisector = ed_vec + line_vec;
                if bisector.magnitude() == 0.0 { continue };

                let b = match intersect(i, bisector, node_p, node.bisector() ){
                    Some(b)=> b,
                    None => { error!("cant compute b: parallel bisectors"); continue },
                };
                // Check eligebility of b
                // a valid b should lie within the area limited by the edge and the bisectors of its two vertices:
                let x_start = cross2d(&edge_start.bisector().normalize(),
                    &(b-edge_start_p).normalize()) < f32::EPSILON;
                let x_end = cross2d(&edge_end.bisector().normalize(),
                    &(b-edge_end_p).normalize()) > - f32::EPSILON;
                let x_edge = cross2d(&edge_vec.normalize(),
                    &(b-edge_start_p).normalize()) > f32::EPSILON;
                // check if b lies infront of the reflex vertex
                let in_front = cross2d(&(b-node_p),&Vector2::new(node.bisector().y,-node.bisector().x)) < 0.0;

                if !(x_start && x_end && x_edge && in_front) {
                    debug!(" - discarding candidate for edge:({}-{}) b: ({:>7.4},{:>7.4}) [{} {} {} {}\x1b[0m]",
                    edge_start.ndx,
                    edge_end.ndx,
                    b.x,
                    b.y,
                    if x_start {"\x1b[032mx_start"} else {"\x1b[031mx_start"},
                    if x_edge {"\x1b[032mx_edge"} else {"\x1b[031mx_edge"},
                    if x_end {"\x1b[032mx_end"} else {"\x1b[031mx_end"},
                    if in_front {"\x1b[032min_front"} else {"\x1b[031min_front"},
                    );
                    continue;
                };
                info!("  - found candidate for edge:({}-{}) b: ({:>7.4},{:>7.4})",edge_start.ndx,edge_end.ndx,b.x,b.y);


                let t = (node_p - b).magnitude() / node.bisector().magnitude();
                let b = [OrderedFloat(b[0]),OrderedFloat(b[1])];

                events.push(Event {
                    time: OrderedFloat(t+node_v.time),
                    node: *node,
                    event_type: EventType::Split{split_point: b},
                });
            }
        Ok(events)
    }
    pub (super) fn handle_split_event(&mut self, events:&mut PriorityQueue<Event, OrderedFloat<f32>>,event: Event) -> Result<bool, SkeletonError> {
        let b = match event.event_type { 
            EventType::Split{split_point: b} => Point2::new(b[0].0,b[1].0),
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
            // split event has become a edge event since split event calculation, a edge event is expected to follow
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
        let edge_start_v = &self.vertices[edge_start.vertex_ndx];
        let edge_start_possition = edge_start_v.coords + edge_start.bisector() * (time-edge_start_v.time);

        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_start = &self.shrining_polygon.nodes[node.next_ndx];
        let s_vert_start_v = &self.vertices[s_vert_start.vertex_ndx];
        let s_vert_start_possition = s_vert_start_v.coords + s_vert_start.bisector() * (time-s_vert_start_v.time);

        // add new vertex to vert_ref list
        let bisect = match bisector(b, s_vert_start_possition, edge_start_possition){
            Ok(bisector) => bisector,
            Err(error)   => return Err(SkeletonError::SplitEventError(format!("s_vert_start:{} edge_start:{} {error}",s_vert_start.ndx,edge_start.ndx)))
        };
        let left_node = Node::new()
            .next_ndx( node.next_ndx)
            .prev_ndx( edge_start.ndx)
            .bisector( bisect )
            .vertex_ndx( self.vertices.len()-1 );

        let edge_end_v = &self.vertices[edge_end.vertex_ndx];
        let edge_end_possition = edge_end_v.coords + edge_end.bisector() * (time-edge_end_v.time);

        // ============= Secound edge loop ================
        // splitting vertex's neighbour forming a close loop with edge_start vertex:
        let s_vert_end = &self.shrining_polygon.nodes[node.prev_ndx];
        let s_vert_end_v = &self.vertices[s_vert_end.vertex_ndx];
        let s_vert_end_possition = s_vert_end_v.coords + s_vert_end.bisector() * (time-s_vert_end_v.time);

        // add new vertex to vert_ref list
        let bisector = match bisector(b, edge_end_possition, s_vert_end_possition ){
            Ok(bisector) => bisector,
            Err(error)   => return Err(SkeletonError::SplitEventError(format!("edge_end:{} s_vert_end:{} {error}",edge_end.ndx, s_vert_end.ndx)))
        };

        let right_node = Node::new()
            .next_ndx( edge_end.ndx)
            .prev_ndx( node.prev_ndx)
            .bisector( bisector)
            .vertex_ndx( self.vertices.len()-1);

        let [left_node,right_node] = self.shrining_polygon.split(left_node, right_node);

        // Find new events for the new verices
        self.find_events(events,left_node)?;
        self.find_events(events,right_node)?;
        // ToDo: it might also be nessesary to find events for the previous vertices
        self.find_edge_event(events,left_node.prev_ndx)?;
        self.find_edge_event(events,right_node.prev_ndx)?;
        Ok(true)
    }
}

pub (super) fn is_point_on_edge(
    point: &Point2<f32>,
    edge_start: &Point2<f32>,
    edge_end: &Point2<f32>,
) -> Result<bool, SkeletonError> {
    let edge_vec = edge_end - edge_start;
    let point_vec = point - edge_start;

    let edge_length_sq = edge_vec.norm_squared();
    if edge_length_sq < f32::EPSILON.powi(2){//1e-5 {
        return Err(SkeletonError::ComputationError(
            "Edge length too small".to_string(),
        ));
    }

    let t = edge_vec.dot(&point_vec) / edge_length_sq;
    let distance_from_edge = (point_vec - t*edge_vec).magnitude();
    Ok(distance_from_edge < 1e-3 )
}
