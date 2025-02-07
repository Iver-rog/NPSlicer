use nalgebra::Vector2;
use ordered_float::OrderedFloat;
use std::collections::HashSet;
use std::fmt::{self, Display, Formatter};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Node {
    pub ndx: usize,
    pub next_ndx: usize,
    pub prev_ndx: usize,
    pub bisector: [OrderedFloat<f32>;2],
    pub vertex_ndx: usize,
}
impl Node {
    pub fn bisector(&self)-> Vector2<f32>{
        Vector2::new(*self.bisector[0],*self.bisector[1])
    }
    pub fn new() -> NodeBuilder{
        NodeBuilder::default()
    }
}
impl Display for Node{
    fn fmt(&self, b:&mut std::fmt::Formatter<'_>) -> Result<(),std::fmt::Error>{
        write!(b, "ndx: {} next: {} prev: {} bisector: [{} {}] vert_ndx: {}",
            self.ndx,
            self.next_ndx,
            self.prev_ndx,
            self.bisector[0],
            self.bisector[1],
            self.vertex_ndx,
            )?;
        Ok(())
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
    pub fn ndx(&mut self,ndx:usize) { self.ndx = Some(ndx) }
    pub fn next_ndx(mut self,ndx:usize) -> NodeBuilder { self.next_ndx = Some(ndx); return self }
    pub fn prev_ndx(mut self,ndx:usize) -> NodeBuilder { self.prev_ndx = Some(ndx); return self }
    pub fn bisector(mut self,bisector:Vector2<f32>) -> NodeBuilder { self.bisector = Some(bisector); return self }
    pub fn vertex_ndx(mut self,vertex_ndx:usize) -> NodeBuilder { self.vertex_ndx = Some(vertex_ndx); return self }
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
    pub fn insert(&mut self, node:Node) {
        self.active_nodes.insert(self.nodes.len());
        self.nodes.push(node);
    }
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

    pub fn next(&self,strat_node:Node) -> Node{
        self.nodes[strat_node.next_ndx]
    }
    pub fn prev(&self,strat_node:Node) -> Node{
        self.nodes[strat_node.prev_ndx]
    }
    pub fn len(&self) -> usize {
        self.active_nodes.len()
    }
    pub fn contains(&self, index:&usize) -> bool {
        self.active_nodes.contains(index)
    }
}
// Itterators
#[allow(unused)]
impl Nodes {
    pub fn active_nodes_iter(&self) -> std::collections::hash_set::Iter<'_, usize> {
        self.active_nodes.iter()
    }
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
