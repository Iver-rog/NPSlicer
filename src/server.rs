use core::result::Result;
use std::io::Write;
use std::net::TcpStream;
use std::net::ToSocketAddrs;
use std::thread::sleep;
use std::time::Duration;
use std::process::Command;
use std::thread;

use std::io;

use serde::{Deserialize, Serialize};
use serde_json;

use nalgebra::{Point2,};
use stl_io::IndexedTriangle;
use stl_io::Vector;

use crate::geo::*;
use crate::gcode;

const BLENDER_HOST: &str = "localhost";
const BLENDER_PORT: u16 = 9000;
const BLENDER_ADR: &str = "localhost:9000";

#[derive(Debug)]
pub struct Blender {
    tcp:TcpStream
}
#[derive(Serialize, Deserialize)]
pub enum BlenderMsg{
    CreateMesh(BlenderObj),
    ClearSeene,
}
#[derive(Serialize, Deserialize)]
pub struct BlenderObj{
    collection: String,
    name: String,
    vertices: Vec<[f32;3]>,
    edges: Vec<[usize;2]>,
    faces: Vec<Vec<usize>>,
}
fn connect_to_blender() -> Result<Blender,String>{
    const MAX_RETRIES: u32 = 5;
    const RETRY_DELAY_MS: u64 = 500;
    for i in 0..MAX_RETRIES {
        if let Ok(tcp) = TcpStream::connect((BLENDER_HOST, BLENDER_PORT)) {
            return Ok(Blender{tcp})
        }
        thread::sleep(Duration::from_millis(RETRY_DELAY_MS));
    }
    Err("could_not_connect_to_blender :(".into())
}
fn connect_to_blender2() -> Result<TcpStream,String>{
    const MAX_RETRIES: u32 = 5;
    const RETRY_DELAY_MS: u64 = 500;
    for i in 0..MAX_RETRIES {
        if let Ok(tcp) = TcpStream::connect((BLENDER_HOST, BLENDER_PORT)) {
            return Ok(tcp)
        }
        thread::sleep(Duration::from_millis(RETRY_DELAY_MS));
    }
    Err("could_not_connect_to_blender :(".into())
}
fn connect_or_launch_blender() -> Blender {
    match connect_to_blender() {
        Ok(blender) => {
            println!("connected to blender");
            return blender
        },
        Err(error) => {
            launch_blender().unwrap();
            connect_to_blender().expect("could not connect to blender:(")
        }
    }
}
fn launch_blender() -> std::io::Result<()> {
    println!("launching blender");
    Command::new("blender")
        // .arg("--background")
        .arg("--python")
        .arg("/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/py/server.py")
        .spawn()?;
    Ok(())
}
#[test]
fn blender_tcp_connection_test(){
    let mut b = Blender::new();
    let mut msg = BlenderMsg::CreateMesh(
        BlenderObj { 
            collection: "test_collection".into(), 
            name: "tset_obj_halla".into(),
            vertices: vec![[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0]],
            edges: vec![[0,1],[1,2],[2,0]],
            faces: vec![vec![0,1,2] ]
        }
    );
    b.send(msg);
    assert!(false)
}

impl Blender {
    pub fn new() -> Blender{
        connect_or_launch_blender()
    }
    fn send(&mut self, msg: BlenderMsg) {
        let json = serde_json::to_vec(&msg).unwrap();
        let len = (json.len() as u32).to_be_bytes(); // 4 bytes, big endian
        match self.tcp.write_all(&len){
            Ok(_) => (),
            Err(error) => {
                println!("Connection lost: {error}");
                self.tcp = connect_to_blender2().unwrap();
                self.tcp.write_all(&len).unwrap();
            }
            
        };
        self.tcp.write_all(&json).unwrap();
        // self.tcp.write_all(b"\n").unwrap();  // Important: delimiter
        self.tcp.flush().unwrap();
    }
    // pub fn send(&mut self, obj:BlenderMsg) {
    //     let msg = serde_json::to_string(&obj).unwrap();
    //     self.tcp.write_all(msg.as_bytes()).unwrap();
    //     self.tcp.write_all(b"\n").unwrap();
    // }
    pub fn save_mesh(&self, tris:&Vec<IndexedTriangle>, vertices:&Vec<Vector<f32>>,name:String){

        // todo!()
    }
    pub fn show(self){
        todo!()
    }
    pub fn apply_boolean(self){
        // todo!()
    }
    // fn create_python_lib(self) -> Result<(),io::Error>{
    //     todo!()
    // }
    pub fn n_gon(&mut self,vertices:Vec<[f32;3]>, edges:Vec<[usize;2]>, faces:Vec<Vec<usize>> ){
        let name = "TEST".into();
        let collection = "collection_name".into();
        let obj = BlenderObj { name, collection, vertices, edges, faces };
        self.send(BlenderMsg::CreateMesh(obj))
    }
    pub fn path(&mut self, path:&gcode::Path){
        todo!()
    }
    pub fn polygon3d(&mut self, polygon:&Polygon3d) {
        todo!()
    }
    pub fn polygon(&mut self, polygon:&Polygon,h:f32) {
        let obj = blender_obj_from_polygon(polygon, h,"polygon","debug");
        self.send(BlenderMsg::CreateMesh(obj));
    }
    pub fn contour(&mut self, contour:&Contour,h:f32){
        todo!()
    }
    pub fn contour3d(&mut self, contour:&Contour3d){
        todo!()
    }
    pub fn edge_loop(&mut self, edge_loop:&Vec<usize>,stl:&stl_io::IndexedMesh){
        todo!()
    }
    pub fn edge_loop_points(&mut self, edge_loop:&Vec<[f32;3]>){
        todo!()
    }
    pub fn line(&mut self, points:&Vec<Point2<f32>>,z_height:f32){
        todo!()
    }
    pub fn line_body2d(&mut self,points:&Vec<[f32;2]>,edges:Vec<[usize;2]>) {
        todo!()
    }
    pub fn line_body3d(&mut self,points:Vec<[f32;3]>,edges:Vec<[usize;2]>) {
        todo!()
    }
    pub fn line_body_points(&mut self, edges:&Vec<[[f32;2];2]>) {
        todo!()
    }
}
fn blender_obj_from_polygon<T:Into<String>>(polygon:&Polygon,h:f32,name:T,collection:T) -> BlenderObj {
    let mut vertices:Vec<[f32;3]> = Vec::new();
    let mut edges:Vec<[usize;2]> = Vec::new();
    for hole in polygon.contours(){
        let offset = vertices.len();
        vertices.extend(hole.points.iter().map(|p|[p.x,p.y,h]));
        edges.extend((offset..(vertices.len()-1)).map(|i|[i,i+1]));
        edges.push([offset,vertices.len()-1]);
    }
    BlenderObj{
        collection:collection.into(),
        name:name.into(),
        vertices,
        edges,
        faces:vec![],
    }
}
impl From<&Polygon> for BlenderObj {
    fn from(polygon:&Polygon) -> Self {
        blender_obj_from_polygon(polygon, 0.0,"polygon","debug")
    }
}
