use std::io::Write;
use std::net::TcpStream;
use std::time::Duration;
use std::process::Command;
use std::thread;
use std::path::Path;

use serde::{Deserialize, Serialize};
use serde_json;

use nalgebra::Point2;
use stl_io::IndexedTriangle;
use stl_io::Vector;

use crate::geo::*;
use crate::gcode;

const BLENDER_HOST: &str = "localhost";
const BLENDER_PORT: u16 = 9000;

#[derive(Debug)]
pub struct Blender {
    tcp:TcpStream
}
#[derive(Serialize, Deserialize)]
pub enum BlenderMsg{
    CreateMesh(BlenderObj),
    LoadSTL(String,String),
    ExportLayers(String),
}
#[derive(Serialize, Deserialize)]
pub struct BlenderObj{
    collection: String,
    name: String,
    vertices: Vec<[f32;3]>,
    edges: Vec<[usize;2]>,
    faces: Vec<Vec<usize>>,
}
impl BlenderObj {
    fn from_mesh<T:Into<String>>(mesh:BlenderMesh,name:T,collection:T) -> Self {
        Self{
            name: name.into(),
            collection: collection.into(),
            vertices: mesh.vertices,
            edges: mesh.edges,
            faces: mesh.faces,
        }
    }
}
pub struct BlenderMesh{
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
    // for _ in 0..MAX_RETRIES {
    for _ in 0..MAX_RETRIES {
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
        Err(_) => {
            launch_blender().unwrap();
            connect_to_blender().expect("could not connect to blender:(")
        }
    }
}
fn launch_blender() -> std::io::Result<()> {
    println!("launching blender");
    let python_server_path = Path::new("../py/server.py");
    Command::new("blender")
        // .arg("--background")
        .arg("--python")
        .arg(python_server_path)
        .spawn()?;
    Ok(())
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
        self.tcp.flush().unwrap();
    }
    /// export layers form blender to disk at the spesified path
    pub fn export_layers(&mut self, path:&str){
        let binding = std::path::absolute(path).unwrap();
        let abs_path = binding.to_string_lossy();
        self.send(BlenderMsg::ExportLayers(abs_path.into()))
    }
    /// loads the stl file at the given path in blender
    pub fn load_mesh<T>(&mut self,path:T ,name:&str)
    where T: std::convert::AsRef<std::path::Path>{
        let binding = std::path::absolute(path).unwrap();
        let abs_path = binding.to_string_lossy();
        self.send(BlenderMsg::LoadSTL(abs_path.into(),name.into()))
    }
    pub fn display2d<O: Into3d<BlenderMesh>,T:Into<String>>(&mut self,obj:O,h:f32,name:T,collection:T) {
        // let mesh = BlenderMesh::into2d(obj, h);
        let blend_obj = BlenderObj::from_mesh(obj.into2d(h),name,collection);
        self.send(BlenderMsg::CreateMesh(blend_obj))
    }
    pub fn display3d<O:Into<BlenderMesh>,T:Into<String>>(&mut self,obj:O,name:T,collection:T) {
        let blend_obj = BlenderObj::from_mesh(obj.into(),name,collection);
        self.send(BlenderMsg::CreateMesh(blend_obj))
    }
    pub fn n_gon(&mut self,vertices:Vec<[f32;3]>, edges:Vec<[usize;2]>, faces:Vec<Vec<usize>> ){
        let name = "n_gon".into();
        let collection = "n_gons".into();
        let obj = BlenderObj { name, collection, vertices, edges, faces };
        self.send(BlenderMsg::CreateMesh(obj))
    }
    pub fn n_gon_result<T:Into<String>>(&mut self,vertices:Vec<[f32;3]>, edges:Vec<[usize;2]>, faces:Vec<Vec<usize>>, name:T){
        let collection = "result".into();
        let obj = BlenderObj { name:name.into(), collection, vertices, edges, faces };
        self.send(BlenderMsg::CreateMesh(obj))
    }
    pub fn path(&mut self, path:&gcode::Path){
        let name = "Path".into();
        let collection = format!("{}",path.path_type);
        self.display3d(path, name, collection);
    }
}

impl From<&crate::gcode::Path> for BlenderMesh {
    fn from(value:&crate::gcode::Path) -> BlenderMesh {
        let vertices:Vec<_> = value.points.iter().map(|p|[p.x,p.y,p.z]).collect();
        let edges = (0..vertices.len()-1).map(|i|[i,i+1]).collect();
        BlenderMesh{vertices,edges,faces:vec![]}
    }
}


trait From2d<T> {
    fn from2d(obj:T,h:f32) -> Self;
}
pub trait Into3d<T>: Sized {
    fn into2d(self,h:f32) -> T;
}
impl<T: From2d<U>, U> Into3d<T> for U {
    fn into2d(self,h:f32) -> T {
        T::from2d(self,h)
    }
}

impl From2d<&Polygon> for BlenderMesh {
    fn from2d(polygon:&Polygon,h:f32) -> Self {
        let mut vertices:Vec<[f32;3]> = Vec::new();
        let mut edges:Vec<[usize;2]> = Vec::new();
        for hole in polygon.contours(){
            let offset = vertices.len();
            vertices.extend(hole.points().map(|p|[p.x,p.y,h]));
            edges.extend((offset..(vertices.len()-1)).map(|i|[i,i+1]));
            edges.push([offset,vertices.len()-1]);
        }
        BlenderMesh{ vertices, edges, faces:vec![] }
    }
}

impl From2d<&Contour> for BlenderMesh {
    fn from2d(contour:&Contour,h:f32) -> Self {
        if contour.points.len() == 0 {println!("Blender Error: attempted to displey empty loop");}
        let vertices: Vec<[f32;3]> = contour.points()
            .map(|p| [ p.x, p.y, h ] )
            .collect();
        let mut edges:Vec<[usize;2]> = (0..vertices.len()-1)
            .map(|i| [i, i+1])
            .collect();
        edges.push([edges.len(),0]);

        return BlenderMesh{ vertices, edges, faces:vec![] }
    }
}

impl From<&Contour3d> for BlenderMesh {
    fn from(contour:&Contour3d) -> Self {
        if contour.0.len() == 0 {println!("Blender Error: attempted to displey empty loop");}
        let vertices: Vec<[f32;3]> = contour.points()
            .map(|p| [ p.x, p.y, p.z ] )
            .collect();
        let mut edges:Vec<[usize;2]> = (0..vertices.len()-1)
            .map(|i| [i, i+1])
            .collect();
        edges.push([edges.len(),0]);

        return BlenderMesh{ vertices, edges, faces:vec![] }
    }
}

impl From<&Polygon3d> for BlenderMesh {
    fn from(polygon:&Polygon3d) -> Self {
        let mut vertices:Vec<[f32;3]> = Vec::new();
        let mut edges:Vec<[usize;2]> = Vec::new();
        for hole in polygon.contours(){
            let offset = vertices.len();
            vertices.extend(hole.points().map(|p|[p.x,p.y,p.z]));
            edges.extend((offset..(vertices.len()-1)).map(|i|[i,i+1]));
            edges.push([offset,vertices.len()-1]);
        }
        BlenderMesh{ vertices, edges, faces:vec![] }
    }
}

impl From<&crate::skeleton::StraightSkeleton> for BlenderMesh {
    fn from(skeleton:&crate::skeleton::StraightSkeleton) -> Self {
        let vertices = skeleton.vertices.iter().map(|p|[p.x,p.y,p.z]).collect();
        let edges = skeleton.edges.clone();
        BlenderMesh{
            vertices,
            edges,
            faces:vec![] 
        }
    }
}

impl From<&crate::skeleton::SkeletonBuilder> for BlenderMesh {
    fn from(skeleton:&crate::skeleton::SkeletonBuilder) -> Self {
        let vertices = skeleton.vertices.iter().map(|v|[v.coords.x,v.coords.y,v.time]).collect();
        let edges = skeleton.edges.iter().map(|e|[e.start,e.end]).collect();
        BlenderMesh{
            vertices,
            edges,
            faces:vec![] 
        }
    }
}
