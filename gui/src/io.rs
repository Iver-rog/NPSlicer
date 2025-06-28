
use rfd;
use std::path::PathBuf;
use super::Error;
use tokio::fs::File;
use tokio::io::BufReader;
use tokio::io;

use stl_io;

use std::path::Path;

pub async fn pick_file() -> Result<PathBuf,Error>{
    rfd::AsyncFileDialog::new()
        .set_title("Choose a STL file for slicing")
        .add_filter("stl", &["stl"])
        .add_filter("g-code", &["gcode"])
        .pick_file()
        .await
        .map(|file_handle|PathBuf::from(file_handle))
        .ok_or(Error::DialogClosed)
}
pub async fn load_stl<T:AsRef<Path>>(path:T) -> Result<(),io::Error> {
    // let mut file = File::open(path).await?;
    // let mut reader = BufReader::new(file);
    let file = std::fs::File::open(path)?;
    let mut reader = std::io::BufReader::new(file);
    let mesh = stl_io::read_stl(&mut reader)?;
    Ok(())
}
pub struct VertexBuffer { 
        data:Vec<Vertex>,
    }
pub trait IntoVertexBuffer {
    fn into_vertex_buffer(data:Self) -> VertexBuffer;
}
impl IntoVertexBuffer for stl_io::IndexedMesh {
    fn into_vertex_buffer(data:Self) -> VertexBuffer {
        let vertices:Vec<glam::Vec3> = data.vertices.into_iter()
            .map(|v|glam::Vec3{x:v.0[0], y:v.0[1], z:v.0[2]})
            .collect();
        let buffer = data.faces.into_iter()
            .flat_map(|f| f.vertices.into_iter().map(move |v| (v,f.normal)))
            .map(|(v,n)| {
                let normal = glam::vec3(n[0], n[1], n[2]);
                Vertex{
                    pos: vertices[v].clone(),
                    normal,
                    color: glam::vec4(0.5, 0.5, 0.5, 1.0),
                }
            })
            .collect();
        return VertexBuffer{ data:buffer }
    }
}
pub struct Vertex {
    pos: glam::Vec3,
    normal: glam::Vec3,
    color: glam::Vec4
}

pub struct IndexedMesh {
    vertices: Vec<glam::Vec3>,
    triangles: Vec<IndexedFace>,
}
pub struct IndexedFace {
    vertices: glam::UVec3,
    normal: glam::Vec3,
}

impl From<stl_io::IndexedMesh> for IndexedMesh {
    fn from(stl:stl_io::IndexedMesh) -> Self {
        let vertices:Vec<glam::Vec3> = stl.vertices.into_iter()
            .map(|v|glam::Vec3{x:v.0[0], y:v.0[1], z:v.0[2]})
            .collect();
        let faces:Vec<IndexedFace> = stl.faces.into_iter()
            .map(|f|f.into())
            .collect();
        todo!()
    }
}

impl From<stl_io::IndexedTriangle> for IndexedFace {
    fn from(tri:stl_io::IndexedTriangle) -> Self {
        let v = tri.vertices;
        let n = tri.normal;
        Self{
            vertices: glam::UVec3{x:v[0] as u32, y:v[1] as u32, z:v[2] as u32},
            normal: glam::Vec3{x:n.0[0], y:n.0[1], z:n.0[2]}
        }
    }
}
