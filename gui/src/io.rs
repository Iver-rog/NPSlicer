
use super::Error;
use crate::scene::pipeline::vertex::Vertex;

use stl_io;
use rfd;

use std::path::PathBuf;


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
pub trait IntoVertexBuffer {
    fn into_vertex_buffer(data:Self) -> Vec<Vertex>;
}
impl IntoVertexBuffer for &stl_io::IndexedMesh {
    fn into_vertex_buffer(data:Self) -> Vec<Vertex> {
        let vertices:Vec<glam::Vec3> = data.vertices.iter()
            .map(|v|glam::Vec3{x:v.0[0], y:v.0[1], z:v.0[2]})
            .collect();

        data.faces.iter()
            .flat_map(|f| f.vertices.into_iter().map(move |v| (v,f.normal)))
            .map(|(v,n)| {
                let normal = glam::vec3(n[0], n[1], n[2]);
                Vertex{
                    pos: vertices[v].clone(),
                    normal,
                    tangent: glam::vec3(0.5, 0.5, 0.5),
                    uv: glam::vec2(0.5, 0.5),
                }
            })
            .collect()
    }
}
