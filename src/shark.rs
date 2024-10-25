use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    mesh::traits::Mesh,
    voxel::prelude::*,
};
//use nalgebra_glm::Vec3;
use std::path::Path;


pub fn main() {
    let voxel_size = 0.2;

    let stl: PolygonSoup<f32> = StlReader::new()
        .read_stl_from_file(Path::new("../write.stl"))
        .expect("Read mesh");

    //for data in stl.edges(){
    //  println!("edge: {} point1: {:?} point2: {:?}",
    //      data, 
    //      stl.edge_positions(&data).0, 
    //      stl.edge_positions(&data).1
    //      );
    //}

    let mut overhanging_faces = Vec::new();
    for face in stl.faces(){
        let normal_z = stl.face_normal(&face).data.0[0][2];
        if normal_z < 0.0 {
          overhanging_faces.push(face);
        }
    }
    for face in overhanging_faces{
      println!("edge: {} point1: {:?} point2: {:?}",
          face, 
          stl.edge_positions(&face).0, 
          stl.edge_positions(&face).1
          );
    };
    //println!("{overhanging_faces:?}");
    PolygonSoup{
        Vec![[1,2,3]]
    };

    dbg!(&stl);

    StlWriter::new()
        .write_stl_to_file(&stl, Path::new("../overhang.stl"))
        .expect("Should write mesh to STL");
}

fn write_volume_to_stl(volume: &Volume, path: &str) {
    let vertices = MarchingCubesMesher::default()
        .with_voxel_size(volume.voxel_size())
        .mesh(volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new(path))
        .expect("Should write mesh to STL");
}
