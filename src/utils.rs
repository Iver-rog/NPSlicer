#![allow(unused)]
use std::{os::unix::process::CommandExt, process::Command};
use std::fs::{self, File, OpenOptions};
use std::path::Path;
use std::io::{self, prelude::*, BufWriter};
use std::fmt::Display;
use stl_io::{IndexedTriangle, Triangle, Vector};

#[derive(Debug)]
pub struct Blender<'a> {
    tmp_path:Box<Path>,
    mesh_path:Box<Path>,
    input_mesh:Option<&'a Path>,
    output_meshes:Vec<&'a Path>,
    line_objects:Vec<(Vec<[f32;3]>,Vec<[usize;2]>)>
}

impl <'a> Blender<'a> {
    pub fn new()->Blender<'a>{

        let tmp = if Path::new("./tmp").exists(){
            Path::new("./tmp")
        } else if Path::new("../tmp").exists(){
            Path::new("../tmp")
        } else {
            panic!("could not find tmp directory");
        };
        fs::remove_dir_all(tmp).unwrap();
        fs::create_dir(tmp).unwrap();
        let mesh = tmp.join("mesh");
        fs::create_dir(mesh.clone()).unwrap();
        println!("{tmp:?}");

        return Blender{
            tmp_path:tmp.into(),
            mesh_path:mesh.into(),
            input_mesh:None,
            output_meshes:Vec::new(),
            line_objects:Vec::new()
        }
    }
    pub fn save_mesh(&self, tris:&Vec<IndexedTriangle>, vertices:&Vec<Vector<f32>>,name:String){
        let out:Vec<Triangle> = tris.iter()
            .map(|tri|
              Triangle {
                normal: tri.normal,
                vertices: [
                  vertices[tri.vertices[0]],
                  vertices[tri.vertices[1]],
                  vertices[tri.vertices[2]],
                ]
              }
            )
            .collect();

        let file_path = self.mesh_path.join(name+".stl");
        let mut file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(file_path)
            .expect("Failed to create output file");

        stl_io::write_stl(&mut file, out.iter()).unwrap();
    }

    pub fn show(self){
        // Create Python library for blender script to open
        self.create_python_lib();
        // Launch blender and plot output generated by this program
        let python_launch_script = "/home/iver/Documents/NTNU/prosjekt/layer-gen-rs/mesh/view.py";
        let _ = Command::new("blender")
            .arg("-P")
            .arg(python_launch_script)
            .exec();
    }

    fn create_python_lib(self) -> Result<(),io::Error>{
        let path_name = self.tmp_path.join("layer_gen_data.py");
        let file = File::create(path_name)
            .expect("could not create layer_gen_data.py file");
        let mut f = BufWriter::new(file);

        // ============ Line bodies ==============
        writeln!(f,"def get_points():")?;
        writeln!(f,"    return [")?;
        for (i,line_object) in self.line_objects.iter().enumerate() {
            write_points(&mut f,line_object.0.to_vec());
            if i != self.line_objects.len()-1 { writeln!(f,",")?; } 
            else { writeln!(f,"")?; }
        }

        writeln!(f,"    ]")?;
        writeln!(f,"def get_lines():")?;
        writeln!(f,"    return [")?;
        for (i,line_object) in self.line_objects.iter().enumerate(){
            write_lines(&mut f,line_object.1.to_vec());
            if i != self.line_objects.len()-1 { writeln!(f,",")?;} 
            else { writeln!(f,"")?; }
        }
        writeln!(f,"    ]")?;

        // ============ mesh directory ==============
        writeln!(f,"def get_mesh_dir():")?;
        let mesh_path = self.mesh_path.to_str().unwrap();
        println!("mesh path: {mesh_path}");
        writeln!(f,"    return '{mesh_path}' ")?;
        return Ok(())
    }

    pub fn edge_loop(&mut self, edge_loop:&Vec<usize>,stl:&stl_io::IndexedMesh){
        let points: Vec<[f32;3]>= edge_loop.clone().into_iter()
            .map(|index| { [
                stl.vertices[index][0],
                stl.vertices[index][1],
                stl.vertices[index][2]
            ] } )
            .collect();
        let mut edges:Vec<[usize;2]> = (0..points.len()-1)
            .map(|i| [i, i+1])
            .collect();
        edges.push([edges.len(),0]);

        self.line_objects.push( (points, edges) );
    }

    pub fn edge_loop_points(&mut self, edge_loop:&Vec<[f32;3]>){
        let points: Vec<[f32;3]>= edge_loop.clone().into_iter()
            .map(|[x,y,z]| { [ x, y, z ] } )
            .collect();
        let mut edges:Vec<[usize;2]> = (0..points.len()-1)
            .map(|i| [i, i+1])
            .collect();
        edges.push([edges.len(),0]);

        self.line_objects.push( (points, edges) );
    }
    pub fn line_body(&mut self,points:&Vec<[f32;2]>,edges:Vec<[usize;2]>) {
        let points3d = points.clone().into_iter().map(|[p1,p2]|[p1,p2,0.0]).collect();
        self.line_objects.push((points3d,edges.clone()));
    }
    pub fn line_body_points(&mut self, edges:&Vec<[[f32;2];2]>) {
        let points3d: Vec<[f32;3]> = edges.iter().flatten()
            .map(|[ x, y ]| [*x, *y, 0.0] )
            .collect();

        let mut edges_as_ref = (0..edges.len())
            .into_iter()
            .map(|i| [2*i ,2*i+1] )
            .collect();

        self.line_objects.push((points3d,edges_as_ref));
    }
}

pub fn print_component_info(components: &[Vec<IndexedTriangle>]) {
    println!("Found {} separate mesh components:", components.len());
    for (i, component) in components.iter().enumerate() {
        println!("Component {} contains {} triangles", i + 1, component.len());
    }
}

fn write_points<T:Display,W: ?Sized + Write>(f:&mut io::BufWriter<W> ,points:Vec<[T;3]>)->Result<(),io::Error>{

        writeln!(f,"        [")?;
        for (i,vertex) in points.iter().enumerate(){
            write!(f,"          ({},{},{})",
                vertex[0],
                vertex[1],
                vertex[2],
                )?;
            if i != points.len()-1 {
                writeln!(f,",")?;
            }else{
                writeln!(f,"")?;
            }
        }
        write!(f,"        ]")?;
        return Ok(())
}
fn write_lines<W: ?Sized + Write>(f:&mut io::BufWriter<W>, edges:Vec<[usize;2]>)->Result<(),io::Error>{
    writeln!(f,"        [")?;
    for (i,edge) in edges.iter().enumerate() {
        write!(f,"            ({},{})",
            edge[0],
            edge[1]
            )?;
        if i != edges.len()-1 { writeln!(f,",")?; }
        else { writeln!(f,"")?; }
    }
    write!(f,"        ]")?;
    return Ok(())
}
