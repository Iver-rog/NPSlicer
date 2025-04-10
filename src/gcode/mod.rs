use nalgebra::{Point2, Point3};
use stl_io::IndexedMesh;

use core::iter::Iterator;
use core::{f32, isize};
use std::io::BufReader;
use std::fs::{self,File};
use std::iter;

use crate::geo::*;
use crate::stl_op;

mod projection;
use projection::MeshCollider;

mod gcodefile;
use gcodefile::{GcodeFile,Settings};

pub enum ToolPathType{
    Travel,
    Perimeter,
    Infill,
}
pub struct ToolPathLine{
    line:Vec<Point3<f32>>,
    line_type:ToolPathType,
}
pub fn contour_and_mesh_colider_from_mesh(mesh:stl_io::IndexedMesh) -> (Vec<Polygon>,MeshCollider){
    let (indexed_contours,indexed_edges) = stl_op::extract_perimeters_and_edges(&mesh.faces);
    // indexed_contours.iter().for_each(|contour| blender.edge_loop(contour, &mesh) );

    let vertices = mesh.vertices.into_iter().map(|v|Point3::new( v[0],v[1],v[2]) ).collect();

    let mesh_collider = MeshCollider{
        faces:mesh.faces,
        edges:indexed_edges,
        vertices,
    };

    let mut contours:Vec<Contour> = indexed_contours.into_iter()
        .map(|indexed_contour|{
            indexed_contour.into_iter()
                .map(|vertex_ndx|{
                    let v = mesh_collider.vertices[vertex_ndx];
                    Point2::new(v[0],v[1])
                })
                .collect::<Vec<Point2<f32>>>()
        })
        .map(|contour| Contour::from(contour) )
        .collect();

    contours.iter_mut().for_each(|contour| contour.simplify(0.05));

    let polygons = polygons_from_contours(contours);
    return (polygons,mesh_collider)
}
pub fn generate_perimeter_offsets(polygon:Polygon,nr_of_perimeters:usize,layer_w:f32) -> impl Iterator<Item = Polygon>{
    iter::repeat(polygon.clone())
        .take(nr_of_perimeters)
        .enumerate()
        // .flat_map(|(i,polygons)| polygons.into_iter().map(move |polygon|(i,polygon)))
        .flat_map(move |(i,polygon)| polygon.offset(-((i as f32 + 0.5)*layer_w)).into_iter() )
}

pub fn main(blender:&mut crate::Blender) {
    // let mut args = env::args();
    // let path = args.next().expect("first arg should be the path");
    // let mesh_layer_dir = args.next().expect("missing argument: stl-layers directory");
    let layer_h = 0.2;
    let layer_w = 0.21;
    let nr_of_perimeters = 2;
    let brim = 5;

    let mesh_layer_dir = "curving_overhang/p0.2";
    let mut mesh_layers = import_layers(&mesh_layer_dir)
        .map(|layer_mesh| contour_and_mesh_colider_from_mesh(layer_mesh));

    println!("layer: 0");
    let (first_layer_perimeters,_) = mesh_layers.next().expect("missing first layer");

    first_layer_perimeters.into_iter()
            .flat_map(|polygon|{
                iter::repeat(polygon.clone())
                    .take(nr_of_perimeters + brim)
                    .enumerate()
            })
            .flat_map(|(i,polygon)| {
                let offset = (( (brim as isize)-(i as isize) )as f32 - 0.5)*layer_w;
                polygon.offset(offset).into_iter() 
            })
            .for_each(|polygon| blender.polygon(&polygon, layer_h) );

    let remaining_layer_perimeters = mesh_layers.enumerate()
        .map(|(layer_nr,(mut polygons, mesh_collider))|{
            polygons.into_iter()
                .flat_map(|polygon|{
                    iter::repeat(polygon.clone())
                        .take(nr_of_perimeters)
                        .enumerate()
                })
                .flat_map(|(i,polygon)| polygon.offset(-((i as f32 + 0.5)*layer_w)).into_iter() )
                .map(|polygon| polygon.project_onto(&mesh_collider) )
                .for_each(|polygon3d| blender.polygon3d(&polygon3d) );
        });

    // for (layer_nr,(mut polygons, mesh_collider)) in mesh_layers.enumerate(){
    //     println!("layer: {layer_nr}");
    //     // blender.save_mesh(&mesh.faces,&mesh.vertices,format!("mesh_layer: {layer_nr}"));
    //
    //     // let (mut polygons, mesh_collider) = contour_and_mesh_colider_from_mesh(mesh);
    //
    //     // polygons.iter_mut().for_each(|polygon| polygon.set_start());
    //
    //     // polygons.iter().for_each(|polygon|blender.polygon(polygon, 10.0));
    //
    //     polygons.into_iter()
    //         .flat_map(|polygon|{
    //             iter::repeat(polygon.clone())
    //                 .take(nr_of_perimeters)
    //                 .enumerate()
    //         })
    //         .flat_map(|(i,polygon)| polygon.offset(-((i as f32 + 0.5)*layer_w)).into_iter() )
    //         .map(|polygon| polygon.project_onto(&mesh_collider) )
    //         .for_each(|polygon3d| blender.polygon3d(&polygon3d) );
    // }
}
#[test]
fn infill_test(){
    let vertices = vec![
        Point3::new( 0.0,  0.0,  0.0),
        Point3::new(-2.5, -2.5, -2.5),
        Point3::new( 1.5,  1.5,  1.5),
    ];
    let edges = vec![
        // IndexedEdge(0,1),
        IndexedEdge(1,2),
    ];

    let mesh = MeshCollider { faces:vec![], edges, vertices, };

    let bounds = Polygon::from_unchecked(vec![Contour::from(vec![
        Point2::new(0.0, 0.0),
        Point2::new(10.0, 0.0),
        Point2::new(10.0, 10.0),
        Point2::new(0.0, 10.0),
    ])]);

    generate_infill(bounds, mesh, true);

}

fn generate_infill(bounds:Polygon,mesh:MeshCollider,allong_x:bool)->(){
    let aabb = &bounds.outer_loop().aabb;
    let (min,max) = ( aabb.x_min.floor() as isize, aabb.x_max.ceil() as isize);
    let range = (max-min) as usize;
    let mut yz_vals:Vec<Vec<(f32,Option<f32>)>> = vec![Vec::new();range];

    for (e1,e2) in mesh.edges.iter()
        .map(|edge|(mesh.vertices[edge.0],mesh.vertices[edge.1]))
        .map(|(e1,e2)| if e1.x < e2.x { (e1,e2) }else{ (e2,e1) }){
            let ay = (e2.y-e1.y)/(e2.x-e1.x);
            let az = (e2.z-e1.z)/(e2.x-e1.x);

            let residual = e1.x%e1.x.ceil();
            let x_range = ((e2.x.floor() as isize) - (e1.x.ceil() as isize)) as usize;

            let y = if ay == 0.0 { vec![e1.y; x_range] }else{
                (0..=x_range).into_iter().map(|x|x as f32)
                    .map(|x|ay*(x-residual)+e1.y).collect()
            };
            let z = if az == 0.0 { vec![e1.z; x_range] }else{
                (0..=x_range).into_iter().map(|x|x as f32)
                    .map(|x|az*(x-residual)+e1.z).collect()
            };
            y.into_iter().zip(z.into_iter()).enumerate()
                .map(|(x,(y,z))|{ 
                    let x_ndx = (x as isize) + (e1.x.ceil() as isize + min);
                    println!("{x_ndx}");
                    (x_ndx,y,z)
                })
                // .filter(|(x_ndx,y,z)| x_ndx.is_negative())
                .filter_map(|(x_ndx,y,z)| 
                    if let Ok(x_ndx) = usize::try_from(x_ndx){ Some((x_ndx,y,z)) }else{None}
                )
                .filter(|(x_ndx,y,z)| *x_ndx < range)
                .for_each(|(x_ndx,y,z)| yz_vals[x_ndx].push((y,Some(z))) )

            // for x in (0..=x_range).into_iter().map(|x| (x as f32)  ){//+e1.x.ceil()) {
            //     let y = ay*(x-residual)+e1.y;
            //     let z = az*(x-residual)+e1.z;
            //     println!("n:{x} x:{:.2} y:{y:.2} z:{z:.2}",(x as f32)+e1.x.ceil());
            // }
    }

    // Add contour intersections
    for (e1,e2) in bounds.all_edges()
        .map(|(e1,e2)| if e1.x < e2.x { (e1,e2) }else{ (e2,e1) }){
            let ay = (e2.y-e1.y)/(e2.x-e1.x);

            let residual = e1.x%e1.x.ceil();
            let x_range = ((e2.x.floor() as isize) - (e1.x.ceil() as isize)) as usize;

            let y = if ay == 0.0 { vec![e1.y; x_range] }else{
                (0..x_range).into_iter().map(|x|x as f32)
                    .map(|x|ay*(x-residual)+e1.y).collect()
            };

            y.into_iter().enumerate()
                .map(|(x,y)|{ 
                    let x_ndx = (x as isize) + (e1.x.ceil() as isize + min);
                    (x_ndx,y)
                })
                .filter_map(|(x_ndx,y)| 
                    if let Ok(x_ndx) = usize::try_from(x_ndx){ Some((x_ndx,y)) }else{None}
                )
                // .filter(|(x_ndx,y)| *x_ndx < range)
                .for_each(|(x_ndx,y)| yz_vals[x_ndx].push((y,None)) )
        }
    for collumn in yz_vals {
        println!("");
        for (y,z) in collumn{
            match z {
                Some(z) => print!(" y:{y:.1} z:{z:.1} |"),
                None => print!(" y:{y:.1} z:None|"),
            }
        }
    }
    // dbg!(yz_vals);
    todo!()
}

impl Polygon {
    fn set_start(&mut self) {
        self.0.iter_mut().for_each(|contour| contour.set_start() );
    }
}

impl Contour {
    fn set_start(&mut self) {
        let start_point_ndx = self.points()
            .map(|p|p.x+p.y)
            .enumerate()
            .max_by(|(_,a),(_,b)|a.total_cmp(b))
            .map(|(index,_)| index)
            .expect("contour contains NAN coordinates");

        self.points.rotate_left(start_point_ndx);
    }
}

pub fn import_layers(layer_dir:&str) -> impl Iterator<Item = IndexedMesh> {
    let paths = fs::read_dir(format!("../{}",layer_dir)).expect("could not find directory");

    let mut stl_files:Vec<_> = paths.filter_map(|path|path.ok())
        // .map(|entry| entry.path() )
        .filter(|path| path.file_type().unwrap().is_file() )
        .filter(|path| path.path().extension().unwrap() == "stl" )
        .collect();

    stl_files.sort_by_key(|dir|dir.file_name());

    stl_files.into_iter()
        .map(|file| File::open(file.path()).unwrap() )
        .map(|file| BufReader::new(file) )
        .map(|mut reader| stl_io::read_stl(& mut reader).expect("Failed to parse STL file") )
}
