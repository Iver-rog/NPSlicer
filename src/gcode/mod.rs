use nalgebra::{Point2, Point3, Rotation3, Similarity3, Vector3};
use stl_io::IndexedMesh;

use std::f32::consts::PI;
use std::iter::Iterator;
use std::io::BufReader;
use std::fs::{self, File};
use std::iter;

use crate::geo::*;
use crate::stl_op;
use crate::Blender;

mod projection;
use projection::MeshCollider;

mod gcodefile;
use gcodefile::{GcodeFile,Settings};

mod path;
pub use path::Path;

#[cfg(test)]
mod test;

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

pub fn main(blender:&mut Blender) {
    // let mut args = env::args();
    // let path = args.next().expect("first arg should be the path");
    // let mesh_layer_dir = args.next().expect("missing argument: stl-layers directory");
    let mut settings = Settings::default();

    let layer_h = 0.2;
    // let layer_w = 0.21;
    let layer_w = settings.perimeter_line_width;
    let nr_of_perimeters = 2;
    let brim = 25;
    // let infill_scale = (settings.infill_percentage as f32/100.0)*settings.perimeter_line_width;
    let infill_scale = 1.0;

    // let mesh_layer_dir = "../curving_overhang/p0.2";
    let mesh_layer_dir = "../simple_overhang";
    // let mesh_layer_dir = "../wine_glass";
    // let mesh_layer_dir = "../flat_wine_glass";

    let mut mesh_layers = import_layers(&mesh_layer_dir)
        .map(|layer_mesh| contour_and_mesh_colider_from_mesh(layer_mesh));

    println!("layer: 0");
    let (first_layer_perimeters,_) = mesh_layers.next().expect("missing first layer");
    let mut gcodefile = GcodeFile::new(&mesh_layer_dir,&settings);

    let first_layer_infill = first_layer_perimeters.clone().into_iter()
        .flat_map(|polygon| polygon.offset(-(nr_of_perimeters as f32)*layer_w))
        .flat_map(|polygon| generate_infill_allong_x_2d(polygon, layer_h , 1./settings.perimeter_line_width) );

    let first_layer_paths = first_layer_perimeters.into_iter()
            .flat_map(|polygon|{
                iter::repeat(polygon.clone())
                    .take(nr_of_perimeters + brim)
                    .enumerate()
            })
            .flat_map(|(i,polygon)| {
                let offset = (((brim as isize) - (i as isize)) as f32 - 0.5)*layer_w;
                polygon.offset(offset).into_iter() 
            })
            .flat_map(|polygon| polygon.0.into_iter() )
            .map(|contour| Contour3d::from_contour(contour,layer_h) )
            .map(|contour3d| Path::from(contour3d) )
            .map(|mut path|{
                path.shorten_ends(settings.perimeter_line_width/2.);
                return path
            })
            .chain(first_layer_infill)
            .inspect(|path| blender.path(&path) );

    gcodefile.layer(first_layer_paths);

   
    for (layer_nr,(mut polygons, mesh_collider)) in mesh_layers.enumerate()
        .inspect(|(layer_nr,_)|{println!("layer: {}",layer_nr+1);}){

            let mesh_collider_copy = mesh_collider.clone();

            let infill:Vec<Path> = polygons.iter().cloned()
                .flat_map(|polygon| polygon.offset(-((nr_of_perimeters as f32) + 0.5) *layer_w).into_iter() )
                .flat_map(|offset_polygon|{
                    let angle = if layer_nr%2 == 0 {PI*0.5}else{0.0}; 
                    generate_3d_infill(offset_polygon, &mesh_collider_copy,angle,infill_scale)
                    })
                .collect();

            let layer_paths = polygons.into_iter()
                .flat_map(|polygon|{
                    iter::repeat(polygon.clone())
                        .take(nr_of_perimeters)
                        .enumerate()
                })
                .flat_map(|(i,polygon)| polygon.offset(-((i as f32 + 0.5)*layer_w)).into_iter() )
                .map(move |polygon| polygon.project_onto(&mesh_collider) )
                .flat_map(|polygon| polygon.0.into_iter() )
                .map(|contour3d| Path::from(contour3d) )
                .map(|mut path|{
                    path.shorten_ends(settings.perimeter_line_width/2.);
                    return path
                    })
                .chain(infill.into_iter())
                .inspect(|path|blender.path(&path));

            gcodefile.layer(layer_paths);

        }



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


#[derive(Copy,Clone,Debug)]
enum PntType {
    Perimeter,
    Inside
}

fn sample_points<'a,I>(
    edges:I,
    aabb:&AABB,
    range:usize,
    min:isize,
    yz_vals:&mut Vec<Vec<(f32,f32,PntType)>>,
    tag:PntType)
where 
    I: Iterator<Item = (Point3<f32>,Point3<f32>)> + 'a,
{
    for (e1,e2) in edges
        // .map(|edge|(mesh.vertices[edge.0],mesh.vertices[edge.1]))
        .map(|(e1,e2)| if e1.x < e2.x { (e1,e2) }else{ (e2,e1) }){

            let residual = e1.x.ceil()-e1.x;

            // f(x) = a*x + b
            let ay = (e2.y-e1.y)/(e2.x-e1.x);
            let by = (e1.y) + ay*residual;
            let az = (e2.z-e1.z)/(e2.x-e1.x);
            let bz = (e1.z) + az*residual;

            let (x_min,x_max) = ((e1.x.ceil() as isize) , (e2.x.floor() as isize));

            let x_vals = (0..(1+x_max - x_min)).map(|x| x as f32);

            let y = match ay == 0.0 {
                true => vec![e1.y; x_vals.len()],
                false => x_vals.clone().map(|x| ay * x + by ).collect(),
            };

            let z = match az == 0.0 {
                true => vec![e1.z; x_vals.len()],
                false => x_vals.clone().map(|x| az * x + bz ).collect(),
            };

            y.into_iter().zip(z.into_iter())
                .enumerate()
                .map(|(i,(y,z))|{ 
                    let x_ndx = i as isize + x_min - min;
                    (x_ndx,y,z)
                })
                .filter_map(|(x_ndx,y,z)| usize::try_from(x_ndx).ok().map(|x_ndx|(x_ndx,y,z)) )
                .filter(|(x_ndx,y,z)| *x_ndx < range+1)
                .filter(|(x_ndx,y,z)| (aabb.y_min <= *y) && (*y <= aabb.y_max) )
                .for_each(|(x_ndx,y,z)| yz_vals[x_ndx].push((y,z,tag)) )
    }
}

// Generates infill paths inside the bounds.
// scale = 1/line_spacing
// Rotation is the angle in radians between the x-axis and the direction of the infill.
fn generate_3d_infill(bounds:Polygon,mesh:&MeshCollider,angle:f32 ,scale:f32) -> Vec<Path> {

    let mut bounds3d = bounds.project_onto(mesh);
    bounds3d.rotate_scale(angle,scale);

    let rot = Rotation3::from_axis_angle(&Vector3::z_axis(),angle);
    // Similarity3::f
    // let transform = scale * rot;
    let translation = Vector3::zeros();
    let axisangle = Vector3::z();
    let transform = Similarity3::new(translation,axisangle,scale);
    let reverse_transform = Similarity3::new(translation,-axisangle,1./scale);

    let aabb = &bounds3d.outer_loop().aabb();
    let (min,max) = ( aabb.x_min.floor() as isize, aabb.x_max.ceil() as isize);

    let offset = -aabb.x_min.floor();
    let range = (max-min) as usize;
    let mut yz_vals:Vec<Vec<(f32,f32,PntType)>> = vec![Vec::new();range+1];

    let edges = mesh.edges.iter()
        .map(|edge|(&mesh.vertices[edge.0],&mesh.vertices[edge.1]))
        .map(|(e1,e2)| (transform.transform_point(e1),transform.transform_point(e2)) );

    let tag = PntType::Inside;

    sample_points(edges, aabb, range, min, &mut yz_vals, tag);

    let edges = bounds3d.all_edges()
        .map(|(e1,e2)|(e1*1.,e2*1.));
    let tag = PntType::Perimeter;

    sample_points(edges, aabb, range, min, &mut yz_vals, tag);

    // sort the tables by y value in alternating increasing/decrasing order:
    for (i,collumn) in yz_vals.iter_mut().enumerate(){
        if i%2 == 0 {
            collumn.sort_unstable_by(|(y1,_,_),(y2,_,_)| y1.partial_cmp(y2).unwrap());
        } else {
            collumn.sort_unstable_by(|(y1,_,_),(y2,_,_)| y2.partial_cmp(y1).unwrap());
        }
    }

    let mut collums = Vec::new();
    for (x,yz) in yz_vals.into_iter()
        .enumerate()
        .map(|(x,yz)| ((x as isize + min) as f32 ,yz)){

        let mut segment = Vec::new();
        let mut within_bounds = false;

        for (y,z,tag) in yz {
            let point = rot.inverse_transform_point(&(Point3::new(x,y,z)*1./scale));
            match tag {
                // found a mesh point keep it if it is inside the contur
                PntType::Inside => if within_bounds {
                    segment.push( point );
                },
                // point on the bounding contour which implies crossing 
                // the boundary between inside and outside the bounds
                PntType::Perimeter => {
                    segment.push( point );
                    if within_bounds { // completed the line segment
                        collums.push( Path{points:segment} );
                        segment = Vec::new();
                    }
                    within_bounds = !within_bounds;
                },
            };
        }
    }
    return collums

}

fn generate_infill_allong_x_2d(bounds:Polygon, height:f32, scale:f32) -> Vec<Path> {
    let aabb = &bounds.outer_loop().aabb;
    let (min,max) = ( (aabb.x_min*scale).floor() as isize, (aabb.x_max*scale).ceil() as isize);

    let range = (max-min) as usize;
    let mut y_vals:Vec<Vec<f32>> = vec![Vec::new();range+1];


    // Add contour intersections
    for (e1,e2) in bounds.all_edges()
        .map(|(e1,e2)| (e1*scale,e2*scale))
        .map(|(e1,e2)| if e1.x < e2.x { (e1,e2) }else{ (e2,e1) }){

            let residual = e1.x.ceil()-e1.x;

            let ay = (e2.y-e1.y)/(e2.x-e1.x);
            let by = (e1.y) + ay*residual;

            let (x_min,x_max) = ((e1.x.ceil() as isize) , (e2.x.ceil() as isize));

            let x_vals = (0..(x_max - x_min)).map(|x| x as f32);

            let y = match ay == 0.0 {
                true => vec![e1.y; x_vals.len()],
                false => x_vals.clone().map(|x| ay * x + by ).collect(),
            };

            y.into_iter()
                .enumerate()
                .map(|(i,y)|{ 
                    let x_ndx = i as isize + x_min - min;
                    (x_ndx,y)
                })
                .filter_map(|(x_ndx,y)| usize::try_from(x_ndx).ok().map(|x_ndx|(x_ndx,y)) )
                .filter(|(x_ndx,y)| *x_ndx < range+1)
                .filter(|(x_ndx,y)| (aabb.y_min*scale <= *y) && (*y <= aabb.y_max*scale) )
                .for_each(|(x_ndx,y)| y_vals[x_ndx].push(y) )
        }

    // sort the table:
    let mut paths = Vec::new();
    for (i,collumn) in y_vals.iter_mut().enumerate(){
        let x = (i as isize + min) as f32;
        let x = x/scale;
        if i%2 == 0 {
            collumn.sort_unstable_by(|t1,t2| t1.partial_cmp(t2).unwrap());
        } else {
            collumn.sort_unstable_by(|t1,t2| t2.partial_cmp(t1).unwrap());
        }
        let mut collumn = collumn.into_iter()
            .map(|y| *y/scale);
        while let Some(start) = collumn.next(){
            if let Some(end) = collumn.next() {
                let start_p = Point3::new(x,start,height);
                let end_p = Point3::new(x,end,height);
                paths.push(Path{ points:vec![start_p,end_p] })
            }
        }

    }
    return paths
}
impl Polygon {
    fn set_start(&mut self) {
        self.0.iter_mut().for_each(|contour| contour.set_start() );
    }
}

impl Contour3d {
    fn set_start(&mut self) {
        let start_point_ndx = self.points()
            .map(|p|p.x-p.y)
            .enumerate()
            .max_by(|(_,a),(_,b)|a.total_cmp(b))
            .map(|(index,_)| index)
            .expect("contour contains NAN coordinates");

        self.0.rotate_left(start_point_ndx);
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
    let paths = fs::read_dir(layer_dir).expect("could not find directory");

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
