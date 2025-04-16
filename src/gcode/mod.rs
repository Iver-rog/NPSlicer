use nalgebra::{Point2, Point3};
use stl_io::IndexedMesh;

use std::iter::Iterator;
use std::io::BufReader;
use std::fs::{self, File};
use std::iter;

use crate::geo::*;
use crate::stl_op;
use crate::Blender;

mod projection;
use projection::{MeshCollider,project_point_onto};

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

    // let mesh_layer_dir = "../curving_overhang/p0.2";
    let mesh_layer_dir = "../simple_overhang";
    let mut mesh_layers = import_layers(&mesh_layer_dir)
        .map(|layer_mesh| contour_and_mesh_colider_from_mesh(layer_mesh));

    println!("layer: 0");
    let (first_layer_perimeters,_) = mesh_layers.next().expect("missing first layer");
    let mut gcodefile = GcodeFile::new(&mesh_layer_dir,&settings);

    let first_layer_infill = first_layer_perimeters.clone().into_iter()
        .flat_map(|polygon| polygon.offset(-(nr_of_perimeters as f32)*layer_w))
        .flat_map(|polygon| generate_infill_allong_x_2d(polygon, layer_h ,0.45) );

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
                .flat_map(|offset_polygon| match layer_nr%2 ==0 {
                    true => generate_infill_allong_y(offset_polygon, &mesh_collider_copy),
                    false => generate_infill_allong_x(offset_polygon, &mesh_collider_copy),
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

// Generates infill paths inside the bounds.
// Spacing is the distance in mm between infill lines.
// Rotation is the angle in radians between the x-axis and the direction of the infill.
fn generate_infill_allong_y(bounds:Polygon,mesh:&MeshCollider) -> Vec<Path> {
    let aabb = &bounds.outer_loop().aabb;
    let (min,max) = ( aabb.y_min.floor() as isize, aabb.y_max.ceil() as isize);
    // let (min,max) = ( aabb.x_min.ceil() as isize, aabb.x_max.floor() as isize);

    let offset = -aabb.y_min.floor();
    let range = (max-min) as usize;
    let mut xz_vals:Vec<Vec<(f32,Option<f32>)>> = vec![Vec::new();range+1];

    for (e1,e2) in mesh.edges.iter()
        .map(|edge|(mesh.vertices[edge.0],mesh.vertices[edge.1]))
        .map(|(e1,e2)| if e1.y < e2.y { (e1,e2) }else{ (e2,e1) }){

            let residual = e1.y.ceil()-e1.y;

            // f(y) = a*y + b
            let ax = (e2.x-e1.x)/(e2.y-e1.y);
            let bx = (e1.x) + ax*residual;
            let az = (e2.z-e1.z)/(e2.y-e1.y);
            let bz = (e1.z) + az*residual;

            let (y_min,y_max) = ((e1.y.ceil() as isize) , (e2.y.floor() as isize));

            let y_vals = (0..(1+y_max - y_min)).map(|y| y as f32);

            let x = match ax == 0.0 {
                true => vec![e1.x; y_vals.len()],
                false => y_vals.clone().map(|y| ax * y + bx ).collect(),
            };

            let z = match az == 0.0 {
                true => vec![e1.z; y_vals.len()],
                false => y_vals.clone().map(|y| az * y + bz ).collect(),
            };

            x.into_iter().zip(z.into_iter())
                .enumerate()
                .map(|(i,(x,z))|{ 
                    let y_ndx = i as isize + y_min - min;
                    (y_ndx,x,z)
                })
                .filter_map(|(y_ndx,x,z)| usize::try_from(y_ndx).ok().map(|y_ndx|(y_ndx,x,z)) )
                .filter(|(y_ndx,x,z)| *y_ndx < range+1)
                .filter(|(y_ndx,x,z)| (aabb.x_min <= *x) && (*x <= aabb.y_max) )
                .for_each(|(y_ndx,x,z)| xz_vals[y_ndx].push((x,Some(z))) )
    }

    // Add contour intersections
    for (e1,e2) in bounds.all_edges()
        .map(|(e1,e2)| if e1.y < e2.y { (e1,e2) }else{ (e2,e1) }){

            let residual = e1.y.ceil()-e1.y;

            let ax = (e2.x-e1.x)/(e2.y-e1.y);
            let bx = (e1.x) + ax*residual;

            let (y_min,y_max) = ((e1.y.ceil() as isize) , (e2.y.ceil() as isize));

            let y_vals = (0..(y_max - y_min)).map(|y| y as f32);

            let x = match ax == 0.0 {
                true => vec![e1.x; y_vals.len()],
                false => y_vals.clone().map(|y| ax * y + bx ).collect(),
            };

            x.into_iter()
                .enumerate()
                .map(|(i,x)|{ 
                    let y_ndx = i as isize + y_min - min;
                    (y_ndx,x)
                })
                .filter_map(|(y_ndx,x)| usize::try_from(y_ndx).ok().map(|y_ndx|(y_ndx,x)) )
                .filter(|(y_ndx,x)| *y_ndx < range+1)
                .filter(|(y_ndx,x)| (aabb.x_min <= *x) && (*x <= aabb.x_max) )
                .for_each(|(y_ndx,x)| xz_vals[y_ndx].push((x,None)) )
        }

    // sort the table:
    for (i,collumn) in xz_vals.iter_mut().enumerate(){
        if i%2 == 0 {
            collumn.sort_unstable_by(|(t1,_),(t2,_)| t1.partial_cmp(t2).unwrap());
        } else {
            collumn.sort_unstable_by(|(t1,_),(t2,_)| t2.partial_cmp(t1).unwrap());
        }
    }

    let mut collums = Vec::new();
    for (y,xz) in xz_vals.into_iter()
        .enumerate()
        .map(|(y,xz)| ((y as isize + min) as f32 ,xz)){

        let mut segment = Vec::new();
        let mut within_bounds = false;

        for (x,z) in xz {
            match z {
                // found a mesh point keep it if it is inside the contur
                Some(z) => if within_bounds {
                    segment.push(Point3::new(x,y,z));
                },
                // point on the bounding contour which implies crossing 
                // the boundary between inside and outside the bounds
                None => {
                    let point3d = project_point_onto(&Point2::new(x,y),&mesh);
                    if within_bounds { // completed the line segment
                        segment.push( point3d );
                        collums.push( Path{points:segment} );
                        segment = Vec::new();
                    } else {
                        segment.push(point3d)
                    }
                    within_bounds = !within_bounds;
                    (x,0.0);
                },
            };
        }
    }
    return collums
}

fn generate_infill_allong_x(bounds:Polygon,mesh:&MeshCollider) -> Vec<Path> {
    let aabb = &bounds.outer_loop().aabb;
    let (min,max) = ( aabb.x_min.floor() as isize, aabb.x_max.ceil() as isize);
    // let (min,max) = ( aabb.x_min.ceil() as isize, aabb.x_max.floor() as isize);

    let offset = -aabb.x_min.floor();
    let range = (max-min) as usize;
    let mut yz_vals:Vec<Vec<(f32,Option<f32>)>> = vec![Vec::new();range+1];

    for (e1,e2) in mesh.edges.iter()
        .map(|edge|(mesh.vertices[edge.0],mesh.vertices[edge.1]))
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
                .for_each(|(x_ndx,y,z)| yz_vals[x_ndx].push((y,Some(z))) )
    }

    // Add contour intersections
    for (e1,e2) in bounds.all_edges()
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
                .filter(|(x_ndx,y)| (aabb.y_min <= *y) && (*y <= aabb.y_max) )
                .for_each(|(x_ndx,y)| yz_vals[x_ndx].push((y,None)) )
        }

    // sort the table:
    for (i,collumn) in yz_vals.iter_mut().enumerate(){
        if i%2 == 0 {
            collumn.sort_unstable_by(|(t1,_),(t2,_)| t1.partial_cmp(t2).unwrap());
        } else {
            collumn.sort_unstable_by(|(t1,_),(t2,_)| t2.partial_cmp(t1).unwrap());
        }
    }

    let mut collums = Vec::new();
    for (x,yz) in yz_vals.into_iter()
        .enumerate()
        .map(|(x,yz)| ((x as isize + min) as f32 ,yz)){

        let mut segment = Vec::new();
        let mut within_bounds = false;

        for (y,z) in yz {
            match z {
                // found a mesh point keep it if it is inside the contur
                Some(z) => if within_bounds {
                    segment.push(Point3::new(x,y,z));
                },
                // point on the bounding contour which implies crossing 
                // the boundary between inside and outside the bounds
                None => {
                    let point3d = project_point_onto(&Point2::new(x,y),&mesh);
                    if within_bounds { // completed the line segment
                        segment.push( point3d );
                        collums.push( Path{points:segment} );
                        segment = Vec::new();
                    } else {
                        segment.push(point3d)
                    }
                    within_bounds = !within_bounds;
                    (y,0.0);
                },
            };
        }
    }
    return collums
}

fn generate_infill_allong_x_2d(bounds:Polygon, height:f32, spacing:f32) -> Vec<Path> {
    let scale = 1.0/spacing;
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
