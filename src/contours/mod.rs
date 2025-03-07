use nalgebra::Point2;
use nalgebra_glm::cross2d;

#[cfg(test)]
mod test;
pub mod boolean;

#[derive(Debug,Clone,PartialEq)]
struct AABB{
    x_max:f32,
    x_min:f32,
    y_max:f32,
    y_min:f32,
}
impl AABB {
    fn point_is_inside(&self,point:&Point2<f32>) -> bool {
        self.x_min <= point.x && point.x <= self.x_max &&
        self.y_min <= point.y && point.y <= self.y_max
    }
}

#[derive(Debug,Clone,PartialEq)]
pub struct Polygon{
    pub outer_loop: Contour,
    pub holes: Vec<Contour>,
}
impl Polygon {
    pub fn invert(&mut self){
        self.outer_loop.reverse_order();
        self.holes.iter_mut().for_each(|c|c.reverse_order());
    }
    pub fn new(mut outer_loop:Contour,mut holes:Vec<Contour>)->Self{
        if outer_loop.area.is_sign_negative() {
            outer_loop.reverse_order();
        }
        for hole in holes.iter_mut() {
            if hole.area.is_sign_positive(){
                hole.reverse_order();
            }
        }
        Self{
            outer_loop,
            holes,
        }
    }
    pub fn area(&self) -> f32 {
        let area = self.outer_loop.area;
        let hole_area: f32 = self.holes.iter().map(|contour| contour.area ).sum();
        return area - hole_area
    }
    pub fn simplify(&mut self, min_a:f32){
        self.outer_loop.simplify(min_a);
        if (self.outer_loop.area < min_a) | (self.outer_loop.points.len() < 3){
            self.outer_loop.points.clear();
            self.outer_loop.area = 0.0;
            self.holes.clear();
        };
        self.holes.iter_mut().for_each(|hole|hole.simplify(min_a));
        self.holes.retain(|c|c.area.abs() > min_a);
    }
    pub fn flatten(self) -> Vec<Contour>{
        let mut a = self.holes;
        a.push(self.outer_loop);
        a
    }
}
#[derive(Debug,Clone,PartialEq)]
pub struct Contour{
    pub area:f32,
    aabb:AABB,
    pub points: Vec<Point2<f32>>
}
impl Contour {
    pub fn simplify(&mut self, min_a:f32){
        let len = self.points.len();
        for i in (0..len).rev(){
            let i_pluss = (i+1)%self.points.len();
            let i_minus = if i == 0 {self.points.len().saturating_sub(1)}else{i.saturating_sub(1)};

            let p = self.points[i];
            let p_p = self.points[i_pluss];
            let p_m = self.points[i_minus];

            let v1 = p_p - p;
            let v2 = p_m - p;

            let area_x2 = cross2d(&v1, &v2);
            if area_x2.abs() < min_a*2. {self.points.remove(i);}
        }
        if self.points.len() < 3 {self.points.clear(); self.area = 0.0;}
    }
    pub fn reverse_order(&mut self) {
        self.points.reverse();
        self.area = self.area * -1.0;
    }
    pub fn new(points:Vec<Point2<f32>>) -> Self {
        let first_point = points[0];
        let last_point = points[points.len()-1];

        let mut aabb = AABB{
            x_max: first_point.x,
            x_min: first_point.x,
            y_max: first_point.y,
            y_min: first_point.y,
        };
        let mut area = last_point.x*first_point.y-first_point.x*last_point.y;

        let mut prev_point = first_point.clone();
        for point in points.iter().skip(1) {
            aabb.x_max = aabb.x_max.max(point.x);
            aabb.x_min = aabb.x_min.min(point.x);
            aabb.y_max = aabb.y_max.max(point.y);
            aabb.y_min = aabb.y_min.min(point.y);
            
            area += prev_point.x*point.y-point.x*prev_point.y;
            prev_point = *point;
        }

        return Contour{
            area: area/2.0,
            aabb,
            points,
        }
    }
    pub fn point_is_inside(&self,point:&Point2<f32>)->bool{
        // returns true if a point is on or inside the contour
        if !self.aabb.point_is_inside(&point) { return false }

        let points_offset_by_one = self.points.iter()
            .skip(1)
            .chain( self.points.iter() );

        let intersections: usize = self.points.iter()
            .zip(points_offset_by_one)
            // cast a ray from the test point towards the right direction allong 
            // the x-axis and check if the ray intersects the edge
            .map(|(p1,p2)|{
                // check if the two points of the edge are on opposite sides of 
                // the horizontal line at test point's x value
                ((p1.y < point.y) != (p2.y <= point.y)) && 
                // find where the edge intersects the horizontal line at test point's x value 
                // and check if the crossing point lies on the right side of the test point
                point.x <= ((point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x)
            })
            .map(|bool| match bool { true => 1, false => 0, } )
            .sum();
        return intersections % 2 == 1
    }
    pub fn x_distance_to_contour(&self,point:&Point2<f32>)->Option<f32>{
        // returns true if a point is on or inside the contour
        if !self.aabb.point_is_inside(&point) { return None }

        let points_offset_by_one = self.points.iter()
            .skip(1)
            .chain( self.points.iter() );

        let intersections: Vec<f32> = self.points.iter()
            .zip(points_offset_by_one)
            // cast a ray from the test point towards the right direction allong 
            // the x-axis and check if the ray intersects the edge
            .filter_map(|(p1,p2)|{
                // check if the two points of the edge are on opposite sides of 
                // the horizontal line at test point's x value
                if (p1.y < point.y) == (p2.y <= point.y) { return None }
                // find where the edge intersects the horizontal line at test point's x value 
                // and check if the crossing point lies on the right side of the test point
                else {
                    let d_x = (point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x;
                    if point.x <= d_x { return Some(d_x-point.x)}
                    else {return None;}
                }
            })
            .collect();
        if intersections.len() % 2 == 1 { 
            return Some(intersections.into_iter()
                .fold(f32::INFINITY, |a, b| a.min(b))
                )
        }
        else { return None }
    }
}

pub fn polygons_from_contours(mut contours:Vec<Contour>)->Vec<Polygon>{
    let mut contour_inside_ref = vec![None;contours.len()];
    for (i,contour) in contours.iter().enumerate() {
        let test_point = contour.points[0];

        let contour_i_is_inside = contours.iter()
            .enumerate()
            .filter(|(n,_)|*n!=i)
            .filter_map(|(n,intersection_contour)|
                match intersection_contour.x_distance_to_contour(&test_point){
                    Some(distance) => Some((n,distance)),
                    None => None,
            })
            .enumerate() // count how many intersections occur
            .reduce(|(_,(n_min,min_distance)),(n_intersect,(n_new,new_distance))| {
                if new_distance < min_distance {(n_intersect,(n_new,new_distance))} else {(n_intersect,(n_min,min_distance))}
            });

        match contour_i_is_inside{
            Some((n_intersect,(n,_))) => { 
                // n_intersect is zero based eg n_intersect = 0 => 1 intersection
                if n_intersect % 2 == 0 { 
                    assert!(contour_inside_ref[i]==None);
                    contour_inside_ref[i]=Some(n);
                } 
            },
            None => (),
        }
    }
    let mut polygons = vec![(None,Vec::new());contours.len()];
    for (i,is_inside) in contour_inside_ref.into_iter().enumerate().rev(){
        match is_inside {
            Some(i) => { polygons[i].1.push( contours.pop().unwrap() ) },
            None => polygons[i].0 = Some(contours.pop().unwrap()),
        }
    }

    polygons.into_iter()
        .filter_map(|(option_outer_loop,holes)| match option_outer_loop {
            Some(outer_loop) => Some((outer_loop,holes)),
            None => None,
        })
        .map(|(outer_loop,holes)| Polygon::new(outer_loop,holes) )
        .collect()
}
