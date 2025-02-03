use nalgebra::Point2;

#[cfg(test)]
mod test;

#[derive(Debug,Clone,PartialEq)]
pub struct AABB{
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
    outer_loop: Contour,
    holes: Vec<Contour>,
}
impl Polygon {
    fn new(mut outer_loop:Contour,mut holes:Vec<Contour>)->Self{
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
}
#[derive(Debug,Clone,PartialEq)]
pub struct Contour{
    area:f32,
    aabb:AABB,
    points: Vec<Point2<f32>>
}
impl Contour {
    pub fn reverse_order(&mut self) {
        self.points.reverse();
        self.area = self.area * -1.0;
    }
    pub fn new(points:Vec<Point2<f32>>) -> Self {
        let first_point = points[0];

        let mut aabb = AABB{
            x_max: first_point.x,
            x_min: first_point.x,
            y_max: first_point.y,
            y_min: first_point.y,
        };
        let mut area = 0.0;
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

pub fn polygons_from_contours(contours:Vec<Contour>)->Vec<Polygon>{
    let mut polygons = vec![Some(Vec::new());contours.len()];
    for (i,contour) in contours.iter().enumerate() {
        let test_point = contour.points[0];

        let mut contour_i_is_inside = None;
        let mut contour_i_is_exterior = true;
        for (n,intersection_contour) in contours.iter().enumerate(){
            if i == n {continue}
            match intersection_contour.x_distance_to_contour(&test_point) {
                Some(new_distance) => {
                    match contour_i_is_inside {
                        Some((_contour,old_distance)) => {
                            if new_distance < old_distance { 
                            contour_i_is_inside = Some((n,new_distance))
                            }
                        }
                        None => {
                            contour_i_is_inside = Some((n,new_distance));
                            contour_i_is_exterior = false;
                        },
                    }
                }
                None => (),
            }
            println!("{contour_i_is_inside:?}");
        }
        if !contour_i_is_exterior {
            match contour_i_is_inside{
                Some((n,_)) => { 
                    match &mut polygons[n]{
                        Some(vector) => {
                            vector.push(i);
                            polygons[i] = None;
                        },
                        none => panic!(),
                    }
                },
                None => (),
            }
        }
    }
    polygons.iter()
        .enumerate()
        .filter_map(|(i,p)| match p {
            Some(x)=> Some((i,x)),
            None => None,
        } )
        .map(|(contour_ndx,holes_ndxes)|{
            let contour = contours[contour_ndx].clone();
            let holes = holes_ndxes.into_iter().map(|ndx|contours[*ndx].clone()).collect();
            Polygon::new(contour,holes)
        }).collect()
}
