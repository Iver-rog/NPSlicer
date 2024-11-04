use nalgebra::{Point2, Vector2};
use std::collections::HashSet;

#[derive(Debug, Clone)]
struct Circle {
    center: Point2<f64>,
    radius: f64,
}

#[derive(Debug, Clone)]
pub struct Edge {
    start: Point2<f64>,
    end: Point2<f64>,
}

impl Edge {
    fn new(start: Point2<f64>, end: Point2<f64>) -> Self {
        Edge { start, end }
    }

    fn vector(&self) -> Vector2<f64> {
        self.end - self.start
    }

    fn length(&self) -> f64 {
        self.vector().magnitude()
    }

    fn distance_to_point(&self, point: &Point2<f64>) -> f64 {
        let edge_vec = self.vector();
        let point_vec = point - self.start;
        
        let t = (point_vec.dot(&edge_vec)) / edge_vec.dot(&edge_vec);
        
        if t <= 0.0 {
            (point - self.start).magnitude()
        } else if t >= 1.0 {
            (point - self.end).magnitude()
        } else {
            let projection = self.start + edge_vec * t;
            (point - projection).magnitude()
        }
    }
}

#[derive(Debug)]
pub struct MedialAxisTransform {
    polygon: Vec<Edge>,
    skeleton: Vec<Edge>,
    tolerance: f64,
}

impl MedialAxisTransform {
    pub fn new(vertices: Vec<Point2<f64>>, tolerance: f64) -> Self {
        let mut edges = Vec::new();
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            edges.push(Edge::new(vertices[i], vertices[j]));
        }
        
        MedialAxisTransform {
            polygon: edges,
            skeleton: Vec::new(),
            tolerance,
        }
    }

    pub fn compute(&mut self) {
        let mut centers = HashSet::new();
        
        // Start with circles at vertices
        for edge in &self.polygon {
            self.grow_circle_from_point(&edge.start, &mut centers);
        }
        dbg!(&centers);
        
        // Connect nearby circle centers to form the skeleton
        self.connect_centers(&centers);
    }

    fn grow_circle_from_point(&self, start: &Point2<f64>, centers: &mut HashSet<(i32, i32)>) {
        let mut circle = Circle {
            center: *start,
            radius: f64::INFINITY,
        };

        // Find initial radius by getting minimum distance to edges
        for edge in &self.polygon {
            let dist = edge.distance_to_point(&circle.center);
            circle.radius = circle.radius.min(dist);
        }

        // Discretize circle center for storage in HashSet
        let x = (circle.center.x / self.tolerance).round() as i32;
        let y = (circle.center.y / self.tolerance).round() as i32;
        centers.insert((x, y));

        // Shrink circle while maintaining contact with polygon
        while circle.radius > self.tolerance {
            let mut shrink_vector = Vector2::zeros();
            let mut contact_count = 0;

            for edge in &self.polygon {
                let dist = edge.distance_to_point(&circle.center);
                if (dist - circle.radius).abs() < self.tolerance {
                    contact_count += 1;
                    let normal = (circle.center - edge.start).normalize();
                    shrink_vector += normal;
                }
            }

            if contact_count < 2 {
                break;
            }

            // Move center in direction of average contact normals
            circle.center += shrink_vector.normalize() * self.tolerance;
            
            // Update radius
            circle.radius = f64::INFINITY;
            for edge in &self.polygon {
                let dist = edge.distance_to_point(&circle.center);
                circle.radius = circle.radius.min(dist);
            }

            // Store discretized center
            let x = (circle.center.x / self.tolerance).round() as i32;
            let y = (circle.center.y / self.tolerance).round() as i32;
            centers.insert((x, y));
        }
    }

    fn connect_centers(&mut self, centers: &HashSet<(i32, i32)>) {
        let mut visited = HashSet::new();
        
        for &(x, y) in centers {
            if visited.contains(&(x, y)) {
                continue;
            }
            
            visited.insert((x, y));
            
            // Look for nearby centers to connect
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let next = (x + dx, y + dy);
                    if centers.contains(&next) && !visited.contains(&next) {
                        let p1 = Point2::new(
                            x as f64 * self.tolerance,
                            y as f64 * self.tolerance
                        );
                        let p2 = Point2::new(
                            next.0 as f64 * self.tolerance,
                            next.1 as f64 * self.tolerance
                        );
                        self.skeleton.push(Edge::new(p1, p2));
                    }
                }
            }
        }
    }

    pub fn get_skeleton(&self) -> Vec<[[f32;2];2]> {
        self.skeleton.clone().into_iter()
            .map(|edge| {
                let p1 = [edge.start[0] as f32, edge.start[1] as f32];
                let p2 = [edge.end[0] as f32, edge.end[1] as f32];
                return [p1,p2]
            })
            .collect()
    }
}

