use crate::contours::{Polygon,Contour};
use crate::contours::polygons_from_contours;
use nalgebra::Point2;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

impl Polygon{
    pub fn subtract(mut self, mut sub_shape:Self) -> Vec<Polygon>{
        // subtracts sub_shape from self
        let mut a = self.holes;
        a.push(self.outer_loop);

        let subj: Vec<Vec<[f32;2]>>= a.into_iter().map(|contour|contour.points.into_iter().map(|p|[p.x,p.y]).collect()).collect();

        let mut b = sub_shape.holes;
        b.push(sub_shape.outer_loop);

        let clip: Vec<Vec<[f32;2]>> = b.into_iter().map(|contour|contour.points.into_iter().map(|p|[p.x,p.y]).collect()).collect();

        let result = subj.overlay(&clip, OverlayRule::Difference, FillRule::EvenOdd);

        result.into_iter().map(|polygon|{
            let contours = polygon.into_iter().map(|contour| Contour::new(contour.into_iter().map(|p|Point2::new(p[0],p[1])).collect())).collect();
            polygons_from_contours(contours).into_iter().next().unwrap()
        }).collect()
    }
}
