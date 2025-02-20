use crate::contours::{Polygon,Contour};
use crate::contours::polygons_from_contours;
use nalgebra::Point2;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

impl Polygon{
    pub fn subtract(self, sub_shape:Self) -> Vec<Polygon>{
        // subtracts sub_shape from self
        let subj:Vec<Vec<[f32;2]>> = self.flatten()
            .into_iter()
            .map(|contour| contour.points.into_iter().map(|p|[p.x,p.y]).collect() )
            .collect();

        let clip:Vec<Vec<[f32;2]>> = sub_shape.flatten()
            .into_iter()
            .map(|contour| contour.points.into_iter().map(|p|[p.x,p.y]).collect() )
            .collect();

        let result = subj.overlay(&clip, OverlayRule::Difference, FillRule::EvenOdd);

        return result.into_iter().map(|polygon|{
            let contours = polygon.into_iter().map(|contour| Contour::new(contour.into_iter().map(|p|Point2::new(p[0],p[1])).collect())).collect();
            polygons_from_contours(contours).into_iter().next().unwrap()
        }).collect()
    }
}
pub fn filtered_boolean(poly:Vec<Polygon>,mask:Vec<Polygon>, mode:OverlayRule) -> Vec<Polygon> {
    let subj:Vec<Vec<[f32;2]>> = poly.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c| 
            c.points.into_iter()
            .map(|p|[p.x,p.y])
            .collect()
            )
        .collect();

    let clip:Vec<Vec<[f32;2]>> = mask.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c| 
            c.points.into_iter()
            .map(|p|[p.x,p.y])
            .collect()
            )
        .collect();

    let result = subj.overlay(&clip, mode, FillRule::EvenOdd);

    return result.into_iter()
        .map(|contours|{
            contours.into_iter()
                .filter_map(|contour|{
                    let last_ndx = contour.len().saturating_sub(1);
                    let mut result = Vec::new();
                    for i in (0..=last_ndx).into_iter(){
                            let i_m = if result.len() == 0 {last_ndx} else {result.len()-1};
                            let i_p = (i+1)%(last_ndx+1);
                            let (p1,p2,p3)=(contour[i_m],contour[i],contour[i_p]);
                            let (p1,p2,p3)=(Point2::new(p1[0],p1[1]),Point2::new(p2[0],p2[1]),Point2::new(p3[0],p3[1]));
                            let v1 = p2-p1;
                            let v2 = p3-p2;
                            if v1.magnitude() > 0.1 && (v1.normalize()+v2.normalize()).magnitude() > 0.3 {result.push(p2)}
                        }
                    if result.len() < 2 {None}else{Some(Contour::new(result))}
                    })
                .filter(|contour| contour.area.abs() > 1.0)
                .collect()
        })
    .map(|contours| polygons_from_contours(contours).into_iter() )
    .flatten().collect()
}
pub fn boolean(poly:Vec<Polygon>,mask:Vec<Polygon>, mode:OverlayRule) -> Vec<Polygon> {
    let subj:Vec<Vec<[f32;2]>> = poly.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c|
            c.points.into_iter()
            .map(|p|[p.x,p.y])
            .collect()
            )
        .collect();

    let clip:Vec<Vec<[f32;2]>> = mask.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c|
            c.points.into_iter()
            .map(|p|[p.x,p.y])
            .collect()
            )
        .collect();

    let result = subj.overlay(&clip, mode, FillRule::EvenOdd);

    return result.into_iter()
        .map(|p_loop|{
            let contours = p_loop.into_iter()
                .map(|contour| Contour::new(
                    contour.into_iter()
                    .map(|p| Point2::new(p[0],p[1]) )
                    .collect()
                    )
                )
                .filter(|contour| contour.area.abs() > 0.1)
                .collect();
            polygons_from_contours(contours).into_iter()
        }).flatten().collect()
}
