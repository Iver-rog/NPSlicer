#[cfg(test)]
mod test;
mod tagged_bool;
pub use tagged_bool::tagged_boolean;

use crate::geo::{Polygon,Contour,polygons_from_contours};
use crate::skeleton::SkeletonBuilder;
use std::fmt::Display;
use nalgebra::Point2;
use i_overlay::i_float::float::compatible::FloatPointCompatible;
use i_overlay::string::clip::ClipRule;
use i_overlay::core::{
    fill_rule::FillRule,
    overlay_rule::OverlayRule
};
use i_overlay::float::{
    single::SingleFloatOverlay,
    clip::FloatClip,
};
use i_overlay::mesh::{
    style::{LineJoin,OutlineStyle,LineCap,StrokeStyle},
    outline::offset::OutlineOffset,
    stroke::offset::StrokeOffset,
};

#[derive(Copy,Debug,Clone)]
struct IOverlayCompatibleType(Point2<f32>);
impl From<IOverlayCompatibleType> for nalgebra::Point2<f32> {
    fn from(t:IOverlayCompatibleType) -> Point2<f32> { t.0 }
}
impl Display for IOverlayCompatibleType{
    fn fmt(&self,b:&mut std::fmt::Formatter<'_>) -> Result<(),std::fmt::Error>{
        write!(b,"({},{})",self.0.x,self.0.y)
    }
}
impl From<Point2<f32>> for IOverlayCompatibleType{
    fn from(p:Point2<f32>)->Self{
        Self(p)
    }
}

impl FloatPointCompatible<f32> for IOverlayCompatibleType {
     fn from_xy(x: f32, y: f32) -> Self {
         Self ( Point2::new(x,y) )
     }
     fn x(&self) -> f32 { self.0.x }
     fn y(&self) -> f32 { self.0.y }
}

impl Polygon{
    pub fn intersect(&self, sub_shape:Vec<Self>) -> Vec<Polygon>{
        // subtracts sub_shape from self
        let subj:Vec<Vec<IOverlayCompatibleType>> = self.0
            .iter()
            .cloned()
            .map(|contour| contour.points.into_iter().map(|p|p.into()).collect() )
            .collect();

        let clip:Vec<Vec<Vec<IOverlayCompatibleType>>> = sub_shape.into_iter()
            .map(|polygon|{
                let polygon2:Vec<Vec<_>> = polygon.flatten()
                    .into_iter()
                    .map(|contour| contour.points.into_iter().map(|p|p.into()).collect() )
                    .collect();
                polygon2
            }).collect();

        let result = subj.overlay(&clip, OverlayRule::Intersect, FillRule::EvenOdd);

        return result.into_iter()
            .map(|polygon|{
                let contours = polygon.into_iter()
                    .map(|contour| Contour::from_points(contour.into_iter().map(|p|p.into()).collect()) )
                    .collect();
                polygons_from_contours(contours).into_iter().next().unwrap()
            }).collect()
    }
    pub fn subtract(&self, sub_shape:Vec<Self>) -> Vec<Polygon>{
        // subtracts sub_shape from self
        let subj:Vec<Vec<IOverlayCompatibleType>> = self.0
            .iter()
            .cloned()
            .map(|contour| contour.points.into_iter().map(|p|p.into()).collect() )
            .collect();

        let clip:Vec<Vec<Vec<IOverlayCompatibleType>>> = sub_shape.into_iter()
            .map(|polygon|{
                let polygon2:Vec<Vec<_>> = polygon.flatten()
                    .into_iter()
                    .map(|contour| contour.points.into_iter().map(|p|p.into()).collect() )
                    .collect();
                polygon2
            }).collect();

        let result = subj.overlay(&clip, OverlayRule::Difference, FillRule::EvenOdd);

        return result.into_iter()
            .map(|polygon|{
                let contours = polygon.into_iter()
                    .map(|contour| Contour::from_points(contour.into_iter().map(|p|p.into()).collect()) )
                    .collect();
                polygons_from_contours(contours).into_iter().next().unwrap()
            }).collect()
    }

    pub fn offset(self,distance:f32)->Vec<Polygon>{
        if self.outer_loop().points.len() == 0 {panic!("BBBBB Bad input shape no outerloop")}
        let shape:Vec<Vec<IOverlayCompatibleType>> =  self.0.into_iter()
            .map(|c|c.points.into_iter().map(|p|p.into()).collect() )
            .collect();
        if shape.len() == 0 {panic!("BBBBB Bad input shape no polygons")}
        let style = OutlineStyle::new(distance).line_join(LineJoin::Round(0.5));
        let shapes = shape.outline(&style);

        let p:Vec<Contour> = shapes.into_iter()
            .flatten()
            .map(|c| Contour::from_points( c.into_iter().map(|p|p.into()).collect() ) )
            .collect();
        polygons_from_contours(p)
    }
    fn into_ioverlay_type(self)->Vec<Vec<IOverlayCompatibleType>>{
        self.0.into_iter()
            .map(|c|c.points.into_iter().map(|p|p.into()).collect() )
            .collect()
    }
}

impl From<Vec<Vec<IOverlayCompatibleType>>> for Polygon{
    fn from(c:Vec<Vec<IOverlayCompatibleType>>)-> Self{
        let mut contours = c.into_iter();
        let outer_loop = match contours.next() {
            Some(contour) => Contour::from_points(contour.into_iter().map(|p|p.into()).collect()),
            None => return Polygon::new(Contour::from_points(vec![]),vec![]),
        };
        let holes:Vec<Contour> = contours.map(|c| Contour::from_points(
                    c.into_iter().map(|p|p.into()).collect()
                    )
                ).collect();

        return Polygon::new( outer_loop, holes )
    }
}

pub fn offset_line(line:Vec<Point2<f32>>,d_x:f32)->Vec<Polygon>{
    let style = StrokeStyle::new(d_x)
        // Miter: corners with a edge sharper than the input min_angle (in degrees) are beveled 
        .line_join(LineJoin::Miter(0.0))// <- don't bevel corners (ever)
        .start_cap(LineCap::Butt)
        .end_cap(LineCap::Butt);
    let line:Vec<IOverlayCompatibleType> = line.into_iter().map(|p|p.into()).collect();
    let polygons = line.stroke(style, true);
    let p:Vec<Contour> = polygons.into_iter()
        .flatten()
        .map(|c| Contour::from_points( c.into_iter().map(|p|p.into()).collect() ) )
        .collect();
    polygons_from_contours(p)
}

pub fn clip_poly(obj:Vec<Polygon>,mask:Vec<Polygon>)-> Vec<Vec<Point2<f32>>>{
    let clip_rule = ClipRule { invert: false, boundary_included: false };
    let mask:Vec<Vec<Vec<IOverlayCompatibleType>>> = mask.into_iter()
        .map(|p|p.into_ioverlay_type())
        .collect();
    obj.into_iter()
        .map(|p|p.into_ioverlay_type())
        .flatten()
        .map(|mut p|{p.push(p[0]);p})
        .map(|p|p.clip_by(&mask, FillRule::NonZero, clip_rule)) // NOTE: wrong fillrule?
        .flatten()
        .map(|c|c.into_iter().map(|p|p.into()).collect())
        .collect()
}
pub fn ss_offset(polygons:Vec<Polygon>,distance:f32)-> Vec<Polygon>{
    let mut skeleton = SkeletonBuilder::new();
    if distance.is_sign_negative() {
        polygons.into_iter()
            .map(|mut polygon|{polygon.invert();polygon})
            .map(|p|{skeleton.add_polygon(p)})
            .for_each(|result| assert!(result.is_ok()));
    }else{
        for polygon in polygons {
            skeleton.add_polygon(polygon).unwrap();
        }
    }
    skeleton.offset_polygon(distance.abs()).unwrap_or(Vec::new())
}

pub fn offset(polygons:Vec<Polygon>,distance:f32)-> Vec<Polygon>{
    let shapes:Vec<Vec<Vec<IOverlayCompatibleType>>> = polygons.into_iter()
        .map(|p|p.into_ioverlay_type())
        .collect();

    let style = OutlineStyle::new(distance).line_join(LineJoin::Miter(0.0));
    let shapes = shapes.outline(&style);

    let p:Vec<Contour> = shapes.into_iter()
        .flatten()
        .map(|c| Contour::from_points( c.into_iter().map(|p|p.into()).collect() ) )
        .collect();
    polygons_from_contours(p)
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

    let result = subj.overlay(&clip, mode, FillRule::NonZero);

    return result.into_iter()
        .map(|polygon| Polygon(
            polygon.into_iter()
                .map(|contour| Contour::from_points(
                    contour.into_iter()
                    .map(|p| Point2::new(p[0],p[1]) )
                    .collect()
                    )
                )
                .collect()
            )
        )
        .collect()
}
