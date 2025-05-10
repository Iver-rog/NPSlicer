use crate::geo::{Contour, FromUnChecked, Polygon};
use std::collections::HashSet;
use nalgebra::Point2;

use i_overlay::vector::edge::VectorEdge;
use i_overlay::core::{
    overlay::ContourDirection,
    fill_rule::FillRule,
    overlay_rule::OverlayRule
};
use i_overlay::float::overlay::{ FloatOverlay, OverlayOptions };
use i_overlay::i_float::float::point::FloatPoint;
use ordered_float::OrderedFloat;

#[cfg(test)]
use crate::contour;

/// byte flags used by the i_overlay crate:
type SegmentFill = u8;

// pub const NONE: SegmentFill = 0;

// pub const SUBJ_TOP: SegmentFill = 0b0001;
// pub const SUBJ_BOTTOM: SegmentFill = 0b0010;
pub const CLIP_TOP: SegmentFill = 0b0100;
pub const CLIP_BOTTOM: SegmentFill = 0b1000;

// pub const SUBJ_BOTH: SegmentFill = SUBJ_TOP | SUBJ_BOTTOM;
// pub const CLIP_BOTH: SegmentFill = CLIP_TOP | CLIP_BOTTOM;
// pub const BOTH_TOP: SegmentFill = SUBJ_TOP | CLIP_TOP;
// pub const BOTH_BOTTOM: SegmentFill = SUBJ_BOTTOM | CLIP_BOTTOM;

// pub const ALL: SegmentFill = SUBJ_BOTH | CLIP_BOTH;


/// Performes a boolean opperation while keeping trach of which of the 
/// edges in the result were part of the cliping shape.
pub fn tagged_boolean(subj:Vec<Polygon>,clip:Vec<Polygon>,overlay_rule:OverlayRule) -> (Vec<Polygon>,Vec<Vec<Vec<bool>>>){
    let subj:Vec<Vec<FloatPoint<f32>>> = subj.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c| c.points.into_iter()
            .map(|p|FloatPoint{x:p.x, y:p.y})
            .collect() )
        .collect();
    let clip:Vec<Vec<FloatPoint<f32>>> = clip.into_iter()
        .map(|p| p.flatten() )
        .flatten()
        .map(|c| c.points.into_iter()
            .map(|p|FloatPoint{x:p.x, y:p.y})
            .collect() )
        .collect();
    
    let fill_rule = FillRule::NonZero;

    let overlay_options = OverlayOptions{
            preserve_input_collinear: false,
            output_direction: ContourDirection::CounterClockwise,
            preserve_output_collinear: true,
            min_output_area: 0.0,
            clean_result: true,
        };

    let overlay = FloatOverlay::with_subj_and_clip_and_options( &subj, &clip, overlay_options);
    let result:Vec<Vec<Vec<FloatPoint<f32>>>> = overlay.clone().overlay(overlay_rule, fill_rule);
    let graph = overlay.into_graph(fill_rule);
    let vector_edges:Vec<VectorEdge> = graph.graph.extract_separate_vectors();

    // filter out the edges that dont bellong to the subj-shape
    let subj_edges:HashSet<[[OrderedFloat<f32>;2];2]> = vector_edges.iter()
        .filter(|v_e|{
            // let flag = v_e.fill & 0b0011u8;
            // (flag == SUBJ_TOP) | (flag == SUBJ_BOTTOM)
            let flag = v_e.fill & 0b1100u8;
            (flag == CLIP_TOP) | (flag == CLIP_BOTTOM)
        })
        .map(|v_e| [graph.adapter.int_to_float(&v_e.a),graph.adapter.int_to_float(&v_e.b)] )
        .map(|[a,b]| [[OrderedFloat(a.x),OrderedFloat(a.y)],[OrderedFloat(b.x),OrderedFloat(b.y)]] )
        .collect();

    let (polygons,tags):(Vec<Polygon>,Vec<_>) = result.into_iter()
        .map(|polygon|{
            let (contours,tags):(Vec<Contour>,Vec<_>) = polygon.into_iter()
                .map(|contour|{
                    let (points,tags):(Vec<Point2<f32>>,Vec<bool>) = contour.iter()
                    // contour.iter()
                        .zip(contour.iter().cycle().skip(1))
                        .map(|(p1,p2)|[ [OrderedFloat(p1.x),OrderedFloat(p1.y)] , [OrderedFloat(p2.x),OrderedFloat(p2.y)] ])
                        .map(|[e1,e2]|{
                            let edge1:[[OrderedFloat<f32>;2];2] = [e1,e2];
                            let edge2:[[OrderedFloat<f32>;2];2] = [e2,e1];
                            let subj_edge = subj_edges.contains(&edge1) | subj_edges.contains(&edge2);
                            let point1 = Point2::new( e1[0].0, e1[1].0 );
                            let point2 = Point2::new( e2[0].0, e2[1].0 );
                            let edge = [point1,point2];
                            // (edge,subj_edge)
                            (point1,subj_edge)
                        })
                        // .scan(None,|state:&mut Option<([Point2<f32>;2],bool)>,(edge,tag)|{
                        //     let remove_point = match *state{ 
                        //         None => false,
                        //         Some((prev_edge,prev_tag)) => {
                        //             if prev_tag == tag {
                        //                 let edge_vec = edge[1] - edge[0];
                        //                 let prev_edge_vec = prev_edge[1] - prev_edge[0];
                        //                 if (edge_vec.normalize() - prev_edge_vec.normalize()).magnitude() < 1e-4 
                        //                 { true } else { false }
                        //             } else {
                        //                 false
                        //             }
                        //         }
                        //     };
                        //     *state = Some((edge,tag));
                        //     match remove_point {
                        //         true => None,
                        //         false => Some((edge[0],tag))
                        //     }
                        //     })
                        .unzip(); (Contour::from_points(points),tags)
                        // .collect::<Vec<_>>()
                        })
                .unzip();
                (Polygon::from_unchecked(contours),tags)
                // .collect::<Vec<_>>()
        })
        .unzip();

    return (polygons,tags)

}

#[cfg(test)]
fn subj()->Vec<Polygon>{
    vec![Polygon(vec![
            contour!(
                [1.0, 0.0],
                [5.0, 0.0],
                [5.0, 4.0],
                [1.0, 4.0]
            )
        ])
    ]
}
#[cfg(test)]
fn clip()->Vec<Polygon>{
    vec![Polygon(vec![
            contour!(
                [0.0, 2.0],
                [2.0, 0.0],
                [4.0, 0.0],
                [6.0, 2.0],
                [4.0, 4.0],
                [2.0, 4.0]
            )
        ])
    ]
}


#[test]
pub fn tagged_bool_intersect_test(){
    let subj = subj();
    let clip = clip();
    let correct_result = [
        // contour point        is part of subj?
        (Point2::new(1.0,3.0),  false),
        (Point2::new(1.0,1.0),  true),
        (Point2::new(2.0,0.0),  true),
        (Point2::new(4.0,0.0),  true),
        (Point2::new(5.0,1.0),  false),
        (Point2::new(5.0,3.0),  true),
        (Point2::new(4.0,4.0),  true),
        (Point2::new(2.0,4.0),  true),
    ];

    let (polygons,tags) = tagged_boolean(subj, clip, OverlayRule::Intersect);
    let contour = polygons.first().unwrap().0.first().unwrap();
    let tags = tags.first().unwrap().first().unwrap();
    let contour_tag = contour.points.iter().zip(tags.iter());

    for (i,(point,tag)) in contour_tag.enumerate() {
        let (correct_point,correct_tag) = correct_result[i];
        assert_eq!(*point,correct_point,"point at ndx: {i} did not match");
        assert_eq!(*tag,correct_tag,"tag at ndx: {i} did not match");
    }
}

#[test]
pub fn tagged_bool_union_test(){
    let subj = subj();
    let clip = clip(); 

    let correct_result = [
        // contour point        is part of clip?
        (Point2::new(1.0,3.0),  true),
        (Point2::new(0.0,2.0),  true),
        (Point2::new(1.0,1.0),  false),
        (Point2::new(1.0,0.0),  false),
        (Point2::new(2.0,0.0),  true),
        (Point2::new(4.0,0.0),  false),
        (Point2::new(5.0,0.0),  false),
        (Point2::new(5.0,1.0),  true),
        (Point2::new(6.0,2.0),  true),
        (Point2::new(5.0,3.0),  false),
        (Point2::new(5.0,4.0),  false),
        (Point2::new(4.0,4.0),  true),
        (Point2::new(2.0,4.0),  false),
        (Point2::new(1.0,4.0),  false),
    ];

    let (polygons,tags) = tagged_boolean(subj, clip, OverlayRule::Union);
    let contour = polygons.first().unwrap().0.first().unwrap();
    let tags = tags.first().unwrap().first().unwrap();
    let contour_tag = contour.points.iter().zip(tags.iter());

    for (i,(point,tag)) in contour_tag.enumerate() {
        let (correct_point,correct_tag) = correct_result[i];
        assert_eq!(*point,correct_point,"point at ndx: {i} did not match");
        assert_eq!(*tag,correct_tag,"tag at ndx: {i} did not match");
    }
}

