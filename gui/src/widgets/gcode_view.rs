
use iced::{ Element, Shrink, Fill, Color };
use iced::widget::text::Span;
use iced::widget::{Column, column, container, text};
use core::iter::Iterator;
use std::cmp::{min,max};

use crate::test_gcode;

use super::transparent_box;

pub fn gcode_view(gcode:&str, line_nr:u32) -> Element<'_, crate::Message> {

    let current_line:usize = line_nr as usize;
    let nr_lines_to_show = 20;
    let nr_of_lines = gcode.split("\n").count();
    assert!(current_line<nr_of_lines);

    let mut nr_lines_before = min((nr_lines_to_show-1)/2,current_line);
    let nr_lines_after = min(nr_lines_to_show-1-nr_lines_before,nr_of_lines-(current_line+1));
    nr_lines_before = max(nr_lines_before,nr_lines_to_show.saturating_sub(nr_lines_after+1));

    let line_iter = gcode.split("\n").map(|line|Element::from(text::Rich::with_spans(highlight_gcode(line))));
    let first5 = line_iter.clone().skip(current_line-nr_lines_before).take(nr_lines_before);
    let selected = line_iter.clone().skip(current_line).next().unwrap();
    let last5 = line_iter.clone().skip(current_line+1).take(nr_lines_after);
    let x = selected.as_widget().size();
    dbg!(x);

    Element::from(
        container(column![
            Column::with_children(first5).padding([0,5]),
            container(selected).style(container::bordered_box).align_left(Fill).padding([0,5]),
            Column::with_children(last5).padding([0,5]),
        ])
        .style(transparent_box)
        .align_left(Shrink)
        .padding(5)
    )
}


// fn highlight_gcode(code: &str) -> Vec<Span<'static>> {
fn highlight_gcode<'a>(code: &'a str) -> Vec<Span<'a>> {
    let mut spans = Vec::new();

    // for line in code.lines() {
    for line in code.split_inclusive('\n') {
        let (code_part,comment_part) = if let Some(pos) = line.find(';') {
                let (code_part,comment_part) = line.split_at(pos);
                (code_part,Some(comment_part))
            } else {
                (line, None)
            };

        for token in code_part.split_inclusive(char::is_whitespace) {
            let mut chars = token.chars();

            if let Some(first) = chars.next() {
                match first {
                    'G' | 'M' => {
                        spans.push( Span::new(token).color(Color::from_rgb(0.8, 0.2, 0.2)) );
                    }
                    'X' | 'Y' | 'Z' | 'E' | 'F' | 'S' => {
                        spans.push( Span::new(first).color(Color::from_rgb(0.2, 0.6, 1.0)) );
                        if first.len_utf8() < token.len(){
                            let rest:&str = &token[first.len_utf8()..];
                            spans.push( Span::new(rest).color(Color::from_rgb(0.6, 0.8, 1.0)) );
                        }
                    }
                    _ => { spans.push( Span::new(token) ); }
                }
            }
        }

        if let Some(comment) = comment_part {
            spans.push( Span::new(comment).color(Color::from_rgb(0.5, 0.5, 0.5)) );
        }
    }
    return spans
}

use std::ops::Index;
use std::ops::Range;

#[derive(Debug)]
pub struct GcodeView {
    s:String,
    layers: Vec<usize>,
    lines:  Vec<usize>
}
impl GcodeView {
    pub fn new(gcode:String) -> Self {
        //TODO: fix panic when input string is short

        let mut lines = Vec::with_capacity(gcode.len()/25); //gestimate how much space is needed
        let mut layers = Vec::with_capacity(gcode.len()/1139); //gestimate how much space is needed
        lines.push(0);
        let mut line_start = 0;
        for line_idx in gcode.match_indices('\n').map(|(idx,_)| idx+1){
            lines.push(line_idx);
            let line = &gcode[line_start..line_idx];
            if line.contains(";LAYER_CHANGE") {
                layers.push(lines.len()-1)
            }
            line_start = line_idx;
        }
        let string_last_idx = gcode.len();
        lines.push(string_last_idx);
        layers.push(lines.len()-2);

        GcodeView{
            s:gcode,
            layers,
            lines,
        }
    }
    pub fn as_str(&self) -> &str { &self.s }
    pub fn lines(&self) -> impl Iterator<Item = &str> {
        self.lines.iter()
            .zip( self.lines.iter().skip(1) )
            .map(|(start,end)| &self.s[*start..(*end)] )
    }
    pub fn nr_of_lines_in_layer(&self, layer_idx:usize) -> usize {
        self.layers[layer_idx+1] - self.layers[layer_idx]
    }
    pub fn nr_of_layers(&self) -> usize {
        self.layers.len()-1
    }
    pub fn view(&self,layer:u32, line:u32) -> Element<'_, crate::Message> {
        let current_line = self.layers[layer as usize] + (line as usize);

        let nr_lines_to_show = min(20,self.lines.len()-1);
        let nr_of_lines = self.lines.len()-1;
        // assert!(current_line<nr_of_lines);

        let mut nr_lines_before = min((nr_lines_to_show-1)/2,current_line);
        let nr_lines_after = min(nr_lines_to_show-1-nr_lines_before,nr_of_lines-(current_line+1));
        nr_lines_before = max(nr_lines_before,nr_lines_to_show.saturating_sub(nr_lines_after+1));


        // let first5 = text::Rich::with_spans(highlight_gcode(&self[(current_line-nr_lines_before)..current_line]));
        // let selected = text::Rich::with_spans(highlight_gcode(&self[current_line]));
        // let last5 = text::Rich::with_spans(highlight_gcode(&self[(current_line+1)..(current_line+nr_lines_after+1)]));
        let first5 = self.lines()
            .map(|line|Element::from(text::Rich::with_spans(highlight_gcode(line))))
            .skip(current_line-nr_lines_before)
            .take(nr_lines_before);
        let selected = self.lines()
            .map(|line|Element::from(text::Rich::with_spans(highlight_gcode(line))))
            .skip(current_line)
            .next()
            .unwrap();
        let last5 = self.lines()
            .map(|line|Element::from(text::Rich::with_spans(highlight_gcode(line))))
            .skip(current_line+1)
            .take(nr_lines_after);

        // dbg!(selected.as_widget().line_height());

        Element::from(
            container(column![
                // container(first5).padding([0,5]),
                Column::with_children(first5).padding([0,5]),
                container(selected).style(container::bordered_box).align_left(Fill).padding([0,5]),
                // container(last5).padding([0,5]),
                Column::with_children(last5).padding([0,5]),
            ])
            .style(transparent_box)
            .align_left(Shrink)
            .padding(5)
        )
    }
}
impl Index<usize> for GcodeView{
    type Output = str;
    fn index(&self,index:usize) -> &Self::Output{
        &self.s[self.lines[index]..self.lines[index+1]]
    }
}
impl Index<Range<usize>> for GcodeView{
    type Output = str;
    fn index(&self,range:Range<usize>) -> &Self::Output{
        &self.s[self.lines[range.start]..self.lines[range.end]]
    }
}


#[test]
fn test_gcode_view(){
    let gcode_str = ";halla
G1 X123 Y34 Z328
;LAYER_CHANGE
G1 X3829 Y2389
G1 E2389
;LAYER_CHANGE
G2
;LAYER_CHANGE
G2
;LAYER_CHANGE
G1
G2
G3
G4
G5
;LAYER_CHANGE
G1
G2
G3
G4
G5
G6
G6";
//     let gcode_str = ";halla
// G1 X123 Y34 Z328
// ;LAYER_CHANGE
// G1 X3829 Y2389
// G1 E2389
// ;LAYER_CHANGE
// G2
// G3";
    let gcode = GcodeView::new(String::from(gcode_str));
    dbg!(&gcode);
    dbg!(&gcode[0]);
    dbg!(&gcode[1]);
    dbg!(&gcode[2]);
    dbg!(&gcode[3]);
    dbg!(&gcode[4]);
    dbg!(&gcode[5]);
    dbg!(&gcode[6]);
    dbg!(&gcode[7]);
    // dbg!(&gcode[8]);

    assert!(false)
}

// #[test]
// fn test_gcode_view2(){
//     let gcode = GcodeView::new(String::from(test_gcode::TEST_GCODE));
//     let layer_nr = gcode.nr_of_layers()
//     gcode.view(, gcode.nr_of_lines_in_layer(layer_idx))
// }
//
