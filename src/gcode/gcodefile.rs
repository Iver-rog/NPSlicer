use std::io::{self,BufWriter,Write};
use std::fs::File;
use std::f32::consts::PI;

use nalgebra::Point3;

use super::Path;

pub struct Settings{
    pub layer_height: f32,
    pub infill_percentage: usize,// %
    pub nozzle_diameter: f32, // mm
    pub perimeter_line_width: f32,
    pub infill_line_width: f32,
    pub n_perimeters: usize,
    pub infill_overlap_percentage: usize,
    pub z_hop: f32,

    pub max_staydown_distance: f32,
    pub min_edge_length: f32,   // mm
    pub filament_diameter: f32, // mm
    pub layer_fan_start: usize,
    pub perimeter_start_xy: [isize;2],
    pub translate_xy: [f32;2],

    // my fields
    pub max_nonplanar_thicness:f32, // mm  the largest z-distance a non planar layer can have
    pub travel_feedrate:usize, // mm/s?
    pub retract_feedrate:usize,// mm/s?
    pub extrusion_feedrate:usize,
    pub non_planar_travel_feedrate:usize,
    pub fan_start:usize, // at which layer the fan should turn on
    pub retract_distance: usize,// nm (10⁻³ mm)  retract distance(movement of extruder) during travel moves
}
impl Default for Settings {
    fn default() -> Self {
        let layer_thickness = 0.2;
        let nozzle_diameter = 0.6;
        let infill_percentage = 10;
        Self {
            layer_height: layer_thickness,
            infill_percentage,
            nozzle_diameter,
            perimeter_line_width: 0.45,
            infill_line_width: 0.6,
            n_perimeters: 2,
            infill_overlap_percentage: 0,
            z_hop: layer_thickness,
               
            max_staydown_distance: 1.5 * nozzle_diameter/((infill_percentage as f32)/100.),
            min_edge_length: 0.5,//mm
            filament_diameter: 1.75,//mm
            layer_fan_start: 3,
            perimeter_start_xy: [-30, -15],
            translate_xy: [100., 100.],

            max_nonplanar_thicness: 50., //mm
            travel_feedrate: 10800,//mm/s?
            retract_feedrate: 2500,//mm/s?
            extrusion_feedrate: 1500,//mm/s?
            non_planar_travel_feedrate: 5040,//mm/s?
            fan_start:3,
            retract_distance: 800 // nm (10⁻³ mm)
        }
    }
}

pub struct GcodeFile<'a>{
    file: BufWriter<File>,
    settings: &'a Settings,
    n_layer: usize,
}

impl GcodeFile<'_> {
    pub fn new<'a>(path:&'_ str, settings:&'a Settings) -> GcodeFile<'a> {
        // let mut file = File::options()
        //     .read(true)
        //     .open(format!("{path}.gcode"))
        //     .unwrap();
        //
        println!("{path}");
        let mut file = File::create(format!("{path}/gcode.gcode")).unwrap();
        let mut buffer = BufWriter::new(file);

        writeheader(&mut buffer, settings).unwrap();

        return GcodeFile { file:buffer, n_layer:0, settings }
    }

    pub fn layer(&mut self,mut paths: impl Iterator<Item = Path>) -> Result<(),io::Error> {
        let f = &mut self.file;
        let s = self.settings;

        writeln!(f,"\n;Start layer {}",self.n_layer)?;
        writeln!(f,"M117 Layer {}",self.n_layer)?;
        if s.fan_start == self.n_layer { writeln!(f,"M106 ; Fan on")?; }

        self.n_layer += 1;
        let safe_z = self.n_layer as f32 * s.layer_height;
        let [x_offset,y_offset] = s.translate_xy;

        writeln!(f,"G92 E0.0 ; Reset extruder distance")?;
        writeln!(f,"G1 Z{:.3} F{:.3}",safe_z,s.travel_feedrate)?;
        writeln!(f,"G1 E{} F{} ; De-retract",(s.retract_distance as f32)/1000.,s.retract_feedrate)?;  // De-Retraction
        let mut end_of_previous_layer:Option<Point3<f32>> = None; 

        for path in paths {
            let end = path.points.last().clone().unwrap();
            let mut points = path.points.iter();
            let start = points.next().unwrap();

            let should_retract = match end_of_previous_layer {
                Some(prev_end) => { (start-prev_end).magnitude() > s.max_staydown_distance },
                None => false, // <- first path in new layer no need for retraction
            };

            if should_retract {
                writeln!(f,"G1 E-{} F{} ; Retract",(s.retract_distance as f32)/1000.,s.retract_feedrate)?;  // Retraction
                writeln!(f,"G1 Z{:.3} F{:.3}",safe_z,s.travel_feedrate)?;
                writeln!(f,"G1 X{:.3} Y{:.3} F{:.3}",start.x+x_offset,start.y+y_offset,s.travel_feedrate)?;
                writeln!(f,"G1 Z{:.3} F{:.3}",start.z,s.travel_feedrate)?;
                writeln!(f,"G1 E{} F{} ; De-retract",(s.retract_distance as f32)/1000.,s.retract_feedrate)?;  // De-Retraction
                writeln!(f,"G92 E0.0 ; Reset extruder distance")?;    // Reset extruder distance
            } else {
                writeln!(f,"G1 X{:.3} Y{:.3} F{:.3}",start.x+x_offset,start.y+y_offset,s.travel_feedrate)?;
                writeln!(f,"G1 Z{:.3} F{:.3}",start.z,s.travel_feedrate)?;
            }

            let extrusion_area = PI*(s.filament_diameter/2.).powi(2); 
            let line_area = s.layer_height*s.perimeter_line_width;
            let extrusion_multiplier = line_area/extrusion_area;

            let mut prev_point = start;
            for point in points{
                // let line_length = (point.xy() - prev_point.xy()).magnitude();
                let line_length = (point - prev_point).magnitude();
                let extrusion_length = line_length*extrusion_multiplier;

                let (x,y,z) = (point.x+x_offset, point.y+y_offset, point.z);

                writeln!(f,"G1 X{x:.3} Y{y:.3} Z{z:.3} E{extrusion_length:.5} F{}",s.extrusion_feedrate)?;

                prev_point = point;
            }
            end_of_previous_layer = Some(*prev_point);
        }

        writeln!(f,"G1 E-{} F{} ; Retract",(s.retract_distance as f32)/1000.,s.retract_feedrate)?;  // Retraction
        writeln!(f,"G1 Z{:.3} F{}",safe_z, s.travel_feedrate)?;
        writeln!(f,";End of layer {}",self.n_layer)?;
        Ok(())
    }
}

fn writeheader<W: Write>(f:&mut BufWriter<W>,settings:&Settings) -> Result<(),io::Error> {
    writeln!(f,"; Generated by layer-gen-rs")?;
    writeln!(f,"; Layer thickness: {}",settings.layer_height)?;
    writeln!(f,"; Nozzle diameter: {}",settings.nozzle_diameter)?;
    writeln!(f,"; Filament diameter: {}",settings.filament_diameter)?;
    writeln!(f,"; Perimeter line width: {}",settings.perimeter_line_width)?;
    writeln!(f,"; Infill line width: {}",settings.infill_line_width)?;
    // writeln!(f,"; Infill angle: {}",settings.infill_angle)?;
    // writeln!(f,"; Infill line spacing: {}",settings.infill_line_spacing)?;
    writeln!(f,"; Layer fan start: {}",settings.layer_fan_start)?;
    writeln!(f,"; Max staydown distance: {}",settings.max_staydown_distance)?;
    // writeln!(f,"; Seam position: {}",settings.seam_position)?;
    // writeln!(f,"; Translate XY: {translate_xy}")?;

    // Starting gcode from PrusaSlicer MK3 config
    writeln!(f,"M201 X1000 Y1000 Z200 E5000 ; sets maximum accelerations, mm/sec^2")?;
    writeln!(f,"M203 X200 Y200 Z12 E120 ; sets maximum feedrates, mm / sec")?;
    writeln!(f,"M204 S1250 T1250 ; sets acceleration (S) and retract acceleration (R), mm/sec^2")?;
    writeln!(f,"M205 X8.00 Y8.00 Z0.40 E4.50 ; sets the jerk limits, mm/sec")?;
    writeln!(f,"M205 S0 T0 ; sets the minimum extruding and travel feed rate, mm/sec")?;
    writeln!(f,"G90 ; use absolute coordinates")?;
    writeln!(f,"M83 ; extruder relative mode")?;
    writeln!(f,"M104 S180 ; set extruder preheat temp")?;
    writeln!(f,"M106 ; Fan on")?;
    writeln!(f,"M140 S60 ; set bed temp")?;
    writeln!(f,"M190 S60 ; wait for bed temp")?;
    writeln!(f,"M104 S215 ; set extruder temp")?;

    writeln!(f,"G28 W ; home all without mesh bed level")?;
    writeln!(f,"G80 ; mesh bed leveling")?;
    writeln!(f,"M104 S215 ; set extruder temp")?;
    writeln!(f,"M109 S215 ; wait for extruder temp")?;

    // Purge f,line
    writeln!(f,"G1 Z0.2 F720")?;
    writeln!(f,"G1 Y-3 F1000 ; go outside print area")?;
    writeln!(f,"G92 E0")?;
    writeln!(f,"M107 ; Fan off")?;
    writeln!(f,"G1 X60 E9 F1000 ; intro line")?;
    writeln!(f,"G1 X100 E12.5 F1000 ; intro line")?;
    writeln!(f,"G92 E0.0")?;

    writeln!(f,"M221 S100 ; M221 - Set extrude factor override percentage")?;
    writeln!(f,"G21 ; set units to millimeters")?;
    writeln!(f,"G90 ; use absolute coordinates")?;
    writeln!(f,"M83 ; use relative distances for extrusion")?;
    writeln!(f,"M900 K0.04 ; Filament gcode LA 1.5")?;
    writeln!(f,"M900 K18 ; Filament gcode LA 1.0")?;
    writeln!(f,"M107 ; Fan off")?;
    Ok(())
}

// NOTE: Might not be a good idea to do it this way
#[allow(unused)]
impl Drop for GcodeFile<'_> {
    fn drop(&mut self){
        let f = &mut self.file;
        let s = self.settings;
        writeln!(f,"\n;end prosedure");

        // Wipe (set relative mode, move to X2.0, Y2.0)
        writeln!(f,"G91 ; relative mode");
        writeln!(f,"G1 X2.0 Y2.0 E-0.4 F{} ; wipe and retract",s.travel_feedrate);
        writeln!(f,"G01 E-0.1 F{} ; retract some more",s.travel_feedrate);
        writeln!(f,"G90 ; absolute mode");

        // Turn off heaters and fan
        writeln!(f,"M104 S0 ; turn off extruder");
        writeln!(f,"M140 S0 ; turn off bed");
        writeln!(f,"M107 ; fan off");

        // Move up
        writeln!(f,"G1 Z{} F{} ; move up",s.max_nonplanar_thicness,s.travel_feedrate);

        // Present print
        writeln!(f,"G1 Y200 F{} ; present print",(s.travel_feedrate/2));

        // Home x
        writeln!(f,"G28 X ; home x");

        // Turn off motors
        writeln!(f,"M84 ; disable motors");
    }
}
