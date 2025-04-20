use std::io::{self,BufWriter,Write};
use std::fs::File;
use std::f32::consts::PI;

use nalgebra::{partial_max, Point3};

use super::path::PathType;
use super::Path;

pub struct Settings{
    pub nozzle_diameter: f32,       // mm
    pub layer_height: f32,          // mm
    pub infill_percentage: usize,   // %
    pub nr_of_perimeters: usize,    // int
    pub perimeter_line_width: f32,  // mm
    pub infill_line_width: f32,     // mm

    /// Wether to print perimeters from the outside in (true)
    /// or from the inside out (false)
    pub outer_wall_first: bool,

    /// [int] The number of brim contours. set to zero to dissable brims
    pub brim: usize,
    
    /// [%] The overlap between the infill lines and inner most contour
    pub infill_overlap_percentage: usize,

    /// [mm] The maximum lenght a travel move can have without requiering a 
    /// retract opperation. Used to allow the hotend to travel to the 
    /// start of the next path without moving up to the safe z-height.
    pub max_staydown_distance: f32,

    /// [mm] Diameter of the fillament (usually 1.75mm)
    pub filament_diameter: f32,

    /// [(x,y) mm] A displacement applied to the model during gcode parsing.
    /// Used to center the model on the build plate.
    pub translate_xy: [f32;2],

    /// [int] The layer number at which the fan should turn on (first layer is 0)
    pub fan_start:usize,

    /// [nm (10⁻³ mm)] The length of fillament to retract before travel moves
    pub retract_distance: usize,

    pub feedrates:Feedrates,
}

pub struct Feedrates {
    /// [mm/min] The feedrate of the axis during first layer perimeter
    pub initial_layer:usize,

    /// [mm/min] The feedrate of the axis during first layer perimeter
    pub initial_layer_infill:usize,

    /// [mm/min] The feedrate for the outer wall during normal printing
    pub outer_wall:usize,

    /// [mm/min] The feedrate for the inner walls during normal printing
    pub inner_wall:usize,

    /// [mm/min] The feedrate for infill during normal printing
    pub infill:usize,

    /// [mm/min] The feedrate of the axis during travel moves
    pub travel:usize,

    /// [mm/min] The feedrate of the extruder during retraction
    pub retract:usize,

    /// [mm/min] The feddrate of the z-axis during strictly vertical movement
    pub z_max:usize,
}

impl Default for Settings {
    fn default() -> Self {
        let nozzle_diameter = 0.6;
        let infill_percentage = 10;
        Self {
            layer_height: 0.2,
            infill_percentage,
            nozzle_diameter,
            perimeter_line_width: 0.45,
            infill_line_width: 0.6,
            nr_of_perimeters: 2,
            infill_overlap_percentage: 0,
            outer_wall_first: true,
               
            max_staydown_distance: 1.5 * nozzle_diameter/((infill_percentage as f32)/100.),
            filament_diameter: 1.75,//mm
            translate_xy: [100., 100.],

            fan_start:3,
            retract_distance: 800, // nm (10⁻³ mm)
            brim: 20,
            feedrates:Feedrates::default(),
        }
    }
}

impl Default for Feedrates {
    fn default() -> Self {
        Self {
            initial_layer:1200,
            initial_layer_infill:2400,
            outer_wall:1600, //orca 45mm/s = 2700mm/min
            inner_wall:1800, //orca 60mm/s = 3600mm/min
            infill:2500, //orca 70mm/s = 4200mm/min
            travel: 10800,
            retract: 2500,
            z_max: 720,
        }
    }
}

pub struct GcodeFile<'a>{
    file: BufWriter<File>,
    settings: &'a Settings,
    n_layer: usize,
    safe_z: f32,
}

impl GcodeFile<'_> {
    pub fn new<'a>(path:&'_ str, settings:&'a Settings) -> GcodeFile<'a> {

        let mut file = File::create(format!("{path}/gcode.gcode")).unwrap();
        let mut buffer = BufWriter::new(file);

        writeheader(&mut buffer, settings).unwrap();
        let safe_z = settings.layer_height;

        return GcodeFile{ file:buffer, settings, n_layer:0, safe_z }
    }

    pub fn layer(&mut self,mut paths: impl Iterator<Item = Path>) -> Result<(),io::Error> {
        let f = &mut self.file;
        let s = self.settings;

        let (outer_wall_feedrate, inner_wall_feedrate) = match self.n_layer == 0 {
            true  => (s.feedrates.initial_layer, s.feedrates.initial_layer),
            false => (s.feedrates.outer_wall,    s.feedrates.inner_wall   ),
        };
        let infill_feedrate = match self.n_layer == 0 {
            true => s.feedrates.initial_layer_infill,
            false => s.feedrates.infill,
        };

        writeln!(f,"\n;Start layer {}",self.n_layer)?;
        writeln!(f,"M117 Layer {}",self.n_layer)?;
        if s.fan_start == self.n_layer { writeln!(f,"M106 ; Fan on")?; }
        self.n_layer += 1;

        let [x_offset,y_offset] = s.translate_xy;

        writeln!(f,"G92 E0.0 ; Reset extruder distance")?;
        writeln!(f,"G1 Z{:.3} F{:.3}",self.safe_z,s.feedrates.z_max)?;
        writeln!(f,"G1 E{} F{} ; De-retract",(s.retract_distance as f32)/1000.,s.feedrates.retract)?;  // De-Retraction
        let mut end_of_previous_layer:Option<Point3<f32>> = None; 

        for path in paths {
            let end = path.points.last().clone().unwrap();
            let mut points = path.points.iter();
            let start = points.next().unwrap();

            let should_retract = match end_of_previous_layer {
                Some(prev_end) => { (start-prev_end).magnitude() > s.max_staydown_distance },
                None => false, // <- first path in new layer no need for retraction
            };

            let (start_x, start_y, start_z) = (start.x+x_offset, start.y+y_offset ,start.z);
            self.safe_z = self.safe_z.max( start_z );

            if should_retract {
                writeln!(f,"G1 E-{} F{} ; Retract",(s.retract_distance as f32)/1000.,s.feedrates.retract)?;  // Retraction
                writeln!(f,"G1 Z{:.3} F{:.3}",self.safe_z,s.feedrates.z_max)?;
                writeln!(f,"G1 X{start_x:.3} Y{start_y:.3} F{:.3}", s.feedrates.travel)?;
                writeln!(f,"G1 Z{start_z:.3} F{:.3}",s.feedrates.z_max)?;
                writeln!(f,"G1 E{} F{} ; De-retract",(s.retract_distance as f32)/1000.,s.feedrates.retract)?;  // De-Retraction
                writeln!(f,"G92 E0.0 ; Reset extruder distance")?;
            } else {
                writeln!(f,"G1 X{start_x:.3} Y{start_y:.3} Z{start_z:.3} F{:.3}", s.feedrates.travel)?;
            }

            let feedrate = match path.path_type {
                PathType::OuterWall => outer_wall_feedrate,
                PathType::InnerWall => inner_wall_feedrate,
                PathType::Infill    => infill_feedrate,
            };
            let line_width = match path.path_type {
                PathType::OuterWall => s.perimeter_line_width,
                PathType::InnerWall => s.perimeter_line_width,
                PathType::Infill    => s.infill_line_width,
            };
            match path.path_type {
                PathType::OuterWall => writeln!(f,";OuterWall")?,
                PathType::InnerWall => writeln!(f,";InnerWall")?,
                PathType::Infill    => writeln!(f,";Infill")?,
            };

            let extrusion_area = PI*(s.filament_diameter/2.).powi(2); 
            let line_area = s.layer_height * line_width;
            let extrusion_multiplier = line_area/extrusion_area;

            let mut prev_point = start;
            for point in points{
                self.safe_z = self.safe_z.max( point.z );
                // let line_length = (point.xy() - prev_point.xy()).magnitude();
                let line_length = (point - prev_point).magnitude();
                let extrusion_length = line_length*extrusion_multiplier;

                let (x,y,z) = (point.x+x_offset, point.y+y_offset, point.z);

                writeln!(f,"G1 X{x:.3} Y{y:.3} Z{z:.3} E{extrusion_length:.5} F{feedrate}")?;

                prev_point = point;
            }
            end_of_previous_layer = Some(*prev_point);
        }

        writeln!(f,"G1 E-{} F{} ; Retract",(s.retract_distance as f32)/1000.,s.feedrates.retract)?;  // Retraction
        writeln!(f,"G1 Z{:.3} F{}",self.safe_z, s.feedrates.z_max)?;
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
    writeln!(f,"; fan start: {}",settings.fan_start)?;
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
        writeln!(f,"G1 X2.0 Y2.0 E-0.4 F{} ; wipe and retract",s.feedrates.travel);
        writeln!(f,"G01 E-0.1 F{} ; retract some more",s.feedrates.travel);
        writeln!(f,"G90 ; absolute mode");

        // Turn off heaters and fan
        writeln!(f,"M104 S0 ; turn off extruder");
        writeln!(f,"M140 S0 ; turn off bed");
        writeln!(f,"M107 ; fan off");

        // Move up
        writeln!(f,"G1 Z{} F{} ; move up",self.safe_z + 10.,s.feedrates.travel);

        // Present print
        writeln!(f,"G1 Y200 F{} ; present print",(s.feedrates.travel/2));

        // Home x
        writeln!(f,"G28 X ; home x");

        // Turn off motors
        writeln!(f,"M84 ; disable motors");
    }
}
