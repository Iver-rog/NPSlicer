
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
            nozzle_diameter,
            layer_height: 0.2,
            infill_percentage,
            nr_of_perimeters: 2,
            perimeter_line_width: 0.45,
            infill_line_width: 0.6,
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
