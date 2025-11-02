use glam::{mat4, vec3, vec4, Quat};
use iced::Rectangle;

#[derive(Copy, Clone)]
pub struct Camera {
    pub eye: glam::Vec3,
    target: glam::Vec3,
    up: glam::Vec3,
    fov_y: f32,
    near: f32,
    far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            eye: vec3(-10.0, -70.0, 20.0),
            target: vec3(-10.0, 0.0, 2.0),
            up: glam::Vec3::Z,
            fov_y: 45.0,
            near: 10.0,
            far: 10000.0,
        }
    }
}

pub const OPENGL_TO_WGPU_MATRIX: glam::Mat4 = mat4(
    vec4(1.0, 0.0, 0.0, 0.0),
    vec4(0.0, 1.0, 0.0, 0.0),
    vec4(0.0, 0.0, 0.5, 0.0),
    vec4(0.0, 0.0, 0.5, 1.0),
);

impl Camera {
    pub fn build_view_proj_matrix(&self, bounds: Rectangle) -> glam::Mat4 {
        //TODO looks distorted without padding; base on surface texture size instead?
        let aspect_ratio = ( bounds.width + 280. )/ bounds.height;

        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(
            self.fov_y,
            aspect_ratio,
            self.near,
            self.far,
        );

        OPENGL_TO_WGPU_MATRIX * proj * view
    }

    pub fn position(&self) -> glam::Vec4 {
        glam::Vec4::from((self.eye, 0.0))
    }
    pub fn handle_event(&mut self, event:CameraEvent){
        // let camera_tangent = self.eye.cross(vec3(0.0,0.0,1.0)).normalize();
        let camera_tangent = (self.eye-self.target).cross(vec3(0.0,0.0,1.0)).normalize();
        let camera_up = camera_tangent.cross(self.eye-self.target).normalize();

        let offset = self.eye - self.target;
        let yaw = Quat::from_rotation_z(-event.pan.x*0.01) * offset;
        let pitch_angle = if (offset.normalize().z.abs() > 0.99)
            &&(offset.z.is_sign_positive() == event.pan.y.is_sign_positive()) 
            {0.} else {event.pan.y};
        let pitch = Quat::from_axis_angle(camera_tangent, pitch_angle*0.01) ;
        self.eye = self.target + (pitch*yaw);

        self.eye = self.eye.lerp(self.target, event.zoom*0.05);

        let pan_sensitivity = 0.001 * (self.eye-self.target).length();
        self.eye += event.orbit.x*pan_sensitivity*camera_tangent;
        self.eye += event.orbit.y*pan_sensitivity*camera_up;
        self.target += event.orbit.x*pan_sensitivity*camera_tangent;
        self.target += event.orbit.y*pan_sensitivity*camera_up;

    }
}

#[derive(Debug,Clone,Default,PartialEq)]
pub struct CameraEvent{
    pub pan: iced::Vector,
    pub orbit: iced::Vector,
    pub zoom: f32,
}
