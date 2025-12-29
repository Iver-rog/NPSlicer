pub mod camera;
pub mod pipeline;

use camera::{Camera, CameraEvent};
use pipeline::Pipeline;
use pipeline::vertex::Vertex;

use crate::wgpu;
use crate::Message;
use pipeline::instance::{self, Instance};

use iced::{mouse, Point};
use iced::widget::shader::{self, Viewport};
use iced::{Color, Rectangle};

use glam::Vec3;

#[derive(Clone)]
pub struct Scene {
    pub size: f32,
    pub instances: Vec<Instance>,
    // pub printbed: stl_io::IndexedMesh,
    pub printbed: Vec<Vertex>,
    pub camera: Camera,
    pub show_depth_buffer: bool,
    pub model_color: Color,
}

impl Scene {
    pub fn new() -> Self {

        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/2-test.stl").unwrap();
        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/world axis.stl").unwrap();
        let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/arrow.stl").unwrap();
        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/bunny.stl").unwrap();
        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/stanford-armadillo.stl").unwrap();

        let mut reader = std::io::BufReader::new(file);
        let mesh = stl_io::read_stl(&mut reader).unwrap();
        let mut vertex_buffer = crate::io::IntoVertexBuffer::into_vertex_buffer(&mesh);

        let file2 = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/bunny.stl").unwrap();
        let mut reader2 = std::io::BufReader::new(file2);
        let mesh2 = stl_io::read_stl(&mut reader2).unwrap();
        crate::io::IntoVertexBuffer::append_to_vertex_buffer(&mesh2,&mut vertex_buffer);

        Self {
            size: 0.2,
            instances: vec![
                Instance{scale:1.0, position:Vec3::new(0.0,0.0,0.0), rotation:glam::Quat::IDENTITY},
                Instance{scale:1.0, position:Vec3::new(0.0,0.0,0.0), rotation:glam::Quat::from_rotation_x(3.12/2.)},
                Instance{scale:1.0, position:Vec3::new(0.0,0.0,0.0), rotation:glam::Quat::from_rotation_y(3.12/2.)},
            ],
            printbed: vertex_buffer,
            camera: Camera::default(),
            show_depth_buffer: false,
            model_color: Color::WHITE,
        }
    }

}

#[derive(Default,Debug)]
pub struct InternalState {
    orbiting: bool,
    shift: bool,
    paning: bool,
    prev_cursor_pos: Point,
}

impl shader::Program<Message> for Scene {
    type State = InternalState;
    type Primitive = Primitive;

    fn update(
            &self,
            state: &mut Self::State,
            event: &iced::Event,
            bounds: Rectangle,
            cursor: iced::advanced::mouse::Cursor,
        ) -> Option<shader::Action<Message>> {
        use iced::mouse::Cursor;

        let mut camera_event = camera::CameraEvent::default();

        match cursor {
            Cursor::Available(point) => {
                // Exit and do nothing if cursor is outside the widget
                if !( (bounds.x<point.x) && (point.x<(bounds.x+bounds.width)) && (bounds.y<point.y) && (point.y<(bounds.y+bounds.height)) ){
                    return None;
                }
                if state.paning{
                    camera_event.pan = point - state.prev_cursor_pos;
                };
                if state.orbiting{
                    camera_event.orbit = point - state.prev_cursor_pos;
                }
                state.prev_cursor_pos = point;
            },
            _ => {},
        };

        match event{
            iced::Event::Keyboard(iced::keyboard::Event::ModifiersChanged(modifiers)) => {
                state.shift = modifiers.shift();
            }
            iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                if state.shift {
                    state.orbiting = true;
                } else {
                    state.paning = true;
                }
            },
            iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                state.orbiting = false;
                state.paning = false;
            },
            iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Middle)) => {
                if state.shift {
                    state.orbiting = true;
                } else {
                    state.paning = true;
                }
            },
            iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Middle)) => {
                state.orbiting = false;
                state.paning = false;
            },
            iced::Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                use iced::mouse::ScrollDelta;
                match delta {
                    ScrollDelta::Lines{x:_,y} => { camera_event.zoom = *y; },
                    // ScrollDelta::Pixels{x:_,y} => { camera_event.zoom = *y; },
                    ScrollDelta::Pixels{x,y} => { 
                        if state.shift{ camera_event.zoom = *y; }
                        else if state.paning{ camera_event.orbit = iced::Vector::new(*x, *y); }
                        else { camera_event.pan = iced::Vector::new(*x, *y); }
                    },
                }
            },
            _ => {},
        };

        if camera_event != CameraEvent::default(){
            Some( shader::Action::publish(Message::Camera(camera_event)) )
        } else { None }
    }

    fn draw(
        &self,
        _state: &Self::State,
        _cursor: mouse::Cursor,
        bounds: Rectangle,
    ) -> Self::Primitive {
        Primitive::new(
            &self.instances,
            &self.printbed,
            &self.camera,
            bounds,
            self.show_depth_buffer,
            self.model_color,
        )
    }
}

/// A collection of `Instance`s that can be rendered.
#[derive(Debug)]
pub struct Primitive {
    // vertex_buffer: Vec<crate::scene::pipeline::vertex::Vertex>,
    vertex_buffer: Vec<crate::scene::pipeline::vertex::Vertex>,
    instance: Vec<instance::Raw>,
    uniforms: pipeline::Uniforms,
    show_depth_buffer: bool,
}

impl Primitive {
    pub fn new(
        cubes: &[Instance],
        printbed: &[Vertex],
        camera: &Camera,
        bounds: Rectangle,
        show_depth_buffer: bool,
        model_color: Color,
    ) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, model_color);

        Self {
            instance: cubes
                .iter()
                .map(instance::Raw::from_instance)
                .collect::<Vec<instance::Raw>>(),
            // printbed: crate::io::IntoVertexBuffer::into_vertex_buffer(printbed),
            vertex_buffer:printbed.to_vec(),
            uniforms,
            show_depth_buffer,
        }
    }
}

impl shader::Primitive for Primitive {
    type Pipeline = pipeline::Pipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        // Upload data to GPU
        pipeline.update(
            device,
            queue,
            viewport.physical_size(),
            &self.uniforms,
            self.instance.len(),
            &self.instance,
            &self.vertex_buffer,
        );
    }

    fn render(
        &self,
        pipeline: &Pipeline,
        encoder: &mut iced::wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &Rectangle<u32>,
    ) {
        pipeline.render(
            target,
            encoder,
            *clip_bounds,
            self.instance.len() as u32,
            self.show_depth_buffer,
        );
    }
}

