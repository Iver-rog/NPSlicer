pub mod camera;
pub mod pipeline;

use camera::{Camera, CameraEvent};
use pipeline::Pipeline;
use pipeline::vertex::Vertex;

use crate::wgpu;
use pipeline::instance::{self, Instance};

use iced::{mouse, Point};
use iced::time::Duration;
use iced::widget::shader::{self, Viewport};
use iced::{Color, Rectangle};

use glam::Vec3;
use rand::Rng;
use std::cmp::Ordering;
use std::iter;

pub const MAX: u32 = 500;

#[derive(Clone)]
pub struct Scene {
    pub size: f32,
    pub instances: Vec<Instance>,
    // pub printbed: stl_io::IndexedMesh,
    pub printbed: Vec<Vertex>,
    pub camera: Camera,
    pub show_depth_buffer: bool,
    pub light_color: Color,
}

impl Scene {
    pub fn new() -> Self {

        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/2-test.stl").unwrap();
        let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/printbeds/world axis.stl").unwrap();
        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/bunny.stl").unwrap();
        // let file = std::fs::File::open("/home/iver/Documents/NTNU/Master/layer-gen-rs/mesh/stanford-armadillo.stl").unwrap();
        let mut reader = std::io::BufReader::new(file);
        let mesh = stl_io::read_stl(&mut reader).unwrap();
        let vertex_buffer = crate::io::IntoVertexBuffer::into_vertex_buffer(&mesh);

        Self {
            size: 0.2,
            instances: vec![Instance::new(1.0, Vec3::new(0.0,0.0,0.0))],
            printbed: vertex_buffer,
            camera: Camera::default(),
            show_depth_buffer: false,
            light_color: Color::WHITE,
        }
    }

    pub fn update(&mut self, time: Duration) {
        // self.camera.eye[0] += 1.;
        // for cube in self.cubes.iter_mut() {
        //     cube.update(self.size, time.as_secs_f32());
        // }
    }

    pub fn change_amount(&mut self, amount: u32) {
        let curr_cubes = self.instances.len() as u32;

        match amount.cmp(&curr_cubes) {
            Ordering::Greater => {
                // spawn
                let cubes_2_spawn = (amount - curr_cubes) as usize;

                let mut cubes = 0;
                self.instances.extend(iter::from_fn(|| {
                    if cubes < cubes_2_spawn {
                        cubes += 1;
                        Some(Instance::new(self.size, rnd_origin()))
                    } else {
                        None
                    }
                }));
            }
            Ordering::Less => {
                // chop
                let cubes_2_cut = curr_cubes - amount;
                let new_len = self.instances.len() - cubes_2_cut as usize;
                self.instances.truncate(new_len);
            }
            Ordering::Equal => {}
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

use crate::Message;
// impl<Message> shader::Program<Message> for Scene {
impl shader::Program<Message> for Scene {
    // type State = ();
    type State = InternalState;
    type Primitive = Primitive;

    fn update(
            &self,
            state: &mut Self::State,
            event: &iced::Event,
            _bounds: Rectangle,
            cursor: iced::advanced::mouse::Cursor,
        ) -> Option<shader::Action<Message>> {
        use iced::mouse::Cursor;

        let mut camera_event = camera::CameraEvent::default();

        match cursor {
            Cursor::Available(point) => {
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
                    ScrollDelta::Pixels{x:_,y} => { camera_event.zoom = *y; },
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
            self.light_color,
        )
    }
}

/// A collection of `Cube`s that can be rendered.
#[derive(Debug)]
pub struct Primitive {
    // printbed: Vec<crate::scene::pipeline::vertex::Vertex>,
    printbed: Vec<crate::scene::pipeline::vertex::Vertex>,
    cubes: Vec<instance::Raw>,
    uniforms: pipeline::Uniforms,
    show_depth_buffer: bool,
}

impl Primitive {
    pub fn new(
        cubes: &[Instance],
        // printbed: &stl_io::IndexedMesh,
        printbed: &Vec<Vertex>,
        camera: &Camera,
        bounds: Rectangle,
        show_depth_buffer: bool,
        light_color: Color,
    ) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, light_color);

        Self {
            cubes: cubes
                .iter()
                .map(instance::Raw::from_cube)
                .collect::<Vec<instance::Raw>>(),
            // printbed: crate::io::IntoVertexBuffer::into_vertex_buffer(printbed),
            printbed:printbed.clone(),
            uniforms,
            show_depth_buffer,
        }
    }
}

impl shader::Primitive for Primitive {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
        storage: &mut shader::Storage,
        _bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        if !storage.has::<Pipeline>() {
            storage.store(Pipeline::new(
                device,
                queue,
                format,
                viewport.physical_size(),
                &self.printbed,
            ));
        }

        let pipeline = storage.get_mut::<Pipeline>().unwrap();

        // Upload data to GPU
        pipeline.update(
            device,
            queue,
            viewport.physical_size(),
            &self.uniforms,
            self.cubes.len(),
            &self.cubes,
            &self.printbed,
        );
    }

    fn render(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        storage: &shader::Storage,
        target: &wgpu::TextureView,
        clip_bounds: &Rectangle<u32>,
    ) {
        // At this point our pipeline should always be initialized
        let pipeline = storage.get::<Pipeline>().unwrap();

        // Render primitive
        pipeline.render(
            target,
            encoder,
            *clip_bounds,
            self.cubes.len() as u32,
            self.show_depth_buffer,
        );
    }
}

fn rnd_origin() -> Vec3 {
    Vec3::new(
        rand::thread_rng().gen_range(-4.0..4.0),
        rand::thread_rng().gen_range(-4.0..4.0),
        rand::thread_rng().gen_range(-4.0..2.0),
    )
}
