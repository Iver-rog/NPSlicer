
use crate::wgpu;

use glam::Vec3;


/// A single instance of a cube.
#[derive(Debug, Clone)]
pub struct Instance {
    pub rotation: glam::Quat,
    pub position: Vec3,
    pub scale: f32,
}

impl Default for Instance {
    fn default() -> Self {
        Self {
            rotation: glam::Quat::IDENTITY,
            position: glam::Vec3::ZERO,
            scale: 1.0,
        }
    }
}

#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct Raw {
    transformation: glam::Mat4,
    normal: glam::Mat3,
    _padding: [f32; 3],
}

impl Raw {
    const ATTRIBS: [wgpu::VertexAttribute; 7] = wgpu::vertex_attr_array![
        //cube transformation matrix
        4 => Float32x4,
        5 => Float32x4,
        6 => Float32x4,
        7 => Float32x4,
        //normal rotation matrix
        8 => Float32x3,
        9 => Float32x3,
        10 => Float32x3,
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }
}

impl Raw {
    pub fn from_instance(cube: &Instance) -> Raw {
        Raw {
            transformation: glam::Mat4::from_scale_rotation_translation(
                glam::vec3(cube.scale, cube.scale, cube.scale),
                cube.rotation,
                cube.position,
            ),
            normal: glam::Mat3::from_quat(cube.rotation),
            _padding: [0.0; 3],
        }
    }
}
