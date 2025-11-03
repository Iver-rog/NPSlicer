struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,
    light_color: vec4<f32>,
}

const CUBE_BASE_COLOR: vec4<f32> = vec4<f32>(0.24, 1.0, 0.8, 1.0);

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var sky_texture: texture_cube<f32>;
@group(0) @binding(2) var tex_sampler: sampler;
@group(0) @binding(3) var normal_texture: texture_2d<f32>;

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) tangent: vec3<f32>,
    @location(3) uv: vec2<f32>,
}

struct Instance {
    @location(4) matrix_0: vec4<f32>,
    @location(5) matrix_1: vec4<f32>,
    @location(6) matrix_2: vec4<f32>,
    @location(7) matrix_3: vec4<f32>,
    @location(8) normal_matrix_0: vec3<f32>,
    @location(9) normal_matrix_1: vec3<f32>,
    @location(10) normal_matrix_2: vec3<f32>,
}

struct Output {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
}

@vertex
fn vs_main(vertex: Vertex, cube: Instance) -> Output {
    let cube_matrix = mat4x4<f32>(
        cube.matrix_0, cube.matrix_1, cube.matrix_2, cube.matrix_3
    );

    let normal_matrix = mat3x3<f32>(
        cube.normal_matrix_0, cube.normal_matrix_1, cube.normal_matrix_2
    );

    let world_pos = cube_matrix * vec4<f32>(vertex.position, 1.0);
    let world_normal = normalize(normal_matrix * vertex.normal);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.world_pos = world_pos.xyz;
    out.world_normal = world_normal;
    return out;
}

@fragment
fn fs_main(in: Output) -> @location(0) vec4<f32> {
    let to_camera = normalize(uniforms.camera_pos.xyz - in.world_pos);
    let facing = max(dot(in.world_normal, to_camera), 0.0);

    let brightness = pow(facing, 0.5) * 0.8 + 0.2;

    var color = CUBE_BASE_COLOR * brightness;
    color.a = 1.0;
    return color;
}

