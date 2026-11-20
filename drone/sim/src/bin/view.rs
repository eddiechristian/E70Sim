//! A window that draws the drone in 3D, to look at it and turn it by hand.
//!
//! This is the drawing of the gamepad item in `drone.md`, on its own: no
//! flight code, no physics and no gamepad. The drone is a model file:
//! `models/x500.stl` unless `--model` names another. `models/README.md` says
//! where that file comes from.
//!
//! The drone is kept in the body's axes, x forward, y right and z down, and
//! is turned with the same `Quat` the physics uses. `to_window` is the one
//! place where those axes become the window's.
//!
//! The body's axes are drawn as three arrows from its centre: red forward,
//! green right and blue down.
//!
//! A ghost of the drone, pale and half seen through, stays level where the
//! drone started, to show how far the drone has been moved and turned.
//!
//! B puts the drone on the ball joint of Stage 2, and takes it off again: a
//! ball on a stand, one and a half times as high as the drone, that holds it
//! by its centre. There it turns but does not move, and it is drawn without
//! its legs. `--ball` starts it there.
//!
//! Keys: W and S pitch, A and D roll, Q and E yaw. The arrow keys move it
//! forward, back, left and right, the way its nose points but level, however
//! it is tilted. Page Up and Page Down move it up and down. R puts it level
//! again, back where it started. Drag with the left mouse button to move the
//! camera round the drone, and turn the wheel to move it in and out.
//!
//! Run with `cargo run --release --bin view`, or
//! `cargo run --release --bin view -- --model frame.stl`.
//! `cargo run --bin view -- --help` lists the arguments.
//!
//! The model has a quarter of a million triangles. They are sent to the
//! graphics card once, at the start, and the card turns them for each
//! picture: all this program sends after that is two tables of sixteen
//! numbers, where the camera is and how the drone is turned. macroquad's own
//! `draw_mesh` sends every triangle again for every picture, which took
//! 150 ms a picture here, so the model is drawn with miniquad, the layer
//! under macroquad.

use std::path::{Path, PathBuf};

use clap::{Parser, ValueEnum};
use macroquad::camera::Camera;
use macroquad::miniquad;
use macroquad::prelude as mq;
use physics::quat::{Quat, Rates};
use physics::vec3::Vec3;

// Turning by hand
const TURN_RATE: f32 = 1.5; // rad/s while a key is held

// Moving by hand
const MOVE_SPEED: f32 = 0.5; // m/s while a key is held
const GROUND_SPACING: f32 = 0.1; // m between the lines of the ground
const GROUND_LINES: u32 = 100; // across the ground, so it is 10 m square
const FARTHEST: f32 = 0.5 * GROUND_LINES as f32 * GROUND_SPACING; // m from the middle, each way
const HIGHEST: f32 = 5.0; // m above where it starts

// The camera
const CAMERA_START_DISTANCE: f32 = 1.0; // m from the drone
const CAMERA_NEAREST: f32 = 0.15; // m
const CAMERA_FARTHEST: f32 = 20.0; // m
const DRAG_SPEED: f32 = 0.008; // rad per pixel of mouse movement
const ZOOM_STEP: f32 = 1.1; // each notch of the wheel changes the distance by this factor

// The colour of a model file that gives none, as red, green and blue from
// 0 to 1.
const PLAIN: [f32; 3] = [0.60, 0.62, 0.66];

// The body's axes as they are drawn, each an arrow from the centre: x,
// forward, in red; y, right, in green; z, down, in blue. Each is the axis,
// along the body's axes, and the colour.
const AXES: [([f32; 3], [f32; 3]); 3] = [
    ([1.0, 0.0, 0.0], [0.90, 0.10, 0.10]),
    ([0.0, 1.0, 0.0], [0.10, 0.75, 0.15]),
    ([0.0, 0.0, 1.0], [0.10, 0.30, 0.95]),
];
const ARROW_SHAFT_RADIUS: f32 = 0.008; // m
const ARROW_HEAD_RADIUS: f32 = 0.022; // m
const ARROW_HEAD_LENGTH: f32 = 0.06; // m
const ARROW_SIDES: usize = 24; // flat sides round the shaft and the head

// The ball joint and its stand
const BALL_RADIUS: f32 = 0.03; // m
const BALL_HEIGHT: f32 = 1.5; // the ball's centre off the ground, in heights of the model
const POLE_RADIUS: f32 = 0.012; // m: wider than an arrow's shaft, which runs down it when level
const FOOT_RADIUS: f32 = 0.10; // m
const FOOT_HEIGHT: f32 = 0.01; // m
const BALL_COLOR: [f32; 3] = [0.95, 0.60, 0.10];
const STAND_COLOR: [f32; 3] = [0.25, 0.25, 0.28];

// The model drawn when `--model` is not given.
const DEFAULT_MODEL: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/models/x500.stl");

/// Which axis of the model file points up.
#[derive(Clone, Copy, Debug, ValueEnum)]
enum Up {
    Y,
    Z,
}

#[derive(Parser, Debug)]
#[command(about = "Draw the drone in a window and turn it by hand")]
struct Args {
    /// The model file to draw: .obj, or .stl in its binary form
    #[arg(long, default_value = DEFAULT_MODEL)]
    model: PathBuf,

    /// What one unit of the model file is, in metres. CAD files are mostly
    /// in millimetres: give 0.001 for those
    #[arg(long, default_value_t = 1.0)]
    model_scale: f32,

    /// The axis of the model file that points up
    #[arg(long, value_enum, default_value_t = Up::Z)]
    model_up: Up,

    /// Turn the model this many degrees about its up axis, to the right
    /// seen from above, so that its nose is forward
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    model_turn: f32,

    /// The legs of the model are whatever is more than this many metres
    /// below its top. They are taken off on the ball joint. The default
    /// fits the model in the repo
    #[arg(long, default_value_t = 0.125)]
    legs_below: f32,

    /// Start with the drone on the ball joint. B changes it
    #[arg(long)]
    ball: bool,

    /// Save the first picture to this .png file and stop, in place of
    /// staying open
    #[arg(long)]
    picture: Option<PathBuf>,
}

/// One flat piece of the drone's surface: three corners along the body's
/// axes (forward, right, down), in metres, and a colour.
#[derive(Clone, Copy)]
struct Triangle {
    corners: [Vec3; 3],
    color: [f32; 3],
}

fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// A point given along the world's axes (north, east, down), along the
/// window's (x right, y up, z towards the viewer). The camera starts south
/// of the drone looking north, so east is to the right and north is away:
///    window x = east,  window y = −down,  window z = −north
fn to_window(v: Vec3) -> mq::Vec3 {
    mq::vec3(v.y, -v.z, -v.x)
}

/// A triangle as a model file gives it: three corners of the file's own
/// three numbers each, and a colour if the file has one.
type FileTriangle = ([[f32; 3]; 3], Option<[f32; 3]>);

/// The triangles of an `.obj` file, as the file's own three numbers a
/// corner. Only the `v` lines (a corner) and the `f` lines (a face) are
/// read. A face of more than three corners is cut into triangles that share
/// its first corner. There are no colours.
fn read_obj(text: &str) -> Result<Vec<FileTriangle>, String> {
    let mut points: Vec<[f32; 3]> = Vec::new();
    let mut triangles = Vec::new();

    for (index, line) in text.lines().enumerate() {
        let bad = |what: &str| format!("line {}: {what}", index + 1);
        let mut words = line.split_whitespace();
        match words.next() {
            Some("v") => {
                let mut point = [0.0; 3];
                for number in &mut point {
                    *number = words
                        .next()
                        .and_then(|word| word.parse().ok())
                        .ok_or_else(|| bad("a corner needs three numbers"))?;
                }
                points.push(point);
            }
            Some("f") => {
                // A corner of a face is `7`, `7/2`, `7//3` or `7/2/3`; the
                // first number is the point. It counts from 1, or back from
                // the last point when it is negative.
                let mut face = Vec::new();
                for word in words {
                    let number: i64 = word
                        .split('/')
                        .next()
                        .and_then(|first| first.parse().ok())
                        .ok_or_else(|| bad("a face names its corners by number"))?;
                    let place = if number < 0 {
                        points.len() as i64 + number
                    } else {
                        number - 1
                    };
                    let point = usize::try_from(place)
                        .ok()
                        .and_then(|place| points.get(place))
                        .ok_or_else(|| bad("a face names a corner that is not there"))?;
                    face.push(*point);
                }
                for i in 2..face.len() {
                    triangles.push(([face[0], face[i - 1], face[i]], None));
                }
            }
            _ => {}
        }
    }
    Ok(triangles)
}

/// The triangles of a binary `.stl` file. The file is 80 bytes of heading,
/// the number of triangles as four bytes, and 50 bytes for each triangle:
/// twelve numbers of four bytes (a direction, which is not used here, and
/// three corners) and two spare bytes.
///
/// Some programs, and `models/make_x500.py`, put a colour in the spare
/// bytes: bit 15 set, then five bits each of red, green and blue.
fn read_stl(bytes: &[u8]) -> Result<Vec<FileTriangle>, String> {
    const HEADING: usize = 84;
    const EACH: usize = 50;

    if bytes.len() < HEADING {
        return Err("shorter than the heading of a binary .stl file".into());
    }
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    if bytes.len() != HEADING + count * EACH {
        return Err(
            "not a binary .stl file: its length does not match its count of triangles. \
             A .stl file written as text has to be saved again as binary"
                .into(),
        );
    }

    let number = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
    let mut triangles = Vec::with_capacity(count);
    for n in 0..count {
        let start = HEADING + n * EACH + 12; // past the direction
        let corner = |c: usize| {
            let at = start + c * 12;
            [number(at), number(at + 4), number(at + 8)]
        };
        let spare = u16::from_le_bytes(bytes[start + 36..start + 38].try_into().unwrap());
        let color = (spare & 0x8000 != 0)
            .then(|| [10, 5, 0].map(|shift| ((spare >> shift) & 31) as f32 / 31.0));
        triangles.push(([corner(0), corner(1), corner(2)], color));
    }
    Ok(triangles)
}

/// A model file as triangles along the body's axes, in metres, with the
/// middle of the model at the centre.
fn load_model(path: &Path, scale: f32, up: Up, turn_degrees: f32) -> Result<Vec<Triangle>, String> {
    let kind = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let raw = match kind.as_str() {
        "obj" => read_obj(&String::from_utf8_lossy(&bytes))?,
        "stl" => read_stl(&bytes)?,
        _ => return Err("the file name has to end in .obj or .stl".into()),
    };
    if raw.is_empty() {
        return Err("there are no triangles in it".into());
    }

    // The file's axes become the body's, with the file's x as forward. Both
    // sets are right-handed, so the model is not mirrored:
    //    z up:  forward = x,  right = −y,  down = −z
    //    y up:  forward = x,  right = z,   down = −y
    // Then the turn about the body's z axis, as in `add_box`.
    let (sin, cos) = turn_degrees.to_radians().sin_cos();
    let to_body = |[x, y, z]: [f32; 3]| {
        let (forward, right, down) = match up {
            Up::Z => (x, -y, -z),
            Up::Y => (x, z, -y),
        };
        vec3(
            scale * (forward * cos - right * sin),
            scale * (forward * sin + right * cos),
            scale * down,
        )
    };
    let mut triangles: Vec<Triangle> = raw
        .into_iter()
        .map(|(corners, color)| Triangle {
            corners: corners.map(to_body),
            color: color.unwrap_or(PLAIN),
        })
        .collect();

    // Move the middle of the box that holds the model to the centre.
    let mut low = [f32::INFINITY; 3];
    let mut high = [f32::NEG_INFINITY; 3];
    for corner in triangles.iter().flat_map(|t| t.corners) {
        for (axis, value) in [corner.x, corner.y, corner.z].into_iter().enumerate() {
            low[axis] = low[axis].min(value);
            high[axis] = high[axis].max(value);
        }
    }
    let middle = [0, 1, 2].map(|axis| 0.5 * (low[axis] + high[axis]));
    for triangle in &mut triangles {
        for corner in &mut triangle.corners {
            corner.x -= middle[0];
            corner.y -= middle[1];
            corner.z -= middle[2];
        }
    }

    eprintln!(
        "{}: {} triangles, {:.3} m long, {:.3} m wide, {:.3} m high",
        path.display(),
        triangles.len(),
        high[0] - low[0],
        high[1] - low[1],
        high[2] - low[2],
    );
    Ok(triangles)
}

/// Add an arrow to the triangles: a cylinder from the centre along `axis`,
/// which has to be one of the body's three, with a cone on its end. The tip
/// of the cone is `length` metres from the centre.
fn add_arrow(triangles: &mut Vec<Triangle>, axis: [f32; 3], length: f32, color: [f32; 3]) {
    // The other two axes, which the circles of the arrow are drawn in.
    let [x, y, z] = axis;
    let along = mq::vec3(x, y, z);
    let side_a = mq::vec3(z, x, y);
    let side_b = mq::vec3(y, z, x);

    // A point of the circle of `radius` that is `far` metres along the axis.
    let ring = |far: f32, radius: f32, step: usize| {
        let angle = std::f32::consts::TAU * step as f32 / ARROW_SIDES as f32;
        far * along + radius * (angle.cos() * side_a + angle.sin() * side_b)
    };
    let mut add = |a: mq::Vec3, b: mq::Vec3, c: mq::Vec3| {
        triangles.push(Triangle {
            corners: [a, b, c].map(|p| vec3(p.x, p.y, p.z)),
            color,
        });
    };

    let neck = length - ARROW_HEAD_LENGTH; // where the shaft meets the head
    let tip = length * along;
    for step in 0..ARROW_SIDES {
        let next = step + 1;
        // The side of the shaft: a strip of two triangles.
        let near_a = ring(0.0, ARROW_SHAFT_RADIUS, step);
        let near_b = ring(0.0, ARROW_SHAFT_RADIUS, next);
        let far_a = ring(neck, ARROW_SHAFT_RADIUS, step);
        let far_b = ring(neck, ARROW_SHAFT_RADIUS, next);
        add(near_a, near_b, far_b);
        add(near_a, far_b, far_a);
        // The head: its flat back, and its side up to the tip.
        let head_a = ring(neck, ARROW_HEAD_RADIUS, step);
        let head_b = ring(neck, ARROW_HEAD_RADIUS, next);
        add(neck * along, head_a, head_b);
        add(head_a, head_b, tip);
    }
}

/// Put the triangles of the legs after all the others, and give how many
/// the others are. A triangle is of the legs if its middle is more than
/// `legs_below` metres below the top of the model.
fn legs_last(triangles: &mut [Triangle], legs_below: f32) -> usize {
    // Down is positive, so the top is the least.
    let top = triangles
        .iter()
        .flat_map(|t| t.corners)
        .map(|corner| corner.z)
        .fold(f32::INFINITY, f32::min);
    let is_leg = |t: &Triangle| {
        let [a, b, c] = t.corners;
        (a.z + b.z + c.z) / 3.0 - top > legs_below
    };
    triangles.sort_by_key(is_leg); // keeps the order within each kind
    triangles.iter().filter(|t| !is_leg(t)).count()
}

/// Add a cylinder that stands upright, from `top` to `bottom` metres down
/// the body's z axis, closed at both ends.
fn add_cylinder(
    triangles: &mut Vec<Triangle>,
    top: f32,
    bottom: f32,
    radius: f32,
    color: [f32; 3],
) {
    let ring = |down: f32, step: usize| {
        let angle = std::f32::consts::TAU * step as f32 / ARROW_SIDES as f32;
        vec3(radius * angle.cos(), radius * angle.sin(), down)
    };
    for step in 0..ARROW_SIDES {
        let (top_a, top_b) = (ring(top, step), ring(top, step + 1));
        let (bottom_a, bottom_b) = (ring(bottom, step), ring(bottom, step + 1));
        for corners in [
            [top_a, top_b, bottom_b],
            [top_a, bottom_b, bottom_a],
            [vec3(0.0, 0.0, top), top_a, top_b],
            [vec3(0.0, 0.0, bottom), bottom_a, bottom_b],
        ] {
            triangles.push(Triangle { corners, color });
        }
    }
}

/// Add a ball about the centre.
fn add_ball(triangles: &mut Vec<Triangle>, radius: f32, color: [f32; 3]) {
    const ROUND: usize = 24; // steps round the ball
    const DOWN: usize = 12; // steps from its top to its bottom
    let point = |round: usize, down: usize| {
        let across = std::f32::consts::TAU * round as f32 / ROUND as f32;
        let from_top = std::f32::consts::PI * down as f32 / DOWN as f32;
        vec3(
            radius * from_top.sin() * across.cos(),
            radius * from_top.sin() * across.sin(),
            -radius * from_top.cos(),
        )
    };
    for down in 0..DOWN {
        for round in 0..ROUND {
            let (a, b) = (point(round, down), point(round + 1, down));
            let (c, d) = (point(round, down + 1), point(round + 1, down + 1));
            // At the top and at the bottom one of the two is only a line.
            if down > 0 {
                triangles.push(Triangle {
                    corners: [a, b, d],
                    color,
                });
            }
            if down + 1 < DOWN {
                triangles.push(Triangle {
                    corners: [a, d, c],
                    color,
                });
            }
        }
    }
}

/// Add the ball joint and its stand: a ball about the centre, on a pole
/// that goes `height` metres down to a foot on the ground.
fn add_stand(triangles: &mut Vec<Triangle>, height: f32) {
    add_ball(triangles, BALL_RADIUS, BALL_COLOR);
    add_cylinder(triangles, 0.0, height, POLE_RADIUS, STAND_COLOR);
    add_cylinder(
        triangles,
        height - FOOT_HEIGHT,
        height,
        FOOT_RADIUS,
        STAND_COLOR,
    );
}

/// One corner of a triangle as the graphics card gets it.
#[repr(C)]
struct Corner {
    position: [f32; 3], // m, along the body's axes
    facing: [f32; 3],   // the direction the triangle faces, along the body's axes
    color: [u8; 4],     // red, green, blue and a spare, 0 to 255
}

/// What the graphics card is told for each picture.
#[repr(C)]
struct Tables {
    camera: mq::Mat4, // from the window's axes to the screen
    place: mq::Mat4,  // from the body's axes to the window's
    ghost: f32,       // 0 for the drone, 1 for its ghost
}

// The two small programs the graphics card runs. The first runs for each
// corner: it turns the corner by `Place`, puts it on the screen with
// `Camera`, and works out the light. There is no lamp in macroquad, so the
// light is worked out here: a triangle that faces the lamp gets its full
// colour and one that is edge-on to it gets 35% of it. Which side of a
// triangle is its outside is not looked at; a model file may have that
// either way round. The second program runs for each dot of the screen and
// gives it the colour.
//
// The ghost is the same triangles with `Ghost` at 1: most of the way to one
// pale colour, and letting 65% of what is behind it through.
const CORNER_PROGRAM: &str = r#"#version 100
attribute vec3 in_position;
attribute vec3 in_facing;
attribute vec4 in_color;
uniform mat4 Camera;
uniform mat4 Place;
uniform float Ghost;
varying lowp vec4 color;
void main() {
    gl_Position = Camera * Place * vec4(in_position, 1.0);
    vec3 facing = normalize((Place * vec4(in_facing, 0.0)).xyz);
    vec3 towards_lamp = normalize(vec3(0.4, 1.0, 0.5));
    float light = 0.35 + 0.65 * abs(dot(facing, towards_lamp));
    vec3 lit = in_color.rgb / 255.0 * light;
    vec3 pale = vec3(0.80, 0.90, 1.00) * light;
    color = vec4(mix(lit, pale, 0.8 * Ghost), 1.0 - 0.65 * Ghost);
}"#;

const DOT_PROGRAM: &str = r#"#version 100
varying lowp vec4 color;
void main() {
    gl_FragColor = color;
}"#;

/// The model, as it sits on the graphics card.
struct Drawing {
    pipeline: miniquad::Pipeline,
    // The two steps the ghost is drawn in: see `draw_ghost`.
    ghost_depth: miniquad::Pipeline,
    ghost_color: miniquad::Pipeline,
    bindings: miniquad::Bindings,
    // How many corners, three for each triangle, each part has. The parts
    // are in this order on the card.
    body: i32,
    legs: i32,
    arrows: i32,
    stand: i32,
}

/// How many triangles each part of the drawing has, in the order they are
/// in.
struct Parts {
    body: usize,
    legs: usize,
    arrows: usize,
    stand: usize,
}

impl Drawing {
    /// Send the triangles to the graphics card. Done once. `parts` says
    /// which of them are what.
    fn new(triangles: &[Triangle], parts: Parts) -> Self {
        use miniquad::{
            BlendFactor, BlendState, BlendValue, BufferLayout, BufferSource, BufferType,
            BufferUsage, Comparison, Equation, PipelineParams, ShaderMeta, ShaderSource,
            UniformBlockLayout, UniformDesc, UniformType, VertexAttribute, VertexFormat,
        };

        let mut corners = Vec::with_capacity(triangles.len() * 3);
        for triangle in triangles {
            let [a, b, c] = triangle
                .corners
                .map(|corner| mq::vec3(corner.x, corner.y, corner.z));
            let facing = (b - a).cross(c - a).normalize_or_zero().to_array();
            let [red, green, blue] = triangle.color.map(|part| (part * 255.0) as u8);
            for position in [a, b, c] {
                corners.push(Corner {
                    position: position.to_array(),
                    facing,
                    color: [red, green, blue, 255],
                });
            }
        }
        // The card is given the corners, and the order to take them in:
        // here simply first to last.
        let order: Vec<u32> = (0..corners.len() as u32).collect();

        // SAFETY: macroquad asks for this to be marked: the two layers must
        // not draw at the same moment. Nothing else is drawing here.
        let card = unsafe { mq::get_internal_gl() }.quad_context;
        let corner_buffer = card.new_buffer(
            BufferType::VertexBuffer,
            BufferUsage::Immutable,
            BufferSource::slice(&corners),
        );
        let order_buffer = card.new_buffer(
            BufferType::IndexBuffer,
            BufferUsage::Immutable,
            BufferSource::slice(&order),
        );
        let shader = card
            .new_shader(
                ShaderSource::Glsl {
                    vertex: CORNER_PROGRAM,
                    fragment: DOT_PROGRAM,
                },
                ShaderMeta {
                    images: vec![],
                    uniforms: UniformBlockLayout {
                        // In the order of `Tables`.
                        uniforms: vec![
                            UniformDesc::new("Camera", UniformType::Mat4),
                            UniformDesc::new("Place", UniformType::Mat4),
                            UniformDesc::new("Ghost", UniformType::Float1),
                        ],
                    },
                },
            )
            .expect("the graphics card did not take the two programs");
        let mut pipeline = |params: PipelineParams| {
            card.new_pipeline(
                &[BufferLayout::default()],
                // In the order of `Corner`.
                &[
                    VertexAttribute::new("in_position", VertexFormat::Float3),
                    VertexAttribute::new("in_facing", VertexFormat::Float3),
                    VertexAttribute::new("in_color", VertexFormat::Byte4),
                ],
                shader,
                params,
            )
        };

        Self {
            pipeline: pipeline(PipelineParams {
                // A triangle behind another is hidden by it.
                depth_test: Comparison::LessOrEqual,
                depth_write: true,
                ..Default::default()
            }),
            // No colour, only how far away each dot of the ghost is.
            ghost_depth: pipeline(PipelineParams {
                depth_test: Comparison::LessOrEqual,
                depth_write: true,
                color_write: (false, false, false, false),
                ..Default::default()
            }),
            // The colour, mixed with what is behind by how far the ghost
            // lets it through.
            ghost_color: pipeline(PipelineParams {
                depth_test: Comparison::LessOrEqual,
                depth_write: false,
                color_blend: Some(BlendState::new(
                    Equation::Add,
                    BlendFactor::Value(BlendValue::SourceAlpha),
                    BlendFactor::OneMinusValue(BlendValue::SourceAlpha),
                )),
                ..Default::default()
            }),
            bindings: miniquad::Bindings {
                vertex_buffers: vec![corner_buffer],
                index_buffer: order_buffer,
                images: vec![],
            },
            body: 3 * parts.body as i32,
            legs: 3 * parts.legs as i32,
            arrows: 3 * parts.arrows as i32,
            stand: 3 * parts.stand as i32,
        }
    }

    /// What the graphics card is told: the camera, and the body's axes and
    /// centre along the window's axes.
    fn tables(camera: &mq::Camera3D, attitude: &Quat, centre: mq::Vec3, ghost: f32) -> Tables {
        // Where the body's three axes point in the window, and where its
        // centre is. These are the four columns of the table.
        let forward = to_window(attitude.to_world(vec3(1.0, 0.0, 0.0)));
        let right = to_window(attitude.to_world(vec3(0.0, 1.0, 0.0)));
        let down = to_window(attitude.to_world(vec3(0.0, 0.0, 1.0)));
        Tables {
            camera: camera.matrix(),
            place: mq::Mat4::from_cols(
                forward.extend(0.0),
                right.extend(0.0),
                down.extend(0.0),
                centre.extend(1.0),
            ),
            ghost,
        }
    }

    /// Draw the model and its arrows, turned by `attitude` and with its
    /// centre at `centre`, which is along the window's axes, as seen by
    /// `camera`. The legs are drawn if `legs` says so.
    fn draw(&self, camera: &mq::Camera3D, attitude: &Quat, centre: mq::Vec3, legs: bool) {
        let tables = Self::tables(camera, attitude, centre, 0.0);
        if legs {
            self.send(
                &[&self.pipeline],
                &tables,
                0,
                self.body + self.legs + self.arrows,
            );
        } else {
            self.send(&[&self.pipeline], &tables, 0, self.body);
            self.send(
                &[&self.pipeline],
                &tables,
                self.body + self.legs,
                self.arrows,
            );
        }
    }

    /// Draw the ball joint and its stand, with the ball's centre at
    /// `centre`.
    fn draw_stand(&self, camera: &mq::Camera3D, centre: mq::Vec3) {
        let tables = Self::tables(camera, &Quat::new(), centre, 0.0);
        let first = self.body + self.legs + self.arrows;
        self.send(&[&self.pipeline], &tables, first, self.stand);
    }

    /// Draw the ghost: the model without the arrows, level, with its centre
    /// at `centre`. It has to be drawn after the drone, so that the drone
    /// shows through it.
    ///
    /// It is drawn twice. The first time leaves no colour, only how far
    /// away the nearest of the ghost is at each dot of the screen. The
    /// second time colours only that nearest surface. Drawn once, the
    /// inside of the ghost would show through its outside.
    fn draw_ghost(&self, camera: &mq::Camera3D, centre: mq::Vec3, legs: bool) {
        let tables = Self::tables(camera, &Quat::new(), centre, 1.0);
        let corners = if legs {
            self.body + self.legs
        } else {
            self.body
        };
        self.send(&[&self.ghost_depth, &self.ghost_color], &tables, 0, corners);
    }

    /// Have the graphics card draw `count` corners from `first` on, once
    /// with each of `pipelines`.
    fn send(&self, pipelines: &[&miniquad::Pipeline], tables: &Tables, first: i32, count: i32) {
        // SAFETY: as in `new`. `flush` has macroquad draw what it was asked
        // for so far, the ground, before this draws over it.
        let mut layers = unsafe { mq::get_internal_gl() };
        layers.flush();
        let card = layers.quad_context;
        card.begin_default_pass(miniquad::PassAction::Nothing);
        for pipeline in pipelines {
            card.apply_pipeline(pipeline);
            card.apply_bindings(&self.bindings);
            card.apply_uniforms(miniquad::UniformsSource::table(tables));
            card.draw(first, count, 1);
        }
        card.end_render_pass();
    }
}

/// +1 while `plus` is held, −1 while `minus` is, 0 for both or neither.
fn key_pair(plus: mq::KeyCode, minus: mq::KeyCode) -> f32 {
    let held = |key| if mq::is_key_down(key) { 1.0 } else { 0.0 };
    held(plus) - held(minus)
}

fn window() -> mq::Conf {
    mq::Conf {
        window_title: "Drone".into(),
        window_width: 1000,
        window_height: 700,
        sample_count: 4, // smooths the edges
        ..Default::default()
    }
}

#[macroquad::main(window)]
async fn main() {
    let args = Args::parse();

    let mut triangles = match load_model(
        &args.model,
        args.model_scale,
        args.model_up,
        args.model_turn,
    ) {
        Ok(triangles) => triangles,
        Err(why) => {
            eprintln!("{}: {why}", args.model.display());
            std::process::exit(1);
        }
    };
    let name = args
        .model
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();

    // How high to hold the drone, so that it clears the ground at any
    // attitude: the distance of its farthest corner from its centre.
    let height = triangles
        .iter()
        .flat_map(|t| t.corners)
        .map(|corner| corner.length())
        .fold(0.0, f32::max)
        + 0.05;

    let count = triangles.len();
    let body = legs_last(&mut triangles, args.legs_below);

    // How high the ball of the ball joint is: the height of the model, legs
    // and all, times `BALL_HEIGHT`.
    let (top, bottom) = triangles
        .iter()
        .flat_map(|t| t.corners)
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(top, bottom), c| {
            (top.min(c.z), bottom.max(c.z))
        });
    let ball_height = BALL_HEIGHT * (bottom - top);

    // The body's three axes as arrows. They are part of the drawing, so
    // they turn with the drone. Each reaches as far as the farthest corner,
    // which keeps the tip of one that points down off the ground.
    for (axis, color) in AXES {
        add_arrow(&mut triangles, axis, height - 0.05, color);
    }
    let with_arrows = triangles.len();
    add_stand(&mut triangles, ball_height);
    let drawing = Drawing::new(
        &triangles,
        Parts {
            body,
            legs: count - body,
            arrows: with_arrows - count,
            stand: triangles.len() - with_arrows,
        },
    );
    drop(triangles); // the graphics card has them now

    // On the ball joint the drone turns but does not move, and has no legs.
    let mut on_ball = args.ball;

    let mut attitude = Quat::new();

    // Where the drone's centre is from where it started, along the world's
    // axes: north, east and down, in metres.
    let mut position = vec3(0.0, 0.0, 0.0);
    let mut turned = false; // since the start, or since R

    // Where the camera is: round the drone by `camera_round`, from behind it
    // towards its right, and above level by `camera_up`, both in radians.
    let mut camera_round: f32 = 0.6;
    let mut camera_up: f32 = 0.45;
    let mut camera_distance = CAMERA_START_DISTANCE;
    let mut mouse_before = mq::mouse_position();

    loop {
        let dt = mq::get_frame_time();

        // Turn the drone. The rates are the body's, as a gyro would read.
        if mq::is_key_pressed(mq::KeyCode::R) {
            attitude = Quat::new();
            position = vec3(0.0, 0.0, 0.0);
            turned = false;
        }
        if mq::is_key_pressed(mq::KeyCode::B) {
            on_ball = !on_ball;
            position = vec3(0.0, 0.0, 0.0); // the ball is where the drone started
        }
        let rates = Rates {
            roll: TURN_RATE * key_pair(mq::KeyCode::D, mq::KeyCode::A),
            pitch: TURN_RATE * key_pair(mq::KeyCode::S, mq::KeyCode::W),
            yaw: TURN_RATE * key_pair(mq::KeyCode::E, mq::KeyCode::Q),
        };
        turned |= rates.roll != 0.0 || rates.pitch != 0.0 || rates.yaw != 0.0;
        attitude.update(rates, dt);
        attitude.normalise();

        // Move the drone. Forward is the way its nose points on the ground,
        // which is its yaw, so a tilted drone still moves level. It stays
        // over the ground that is drawn, and no lower than it started: there
        // it clears the ground at any attitude.
        let (sin, cos) = attitude.angles().yaw.sin_cos();
        let speed = if on_ball { 0.0 } else { MOVE_SPEED }; // the ball holds it
        let forward = speed * dt * key_pair(mq::KeyCode::Up, mq::KeyCode::Down);
        let right = speed * dt * key_pair(mq::KeyCode::Right, mq::KeyCode::Left);
        let up = speed * dt * key_pair(mq::KeyCode::PageUp, mq::KeyCode::PageDown);
        position.x = (position.x + forward * cos - right * sin).clamp(-FARTHEST, FARTHEST);
        position.y = (position.y + forward * sin + right * cos).clamp(-FARTHEST, FARTHEST);
        position.z = (position.z - up).clamp(-HIGHEST, 0.0);

        // Move the camera. It looks at the drone, and so follows it.
        let mouse = mq::mouse_position();
        if mq::is_mouse_button_down(mq::MouseButton::Left) {
            camera_round -= DRAG_SPEED * (mouse.0 - mouse_before.0);
            camera_up = (camera_up + DRAG_SPEED * (mouse.1 - mouse_before.1)).clamp(-1.5, 1.5);
        }
        mouse_before = mouse;
        let wheel = mq::mouse_wheel().1;
        if wheel > 0.0 {
            camera_distance /= ZOOM_STEP;
        } else if wheel < 0.0 {
            camera_distance *= ZOOM_STEP;
        }
        camera_distance = camera_distance.clamp(CAMERA_NEAREST, CAMERA_FARTHEST);

        // The camera's place along the world's axes. With `camera_round` at
        // zero it is south of the drone, looking north along its back.
        let level = camera_distance * camera_up.cos();
        let start = mq::vec3(0.0, if on_ball { ball_height } else { height }, 0.0);
        let target = to_window(position) + start;
        let camera = to_window(vec3(
            -level * camera_round.cos(),
            level * camera_round.sin(),
            -camera_distance * camera_up.sin(),
        )) + target;

        mq::clear_background(mq::Color::new(0.80, 0.85, 0.90, 1.0));
        let camera = mq::Camera3D {
            position: camera,
            target,
            up: mq::vec3(0.0, 1.0, 0.0),
            ..Default::default()
        };
        mq::set_camera(&camera);

        // The ground: lines 0.1 m apart.
        mq::draw_grid(GROUND_LINES, GROUND_SPACING, mq::BLACK, mq::GRAY);
        if on_ball {
            drawing.draw_stand(&camera, start);
        }
        drawing.draw(&camera, &attitude, target, !on_ball);

        // The ghost, where the drone started. It is left out while the
        // drone is still exactly there, where it would only make the drone
        // pale.
        let moved = position.x != 0.0 || position.y != 0.0 || position.z != 0.0;
        if moved || turned {
            drawing.draw_ghost(&camera, start, !on_ball);
        }

        // Text, on the flat screen again.
        mq::set_default_camera();
        let angles = attitude.angles();
        let lines = [
            format!("drawing: {name}, {count} triangles"),
            format!(
                "roll {:6.1}   pitch {:6.1}   yaw {:6.1}   degrees",
                angles.roll.to_degrees(),
                angles.pitch.to_degrees(),
                angles.yaw.to_degrees(),
            ),
            format!(
                "north {:6.2}   east {:6.2}   up {:6.2}   metres",
                position.x,
                position.y,
                0.0 - position.z, // not `-`, which gives -0.00 at the start
            ),
            "W S pitch   A D roll   Q E yaw   R level, and back to the start".into(),
            "arrows: forward, back, left, right   Page Up, Page Down: up, down".into(),
            format!(
                "B: {}",
                if on_ball {
                    "take it off the ball joint"
                } else {
                    "put it on the ball joint of Stage 2"
                }
            ),
            "drag: camera   wheel: nearer, farther".into(),
            "axes: red forward, green right, blue down".into(),
            format!("{} pictures a second", mq::get_fps()),
        ];
        for (row, line) in lines.iter().enumerate() {
            mq::draw_text(line, 12.0, 24.0 + 22.0 * row as f32, 22.0, mq::BLACK);
        }

        if let Some(path) = &args.picture {
            mq::get_screen_data().export_png(&path.to_string_lossy());
            return;
        }

        mq::next_frame().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_window_has_north_away_east_right_and_down_down() {
        assert_eq!(to_window(vec3(1.0, 0.0, 0.0)), mq::vec3(0.0, 0.0, -1.0));
        assert_eq!(to_window(vec3(0.0, 1.0, 0.0)), mq::vec3(1.0, 0.0, 0.0));
        assert_eq!(to_window(vec3(0.0, 0.0, 1.0)), mq::vec3(0.0, -1.0, 0.0));
    }

    #[test]
    fn an_arrow_runs_from_the_centre_to_its_tip_along_its_axis() {
        for (axis, color) in AXES {
            let mut triangles = Vec::new();
            add_arrow(&mut triangles, axis, 0.3, color);
            assert_eq!(triangles.len(), 4 * ARROW_SIDES);
            let mut farthest: f32 = 0.0;
            for corner in triangles.iter().flat_map(|t| t.corners) {
                let along = corner.x * axis[0] + corner.y * axis[1] + corner.z * axis[2];
                let off = (corner.length().powi(2) - along * along).max(0.0).sqrt();
                assert!((-1e-6..=0.3 + 1e-6).contains(&along));
                assert!(off <= ARROW_HEAD_RADIUS + 1e-5);
                farthest = farthest.max(along);
            }
            assert!((farthest - 0.3).abs() < 1e-6);
        }
    }

    #[test]
    fn the_legs_of_the_model_in_the_repo_are_found() {
        let mut triangles = load_model(Path::new(DEFAULT_MODEL), 1.0, Up::Z, 0.0).unwrap();
        let lowest = |triangles: &[Triangle]| {
            triangles
                .iter()
                .flat_map(|t| t.corners)
                .map(|c| c.z)
                .fold(f32::NEG_INFINITY, f32::max)
        };
        let all = triangles.len();
        let with_legs = lowest(&triangles);
        let body = legs_last(&mut triangles, 0.125);
        assert_eq!(triangles.len(), all);
        // The legs are a fifth or so of the triangles, and are the lower
        // half of the model: without them it ends 0.15 m higher or more.
        assert!((all / 10..all / 3).contains(&(all - body)));
        assert!(with_legs - lowest(&triangles[..body]) > 0.15);
    }

    #[test]
    fn the_stand_runs_from_the_ball_down_to_the_ground() {
        let mut triangles = Vec::new();
        add_stand(&mut triangles, 0.4);
        let mut top = f32::INFINITY;
        let mut bottom = f32::NEG_INFINITY;
        for corner in triangles.iter().flat_map(|t| t.corners) {
            top = top.min(corner.z);
            bottom = bottom.max(corner.z);
            assert!(corner.x.hypot(corner.y) <= FOOT_RADIUS + 1e-6);
        }
        assert!((top + BALL_RADIUS).abs() < 1e-6);
        assert!((bottom - 0.4).abs() < 1e-6);
    }

    #[test]
    fn an_obj_file_gives_its_faces_as_triangles() {
        let text = "# a square\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1 4//1\nf -4 -3 -2\n";
        let triangles = read_obj(text).unwrap();
        assert_eq!(triangles.len(), 3);
        assert_eq!(
            triangles[1].0,
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
        );
        assert_eq!(
            triangles[2].0,
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]
        );
        assert_eq!(triangles[0].1, None);
    }

    #[test]
    fn an_obj_face_with_a_missing_corner_is_refused() {
        assert!(read_obj("v 0 0 0\nf 1 2 3\n").is_err());
    }

    #[test]
    fn a_binary_stl_file_gives_its_triangles() {
        let mut bytes = vec![0u8; 80];
        bytes.extend(1u32.to_le_bytes());
        for number in [
            0.0f32, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        ] {
            bytes.extend(number.to_le_bytes());
        }
        bytes.extend([0u8; 2]);
        let corners = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        assert_eq!(read_stl(&bytes).unwrap(), [(corners, None)]);

        // Full red, in the spare bytes.
        let end = bytes.len();
        bytes[end - 2..].copy_from_slice(&(0x8000u16 | 31 << 10).to_le_bytes());
        assert_eq!(
            read_stl(&bytes).unwrap(),
            [(corners, Some([1.0, 0.0, 0.0]))]
        );

        bytes.pop();
        assert!(read_stl(&bytes).is_err());
    }

    #[test]
    fn the_model_in_the_repo_loads_and_is_the_size_of_the_x500() {
        let triangles = load_model(Path::new(DEFAULT_MODEL), 1.0, Up::Z, 0.0).unwrap();
        assert!(triangles.len() > 200_000);
        // Its motors are 0.174 m forward and to the side of the centre, so
        // nothing but a propeller tip is much farther out than that.
        let farthest_forward = triangles
            .iter()
            .flat_map(|t| t.corners)
            .map(|c| c.x)
            .fold(0.0, f32::max);
        assert!((0.19..0.22).contains(&farthest_forward));
    }
}
