extern crate sdl2; 

use sdl2::event::Event;
use sdl2::gfx::primitives::DrawRenderer;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::render::Canvas;
use sdl2::video::Window;
use std::time::Duration;
use std::ops::{Add,AddAssign,Div,Mul,Sub,SubAssign};

const ZERO_THRESHOLD: f64 = 0.00001;

const SPEED_CAP: f64 = 100.0;
const FORCE_CAP: f64 = 50.0;

#[derive(Debug, Copy, Clone)]
struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
    const ZERO: Vec2 = Vec2 {x: 0.0, y: 0.0};

    fn length(self) -> f64 {
        (self.x.powf(2.0) + self.y.powf(2.0)).sqrt()
    }

    fn normalized(self) -> Vec2 {
        self / self.length()
    }

    fn length_sub(self, amnt: f64) -> Vec2 {
        if self.length() < amnt || self.length() < ZERO_THRESHOLD {
            Vec2::ZERO
        } else {
            self - self.normalized() * amnt
        }
    }

    fn length_clamped(self, amnt: f64) -> Vec2 {
        if self.length() < amnt {
            self
        } else {
            self.normalized() * amnt
        }
    }

    fn dot(self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    fn project_onto(self, other: Vec2) -> Vec2 {
        other.normalized() * (self.dot(other) / other.length())
    }

    fn rotated90(self, cw: bool) -> Vec2 {
        let invert_x_for_ccw = (self.x < 0.0) ^ (self.y < 0.0);
        if invert_x_for_ccw ^ cw {
            Vec2 {x: self.y * -1.0, y: self.x}
        } else {
            Vec2 {x: self.y, y: self.x * -1.0}
        }
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(mut self, other: Vec2) -> Vec2 {
        self += other;
        self
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, other: Vec2) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(mut self, other: Vec2) -> Vec2 {
        self -= other;
        self
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, other: Vec2) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, rhs: f64) -> Vec2 {
        Vec2 {x: self.x / rhs, y: self.y / rhs}
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Vec2 {
        Vec2 {x: self.x * rhs, y: self.y * rhs}
    }
}

struct RopeSegment {
    pos: Vec2,
    speed: Vec2,
    force: Vec2,
}

impl RopeSegment {
    const MASS: f64 = 0.5;
    const STIFFNESS: f64 = 0.5;
    const DAMPING: f64 = 0.015;
    const LENGTH: f64 = 20.0;

    const STATIC_FRICTION: f64 = 0.0016;
    const KINETIC_FRICTION: f64 = 0.0008;

    fn apply_force_to_linked_segment(&self, linked: &mut RopeSegment) {
        let pull = (self.pos - linked.pos).length_sub(Self::LENGTH);
        if pull.length() < ZERO_THRESHOLD { return; }
        let spring_speed = (self.speed - linked.speed).project_onto(pull);
        let spring_damping = spring_speed * Self::DAMPING;
        let pull_dampened = pull + spring_damping;
        linked.pull(pull_dampened * Self::STIFFNESS);
    }

    fn pull(&mut self, force: Vec2) {
        self.force += force;
    }

    fn tick(&mut self) {
        let mut friction_applied = false;
        if self.speed.length() < ZERO_THRESHOLD {
            self.speed = Vec2::ZERO;
            self.force = self.force.length_sub(Self::STATIC_FRICTION);
            friction_applied = true;
        }
        self.force = self.force.length_clamped(FORCE_CAP);
        self.speed += self.force / Self::MASS;
        if ! friction_applied {
            self.speed = self.speed.length_sub(Self::KINETIC_FRICTION / Self::MASS);
        }
        self.speed = self.speed.length_clamped(SPEED_CAP);
        self.pos += self.speed;
        self.force = Vec2::ZERO;
    }
}

struct Rope {
    cursor: Vec2,
    segments: Vec<RopeSegment>,
}

impl Rope {
    const DRAW_WIDTH: f64 = 10.0;

    fn new(n: usize, pos: Vec2) -> Rope {
        let mut segments = Vec::new();
        for _ in 0..n {
            segments.push(RopeSegment {
                pos,
                speed: Vec2::ZERO,
                force: Vec2::ZERO,
            });
        }
        Rope {cursor: pos, segments}
    }

    fn pull(&mut self, x: f64, y: f64) {
        self.cursor += Vec2 {x, y};
    }

    fn tick(&mut self) {
        let diff = self.cursor - self.segments[0].pos;
        if diff.length() > ZERO_THRESHOLD {
            self.segments[0].force += diff * 0.0005;
        }
        for i in 0..self.segments.len() {
            if i != 0 {
                let (left, right) = self.segments.split_at_mut(i);
                right[0].apply_force_to_linked_segment(&mut left[i-1]);
            }
            if i != self.segments.len() - 1 {
                let (left, right) = self.segments.split_at_mut(i+1);
                left[i].apply_force_to_linked_segment(&mut right[0]);
            }
        }
        for segment in &mut self.segments {
            segment.tick()
        }
    }

    fn draw(&self, canvas: &mut Canvas<Window>) {
        for s in &self.segments {
            canvas.filled_circle(
                s.pos.x as i16,
                s.pos.y as i16,
                (Self::DRAW_WIDTH / 2.0) as i16,
                Color::WHITE,
            ).unwrap();
        }
        for segments in self.segments[..].windows(2) {
            if let [s1, s2] = segments {
                if (s2.pos - s1.pos).length() < ZERO_THRESHOLD {
                    continue;
                }
                let s1norm = (s2.pos - s1.pos).normalized().rotated90(true) * (Self::DRAW_WIDTH / 2.0);
                let s2norm = (s1.pos - s2.pos).normalized().rotated90(false) * (Self::DRAW_WIDTH / 2.0);
                let s1a = s1.pos + s1norm;
                let s1b = s1.pos - s1norm;
                let s2a = s2.pos + s2norm;
                let s2b = s2.pos - s2norm;
                canvas.filled_polygon(
                    &[s1a.x as i16, s1b.x as i16, s2b.x as i16, s2a.x as i16],
                    &[s1a.y as i16, s1b.y as i16, s2b.y as i16, s2a.y as i16],
                    Color::WHITE,
                ).unwrap();
            }
        }

        canvas.filled_circle(
            self.cursor.x.round() as i16,
            self.cursor.y.round() as i16,
            10,
            Color::BLACK,
        ).unwrap();
    }
}

fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    sdl_context.mouse().set_relative_mouse_mode(true);

    let window = video_subsystem.window("Rope", 0, 0)
        .fullscreen_desktop()
        .build()
        .unwrap();
    let mut canvas = window.into_canvas().present_vsync().build().unwrap();
 
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut rope = Rope::new(40, Vec2 {x: 300.0, y: 300.0});
    'running: loop {
        canvas.set_draw_color(Color::GREY);
        canvas.clear();
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                Event::MouseMotion { xrel, yrel, .. } => {
                    rope.pull(f64::from(xrel), f64::from(yrel));
                },
                _ => {}
            }
        }
        for _ in 0..15 {
            rope.tick();
        }

        rope.draw(&mut canvas);
        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
