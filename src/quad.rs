use glam::DVec2;
use std::ops::Add;

const THETA: f64 = 0.5;

#[derive(Clone, Copy, Debug)]
struct AABB {
    half: DVec2,
    mins: DVec2,
    maxs: DVec2,
    width: f64,
}

impl Add<DVec2> for AABB {
    type Output = Self;
    fn add(self, rhs: DVec2) -> Self::Output {
        Self::Output::from(self.mins + rhs, self.maxs + rhs)
    }
}

impl AABB {

    fn new(min: f64, max: f64) -> Self {
        let mins = DVec2::splat(min);
        let maxs = DVec2::splat(max);
        Self::from(mins, maxs)
    }

    fn from(mins: DVec2, maxs: DVec2) -> Self {
        AABB {
            half: (maxs - mins) * 0.5,
            width: maxs.x - mins.x,
            mins,
            maxs,
        }
    }

    #[inline]
    fn contains(&self, point: DVec2) -> bool {
        self.mins.cmple(point).all() &&
        self.maxs.cmpge(point).all()
    }

    // #[inline]
    // fn transformed(&self, delta: DVec2) -> Self {
    //     AABB::from(self.mins + delta, self.maxs + delta)
    // }
}

#[derive(Clone, Copy, Debug)]
pub struct QuadTreeMeta {
    handle: Option<usize>,
    mass: f64,
    position: DVec2,
}

impl QuadTreeMeta {
    pub fn new(handle: Option<usize>, mass: f64, position: DVec2) -> Self {
        QuadTreeMeta { handle, mass, position }
    }
}

impl Add for QuadTreeMeta {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        let mass = self.mass + other.mass;
        let position = self.position.lerp(other.position, other.mass / mass);
        Self::Output::new(None, mass, position)
    }
}

pub struct QuadTreeNode {
    aabb: AABB,
    children: Option<Box<[Self;4]>>,
    meta: Option<QuadTreeMeta>,
}

impl From<AABB> for QuadTreeNode {
    fn from(value: AABB) -> Self {
        QuadTreeNode {
            aabb: value,
            children: None,
            meta: None,
        }
    }
}

impl QuadTreeNode {

    pub fn new(min: f64, max: f64) -> Self {
        let (min, max) = (
            f64::min(min, max),
            f64::max(min, max),
        );
        AABB::new(min, max).into()
    }

    pub fn calculate(&self, meta: QuadTreeMeta) -> DVec2 {
        // if there is meta
        if let Some(self_meta) = self.meta {
            let delta = self_meta.position - meta.position;
            let dir = delta.normalize();
            let dist = delta.length_squared().recip(); //hmmmmm
            let mass = self_meta.mass * meta.mass;
            // if an external node
            if let Some(self_handle) = self_meta.handle && self_handle != meta.handle.unwrap() {
                dir * mass * dist
            }
            // if dist ratio < theta
            else if self.aabb.width * self.aabb.width * dist < THETA {
                dir * mass * dist
            }
            else if self.children.is_some() {
                // otherwise get sum of child forces
                let mut force = DVec2::ZERO;
                for child in self.children.as_ref().unwrap().iter() {
                    force += child.calculate(meta);
                }
                force
            } else {
                DVec2::ZERO
            }
        } else {
            DVec2::ZERO
        }
    }

    pub fn insert(&mut self, new_meta: &QuadTreeMeta) -> bool {
        if !self.aabb.contains(new_meta.position) {
            return false;
        }
        if self.meta.is_some() {
            if self.children.is_none() {
                self.sub_divide();
            }
            self.sub_insert(new_meta);
        }
        self.add_meta(*new_meta);
        true
    }

    fn add_meta(&mut self, other_meta: QuadTreeMeta) {
        self.meta = Some(match self.meta {
            Some(meta) => meta + other_meta,
            None => other_meta
        });
    }

    fn sub_divide(&mut self) {
        let nw = AABB::from(self.aabb.mins, DVec2::from(self.aabb.mins + self.aabb.half));
        let ne = nw + DVec2::X * self.aabb.half.x;
        let sw = nw + DVec2::Y * self.aabb.half.y;
        let se = nw + self.aabb.half;
        self.children = Some(Box::new([
            nw.into(), ne.into(),
            sw.into(), se.into(),
        ]));
        self.sub_insert(&self.meta.unwrap());
    }

    fn sub_insert(&mut self, meta: &QuadTreeMeta) {
        for child in self.children.as_mut().unwrap().iter_mut() {
            if child.insert(meta) { break }
        }
    }

}
