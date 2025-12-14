use glam::DVec2;
use std::ops::Add;

const THETA: f64 = 0.5;

#[derive(Debug)]
pub struct AABB {
    mins: DVec2,
    maxs: DVec2,
    width: f64,
}

impl AABB {

    pub fn new(min: f64, max: f64) -> Self {
        AABB {
            mins: DVec2::new(min, min),
            maxs: DVec2::new(max, max),
            width: max - min,
        }
    }

    fn from(mins: DVec2, maxs: DVec2) -> Self {
        AABB {
            mins,
            maxs,
            width: maxs.x - mins.x,
        }
    }

    pub fn contains(&self, point: DVec2) -> bool {
        self.mins.cmple(point).all() &&
        self.maxs.cmpge(point).all()
    }

    pub fn half_extents(&self) -> DVec2 {
        (self.maxs - self.mins) * 0.5
    }

    pub fn transformed(&self, delta: DVec2) -> Self {
        AABB::from(self.mins + delta, self.maxs + delta)
    }

    pub fn width(&self) -> f64 {
        self.width
    }
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

    pub fn handle(&self) -> Option<usize> {
        self.handle
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn position(&self) -> DVec2 {
        self.position
    }
}

impl Add for QuadTreeMeta {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        let mass = self.mass + other.mass;
        let position = (
            self.position * self.mass +
            other.position * other.mass
        ) * mass.recip();
        QuadTreeMeta::new(None, mass, DVec2::from(position))
    }
}

pub struct QuadTreeNode {
    aabb: AABB,
    children: Option<Box<[Self;4]>>,
    meta: Option<QuadTreeMeta>,
}

impl QuadTreeNode {

    pub fn new(aabb: AABB) -> Self {
        QuadTreeNode {
            aabb,
            children: None,
            meta: None,
        }
    }

    pub fn calculate(&self, meta: QuadTreeMeta) -> DVec2 {
        // if there is meta
        if let Some(self_meta) = self.meta {
            let delta = self_meta.position() - meta.position();
            let dir = delta.normalize();
            let dist = delta.length_squared().recip();
            let mass = self_meta.mass() * meta.mass();
            // if an external node
            if let Some(self_handle) = self_meta.handle() {
                // and not the same body
                if self_handle != meta.handle.unwrap() {
                    return dir * mass * dist;
                }
            }
            // if dist ratio < theta
            if self.aabb.width() * self.aabb.width() * dist < THETA {
                return dir * mass * dist;
            }
            else if self.children.is_some() {
                // otherwise get sum of child forces
                let mut force = DVec2::ZERO;
                for child in self.children.as_ref().unwrap().iter() {
                    force += child.calculate(meta);
                }
                return force;
            }
        }
        return DVec2::ZERO;
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
        let half_extents = self.aabb.half_extents();
        let nw = AABB::from(self.aabb.mins, DVec2::from(self.aabb.mins + half_extents));
        let ne = nw.transformed(DVec2::X * half_extents.x);
        let sw = nw.transformed(DVec2::Y * half_extents.y);
        let se = nw.transformed(half_extents);
        self.children = Some(Box::new([
            Self::new(nw), Self::new(ne),
            Self::new(sw), Self::new(se),
        ]));
        self.sub_insert(&self.meta.unwrap());
    }

    fn sub_insert(&mut self, meta: &QuadTreeMeta) {
        for child in self.children.as_mut().unwrap().iter_mut() {
            if child.insert(meta) { break; }
        }
    }

}
