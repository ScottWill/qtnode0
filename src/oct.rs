use glam::DVec3;
use std::ops::Add;

const THETA: f64 = 0.5;

#[derive(Debug)]
struct AABB {
    half: DVec3,
    mins: DVec3,
    maxs: DVec3,
    width: f64,
}

impl AABB {

    fn new(min: f64, max: f64) -> Self {
        let mins = DVec3::splat(min);
        let maxs = DVec3::splat(max);
        Self::from(mins, maxs)
    }

    fn from(mins: DVec3, maxs: DVec3) -> Self {
        AABB {
            half: (maxs - mins) * 0.5,
            width: maxs.x - mins.x,
            mins,
            maxs,
        }
    }

    #[inline]
    fn contains(&self, point: DVec3) -> bool {
        self.mins.cmple(point).all() &&
        self.maxs.cmpge(point).all()
    }

    #[inline]
    fn transformed(&self, delta: DVec3) -> Self {
        AABB::from(self.mins + delta, self.maxs + delta)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OctTreeMeta {
    handle: Option<usize>,
    mass: f64,
    position: DVec3,
}

impl OctTreeMeta {
    pub fn new(handle: Option<usize>, mass: f64, position: DVec3) -> Self {
        OctTreeMeta { handle, mass, position }
    }
}

impl Add for OctTreeMeta {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        let mass = self.mass + other.mass;
        let position = self.position.lerp(other.position, other.mass / mass);
        OctTreeMeta::new(None, mass, position)
    }
}

pub struct OctTreeNode {
    aabb: AABB,
    children: Option<Box<[Self;8]>>,
    meta: Option<OctTreeMeta>,
}

impl From<AABB> for OctTreeNode {
    fn from(value: AABB) -> Self {
        OctTreeNode {
            aabb: value,
            children: None,
            meta: None,
        }
    }
}

impl OctTreeNode {

    pub fn new(min: f64, max: f64) -> Self {
        let (min, max) = (
            f64::min(min, max),
            f64::max(min, max),
        );
        AABB::new(min, max).into()
    }

    pub fn calculate(&self, meta: OctTreeMeta) -> DVec3 {
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
                let mut force = DVec3::ZERO;
                for child in self.children.as_ref().unwrap().iter() {
                    force += child.calculate(meta);
                }
                force
            } else {
                DVec3::ZERO
            }
        } else {
            DVec3::ZERO
        }
    }

    pub fn insert(&mut self, new_meta: &OctTreeMeta) -> bool {
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

    fn add_meta(&mut self, other_meta: OctTreeMeta) {
        self.meta = Some(match self.meta {
            Some(meta) => meta + other_meta,
            None => other_meta
        });
    }

    fn sub_divide(&mut self) {
        let half_extents = self.aabb.half;
        let dz = DVec3::Z * half_extents.z;
        let nw0 = AABB::from(self.aabb.mins, DVec3::from(self.aabb.mins + half_extents));
        let ne0 = nw0.transformed(DVec3::X * half_extents.x);
        let sw0 = nw0.transformed(DVec3::Y * half_extents.y);
        let se0 = nw0.transformed(half_extents);
        let nw1 = nw0.transformed(dz);
        let ne1 = ne0.transformed(dz);
        let sw1 = sw0.transformed(dz);
        let se1 = se0.transformed(dz);
        self.children = Some(Box::new([
            nw0.into(), ne0.into(),
            sw0.into(), se0.into(),
            nw1.into(), ne1.into(),
            sw1.into(), se1.into(),
        ]));
        self.sub_insert(&self.meta.unwrap());
    }

    fn sub_insert(&mut self, meta: &OctTreeMeta) {
        for child in self.children.as_mut().unwrap().iter_mut() {
            if child.insert(meta) { break }
        }
    }

}
