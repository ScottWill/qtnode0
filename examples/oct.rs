use std::time::Instant;

use glam::DVec3;
use qtnode0::{OctTreeMeta, OctTreeNode};
use rand::Rng as _;
use rayon::iter::{IntoParallelIterator as _, ParallelIterator as _};

fn main() {

    const G: f64 = 6.6743e-2;
    let min = -1.0_f64;
    let max =  1.0_f64;
    let meta_size = 100_000;

    let mut root = OctTreeNode::new(min, max);
    let mut metas = Vec::with_capacity(meta_size);

    let mut rng = rand::rng();

    for i in 0..meta_size {
        let pos = DVec3::new(
            rng.random_range(min..max),
            rng.random_range(min..max),
            rng.random_range(min..max),
        );
        let meta = OctTreeMeta::new(Some(i), 1.0, pos);
        metas.push(meta);
    }

    let now = Instant::now();

    for meta in &metas {
        root.insert(meta);
    }
    println!("creation: {:?}", now.elapsed());
    let now = Instant::now();

    let forces: Vec<DVec3> = metas
        .into_par_iter()
        .map(|meta| root.calculate(meta) * G)
        .collect();

    println!("qt par_calc: {:?}", now.elapsed());

    let f: DVec3 = forces.iter().sum();
    println!("{f:?}");

}
