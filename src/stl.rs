use std::fs;
use std::path::Path;

/// Largest side of the axis-aligned bounding box, in file units.
pub fn max_extent(path: &Path) -> Option<f64> {
    let data = fs::read(path).ok()?;
    let verts = binary(&data).or_else(|| ascii(&data))?;
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for v in verts {
        any = true;
        for i in 0..3 {
            lo[i] = lo[i].min(v[i]);
            hi[i] = hi[i].max(v[i]);
        }
    }
    if !any {
        return None;
    }
    Some((0..3).map(|i| hi[i] - lo[i]).fold(0.0, f64::max))
}

fn binary(data: &[u8]) -> Option<Vec<[f64; 3]>> {
    if data.len() < 84 {
        return None;
    }
    let n = u32::from_le_bytes(data[80..84].try_into().ok()?) as usize;
    if 84usize.checked_add(n.checked_mul(50)?)? != data.len() {
        return None;
    }
    let f = |o: usize| f32::from_le_bytes(data[o..o + 4].try_into().unwrap()) as f64;
    let mut out = Vec::with_capacity(n * 3);
    for t in 0..n {
        let base = 84 + t * 50 + 12;
        for v in 0..3 {
            let o = base + v * 12;
            let p = [f(o), f(o + 4), f(o + 8)];
            if p.iter().any(|x| !x.is_finite()) {
                return None;
            }
            out.push(p);
        }
    }
    Some(out)
}

fn ascii(data: &[u8]) -> Option<Vec<[f64; 3]>> {
    let text = std::str::from_utf8(data).ok()?;
    if !text.trim_start().starts_with("solid") {
        return None;
    }
    let mut out = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        if it.next() == Some("vertex") {
            let p: Vec<f64> = it.filter_map(|s| s.parse().ok()).collect();
            if p.len() == 3 {
                out.push([p[0], p[1], p[2]]);
            }
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ascii_cube_extent() {
        let dir = std::env::temp_dir().join("urdflint-stl-test");
        fs::create_dir_all(&dir).unwrap();
        let p = dir.join("tri.stl");
        fs::write(&p, "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 2 0 0\nvertex 0 3 0.5\nendloop\nendfacet\nendsolid t\n").unwrap();
        assert_eq!(max_extent(&p), Some(3.0));
    }

    #[test]
    fn binary_extent() {
        let dir = std::env::temp_dir().join("urdflint-stl-test");
        fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bin.stl");
        let mut d = vec![0u8; 80];
        d.extend_from_slice(&1u32.to_le_bytes());
        d.extend_from_slice(&[0u8; 12]);
        for v in [[0.0f32, 0.0, 0.0], [500.0, 0.0, 0.0], [0.0, 20.0, 0.0]] {
            for c in v {
                d.extend_from_slice(&c.to_le_bytes());
            }
        }
        d.extend_from_slice(&[0, 0]);
        fs::write(&p, d).unwrap();
        assert_eq!(max_extent(&p), Some(500.0));
    }
}
