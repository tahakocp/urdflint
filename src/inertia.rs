/// Symmetric 3x3 matrix given as [ixx, ixy, ixz, iyy, iyz, izz].
pub fn eigenvalues(m: [f64; 6]) -> [f64; 3] {
    let mut a = [[m[0], m[1], m[2]], [m[1], m[3], m[4]], [m[2], m[4], m[5]]];
    // Cyclic Jacobi: 3x3 converges in a handful of sweeps.
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
        }
    }
    let mut ev = [a[0][0], a[1][1], a[2][2]];
    ev.sort_by(|x, y| x.partial_cmp(y).unwrap());
    ev
}

#[derive(Debug, PartialEq)]
pub enum Verdict {
    Valid,
    NotPositiveDefinite,
    TriangleViolation,
}

/// Relative tolerance on the triangle inequality: CAD exports of flat plates
/// land exactly on equality and rounding pushes them a hair over.
pub fn check(m: [f64; 6]) -> (Verdict, [f64; 3]) {
    let ev = eigenvalues(m);
    let scale = ev[2].abs().max(ev[0].abs()).max(1e-300);
    if ev[0] <= 1e-9 * scale {
        return (Verdict::NotPositiveDefinite, ev);
    }
    if ev[0] + ev[1] < ev[2] * (1.0 - 1e-6) {
        return (Verdict::TriangleViolation, ev);
    }
    (Verdict::Valid, ev)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_is_sorted() {
        let ev = eigenvalues([3.0, 0.0, 0.0, 1.0, 0.0, 2.0]);
        assert_eq!(ev, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn off_diagonal_rotation() {
        // [[2,1],[1,2]] block has eigenvalues 1 and 3; third is 5.
        let ev = eigenvalues([2.0, 1.0, 0.0, 2.0, 0.0, 5.0]);
        for (got, want) in ev.iter().zip([1.0, 3.0, 5.0]) {
            assert!((got - want).abs() < 1e-12, "{ev:?}");
        }
    }

    #[test]
    fn solid_box_is_valid() {
        let (m, w, h, d) = (2.0, 0.3, 0.2, 0.1);
        let ixx = m / 12.0 * (h * h + d * d);
        let iyy = m / 12.0 * (w * w + d * d);
        let izz = m / 12.0 * (w * w + h * h);
        assert_eq!(check([ixx, 0.0, 0.0, iyy, 0.0, izz]).0, Verdict::Valid);
    }

    #[test]
    fn flat_plate_on_the_boundary_is_valid() {
        assert_eq!(check([1.0, 0.0, 0.0, 1.0, 0.0, 2.0]).0, Verdict::Valid);
    }

    #[test]
    fn negative_moment() {
        assert_eq!(check([-0.1, 0.0, 0.0, 1.0, 0.0, 1.0]).0, Verdict::NotPositiveDefinite);
    }

    #[test]
    fn triangle_violation() {
        assert_eq!(check([0.001, 0.0, 0.0, 0.001, 0.0, 1.0]).0, Verdict::TriangleViolation);
    }

    #[test]
    fn coupled_but_not_positive_definite() {
        // Diagonal is positive, off-diagonal is too large: det < 0.
        assert_eq!(check([1.0, 2.0, 0.0, 1.0, 0.0, 1.0]).0, Verdict::NotPositiveDefinite);
    }
}
