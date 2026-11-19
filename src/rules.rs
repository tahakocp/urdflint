use std::fmt;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Severity {
    Error,
    Warning,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Severity::Error => "error",
            Severity::Warning => "warning",
        })
    }
}

pub struct Rule {
    pub code: &'static str,
    pub severity: Severity,
    pub summary: &'static str,
    pub why: &'static str,
}

use Severity::{Error, Warning};

pub const RULES: &[Rule] = &[
    Rule { code: "E001", severity: Error, summary: "file is not well-formed XML",
        why: "Nothing else can be checked. The message carries the parser's line and column." },
    Rule { code: "E002", severity: Error, summary: "root element is not <robot>",
        why: "URDF requires <robot name=\"...\"> as the document element." },
    Rule { code: "E003", severity: Error, summary: "unexpanded xacro input",
        why: "The file uses xacro macros or ${...} expressions. Run `xacro in.urdf.xacro > out.urdf` and lint the output." },
    Rule { code: "E010", severity: Error, summary: "duplicate link name",
        why: "Link names are identifiers. Parsers keep one of the duplicates and drop the other without saying which." },
    Rule { code: "E011", severity: Error, summary: "duplicate joint name",
        why: "Same as E010 for joints. Controllers address joints by name, so the wrong one gets commanded." },
    Rule { code: "E012", severity: Error, summary: "joint refers to a link that is not defined",
        why: "Every <parent link> and <child link> must name an existing <link>. Typos here are the most common reason a model loads with a missing limb." },
    Rule { code: "E013", severity: Error, summary: "link has more than one parent joint",
        why: "A URDF is a tree. Closed chains cannot be expressed with a second parent joint; use mimic joints or a simulator-specific constraint." },
    Rule { code: "E014", severity: Error, summary: "kinematic loop",
        why: "Following parent joints leads back to the same link. No root can be found, so most tools refuse the file." },
    Rule { code: "E015", severity: Error, summary: "more than one root link",
        why: "All links must hang off a single root. A second root usually means a joint was deleted or a name changed." },
    Rule { code: "E020", severity: Error, summary: "unknown joint type",
        why: "Valid types: revolute, continuous, prismatic, fixed, floating, planar." },
    Rule { code: "E021", severity: Error, summary: "revolute or prismatic joint without <limit>",
        why: "The URDF spec makes <limit> mandatory for these two types, including effort and velocity." },
    Rule { code: "E022", severity: Error, summary: "lower limit is greater than upper limit",
        why: "The joint range is empty. Some simulators then lock the joint at an arbitrary value." },
    Rule { code: "E023", severity: Error, summary: "joint axis has zero length",
        why: "A zero axis cannot be normalised; the joint's motion direction is undefined (NaN in several engines)." },
    Rule { code: "E024", severity: Error, summary: "malformed number or vector",
        why: "xyz, rpy, size, scale and inertia values must be finite numbers, with the right count." },
    Rule { code: "E025", severity: Error, summary: "mimic joint refers to an undefined joint",
        why: "<mimic joint=\"...\"> must name another joint in the same file." },
    Rule { code: "E030", severity: Error, summary: "non-positive mass",
        why: "Mass must be greater than zero. Zero mass makes the inverse mass matrix infinite. On a link that no joint moves, a bare zero-mass frame marker is reported as W034 instead." },
    Rule { code: "E031", severity: Error, summary: "inertia matrix is not positive definite",
        why: "A real body always has positive principal moments. Negative or zero ones come from sign errors or from exporting a mesh with flipped normals." },
    Rule { code: "E032", severity: Error, summary: "principal moments break the triangle inequality",
        why: "For any rigid body, the sum of two principal moments is at least the third. Violations mean the numbers cannot belong to any mass distribution, even if the matrix is positive definite." },
    Rule { code: "E040", severity: Error, summary: "mesh file does not exist",
        why: "The path resolves to a location on disk and nothing is there." },
    Rule { code: "E041", severity: Error, summary: "non-positive geometry size or mesh scale",
        why: "Box sides, cylinder and sphere dimensions and mesh scale factors must be greater than zero." },
    Rule { code: "W020", severity: Warning, summary: "limit has no effort or no velocity",
        why: "Both are required by the spec. Some loaders default them to 0, which freezes the joint under torque control." },
    Rule { code: "W021", severity: Warning, summary: "effort or velocity limit is not positive",
        why: "A zero limit means the joint cannot move under a controller that honours limits." },
    Rule { code: "W022", severity: Warning, summary: "revolute limit beyond +-4 pi",
        why: "Likely degrees written where radians are expected." },
    Rule { code: "W030", severity: Warning, summary: "mass is extreme (below 0.1 g or above 10 t)",
        why: "Usually a unit slip (grams or tonnes). Mass ratios above a few thousand between touching bodies also make contact solvers unstable." },
    Rule { code: "W031", severity: Warning, summary: "inertial values look like a placeholder",
        why: "mass 1 with ixx = iyy = izz = 1 is the default that tools write when no one filled it in." },
    Rule { code: "W032", severity: Warning, summary: "moving link has no <inertial>",
        why: "Physics engines treat it as massless or merge it into its parent, and the behaviour differs between engines." },
    Rule { code: "W033", severity: Warning, summary: "principal moments differ by more than 10^4",
        why: "Possible for a long thin rod, rare otherwise. Often a sign that one moment is in the wrong unit." },
    Rule { code: "W034", severity: Warning, summary: "massless placeholder link",
        why: "mass 0 with zero inertia on a link that no joint moves. Fine for a frame marker, but several tools drop or merge such links and the TF frame can vanish with it." },
    Rule { code: "W040", severity: Warning, summary: "mesh extent suggests a unit mistake",
        why: "After scale, the mesh is larger than 20 m or smaller than 1 mm in its biggest dimension. CAD exports in millimetres without scale=\"0.001\" are the usual cause. Checked for STL only." },
    Rule { code: "W041", severity: Warning, summary: "package:// path could not be resolved",
        why: "Pass --package-path NAME=DIR so the file can be checked, or ignore with --allow W041." },
    Rule { code: "W050", severity: Warning, summary: "material is referenced but never defined",
        why: "A <material name> with no colour or texture and no global definition renders as the default." },
];

pub fn find(code: &str) -> Option<&'static Rule> {
    RULES.iter().find(|r| r.code.eq_ignore_ascii_case(code))
}

#[derive(Debug, Clone, PartialEq)]
pub struct Diag {
    pub code: &'static str,
    pub severity: Severity,
    pub line: u32,
    pub message: String,
}

impl Diag {
    pub fn new(code: &'static str, line: u32, message: impl Into<String>) -> Diag {
        let severity = find(code).map(|r| r.severity).expect("known rule code");
        Diag { code, severity, line, message: message.into() }
    }
}
