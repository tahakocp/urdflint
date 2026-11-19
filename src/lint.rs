use crate::inertia::{self, Verdict};
use crate::rules::Diag;
use crate::stl;
use roxmltree::{Document, Node};
use std::borrow::Cow;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

#[derive(Default, Clone)]
pub struct Options {
    /// Skip every check that touches the file system.
    pub no_meshes: bool,
    /// package name -> directory containing that package.
    pub packages: HashMap<String, PathBuf>,
}

#[derive(Default, Debug)]
pub struct Summary {
    pub links: usize,
    pub joints: usize,
    pub dof: usize,
    pub mass: f64,
    pub root: Option<String>,
}

pub struct Report {
    pub diags: Vec<Diag>,
    pub summary: Summary,
}

const JOINT_TYPES: [&str; 6] = ["revolute", "continuous", "prismatic", "fixed", "floating", "planar"];

struct Ctx<'a, 'i> {
    doc: &'a Document<'i>,
    out: Vec<Diag>,
}

impl Ctx<'_, '_> {
    fn line(&self, n: Node) -> u32 {
        self.doc.text_pos_at(n.range().start).row
    }

    fn push(&mut self, code: &'static str, n: Node, msg: String) {
        let line = self.line(n);
        self.out.push(Diag::new(code, line, msg));
    }

    /// Parses `attr` as exactly `count` finite floats. Missing attribute gives None without a finding
    /// when `required` is false.
    fn floats(&mut self, n: Node, attr: &str, count: usize, required: bool) -> Option<Vec<f64>> {
        let Some(raw) = n.attribute(attr) else {
            if required {
                self.push("E024", n, format!("<{}> is missing `{attr}`", n.tag_name().name()));
            }
            return None;
        };
        let parsed: Vec<f64> = raw
            .split_whitespace()
            .filter_map(|s| s.parse::<f64>().ok().filter(|v| v.is_finite()))
            .collect();
        let tokens = raw.split_whitespace().count();
        if parsed.len() != tokens || tokens != count {
            self.push(
                "E024",
                n,
                format!("`{attr}=\"{raw}\"` on <{}> should be {count} finite number(s)", n.tag_name().name()),
            );
            return None;
        }
        Some(parsed)
    }
}

/// Gazebo plugins and sensor blocks often use prefixes (`sensor:`, `controller:`) that are never
/// declared. URDF readers ignore namespaces, roxmltree does not, so rename the prefix in place.
/// Same byte length, so line and column numbers stay correct.
fn strip_unknown_prefixes(src: &str) -> Cow<'_, str> {
    let mut text = Cow::Borrowed(src);
    for _ in 0..16 {
        let prefix = match Document::parse(&text) {
            Err(roxmltree::Error::UnknownNamespace(p, _)) if p != "xacro" => p,
            _ => break,
        };
        let mut out = String::with_capacity(text.len());
        let mut rest: &str = &text;
        let pat = format!("{prefix}:");
        while let Some(i) = rest.find(&pat) {
            let (head, tail) = rest.split_at(i);
            out.push_str(head);
            let at_name_start = head.chars().next_back().is_some_and(|c| c == '<' || c == '/' || c.is_whitespace());
            out.push_str(&prefix);
            out.push(if at_name_start { '_' } else { ':' });
            rest = &tail[pat.len()..];
        }
        out.push_str(rest);
        text = Cow::Owned(out);
    }
    text
}

pub fn lint_str(src: &str, file: &Path, opts: &Options) -> Report {
    let mut report = Report { diags: Vec::new(), summary: Summary::default() };
    let text = strip_unknown_prefixes(src);
    let doc = match Document::parse(&text) {
        Ok(d) => d,
        Err(e) => {
            let code = if src.contains("<xacro:") { "E003" } else { "E001" };
            report.diags.push(Diag::new(code, e.pos().row, e.to_string()));
            return report;
        }
    };
    let root = doc.root_element();
    let mut cx = Ctx { doc: &doc, out: Vec::new() };

    if root.tag_name().name() != "robot" {
        cx.push("E002", root, format!("root element is <{}>", root.tag_name().name()));
        report.diags = cx.out;
        return report;
    }
    let xacro = doc.descendants().find(|n| {
        n.tag_name().namespace().is_some_and(|ns| ns.contains("xacro"))
            || n.attributes().any(|a| a.value().contains("${") || a.value().contains("$("))
    });
    if let Some(n) = xacro {
        cx.push("E003", n, "file contains xacro elements or ${...} expressions".into());
        report.diags = cx.out;
        return report;
    }

    let base = file.parent().unwrap_or(Path::new("."));
    let links: Vec<Node> = root.children().filter(|n| n.has_tag_name("link")).collect();
    let joints: Vec<Node> = root.children().filter(|n| n.has_tag_name("joint")).collect();

    let link_names = check_names(&mut cx, &links, "E010", "link");
    check_names(&mut cx, &joints, "E011", "joint");

    let defined_materials: HashSet<&str> = root
        .children()
        .filter(|n| n.has_tag_name("material"))
        .filter_map(|n| n.attribute("name"))
        .collect();

    let tree = check_tree(&mut cx, &joints, &link_names);
    for j in &joints {
        check_joint(&mut cx, *j, &joints);
    }
    for l in &links {
        let moving = l
            .attribute("name")
            .map(|name| tree.moving.contains(name))
            .unwrap_or(false);
        check_link(&mut cx, *l, moving, base, opts, &defined_materials);
    }

    let mut summary = Summary { links: links.len(), joints: joints.len(), root: tree.root, ..Summary::default() };
    summary.dof = joints
        .iter()
        .filter(|j| match j.attribute("type") {
            Some("revolute" | "continuous" | "prismatic") => !j.children().any(|c| c.has_tag_name("mimic")),
            _ => false,
        })
        .count();
    summary.mass = links
        .iter()
        .filter_map(|l| l.children().find(|c| c.has_tag_name("inertial")))
        .filter_map(|i| i.children().find(|c| c.has_tag_name("mass")))
        .filter_map(|m| m.attribute("value")?.parse::<f64>().ok())
        .filter(|m| m.is_finite() && *m > 0.0)
        .sum();
    cx.out.sort_by_key(|d| d.line);
    report.diags = cx.out;
    report.summary = summary;
    report
}

fn check_names<'a>(cx: &mut Ctx, nodes: &[Node<'a, '_>], code: &'static str, what: &str) -> HashSet<&'a str> {
    let mut seen: HashMap<&str, u32> = HashMap::new();
    for n in nodes {
        let Some(name) = n.attribute("name") else {
            cx.push("E024", *n, format!("<{what}> is missing `name`"));
            continue;
        };
        if let Some(first) = seen.get(name) {
            cx.push(code, *n, format!("{what} `{name}` already defined on line {first}"));
        } else {
            seen.insert(name, cx.line(*n));
        }
    }
    seen.into_keys().collect()
}

struct Tree {
    root: Option<String>,
    /// Links whose parent joint can move.
    moving: HashSet<String>,
}

fn check_tree(cx: &mut Ctx, joints: &[Node], links: &HashSet<&str>) -> Tree {
    let mut parent_of: HashMap<&str, (&str, Node)> = HashMap::new();
    let mut moving = HashSet::new();
    for j in joints {
        let jname = j.attribute("name").unwrap_or("?");
        let end = |tag: &str| j.children().find(|c| c.has_tag_name(tag)).and_then(|c| c.attribute("link"));
        let (Some(parent), Some(child)) = (end("parent"), end("child")) else {
            cx.push("E012", *j, format!("joint `{jname}` needs both <parent link> and <child link>"));
            continue;
        };
        let mut ok = true;
        for (role, name) in [("parent", parent), ("child", child)] {
            if !links.contains(name) {
                cx.push("E012", *j, format!("joint `{jname}` {role} link `{name}` is not defined"));
                ok = false;
            }
        }
        if !ok {
            continue;
        }
        if let Some((first, _)) = parent_of.get(child) {
            cx.push("E013", *j, format!("link `{child}` already has parent `{first}`; `{jname}` adds `{parent}`"));
            continue;
        }
        parent_of.insert(child, (parent, *j));
        if j.attribute("type").is_some_and(|t| t != "fixed") {
            moving.insert(child.to_string());
        }
    }

    // A loop is any cycle in the child -> parent map.
    let mut reported: HashSet<&str> = HashSet::new();
    for start in parent_of.keys() {
        let mut path = vec![*start];
        let mut cur = *start;
        while let Some((p, _)) = parent_of.get(cur) {
            if let Some(pos) = path.iter().position(|n| n == p) {
                let cycle = &path[pos..];
                if cycle.iter().all(|n| reported.insert(n)) {
                    let (_, joint) = parent_of[cycle[0]];
                    cx.push("E014", joint, format!("loop through {}", cycle.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(" -> ")));
                }
                break;
            }
            path.push(p);
            cur = p;
        }
    }

    let mut roots: Vec<&str> = links.iter().copied().filter(|l| !parent_of.contains_key(l)).collect();
    roots.sort();
    if roots.len() > 1 && cx.out.iter().all(|d| d.code != "E014") {
        cx.out.push(Diag::new(
            "E015",
            cx.line(cx.doc.root_element()),
            format!("{} root links: {}", roots.len(), roots.iter().map(|r| format!("`{r}`")).collect::<Vec<_>>().join(", ")),
        ));
    }
    Tree { root: roots.first().filter(|_| roots.len() == 1).map(|s| s.to_string()), moving }
}

fn check_joint(cx: &mut Ctx, j: Node, all: &[Node]) {
    let name = j.attribute("name").unwrap_or("?");
    let kind = j.attribute("type").unwrap_or("");
    if !JOINT_TYPES.contains(&kind) {
        cx.push("E020", j, format!("joint `{name}` has type \"{kind}\""));
        return;
    }
    if let Some(o) = j.children().find(|c| c.has_tag_name("origin")) {
        cx.floats(o, "xyz", 3, false);
        cx.floats(o, "rpy", 3, false);
    }
    // Fixed joints ignore the axis, and exporters leave 0 0 0 there.
    let axis = j.children().find(|c| c.has_tag_name("axis")).filter(|_| kind != "fixed");
    if let Some(a) = axis {
        if let Some(v) = cx.floats(a, "xyz", 3, false) {
            if v.iter().map(|x| x * x).sum::<f64>() < 1e-18 {
                cx.push("E023", a, format!("joint `{name}` axis is 0 0 0"));
            }
        }
    }
    if let Some(m) = j.children().find(|c| c.has_tag_name("mimic")) {
        let target = m.attribute("joint").unwrap_or("");
        if !all.iter().any(|o| o.attribute("name") == Some(target)) {
            cx.push("E025", m, format!("joint `{name}` mimics `{target}`, which is not defined"));
        }
    }
    let limit = j.children().find(|c| c.has_tag_name("limit"));
    let needs_limit = matches!(kind, "revolute" | "prismatic");
    let Some(limit) = limit else {
        if needs_limit {
            cx.push("E021", j, format!("{kind} joint `{name}` has no <limit>"));
        }
        return;
    };
    let get = |cx: &mut Ctx, attr: &str| -> Option<f64> {
        limit.attribute(attr)?;
        cx.floats(limit, attr, 1, false).map(|v| v[0])
    };
    let (lo, hi) = (get(cx, "lower"), get(cx, "upper"));
    let (effort, velocity) = (get(cx, "effort"), get(cx, "velocity"));
    if needs_limit {
        if limit.attribute("effort").is_none() || limit.attribute("velocity").is_none() {
            cx.push("W020", limit, format!("joint `{name}` limit needs both effort and velocity"));
        }
        if let (Some(lo), Some(hi)) = (lo, hi) {
            if lo > hi {
                cx.push("E022", limit, format!("joint `{name}` lower {lo} > upper {hi}"));
            }
            let wide = |v: f64| (4.0 * std::f64::consts::PI..=1000.0).contains(&v.abs());
            if kind == "revolute" && (wide(lo) || wide(hi)) {
                cx.push("W022", limit, format!("joint `{name}` range [{lo}, {hi}] is wider than two turns each way; degrees?"));
            }
        }
    }
    for (label, v) in [("effort", effort), ("velocity", velocity)] {
        if v.is_some_and(|v| v <= 0.0) {
            cx.push("W021", limit, format!("joint `{name}` {label} is {}", v.unwrap()));
        }
    }
}

fn check_link(cx: &mut Ctx, l: Node, moving: bool, base: &Path, opts: &Options, materials: &HashSet<&str>) {
    let name = l.attribute("name").unwrap_or("?");
    match l.children().find(|c| c.has_tag_name("inertial")) {
        Some(i) => check_inertial(cx, i, name, moving),
        None if moving => cx.push("W032", l, format!("link `{name}` is moved by a joint but has no <inertial>")),
        None => {}
    }
    for tag in ["visual", "collision"] {
        for v in l.children().filter(|c| c.has_tag_name(tag)) {
            if let Some(o) = v.children().find(|c| c.has_tag_name("origin")) {
                cx.floats(o, "xyz", 3, false);
                cx.floats(o, "rpy", 3, false);
            }
            if let Some(g) = v.children().find(|c| c.has_tag_name("geometry")) {
                for shape in g.children().filter(|c| c.is_element()) {
                    check_shape(cx, shape, base, opts);
                }
            }
            if let Some(m) = v.children().find(|c| c.has_tag_name("material")) {
                let defined_here = m.children().any(|c| c.has_tag_name("color") || c.has_tag_name("texture"));
                if let Some(mn) = m.attribute("name") {
                    if !defined_here && !materials.contains(mn) {
                        cx.push("W050", m, format!("material `{mn}` is not defined"));
                    }
                }
            }
        }
    }
}

fn zero_inertia(i: Node) -> bool {
    i.children().find(|c| c.has_tag_name("inertia")).is_some_and(|t| {
        ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]
            .iter()
            .all(|a| t.attribute(*a).and_then(|v| v.parse::<f64>().ok()) == Some(0.0))
    })
}

fn check_inertial(cx: &mut Ctx, i: Node, link: &str, moving: bool) {
    let mass_node = i.children().find(|c| c.has_tag_name("mass"));
    let mass = mass_node.and_then(|m| cx.floats(m, "value", 1, true)).map(|v| v[0]);
    if mass_node.is_none() {
        cx.push("E030", i, format!("link `{link}` <inertial> has no <mass>"));
    }
    if let Some(m) = mass {
        let at = mass_node.unwrap();
        if m == 0.0 && zero_inertia(i) && !moving {
            cx.push("W034", at, format!("link `{link}` has mass 0 and zero inertia"));
            return;
        } else if m <= 0.0 {
            cx.push("E030", at, format!("link `{link}` mass is {m}"));
            return;
        } else if !(1e-4..=1e4).contains(&m) {
            cx.push("W030", at, format!("link `{link}` mass is {m} kg"));
        }
    }
    if let Some(o) = i.children().find(|c| c.has_tag_name("origin")) {
        cx.floats(o, "xyz", 3, false);
        cx.floats(o, "rpy", 3, false);
    }
    let Some(t) = i.children().find(|c| c.has_tag_name("inertia")) else {
        cx.push("E031", i, format!("link `{link}` <inertial> has no <inertia>"));
        return;
    };
    let mut v = [0.0; 6];
    for (k, attr) in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"].iter().enumerate() {
        match cx.floats(t, attr, 1, true) {
            Some(x) => v[k] = x[0],
            None => return,
        }
    }
    if mass == Some(1.0) && v == [1.0, 0.0, 0.0, 1.0, 0.0, 1.0] {
        cx.push("W031", t, format!("link `{link}` has mass 1 and identity inertia"));
        return;
    }
    let (verdict, ev) = inertia::check(v);
    match verdict {
        Verdict::NotPositiveDefinite => cx.push(
            "E031",
            t,
            format!("link `{link}` principal moments {:.4e} {:.4e} {:.4e}; smallest is not positive", ev[0], ev[1], ev[2]),
        ),
        Verdict::TriangleViolation => cx.push(
            "E032",
            t,
            format!("link `{link}` principal moments {:.4e} + {:.4e} < {:.4e}", ev[0], ev[1], ev[2]),
        ),
        Verdict::Valid if ev[2] / ev[0] > 1e4 => cx.push(
            "W033",
            t,
            format!("link `{link}` principal moments span {:.1e}x ({:.3e} .. {:.3e})", ev[2] / ev[0], ev[0], ev[2]),
        ),
        Verdict::Valid => {}
    }
}

fn check_shape(cx: &mut Ctx, s: Node, base: &Path, opts: &Options) {
    let tag = s.tag_name().name();
    let positive = |cx: &mut Ctx, vals: &[f64], what: &str| {
        if vals.iter().any(|v| *v <= 0.0) {
            cx.push("E041", s, format!("<{tag}> {what} must be positive"));
        }
    };
    match tag {
        "box" => {
            if let Some(v) = cx.floats(s, "size", 3, true) {
                positive(cx, &v, "size");
            }
        }
        "sphere" => {
            if let Some(v) = cx.floats(s, "radius", 1, true) {
                positive(cx, &v, "radius");
            }
        }
        "cylinder" | "capsule" => {
            for attr in ["radius", "length"] {
                if let Some(v) = cx.floats(s, attr, 1, true) {
                    positive(cx, &v, attr);
                }
            }
        }
        "mesh" => check_mesh(cx, s, base, opts),
        _ => {}
    }
}

fn check_mesh(cx: &mut Ctx, s: Node, base: &Path, opts: &Options) {
    let scale = match s.attribute("scale") {
        Some(_) => cx.floats(s, "scale", 3, false).unwrap_or(vec![1.0; 3]),
        None => vec![1.0; 3],
    };
    if scale.iter().any(|v| *v <= 0.0) {
        cx.push("E041", s, "mesh scale must be positive".into());
    }
    if opts.no_meshes {
        return;
    }
    let Some(file) = s.attribute("filename") else {
        cx.push("E024", s, "<mesh> is missing `filename`".into());
        return;
    };
    let path = match resolve(file, base, opts) {
        Resolved::Path(p) => p,
        Resolved::Unknown(pkg) => {
            cx.push("W041", s, format!("package `{pkg}` not found near this file; mesh `{file}` unchecked"));
            return;
        }
    };
    if !path.is_file() {
        cx.push("E040", s, format!("`{file}` does not exist ({})", path.display()));
        return;
    }
    let is_stl = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("stl"));
    if is_stl {
        if let Some(ext) = stl::max_extent(&path) {
            let size = ext * scale.iter().cloned().fold(0.0, f64::max);
            if !(1e-3..=20.0).contains(&size) {
                cx.push("W040", s, format!("`{file}` is {size:.4} m in its largest dimension after scale"));
            }
        }
    }
}

enum Resolved {
    Path(PathBuf),
    Unknown(String),
}

fn resolve(file: &str, base: &Path, opts: &Options) -> Resolved {
    if let Some(rest) = file.strip_prefix("package://") {
        let (pkg, tail) = rest.split_once('/').unwrap_or((rest, ""));
        if let Some(dir) = opts.packages.get(pkg) {
            return Resolved::Path(dir.join(tail));
        }
        // Workspace layouts put the package next to the file, or one of its ancestors.
        for anc in base.ancestors().take(8) {
            if anc.file_name().is_some_and(|n| n == pkg) {
                return Resolved::Path(anc.join(tail));
            }
            let cand = anc.join(pkg);
            if cand.is_dir() {
                return Resolved::Path(cand.join(tail));
            }
        }
        if let Ok(rpp) = std::env::var("ROS_PACKAGE_PATH") {
            for dir in rpp.split(':').filter(|d| !d.is_empty()) {
                let cand = Path::new(dir).join(pkg);
                if cand.is_dir() {
                    return Resolved::Path(cand.join(tail));
                }
            }
        }
        return Resolved::Unknown(pkg.to_string());
    }
    if let Some(rest) = file.strip_prefix("file://") {
        return Resolved::Path(PathBuf::from(rest));
    }
    let p = Path::new(file);
    Resolved::Path(if p.is_absolute() { p.to_path_buf() } else { base.join(p) })
}
