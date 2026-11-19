use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use urdflint::lint::{lint_str, Options};
use urdflint::rules::RULES;

fn dir(sub: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(sub)
}

fn urdfs(sub: &str) -> Vec<PathBuf> {
    let mut v: Vec<_> = fs::read_dir(dir(sub))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "urdf"))
        .collect();
    v.sort();
    v
}

fn opts() -> Options {
    let mut o = Options::default();
    o.packages.insert("mesh_pkg".into(), dir("good"));
    o
}

#[test]
fn good_files_are_clean() {
    for f in urdfs("good") {
        let rep = lint_str(&fs::read_to_string(&f).unwrap(), &f, &opts());
        assert!(rep.diags.is_empty(), "{}: {:?}", f.display(), rep.diags);
    }
}

#[test]
fn every_bad_file_reports_its_code() {
    for f in urdfs("bad") {
        let code = f.file_name().unwrap().to_str().unwrap()[..4].to_string();
        let rep = lint_str(&fs::read_to_string(&f).unwrap(), &f, &opts());
        let codes: Vec<_> = rep.diags.iter().map(|d| d.code).collect();
        assert!(codes.contains(&code.as_str()), "{}: wanted {code}, got {codes:?}", f.display());
        // fixtures are minimal on purpose: nothing else should fire
        assert_eq!(codes.len(), 1, "{}: {codes:?}", f.display());
    }
}

#[test]
fn every_rule_has_a_fixture() {
    let have: Vec<String> = urdfs("bad")
        .iter()
        .map(|p| p.file_name().unwrap().to_str().unwrap()[..4].to_string())
        .collect();
    for r in RULES {
        assert!(have.iter().any(|h| h == r.code), "no fixture for {}", r.code);
    }
}

#[test]
fn diagnostics_carry_line_numbers() {
    let f = dir("bad").join("E012_unknown_link.urdf");
    let rep = lint_str(&fs::read_to_string(&f).unwrap(), &f, &opts());
    assert_eq!(rep.diags[0].line, 4);
}

#[test]
fn summary_counts_dof_and_mass() {
    let f = dir("good").join("arm.urdf");
    let rep = lint_str(&fs::read_to_string(&f).unwrap(), &f, &opts());
    assert_eq!(rep.summary.links, 4);
    assert_eq!(rep.summary.joints, 3);
    assert_eq!(rep.summary.dof, 2);
    assert!((rep.summary.mass - 6.3).abs() < 1e-9);
    assert_eq!(rep.summary.root.as_deref(), Some("base"));

    let f = dir("good").join("slider_mimic.urdf");
    let rep = lint_str(&fs::read_to_string(&f).unwrap(), &f, &opts());
    assert_eq!(rep.summary.dof, 1, "mimic joint is not an independent DOF");
}

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_urdflint"))
}

#[test]
fn exit_codes() {
    let good = dir("good").join("arm.urdf");
    let bad = dir("bad").join("E022_limit_order.urdf");
    let warn = dir("bad").join("W031_placeholder.urdf");
    assert_eq!(bin().arg(&good).status().unwrap().code(), Some(0));
    assert_eq!(bin().arg(&bad).status().unwrap().code(), Some(1));
    assert_eq!(bin().arg(&warn).status().unwrap().code(), Some(0));
    assert_eq!(bin().arg("--deny-warnings").arg(&warn).status().unwrap().code(), Some(1));
    assert_eq!(bin().args(["--allow", "W031", "--deny-warnings"]).arg(&warn).status().unwrap().code(), Some(0));
    assert_eq!(bin().arg("nonexistent.urdf").status().unwrap().code(), Some(2));
    assert_eq!(bin().arg("--bogus").status().unwrap().code(), Some(2));
}

#[test]
fn output_formats() {
    let bad = dir("bad").join("E023_zero_axis.urdf");
    let out = |fmt: &str| {
        let o = bin().args(["--format", fmt]).arg(&bad).output().unwrap();
        String::from_utf8(o.stdout).unwrap()
    };
    assert!(out("text").contains("error[E023]"));
    let j = out("json");
    assert!(j.starts_with("[{") && j.contains("\"code\":\"E023\""), "{j}");
    let g = out("github");
    assert!(g.starts_with("::error file=") && g.contains("title=E023::"), "{g}");
}

#[test]
fn directory_walk_finds_nested_files() {
    let o = bin().arg("-q").arg(dir("good")).output().unwrap();
    assert!(o.status.success());
    let o = bin().arg("--format").arg("json").arg(dir("bad")).output().unwrap();
    let s = String::from_utf8(o.stdout).unwrap();
    assert!(s.contains("E015") && s.contains("W050"));
}

#[test]
fn explain_and_list() {
    let o = bin().args(["--explain", "e032"]).output().unwrap();
    assert!(String::from_utf8(o.stdout).unwrap().contains("triangle inequality"));
    let o = bin().arg("--list-rules").output().unwrap();
    assert_eq!(String::from_utf8(o.stdout).unwrap().lines().count(), RULES.len());
}
