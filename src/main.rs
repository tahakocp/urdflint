use std::collections::{BTreeSet, HashMap};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use urdflint::lint::{lint_str, Options};
use urdflint::report;
use urdflint::rules::{self, Severity};

const HELP: &str = "\
urdflint: lint URDF robot descriptions

USAGE
    urdflint [OPTIONS] <FILE|DIR>...

Directories are searched recursively for *.urdf files.

OPTIONS
    --format <text|json|github>   output format (default: text)
    --allow <CODE>                silence a rule, repeatable (e.g. --allow W033)
    --deny-warnings               exit 1 on warnings too
    --package-path <NAME=DIR>     where package://NAME/ lives, repeatable
    --no-meshes                   do not look at mesh files
    -q, --quiet                   findings only, no per-file summary
    --list-rules                  print all rules
    --explain <CODE>              describe one rule
    -V, --version
    -h, --help

EXIT STATUS
    0 clean, 1 findings, 2 usage or I/O error
";

struct Args {
    format: String,
    allow: BTreeSet<String>,
    deny_warnings: bool,
    quiet: bool,
    opts: Options,
    paths: Vec<PathBuf>,
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut a = Args {
        format: "text".into(),
        allow: BTreeSet::new(),
        deny_warnings: false,
        quiet: false,
        opts: Options { no_meshes: false, packages: HashMap::new() },
        paths: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    let value = |it: &mut dyn Iterator<Item = String>, flag: &str| it.next().ok_or(format!("{flag} needs a value"));
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{HELP}");
                return Ok(None);
            }
            "-V" | "--version" => {
                println!("urdflint {}", env!("CARGO_PKG_VERSION"));
                return Ok(None);
            }
            "--list-rules" => {
                for r in rules::RULES {
                    println!("{}  {:<7} {}", r.code, r.severity, r.summary);
                }
                return Ok(None);
            }
            "--explain" => {
                let code = value(&mut it, "--explain")?;
                let r = rules::find(&code).ok_or(format!("no rule named {code}"))?;
                println!("{} ({}): {}\n\n{}", r.code, r.severity, r.summary, r.why);
                return Ok(None);
            }
            "--format" => {
                a.format = value(&mut it, "--format")?;
                if !["text", "json", "github"].contains(&a.format.as_str()) {
                    return Err(format!("unknown format `{}`", a.format));
                }
            }
            "--allow" => {
                let c = value(&mut it, "--allow")?.to_uppercase();
                if rules::find(&c).is_none() {
                    return Err(format!("no rule named {c}"));
                }
                a.allow.insert(c);
            }
            "--package-path" => {
                let v = value(&mut it, "--package-path")?;
                let (n, d) = v.split_once('=').ok_or("--package-path expects NAME=DIR")?;
                a.opts.packages.insert(n.to_string(), PathBuf::from(d));
            }
            "--deny-warnings" => a.deny_warnings = true,
            "--no-meshes" => a.opts.no_meshes = true,
            "-q" | "--quiet" => a.quiet = true,
            s if s.starts_with('-') && s.len() > 1 => return Err(format!("unknown option {s}")),
            _ => a.paths.push(PathBuf::from(arg)),
        }
    }
    if a.paths.is_empty() {
        return Err("no input files; try --help".into());
    }
    Ok(Some(a))
}

fn collect(p: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    if p.is_dir() {
        let mut entries: Vec<_> = fs::read_dir(p)?.collect::<Result<_, _>>()?;
        entries.sort_by_key(|e| e.file_name());
        for e in entries {
            let path = e.path();
            if path.is_dir() || path.extension().is_some_and(|x| x == "urdf") {
                collect(&path, out)?;
            }
        }
    } else {
        out.push(p.to_path_buf());
    }
    Ok(())
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(Some(a)) => a,
        Ok(None) => return ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("urdflint: {e}");
            return ExitCode::from(2);
        }
    };
    let mut files = Vec::new();
    for p in &args.paths {
        if let Err(e) = collect(p, &mut files) {
            eprintln!("urdflint: {}: {e}", p.display());
            return ExitCode::from(2);
        }
    }
    if files.is_empty() {
        eprintln!("urdflint: no .urdf files found");
        return ExitCode::from(2);
    }

    let (mut total_err, mut total_warn, mut json_items) = (0usize, 0usize, Vec::new());
    for f in &files {
        let src = match fs::read(f) {
            Ok(b) => String::from_utf8_lossy(&b).into_owned(),
            Err(e) => {
                eprintln!("urdflint: {}: {e}", f.display());
                return ExitCode::from(2);
            }
        };
        let name = f.display().to_string();
        let rep = lint_str(&src, f, &args.opts);
        let diags: Vec<_> = rep.diags.iter().filter(|d| !args.allow.contains(d.code)).collect();
        let errors = diags.iter().filter(|d| d.severity == Severity::Error).count();
        let warnings = diags.len() - errors;
        total_err += errors;
        total_warn += warnings;
        for d in &diags {
            match args.format.as_str() {
                "json" => json_items.push(report::json(&name, d)),
                "github" => println!("{}", report::github(&name, d)),
                _ => println!("{}", report::text(&name, d)),
            }
        }
        if args.format == "text" && !args.quiet && rep.summary.links > 0 {
            println!("{}", report::summary_line(&name, &rep.summary, errors, warnings));
        }
    }
    if args.format == "json" {
        println!("[{}]", json_items.join(","));
    }
    if args.format == "text" && !args.quiet && files.len() > 1 {
        println!("{} files, {total_err} error(s), {total_warn} warning(s)", files.len());
    }
    if total_err > 0 || (args.deny_warnings && total_warn > 0) {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}
