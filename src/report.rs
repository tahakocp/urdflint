use crate::lint::Summary;
use crate::rules::Diag;

pub fn json_escape(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o
}

pub fn text(file: &str, d: &Diag) -> String {
    format!("{file}:{}: {}[{}]: {}", d.line, d.severity, d.code, d.message)
}

pub fn json(file: &str, d: &Diag) -> String {
    format!(
        "{{\"file\":\"{}\",\"line\":{},\"code\":\"{}\",\"severity\":\"{}\",\"message\":\"{}\"}}",
        json_escape(file),
        d.line,
        d.code,
        d.severity,
        json_escape(&d.message)
    )
}

/// Workflow command understood by GitHub Actions; shows up as an inline annotation.
pub fn github(file: &str, d: &Diag) -> String {
    let esc = |s: &str| s.replace('%', "%25").replace('\r', "%0D").replace('\n', "%0A");
    let prop = |s: &str| esc(s).replace(':', "%3A").replace(',', "%2C");
    format!("::{} file={},line={},title={}::{}", d.severity, prop(file), d.line, d.code, esc(&d.message))
}

pub fn summary_line(file: &str, s: &Summary, errors: usize, warnings: usize) -> String {
    let root = s.root.as_deref().map(|r| format!(", root `{r}`")).unwrap_or_default();
    format!(
        "{file}: {} links, {} joints ({} DOF), {:.3} kg{root}; {errors} error(s), {warnings} warning(s)",
        s.links, s.joints, s.dof, s.mass
    )
}
