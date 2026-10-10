//! Mechanism-tree glob patterns for subscriptions (E2).
//!
//! Paths are `/`-separated (`robot/turret/angle`). Pattern syntax:
//! - `*` matches any run of characters inside one segment.
//! - `?` matches one character inside one segment.
//! - `**` as a whole segment matches zero or more whole segments.
//!
//! `robot/turret/**` therefore matches `robot/turret`, `robot/turret/angle` and
//! `robot/turret/pid/kp`.

/// Returns an error message when `pattern` is empty or has empty segments.
pub fn validate(pattern: &str) -> Result<(), &'static str> {
    if pattern.is_empty() {
        return Err("pattern is empty");
    }
    if pattern.split('/').any(str::is_empty) {
        return Err("pattern has an empty segment");
    }
    Ok(())
}

/// True when `path` matches `pattern`.
pub fn matches(pattern: &str, path: &str) -> bool {
    let pat: Vec<&str> = pattern.split('/').collect();
    let segs: Vec<&str> = path.split('/').collect();
    match_segments(&pat, &segs)
}

fn match_segments(pat: &[&str], segs: &[&str]) -> bool {
    match pat.first() {
        None => segs.is_empty(),
        Some(&"**") => (0..=segs.len()).any(|skip| match_segments(&pat[1..], &segs[skip..])),
        Some(p) => match segs.first() {
            Some(s) => match_segment(p.as_bytes(), s.as_bytes()) && match_segments(&pat[1..], &segs[1..]),
            None => false,
        },
    }
}

/// Glob match inside one segment, supporting `*` and `?`.
fn match_segment(pat: &[u8], text: &[u8]) -> bool {
    let (mut p, mut t) = (0usize, 0usize);
    let (mut star_p, mut star_t): (Option<usize>, usize) = (None, 0);
    while t < text.len() {
        if p < pat.len() && (pat[p] == b'?' || pat[p] == text[t]) {
            p += 1;
            t += 1;
        } else if p < pat.len() && pat[p] == b'*' {
            star_p = Some(p);
            star_t = t;
            p += 1;
        } else if let Some(sp) = star_p {
            p = sp + 1;
            star_t += 1;
            t = star_t;
        } else {
            return false;
        }
    }
    while p < pat.len() && pat[p] == b'*' {
        p += 1;
    }
    p == pat.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn globstar_matches_subtree_and_root() {
        assert!(matches("robot/turret/**", "robot/turret"));
        assert!(matches("robot/turret/**", "robot/turret/angle"));
        assert!(matches("robot/turret/**", "robot/turret/pid/kp"));
        assert!(!matches("robot/turret/**", "robot/intake/angle"));
        assert!(!matches("robot/turret/**", "robot/turretx/angle"));
    }

    #[test]
    fn star_and_question_stay_inside_a_segment() {
        assert!(matches("robot/*/angle", "robot/turret/angle"));
        assert!(!matches("robot/*/angle", "robot/turret/pid/angle"));
        assert!(matches("robot/tur*/an?le", "robot/turret/angle"));
        assert!(matches("**/angle", "a/b/angle"));
        assert!(matches("**", "anything/at/all"));
        assert!(!matches("robot", "robot/angle"));
    }

    #[test]
    fn validation_rejects_empty_segments() {
        assert!(validate("").is_err());
        assert!(validate("robot//angle").is_err());
        assert!(validate("robot/**").is_ok());
    }
}
