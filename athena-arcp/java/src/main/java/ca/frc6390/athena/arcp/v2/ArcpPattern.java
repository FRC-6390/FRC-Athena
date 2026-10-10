package ca.frc6390.athena.arcp.v2;

/**
 * Mechanism-tree glob patterns for subscriptions (E2). Same rules as the Rust
 * {@code arcp_core::v2::pattern} module: {@code *} and {@code ?} stay inside one segment,
 * {@code **} as a whole segment matches zero or more segments.
 */
public final class ArcpPattern {

    private ArcpPattern() {
    }

    /** @return null when valid, otherwise the reason it is invalid */
    public static String validationError(String pattern) {
        if (pattern == null || pattern.isEmpty()) {
            return "pattern is empty";
        }
        for (String segment : pattern.split("/", -1)) {
            if (segment.isEmpty()) {
                return "pattern has an empty segment";
            }
        }
        return null;
    }

    public static boolean matches(String pattern, String path) {
        return matchSegments(pattern.split("/", -1), 0, path.split("/", -1), 0);
    }

    private static boolean matchSegments(String[] pat, int pi, String[] segs, int si) {
        if (pi == pat.length) {
            return si == segs.length;
        }
        if (pat[pi].equals("**")) {
            for (int skip = si; skip <= segs.length; skip++) {
                if (matchSegments(pat, pi + 1, segs, skip)) {
                    return true;
                }
            }
            return false;
        }
        return si < segs.length && matchSegment(pat[pi], segs[si]) && matchSegments(pat, pi + 1, segs, si + 1);
    }

    /** Glob match inside one segment supporting {@code *} and {@code ?}. */
    static boolean matchSegment(String pat, String text) {
        int p = 0;
        int t = 0;
        int starP = -1;
        int starT = 0;
        while (t < text.length()) {
            if (p < pat.length() && (pat.charAt(p) == '?' || pat.charAt(p) == text.charAt(t))) {
                p++;
                t++;
            } else if (p < pat.length() && pat.charAt(p) == '*') {
                starP = p;
                starT = t;
                p++;
            } else if (starP >= 0) {
                p = starP + 1;
                starT++;
                t = starT;
            } else {
                return false;
            }
        }
        while (p < pat.length() && pat.charAt(p) == '*') {
            p++;
        }
        return p == pat.length();
    }
}
