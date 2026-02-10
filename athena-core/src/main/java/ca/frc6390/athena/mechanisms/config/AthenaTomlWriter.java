package ca.frc6390.athena.mechanisms.config;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * Minimal TOML writer for Athena config export.
 *
 * <p>This is intentionally "data-only": objects, arrays, and scalar values. It does not attempt to
 * preserve comments or formatting from source files.
 */
final class AthenaTomlWriter {
    private AthenaTomlWriter() {}

    static String write(JsonNode root) {
        if (root == null || root.isNull()) {
            return "";
        }
        if (!root.isObject()) {
            throw new IllegalArgumentException("TOML root must be an object");
        }
        StringBuilder out = new StringBuilder();
        writeTable(out, "", (ObjectNode) root, true);
        return out.toString();
    }

    private static void writeTable(StringBuilder out, String path, ObjectNode obj, boolean isRoot) {
        if (!isRoot) {
            out.append('[').append(path).append(']').append('\n');
        }

        List<Map.Entry<String, JsonNode>> scalars = new ArrayList<>();
        List<Map.Entry<String, ObjectNode>> tables = new ArrayList<>();
        List<Map.Entry<String, ArrayNode>> arrays = new ArrayList<>();

        Iterator<Map.Entry<String, JsonNode>> it = obj.fields();
        while (it.hasNext()) {
            Map.Entry<String, JsonNode> e = it.next();
            String key = e.getKey();
            JsonNode v = e.getValue();
            if (v == null || v.isNull()) {
                continue;
            }
            if (v.isObject()) {
                tables.add(Map.entry(key, (ObjectNode) v));
            } else if (v.isArray()) {
                arrays.add(Map.entry(key, (ArrayNode) v));
            } else {
                scalars.add(e);
            }
        }

        scalars.sort(Comparator.comparing(Map.Entry::getKey));
        tables.sort(Comparator.comparing(Map.Entry::getKey));
        arrays.sort(Comparator.comparing(Map.Entry::getKey));

        for (Map.Entry<String, JsonNode> e : scalars) {
            out.append(formatKey(e.getKey()))
                    .append(" = ")
                    .append(formatValue(e.getValue()))
                    .append('\n');
        }

        for (Map.Entry<String, ArrayNode> e : arrays) {
            String key = e.getKey();
            ArrayNode arr = e.getValue();
            if (arr == null || arr.isEmpty()) {
                continue;
            }
            if (isArrayOfObjects(arr)) {
                for (JsonNode child : arr) {
                    if (child == null || !child.isObject()) {
                        continue;
                    }
                    out.append('\n');
                    out.append("[[").append(pathJoin(path, formatBareKey(key))).append("]]").append('\n');
                    writeInlineTable(out, (ObjectNode) child, pathJoin(path, formatBareKey(key)));
                }
            } else {
                out.append(formatKey(key))
                        .append(" = ")
                        .append(formatArray(arr))
                        .append('\n');
            }
        }

        for (Map.Entry<String, ObjectNode> e : tables) {
            String childPath = pathJoin(path, formatBareKey(e.getKey()));
            out.append('\n');
            writeTable(out, childPath, e.getValue(), false);
        }
    }

    // Writes scalar fields and nested tables for a table that already has its header printed.
    private static void writeInlineTable(StringBuilder out, ObjectNode obj, String path) {
        // Scalars first
        List<Map.Entry<String, JsonNode>> scalars = new ArrayList<>();
        List<Map.Entry<String, ObjectNode>> tables = new ArrayList<>();
        List<Map.Entry<String, ArrayNode>> arrays = new ArrayList<>();
        Iterator<Map.Entry<String, JsonNode>> it = obj.fields();
        while (it.hasNext()) {
            Map.Entry<String, JsonNode> e = it.next();
            String key = e.getKey();
            JsonNode v = e.getValue();
            if (v == null || v.isNull()) {
                continue;
            }
            if (v.isObject()) {
                tables.add(Map.entry(key, (ObjectNode) v));
            } else if (v.isArray()) {
                arrays.add(Map.entry(key, (ArrayNode) v));
            } else {
                scalars.add(e);
            }
        }
        scalars.sort(Comparator.comparing(Map.Entry::getKey));
        tables.sort(Comparator.comparing(Map.Entry::getKey));
        arrays.sort(Comparator.comparing(Map.Entry::getKey));

        for (Map.Entry<String, JsonNode> e : scalars) {
            out.append(formatKey(e.getKey()))
                    .append(" = ")
                    .append(formatValue(e.getValue()))
                    .append('\n');
        }

        for (Map.Entry<String, ArrayNode> e : arrays) {
            String key = e.getKey();
            ArrayNode arr = e.getValue();
            if (arr == null || arr.isEmpty()) {
                continue;
            }
            if (isArrayOfObjects(arr)) {
                for (JsonNode child : arr) {
                    if (child == null || !child.isObject()) {
                        continue;
                    }
                    out.append('\n');
                    out.append("[[").append(pathJoin(path, formatBareKey(key))).append("]]").append('\n');
                    writeInlineTable(out, (ObjectNode) child, pathJoin(path, formatBareKey(key)));
                }
            } else {
                out.append(formatKey(key))
                        .append(" = ")
                        .append(formatArray(arr))
                        .append('\n');
            }
        }

        for (Map.Entry<String, ObjectNode> e : tables) {
            String childPath = pathJoin(path, formatBareKey(e.getKey()));
            out.append('\n');
            writeTable(out, childPath, e.getValue(), false);
        }
    }

    private static boolean isArrayOfObjects(ArrayNode arr) {
        for (JsonNode n : arr) {
            if (n == null || n.isNull()) {
                continue;
            }
            return n.isObject();
        }
        return false;
    }

    private static String pathJoin(String base, String seg) {
        if (base == null || base.isEmpty()) {
            return seg;
        }
        if (seg == null || seg.isEmpty()) {
            return base;
        }
        return base + "." + seg;
    }

    private static String formatKey(String key) {
        return formatBareKey(key);
    }

    private static String formatBareKey(String key) {
        if (key == null) {
            return "\"\"";
        }
        String k = key.trim();
        if (k.isEmpty()) {
            return "\"\"";
        }
        // Accept common bare keys. If it contains anything else, quote it.
        if (k.matches("[A-Za-z0-9_-]+")) {
            return k;
        }
        return quote(k);
    }

    private static String formatValue(JsonNode v) {
        if (v == null || v.isNull()) {
            return "''";
        }
        if (v.isTextual()) {
            return quote(v.asText());
        }
        if (v.isBoolean()) {
            return v.asBoolean() ? "true" : "false";
        }
        if (v.isIntegralNumber()) {
            return Long.toString(v.asLong());
        }
        if (v.isFloatingPointNumber()) {
            double d = v.asDouble();
            if (Double.isFinite(d)) {
                // Avoid scientific notation unless necessary.
                return stripTrailingZeros(Double.toString(d));
            }
            return "0.0";
        }
        if (v.isArray()) {
            return formatArray((ArrayNode) v);
        }
        // Objects should be handled by table writer; fallback to JSON-ish string.
        return quote(v.toString());
    }

    private static String formatArray(ArrayNode arr) {
        List<String> parts = new ArrayList<>();
        for (JsonNode child : arr) {
            if (child == null || child.isNull()) {
                continue;
            }
            if (child.isObject()) {
                // Array-of-tables are handled elsewhere.
                continue;
            }
            parts.add(formatValue(child));
        }
        return "[" + String.join(", ", parts) + "]";
    }

    private static String quote(String s) {
        if (s == null) {
            return "\"\"";
        }
        String escaped = s
                .replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r")
                .replace("\t", "\\t");
        return "\"" + escaped + "\"";
    }

    private static String stripTrailingZeros(String s) {
        if (s == null) {
            return "0.0";
        }
        // "1.23000" -> "1.23", "1.0" stays "1.0" (to keep float identity).
        if (!s.contains(".")) {
            return s;
        }
        int end = s.length();
        while (end > 0 && s.charAt(end - 1) == '0') {
            end--;
        }
        if (end > 0 && s.charAt(end - 1) == '.') {
            end++; // keep one trailing zero for "1.0"
        }
        return s.substring(0, Math.min(end, s.length()));
    }
}

