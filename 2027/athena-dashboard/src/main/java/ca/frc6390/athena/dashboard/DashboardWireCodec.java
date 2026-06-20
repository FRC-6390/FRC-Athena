package ca.frc6390.athena.dashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Collectors;

import ca.frc6390.athena.runtime.diagnostics.DiagnosticEvent;
import ca.frc6390.athena.runtime.diagnostics.DiagnosticsSnapshot;
import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryValue;

/**
 * Dependency-free JSON wire shape for dashboard packets and controls.
 */
public final class DashboardWireCodec {
    private DashboardWireCodec() {
    }

    /**
     * Encodes a robot-to-dashboard packet as JSON.
     *
     * @param packet dashboard packet
     * @return JSON payload
     */
    public static String encodePacket(DashboardPacket packet) {
        StringBuilder json = new StringBuilder();
        json.append('{');
        field(json, "timestamp", packet.timestamp().toString());
        json.append(',');
        json.append("\"hasErrors\":").append(packet.hasErrors());
        json.append(',');
        json.append("\"telemetry\":[");
        json.append(packet.telemetry().values().entrySet().stream()
                .map(entry -> encodeTelemetry(entry.getKey(), entry.getValue()))
                .collect(Collectors.joining(",")));
        json.append("],");
        json.append("\"diagnostics\":[");
        json.append(packet.diagnostics().stream()
                .map(DashboardWireCodec::encodeDiagnostics)
                .collect(Collectors.joining(",")));
        json.append(']');
        json.append('}');
        return json.toString();
    }

    /**
     * Encodes a dashboard-to-robot control message as JSON.
     *
     * @param message control message
     * @return JSON payload
     */
    public static String encodeControl(DashboardControlMessage message) {
        StringBuilder json = new StringBuilder();
        json.append('{');
        field(json, "name", message.name());
        json.append(",\"fields\":{");
        json.append(message.fields().entrySet().stream()
                .map(entry -> quote(entry.getKey()) + ":" + quote(entry.getValue()))
                .collect(Collectors.joining(",")));
        json.append("}}");
        return json.toString();
    }

    /**
     * Decodes the control JSON shape produced by {@link #encodeControl}.
     *
     * @param json JSON payload
     * @return control message
     */
    public static DashboardControlMessage decodeControl(String json) {
        if (json == null || json.isBlank()) {
            throw new IllegalArgumentException("Control payload cannot be blank.");
        }
        String name = readStringField(json, "name");
        String fieldsObject = readObjectField(json, "fields");
        Map<String, String> fields = new LinkedHashMap<>();
        if (!fieldsObject.isBlank()) {
            for (String field : splitTopLevel(fieldsObject)) {
                int separator = field.indexOf(':');
                if (separator <= 0) {
                    throw new IllegalArgumentException("Invalid control field: " + field);
                }
                fields.put(unquote(field.substring(0, separator).trim()), unquote(field.substring(separator + 1).trim()));
            }
        }
        return new DashboardControlMessage(name, fields);
    }

    private static String encodeTelemetry(TelemetryKey key, TelemetryValue value) {
        StringBuilder json = new StringBuilder();
        json.append('{');
        field(json, "path", key.path());
        json.append(',');
        field(json, "type", key.type().name());
        json.append(',');
        json.append("\"value\":");
        if (value.value() instanceof Boolean || value.value() instanceof Number) {
            json.append(value.value());
        } else {
            json.append(quote(value.value().toString()));
        }
        json.append('}');
        return json.toString();
    }

    private static String encodeDiagnostics(DiagnosticsSnapshot snapshot) {
        StringBuilder json = new StringBuilder();
        json.append('{');
        field(json, "channel", snapshot.channel());
        json.append(",\"summary\":{");
        json.append(snapshot.summary().entrySet().stream()
                .map(entry -> quote(entry.getKey()) + ":" + quote(entry.getValue()))
                .collect(Collectors.joining(",")));
        json.append("},\"events\":[");
        json.append(snapshot.events().stream()
                .map(DashboardWireCodec::encodeEvent)
                .collect(Collectors.joining(",")));
        json.append("]}");
        return json.toString();
    }

    private static String encodeEvent(DiagnosticEvent event) {
        StringBuilder json = new StringBuilder();
        json.append('{');
        json.append("\"sequence\":").append(event.sequence()).append(',');
        field(json, "timestamp", event.timestamp().toString());
        json.append(',');
        field(json, "level", event.level().name());
        json.append(',');
        field(json, "message", event.message());
        json.append('}');
        return json.toString();
    }

    private static void field(StringBuilder json, String name, String value) {
        json.append(quote(name)).append(':').append(quote(value));
    }

    private static String quote(String value) {
        return "\"" + escape(value) + "\"";
    }

    private static String escape(String value) {
        StringBuilder escaped = new StringBuilder();
        for (int index = 0; index < value.length(); index++) {
            char c = value.charAt(index);
            if (c == '\\' || c == '"') {
                escaped.append('\\').append(c);
            } else if (c == '\n') {
                escaped.append("\\n");
            } else if (c == '\r') {
                escaped.append("\\r");
            } else if (c == '\t') {
                escaped.append("\\t");
            } else {
                escaped.append(c);
            }
        }
        return escaped.toString();
    }

    private static String readStringField(String json, String name) {
        String marker = quote(name) + ":";
        int start = json.indexOf(marker);
        if (start < 0) {
            throw new IllegalArgumentException("Missing string field " + name + ".");
        }
        return unquote(readJsonValue(json, start + marker.length()));
    }

    private static String readObjectField(String json, String name) {
        String marker = quote(name) + ":";
        int start = json.indexOf(marker);
        if (start < 0) {
            throw new IllegalArgumentException("Missing object field " + name + ".");
        }
        String value = readJsonValue(json, start + marker.length()).trim();
        if (value.length() < 2 || value.charAt(0) != '{' || value.charAt(value.length() - 1) != '}') {
            throw new IllegalArgumentException("Field " + name + " must be an object.");
        }
        return value.substring(1, value.length() - 1).trim();
    }

    private static String readJsonValue(String json, int startIndex) {
        int start = skipWhitespace(json, startIndex);
        if (json.charAt(start) == '"') {
            int end = start + 1;
            boolean escaped = false;
            while (end < json.length()) {
                char c = json.charAt(end);
                if (c == '"' && !escaped) {
                    return json.substring(start, end + 1);
                }
                escaped = c == '\\' && !escaped;
                if (c != '\\') {
                    escaped = false;
                }
                end++;
            }
            throw new IllegalArgumentException("Unterminated string value.");
        }
        if (json.charAt(start) == '{') {
            int depth = 0;
            boolean inString = false;
            boolean escaped = false;
            for (int index = start; index < json.length(); index++) {
                char c = json.charAt(index);
                if (c == '"' && !escaped) {
                    inString = !inString;
                }
                if (!inString) {
                    if (c == '{') {
                        depth++;
                    } else if (c == '}') {
                        depth--;
                        if (depth == 0) {
                            return json.substring(start, index + 1);
                        }
                    }
                }
                escaped = c == '\\' && !escaped;
                if (c != '\\') {
                    escaped = false;
                }
            }
        }
        throw new IllegalArgumentException("Unsupported JSON value.");
    }

    private static java.util.List<String> splitTopLevel(String fieldsObject) {
        java.util.List<String> fields = new java.util.ArrayList<>();
        int start = 0;
        boolean inString = false;
        boolean escaped = false;
        for (int index = 0; index < fieldsObject.length(); index++) {
            char c = fieldsObject.charAt(index);
            if (c == '"' && !escaped) {
                inString = !inString;
            } else if (c == ',' && !inString) {
                fields.add(fieldsObject.substring(start, index).trim());
                start = index + 1;
            }
            escaped = c == '\\' && !escaped;
            if (c != '\\') {
                escaped = false;
            }
        }
        fields.add(fieldsObject.substring(start).trim());
        return fields;
    }

    private static int skipWhitespace(String json, int index) {
        int cursor = index;
        while (cursor < json.length() && Character.isWhitespace(json.charAt(cursor))) {
            cursor++;
        }
        if (cursor >= json.length()) {
            throw new IllegalArgumentException("Missing JSON value.");
        }
        return cursor;
    }

    private static String unquote(String value) {
        String trimmed = value.trim();
        if (trimmed.length() < 2 || trimmed.charAt(0) != '"' || trimmed.charAt(trimmed.length() - 1) != '"') {
            throw new IllegalArgumentException("Expected quoted string: " + value);
        }
        String body = trimmed.substring(1, trimmed.length() - 1);
        StringBuilder unescaped = new StringBuilder();
        boolean escaped = false;
        for (int index = 0; index < body.length(); index++) {
            char c = body.charAt(index);
            if (escaped) {
                unescaped.append(switch (c) {
                    case 'n' -> '\n';
                    case 'r' -> '\r';
                    case 't' -> '\t';
                    default -> c;
                });
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else {
                unescaped.append(c);
            }
        }
        if (escaped) {
            throw new IllegalArgumentException("Dangling escape in string.");
        }
        return unescaped.toString();
    }
}
