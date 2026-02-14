package ca.frc6390.athena.core.diagnostics;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

/**
 * Generic subsystem diagnostics channel with structured events + summary fields.
 */
public final class DiagnosticsChannel {
    public record Event(
            long sequence,
            double timestampSeconds,
            String systemKey,
            String level,
            String category,
            String message,
            String line) {
    }

    private final String name;
    private final BoundedEventLog<Event> log;
    private final ConcurrentMap<String, Object> summaryFields = new ConcurrentHashMap<>();

    public DiagnosticsChannel(String name, int capacity) {
        String resolvedName = name != null ? name.trim() : "";
        if (resolvedName.isEmpty()) {
            throw new IllegalArgumentException("name must not be blank");
        }
        this.name = resolvedName;
        this.log = new BoundedEventLog<>(Math.max(1, capacity));
    }

    public String name() {
        return name;
    }

    public int capacity() {
        return log.capacity();
    }

    public DiagnosticsChannel field(String key, Object value) {
        String resolvedKey = key != null ? key.trim() : "";
        if (resolvedKey.isEmpty()) {
            throw new IllegalArgumentException("field key must not be blank");
        }
        if (value == null) {
            summaryFields.remove(resolvedKey);
        } else {
            summaryFields.put(resolvedKey, value);
        }
        return this;
    }

    public DiagnosticsChannel clearFields() {
        summaryFields.clear();
        return this;
    }

    public DiagnosticsChannel log(String level, String category, String message) {
        String text = message != null ? message.trim() : "";
        if (text.isEmpty()) {
            return this;
        }
        String resolvedLevel = level != null && !level.isBlank() ? level : "INFO";
        String resolvedCategory = category != null && !category.isBlank() ? category : "general";
        String displayName = name.lastIndexOf('/') >= 0
                ? name.substring(name.lastIndexOf('/') + 1)
                : name;
        String line = "[" + displayName + "] "
                + resolvedLevel.toLowerCase(java.util.Locale.ROOT)
                + " " + resolvedCategory + ": " + text;
        log.append((sequence, timestampSeconds) ->
                new Event(sequence, timestampSeconds, name, resolvedLevel, resolvedCategory, text, line));
        return this;
    }

    public DiagnosticsChannel info(String category, String message) {
        return log("INFO", category, message);
    }

    public DiagnosticsChannel warn(String category, String message) {
        return log("WARN", category, message);
    }

    public DiagnosticsChannel error(String category, String message) {
        return log("ERROR", category, message);
    }

    public List<Event> events(int limit) {
        return log.snapshot(limit);
    }

    public int eventCount() {
        return log.count();
    }

    public DiagnosticsChannel clear() {
        log.clear();
        return this;
    }

    public Map<String, Object> summary() {
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("name", name);
        out.put("capacity", capacity());
        out.put("eventCount", eventCount());
        if (!summaryFields.isEmpty()) {
            out.put("fields", Map.copyOf(summaryFields));
        }
        return out;
    }

    public Map<String, Object> snapshot(int limit) {
        Map<String, Object> out = new LinkedHashMap<>(summary());
        out.put("events", events(limit));
        return out;
    }
}
