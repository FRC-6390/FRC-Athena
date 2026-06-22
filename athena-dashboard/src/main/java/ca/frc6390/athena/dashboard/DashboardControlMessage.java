package ca.frc6390.athena.dashboard;

import java.util.Map;

/**
 * Control message sent from a dashboard to robot code.
 *
 * @param name control name
 * @param fields message fields
 */
public record DashboardControlMessage(String name, Map<String, String> fields) {
    public DashboardControlMessage {
        name = name == null || name.isBlank() ? "control" : name;
        fields = fields == null ? Map.of() : Map.copyOf(fields);
    }

    /**
     * Creates a control message with no fields.
     *
     * @param name control name
     * @return control message
     */
    public static DashboardControlMessage of(String name) {
        return new DashboardControlMessage(name, Map.of());
    }

    /**
     * Creates a control message with one field.
     *
     * @param name control name
     * @param key field key
     * @param value field value
     * @return control message
     */
    public static DashboardControlMessage of(String name, String key, String value) {
        return new DashboardControlMessage(name, Map.of(key, value));
    }
}
