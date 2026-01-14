package ca.frc6390.athena.logging;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class ShuffleboardTelemetrySink implements TelemetrySink {
    private final ShuffleboardTab tab;
    private final Map<String, GenericEntry> entries;

    public ShuffleboardTelemetrySink(String tabName) {
        String resolved = tabName == null || tabName.isBlank() ? "Telemetry" : tabName;
        if (!resolved.equals("Athena") && !resolved.startsWith("Athena/")) {
            resolved = "Athena/" + resolved;
        }
        this.tab = Shuffleboard.getTab(resolved);
        this.entries = new HashMap<>();
    }

    @Override
    public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
        GenericEntry entry = entries.computeIfAbsent(key, name -> {
            Object defaultValue = initialValue != null ? initialValue : defaultValue(type);
            return tab.add(name, defaultValue).getEntry();
        });

        return value -> setEntryValue(entry, type, value);
    }

    private static Object defaultValue(TelemetryValueType type) {
        return switch (type) {
            case DOUBLE -> 0.0;
            case BOOLEAN -> false;
            case INTEGER -> 0L;
            case STRING -> "";
            case DOUBLE_ARRAY -> new double[0];
            case BOOLEAN_ARRAY -> new boolean[0];
            case INTEGER_ARRAY -> new long[0];
            case STRING_ARRAY -> new String[0];
        };
    }

    private static void setEntryValue(GenericEntry entry, TelemetryValueType type, Object value) {
        switch (type) {
            case DOUBLE -> entry.setDouble(((Number) value).doubleValue());
            case BOOLEAN -> entry.setBoolean((Boolean) value);
            case INTEGER -> entry.setInteger(((Number) value).longValue());
            case STRING -> entry.setString((String) value);
            case DOUBLE_ARRAY -> entry.setDoubleArray((double[]) value);
            case BOOLEAN_ARRAY -> entry.setBooleanArray((boolean[]) value);
            case INTEGER_ARRAY -> {
                if (value instanceof long[]) {
                    entry.setIntegerArray((long[]) value);
                } else {
                    entry.setIntegerArray(TelemetryRegistry.toLongArray((int[]) value));
                }
            }
            case STRING_ARRAY -> entry.setStringArray((String[]) value);
        }
    }
}
