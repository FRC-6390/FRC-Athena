package ca.frc6390.athena.logging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class NetworkTableTelemetrySink implements TelemetrySink {
    private final NetworkTable table;
    private final String prefix;

    public NetworkTableTelemetrySink(String prefix) {
        this(NetworkTableInstance.getDefault().getTable("Athena"), prefix);
    }

    public NetworkTableTelemetrySink(NetworkTable table, String prefix) {
        this.table = table;
        this.prefix = prefix == null ? "" : prefix.trim();
    }

    @Override
    public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
        String fullKey = prefix.isEmpty() ? key : prefix + "/" + key;
        NetworkTableEntry entry = table.getEntry(fullKey);
        if (initialValue != null) {
            setEntryValue(entry, type, initialValue);
        }
        return value -> setEntryValue(entry, type, value);
    }

    private static void setEntryValue(NetworkTableEntry entry, TelemetryValueType type, Object value) {
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
