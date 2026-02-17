package ca.frc6390.athena.logging;

import java.util.Arrays;
import java.util.Objects;

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
        return switch (type) {
            case DOUBLE -> createDoubleOutput(entry, initialValue);
            case BOOLEAN -> createBooleanOutput(entry, initialValue);
            case INTEGER -> createIntegerOutput(entry, initialValue);
            case STRING -> createStringOutput(entry, initialValue);
            case DOUBLE_ARRAY -> createDoubleArrayOutput(entry, initialValue);
            case BOOLEAN_ARRAY -> createBooleanArrayOutput(entry, initialValue);
            case INTEGER_ARRAY -> createIntegerArrayOutput(entry, initialValue);
            case STRING_ARRAY -> createStringArrayOutput(entry, initialValue);
        };
    }

    private static TelemetryOutput createDoubleOutput(NetworkTableEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final double[] last = new double[1];
        if (initialValue instanceof Number number) {
            double value = number.doubleValue();
            entry.setDouble(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            double current = ((Number) value).doubleValue();
            if (hasLast[0] && Double.compare(last[0], current) == 0) {
                return;
            }
            entry.setDouble(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createBooleanOutput(NetworkTableEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final boolean[] last = new boolean[1];
        if (initialValue instanceof Boolean value) {
            entry.setBoolean(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            boolean current = (Boolean) value;
            if (hasLast[0] && last[0] == current) {
                return;
            }
            entry.setBoolean(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createIntegerOutput(NetworkTableEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final long[] last = new long[1];
        if (initialValue instanceof Number number) {
            long value = number.longValue();
            entry.setInteger(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            long current = ((Number) value).longValue();
            if (hasLast[0] && last[0] == current) {
                return;
            }
            entry.setInteger(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createStringOutput(NetworkTableEntry entry, Object initialValue) {
        final String[] last = new String[1];
        final boolean[] hasLast = new boolean[1];
        if (initialValue instanceof String value) {
            entry.setString(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            String current = (String) value;
            if (hasLast[0] && Objects.equals(last[0], current)) {
                return;
            }
            entry.setString(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createDoubleArrayOutput(NetworkTableEntry entry, Object initialValue) {
        final double[][] last = new double[1][];
        if (initialValue instanceof double[] value) {
            entry.setDoubleArray(value);
            last[0] = copyDoubleArray(value, null);
        }
        return value -> {
            double[] current = (double[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.setDoubleArray(current);
            last[0] = copyDoubleArray(current, last[0]);
        };
    }

    private static TelemetryOutput createBooleanArrayOutput(NetworkTableEntry entry, Object initialValue) {
        final boolean[][] last = new boolean[1][];
        if (initialValue instanceof boolean[] value) {
            entry.setBooleanArray(value);
            last[0] = copyBooleanArray(value, null);
        }
        return value -> {
            boolean[] current = (boolean[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.setBooleanArray(current);
            last[0] = copyBooleanArray(current, last[0]);
        };
    }

    private static TelemetryOutput createIntegerArrayOutput(NetworkTableEntry entry, Object initialValue) {
        final long[][] scratch = new long[1][];
        final long[][] last = new long[1][];
        if (initialValue != null) {
            long[] current = normalizeIntegerArray(initialValue, scratch);
            entry.setIntegerArray(current);
            last[0] = copyLongArray(current, null);
        }
        return value -> {
            long[] current = normalizeIntegerArray(value, scratch);
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.setIntegerArray(current);
            last[0] = copyLongArray(current, last[0]);
        };
    }

    private static TelemetryOutput createStringArrayOutput(NetworkTableEntry entry, Object initialValue) {
        final String[][] last = new String[1][];
        if (initialValue instanceof String[] value) {
            entry.setStringArray(value);
            last[0] = copyStringArray(value, null);
        }
        return value -> {
            String[] current = (String[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.setStringArray(current);
            last[0] = copyStringArray(current, last[0]);
        };
    }

    private static long[] normalizeIntegerArray(Object value, long[][] scratchHolder) {
        if (value instanceof long[] longValues) {
            return longValues;
        }
        if (value instanceof int[] intValues) {
            scratchHolder[0] = toLongArray(intValues, scratchHolder[0]);
            return scratchHolder[0];
        }
        return new long[0];
    }

    private static double[] copyDoubleArray(double[] source, double[] reuse) {
        if (source == null) {
            return null;
        }
        double[] out = reuse;
        if (out == null || out.length != source.length) {
            out = new double[source.length];
        }
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static boolean[] copyBooleanArray(boolean[] source, boolean[] reuse) {
        if (source == null) {
            return null;
        }
        boolean[] out = reuse;
        if (out == null || out.length != source.length) {
            out = new boolean[source.length];
        }
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static long[] copyLongArray(long[] source, long[] reuse) {
        if (source == null) {
            return null;
        }
        long[] out = reuse;
        if (out == null || out.length != source.length) {
            out = new long[source.length];
        }
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static String[] copyStringArray(String[] source, String[] reuse) {
        if (source == null) {
            return null;
        }
        String[] out = reuse;
        if (out == null || out.length != source.length) {
            out = new String[source.length];
        }
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static long[] toLongArray(int[] values, long[] reuse) {
        if (values == null) {
            return new long[0];
        }
        long[] out = reuse;
        if (out == null || out.length != values.length) {
            out = new long[values.length];
        }
        for (int i = 0; i < values.length; i++) {
            out[i] = values[i];
        }
        return out;
    }
}
