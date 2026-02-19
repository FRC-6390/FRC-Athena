package ca.frc6390.athena.logging;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Objects;

import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public final class DataLogTelemetrySink implements TelemetrySink {
    private static final String DEFAULT_REAL_DATALOG_DIR = "/home/lvuser/athena/logs";
    private static final String DEFAULT_SIM_DATALOG_DIR = "athena/logs";

    private final DataLog log;
    private final String prefix;

    public DataLogTelemetrySink(String prefix, String logDirectory, int retentionCount) {
        String resolvedDirectory = resolveLogDirectory(logDirectory);
        ensureLogDirectoryExists(resolvedDirectory);
        pruneOldLogs(resolvedDirectory, retentionCount);
        DataLogManager.start(resolvedDirectory);
        this.log = DataLogManager.getLog();
        this.prefix = prefix == null || prefix.isBlank() ? "" : prefix.trim();
    }

    @Override
    public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
        String fullKey = prefix.isEmpty() ? key : prefix + "/" + key;
        return switch (type) {
            case DOUBLE -> createDoubleOutput(new DoubleLogEntry(log, fullKey), initialValue);
            case BOOLEAN -> createBooleanOutput(new BooleanLogEntry(log, fullKey), initialValue);
            case INTEGER -> createIntegerOutput(new IntegerLogEntry(log, fullKey), initialValue);
            case STRING -> createStringOutput(new StringLogEntry(log, fullKey), initialValue);
            case DOUBLE_ARRAY -> createDoubleArrayOutput(new DoubleArrayLogEntry(log, fullKey), initialValue);
            case BOOLEAN_ARRAY -> createBooleanArrayOutput(new BooleanArrayLogEntry(log, fullKey), initialValue);
            case INTEGER_ARRAY -> createIntegerArrayOutput(new IntegerArrayLogEntry(log, fullKey), initialValue);
            case STRING_ARRAY -> createStringArrayOutput(new StringArrayLogEntry(log, fullKey), initialValue);
        };
    }

    private static TelemetryOutput createDoubleOutput(DoubleLogEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final double[] last = new double[1];
        if (initialValue instanceof Number number) {
            double value = number.doubleValue();
            entry.append(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            double current = ((Number) value).doubleValue();
            if (hasLast[0] && Double.compare(last[0], current) == 0) {
                return;
            }
            entry.append(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createBooleanOutput(BooleanLogEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final boolean[] last = new boolean[1];
        if (initialValue instanceof Boolean value) {
            entry.append(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            boolean current = (Boolean) value;
            if (hasLast[0] && last[0] == current) {
                return;
            }
            entry.append(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createIntegerOutput(IntegerLogEntry entry, Object initialValue) {
        final boolean[] hasLast = new boolean[1];
        final long[] last = new long[1];
        if (initialValue instanceof Number number) {
            long value = number.longValue();
            entry.append(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            long current = ((Number) value).longValue();
            if (hasLast[0] && last[0] == current) {
                return;
            }
            entry.append(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createStringOutput(StringLogEntry entry, Object initialValue) {
        final String[] last = new String[1];
        final boolean[] hasLast = new boolean[1];
        if (initialValue instanceof String value) {
            entry.append(value);
            hasLast[0] = true;
            last[0] = value;
        }
        return value -> {
            String current = (String) value;
            if (hasLast[0] && Objects.equals(last[0], current)) {
                return;
            }
            entry.append(current);
            hasLast[0] = true;
            last[0] = current;
        };
    }

    private static TelemetryOutput createDoubleArrayOutput(DoubleArrayLogEntry entry, Object initialValue) {
        final double[][] last = new double[1][];
        if (initialValue instanceof double[] values) {
            entry.append(values);
            last[0] = copyDoubleArray(values, null);
        }
        return value -> {
            double[] current = (double[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.append(current);
            last[0] = copyDoubleArray(current, last[0]);
        };
    }

    private static TelemetryOutput createBooleanArrayOutput(BooleanArrayLogEntry entry, Object initialValue) {
        final boolean[][] last = new boolean[1][];
        if (initialValue instanceof boolean[] values) {
            entry.append(values);
            last[0] = copyBooleanArray(values, null);
        }
        return value -> {
            boolean[] current = (boolean[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.append(current);
            last[0] = copyBooleanArray(current, last[0]);
        };
    }

    private static TelemetryOutput createIntegerArrayOutput(IntegerArrayLogEntry entry, Object initialValue) {
        final long[][] scratch = new long[1][];
        final long[][] last = new long[1][];
        if (initialValue != null) {
            long[] current = normalizeIntegerArray(initialValue, scratch);
            entry.append(current);
            last[0] = copyLongArray(current, null);
        }
        return value -> {
            long[] current = normalizeIntegerArray(value, scratch);
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.append(current);
            last[0] = copyLongArray(current, last[0]);
        };
    }

    private static TelemetryOutput createStringArrayOutput(StringArrayLogEntry entry, Object initialValue) {
        final String[][] last = new String[1][];
        if (initialValue instanceof String[] values) {
            entry.append(values);
            last[0] = copyStringArray(values, null);
        }
        return value -> {
            String[] current = (String[]) value;
            if (last[0] != null && Arrays.equals(last[0], current)) {
                return;
            }
            entry.append(current);
            last[0] = copyStringArray(current, last[0]);
        };
    }

    private static long[] normalizeIntegerArray(Object value, long[][] scratchHolder) {
        if (value instanceof long[] values) {
            return values;
        }
        if (value instanceof int[] values) {
            scratchHolder[0] = toLongArray(values, scratchHolder[0]);
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

    private static String resolveLogDirectory(String configuredDirectory) {
        if (configuredDirectory != null && !configuredDirectory.isBlank()) {
            return configuredDirectory.trim();
        }
        if (RobotBase.isReal()) {
            return DEFAULT_REAL_DATALOG_DIR;
        }
        return new File(Filesystem.getOperatingDirectory(), DEFAULT_SIM_DATALOG_DIR).getAbsolutePath();
    }

    private static void ensureLogDirectoryExists(String logDirectory) {
        File dir = new File(logDirectory);
        if (dir.isDirectory()) {
            return;
        }
        if (dir.exists() && !dir.isDirectory()) {
            DriverStation.reportWarning(
                    "Telemetry log path exists but is not a directory: " + logDirectory,
                    false);
            return;
        }
        if (!dir.mkdirs()) {
            DriverStation.reportWarning(
                    "Failed to create telemetry log directory: " + logDirectory,
                    false);
        }
    }

    private static void pruneOldLogs(String logDirectory, int retentionCount) {
        int keepCount = Math.max(1, retentionCount);
        File dir = new File(logDirectory);
        if (!dir.isDirectory()) {
            return;
        }

        File[] logFiles = dir.listFiles((ignored, name) -> name != null && name.endsWith(".wpilog"));
        if (logFiles == null || logFiles.length <= keepCount) {
            return;
        }

        Arrays.sort(logFiles,
                Comparator.comparingLong(File::lastModified)
                        .reversed()
                        .thenComparing(File::getName));

        for (int i = keepCount; i < logFiles.length; i++) {
            File file = logFiles[i];
            if (!file.delete()) {
                DriverStation.reportWarning(
                        "Failed to delete old telemetry log: " + file.getAbsolutePath(),
                        false);
            }
        }
    }
}
