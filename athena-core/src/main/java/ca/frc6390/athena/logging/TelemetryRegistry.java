package ca.frc6390.athena.logging;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.core.RobotTime;
import edu.wpi.first.wpilibj.DriverStation;

public final class TelemetryRegistry {
    private static final String DEFAULT_PREFIX = "Athena";
    private static final int DEFAULT_PERIOD_MS = 100;
    private static final int HIGH_BANDWIDTH_MIN_PERIOD_MS = 200;
    private static final String DEFAULT_DATALOG_DIRECTORY = "";
    private static final int DEFAULT_DATALOG_RETENTION_COUNT = 5;
    private volatile boolean enabled = true;
    private final TelemetrySink diskSink;
    private final TelemetrySink networkTablesSink;
    private final List<Entry> entries;
    private final Map<String, Entry> entriesByKey;
    private final int defaultPeriodMs;

    private static final class Entry {
        private final String key;
        private final TelemetryValueType type;
        private final TelemetryOutput diskOutput;
        private final TelemetryOutput networkTablesOutput;
        private final ValueSupplier supplier;
        private final int periodMs;
        private final double epsilon;
        private long lastPublishMs;
        private Object lastValue;

        private Entry(String key,
                      TelemetryValueType type,
                      TelemetryOutput diskOutput,
                      TelemetryOutput networkTablesOutput,
                      ValueSupplier supplier,
                      int periodMs,
                      double epsilon,
                      Object initialValue) {
            this.key = key;
            this.type = type;
            this.diskOutput = diskOutput;
            this.networkTablesOutput = networkTablesOutput;
            this.supplier = supplier;
            this.periodMs = periodMs;
            this.epsilon = epsilon;
            this.lastPublishMs = 0L;
            this.lastValue = snapshotValue(type, initialValue, null);
        }
    }

    @FunctionalInterface
    private interface ValueSupplier {
        Object get() throws Exception;
    }

    public TelemetryRegistry(TelemetrySink diskSink,
                             TelemetrySink networkTablesSink,
                             int defaultPeriodMs) {
        this.diskSink = diskSink;
        this.networkTablesSink = networkTablesSink;
        this.entries = new ArrayList<>();
        this.entriesByKey = new HashMap<>();
        this.defaultPeriodMs = defaultPeriodMs > 0 ? defaultPeriodMs : DEFAULT_PERIOD_MS;
    }

    public static TelemetryRegistry create(TelemetryConfig config) {
        TelemetryConfig resolved = config != null ? config : TelemetryConfig.defaults();
        TelemetrySink disk = resolved.isDiskEnabled()
                ? new DataLogTelemetrySink(
                        resolved.diskPrefix(),
                        resolved.dataLogDirectory(),
                        resolved.dataLogRetentionCount())
                : null;
        TelemetrySink networkTables = resolved.isNetworkTablesEnabled()
                ? new NetworkTableTelemetrySink(resolved.networkTablePrefix())
                : null;
        return new TelemetryRegistry(disk, networkTables, resolved.defaultPeriodMs());
    }

    public void register(Object target) {
        Objects.requireNonNull(target, "target");
        Class<?> type = target.getClass();

        for (Field field : type.getDeclaredFields()) {
            Telemetry telemetry = field.getAnnotation(Telemetry.class);
            if (telemetry == null) {
                continue;
            }
            registerField(target, field, telemetry);
        }

        for (Method method : type.getDeclaredMethods()) {
            Telemetry telemetry = method.getAnnotation(Telemetry.class);
            if (telemetry == null) {
                continue;
            }
            registerMethod(target, method, telemetry);
        }
    }

    public void registerAll(Object... targets) {
        for (Object target : targets) {
            register(target);
        }
    }

    public void tick() {
        double nowSeconds = RobotTime.nowSeconds();
        if (Double.isFinite(nowSeconds)) {
            tick((long) (nowSeconds * 1000.0));
            return;
        }
        tick(System.currentTimeMillis());
    }

    public void tick(long nowMs) {
        if (!enabled || entries.isEmpty()) {
            return;
        }
        for (Entry entry : entries) {
            if (nowMs - entry.lastPublishMs < entry.periodMs) {
                continue;
            }
            Object rawValue;
            try {
                rawValue = entry.supplier.get();
            } catch (Exception ex) {
                DriverStation.reportWarning("Telemetry read failed for " + entry.key + ": " + ex.getMessage(), false);
                continue;
            }
            Object value;
            try {
                value = normalizeValue(entry.type, rawValue);
            } catch (RuntimeException ex) {
                DriverStation.reportWarning("Telemetry normalize failed for " + entry.key + ": " + ex.getMessage(), false);
                continue;
            }
            if (value == null) {
                continue;
            }
            if (!shouldPublish(entry, value)) {
                entry.lastPublishMs = nowMs;
                continue;
            }
            if (entry.diskOutput != null) {
                entry.diskOutput.write(value);
            }
            if (entry.networkTablesOutput != null) {
                entry.networkTablesOutput.write(value);
            }
            entry.lastValue = snapshotValue(entry.type, value, entry.lastValue);
            entry.lastPublishMs = nowMs;
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public int entryCount() {
        return entries.size();
    }

    private void registerField(Object target, Field field, Telemetry telemetry) {
        field.setAccessible(true);
        TelemetryValueType valueType = resolveType(field.getType());
        if (valueType == null) {
            DriverStation.reportWarning("Unsupported telemetry field type on " + field.getName(), false);
            return;
        }
        String key = resolveKey(target.getClass(), field.getName(), telemetry);
        ValueSupplier supplier = () -> field.get(target);
        addEntry(key, valueType, telemetry, supplier);
    }

    private void registerMethod(Object target, Method method, Telemetry telemetry) {
        if (method.getParameterCount() != 0) {
            DriverStation.reportWarning("Telemetry method must have no parameters: " + method.getName(), false);
            return;
        }
        method.setAccessible(true);
        TelemetryValueType valueType = resolveType(method.getReturnType());
        if (valueType == null) {
            DriverStation.reportWarning("Unsupported telemetry method type on " + method.getName(), false);
            return;
        }
        String key = resolveKey(target.getClass(), method.getName(), telemetry);
        ValueSupplier supplier = () -> method.invoke(target);
        addEntry(key, valueType, telemetry, supplier);
    }

    private void addEntry(String key, TelemetryValueType valueType, Telemetry telemetry, ValueSupplier supplier) {
        Object initialValue;
        try {
            initialValue = normalizeValue(valueType, supplier.get());
        } catch (Exception ex) {
            initialValue = null;
        }
        TelemetryOutput diskOutput = shouldLogToDisk(telemetry)
                ? diskSink.create(key, valueType, initialValue)
                : null;
        TelemetryOutput networkTablesOutput = shouldLogToNetworkTables(telemetry)
                ? networkTablesSink.create(key, valueType, initialValue)
                : null;
        int periodMs = telemetry.periodMs() > 0 ? telemetry.periodMs() : defaultPeriodMs;
        if (isHighBandwidthType(valueType) && periodMs < HIGH_BANDWIDTH_MIN_PERIOD_MS) {
            periodMs = HIGH_BANDWIDTH_MIN_PERIOD_MS;
        }
        Entry updated = new Entry(
                key,
                valueType,
                diskOutput,
                networkTablesOutput,
                supplier,
                periodMs,
                telemetry.epsilon(),
                initialValue);
        Entry previous = entriesByKey.put(key, updated);
        if (previous == null) {
            entries.add(updated);
            return;
        }
        int index = entries.indexOf(previous);
        if (index >= 0) {
            entries.set(index, updated);
        } else {
            // Fallback for unexpected external mutation; keep map/list consistent.
            entries.add(updated);
        }
    }

    private boolean shouldLogToDisk(Telemetry telemetry) {
        if (diskSink == null) {
            return false;
        }
        return telemetry.destination() == TelemetryDestination.DISK
                || telemetry.destination() == TelemetryDestination.BOTH;
    }

    private boolean shouldLogToNetworkTables(Telemetry telemetry) {
        if (networkTablesSink == null) {
            return false;
        }
        // Treat NetworkTables as a dashboard sink for any telemetry intended for Shuffleboard/BOTH.
        return telemetry.destination() == TelemetryDestination.SHUFFLEBOARD
                || telemetry.destination() == TelemetryDestination.BOTH;
    }

    private static String resolveKey(Class<?> targetType, String name, Telemetry telemetry) {
        String key = telemetry.key();
        if (key == null || key.isBlank()) {
            return targetType.getSimpleName() + "/" + name;
        }
        return key;
    }

    private static boolean isHighBandwidthType(TelemetryValueType valueType) {
        return valueType == TelemetryValueType.DOUBLE_ARRAY
                || valueType == TelemetryValueType.BOOLEAN_ARRAY
                || valueType == TelemetryValueType.INTEGER_ARRAY
                || valueType == TelemetryValueType.STRING_ARRAY;
    }

    private static TelemetryValueType resolveType(Class<?> type) {
        if (type == double.class || type == Double.class || type == float.class || type == Float.class) {
            return TelemetryValueType.DOUBLE;
        }
        if (type == boolean.class || type == Boolean.class) {
            return TelemetryValueType.BOOLEAN;
        }
        if (type == int.class || type == Integer.class || type == long.class || type == Long.class
                || type == short.class || type == Short.class || type == byte.class || type == Byte.class) {
            return TelemetryValueType.INTEGER;
        }
        if (type == String.class) {
            return TelemetryValueType.STRING;
        }
        if (type.isEnum()) {
            return TelemetryValueType.STRING;
        }
        if (type.isArray()) {
            Class<?> component = type.getComponentType();
            if (component == double.class) {
                return TelemetryValueType.DOUBLE_ARRAY;
            }
            if (component == boolean.class) {
                return TelemetryValueType.BOOLEAN_ARRAY;
            }
            if (component == int.class || component == long.class) {
                return TelemetryValueType.INTEGER_ARRAY;
            }
            if (component == String.class) {
                return TelemetryValueType.STRING_ARRAY;
            }
        }
        return null;
    }

    private static Object normalizeValue(TelemetryValueType type, Object value) {
        if (value == null) {
            return null;
        }
        return switch (type) {
            case DOUBLE -> ((Number) value).doubleValue();
            case BOOLEAN -> (Boolean) value;
            case INTEGER -> ((Number) value).longValue();
            case STRING -> value instanceof Enum ? ((Enum<?>) value).name() : value.toString();
            case DOUBLE_ARRAY -> (double[]) value;
            case BOOLEAN_ARRAY -> (boolean[]) value;
            case INTEGER_ARRAY -> value;
            case STRING_ARRAY -> (String[]) value;
        };
    }

    private static boolean shouldPublish(Entry entry, Object value) {
        if (entry.lastValue == null) {
            return true;
        }
        return switch (entry.type) {
            case DOUBLE -> {
                double current = (double) value;
                double previous = (double) entry.lastValue;
                if (entry.epsilon > 0.0) {
                    yield Math.abs(current - previous) > entry.epsilon;
                }
                yield Double.compare(current, previous) != 0;
            }
            case BOOLEAN -> (boolean) value != (boolean) entry.lastValue;
            case INTEGER -> ((Number) value).longValue() != ((Number) entry.lastValue).longValue();
            case STRING -> !Objects.equals(value, entry.lastValue);
            case DOUBLE_ARRAY -> !Arrays.equals((double[]) value, (double[]) entry.lastValue);
            case BOOLEAN_ARRAY -> !Arrays.equals((boolean[]) value, (boolean[]) entry.lastValue);
            case INTEGER_ARRAY -> !integerArrayEquals(value, entry.lastValue);
            case STRING_ARRAY -> !Arrays.equals((String[]) value, (String[]) entry.lastValue);
        };
    }

    private static Object snapshotValue(TelemetryValueType type, Object value, Object previous) {
        if (value == null) {
            return null;
        }
        return switch (type) {
            case DOUBLE_ARRAY -> copyDoubleArray((double[]) value, previous);
            case BOOLEAN_ARRAY -> copyBooleanArray((boolean[]) value, previous);
            case INTEGER_ARRAY -> copyIntegerArray(value, previous);
            case STRING_ARRAY -> copyStringArray((String[]) value, previous);
            default -> value;
        };
    }

    private static double[] copyDoubleArray(double[] source, Object previous) {
        if (source == null) {
            return null;
        }
        double[] out = previous instanceof double[] existing && existing.length == source.length
                ? existing
                : new double[source.length];
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static boolean[] copyBooleanArray(boolean[] source, Object previous) {
        if (source == null) {
            return null;
        }
        boolean[] out = previous instanceof boolean[] existing && existing.length == source.length
                ? existing
                : new boolean[source.length];
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static Object copyIntegerArray(Object source, Object previous) {
        if (source instanceof long[] sourceLong) {
            long[] out = previous instanceof long[] existing && existing.length == sourceLong.length
                    ? existing
                    : new long[sourceLong.length];
            System.arraycopy(sourceLong, 0, out, 0, sourceLong.length);
            return out;
        }
        if (source instanceof int[] sourceInt) {
            int[] out = previous instanceof int[] existing && existing.length == sourceInt.length
                    ? existing
                    : new int[sourceInt.length];
            System.arraycopy(sourceInt, 0, out, 0, sourceInt.length);
            return out;
        }
        return null;
    }

    private static String[] copyStringArray(String[] source, Object previous) {
        if (source == null) {
            return null;
        }
        String[] out = previous instanceof String[] existing && existing.length == source.length
                ? existing
                : new String[source.length];
        System.arraycopy(source, 0, out, 0, source.length);
        return out;
    }

    private static boolean integerArrayEquals(Object value, Object lastValue) {
        if (value instanceof long[] && lastValue instanceof long[]) {
            return Arrays.equals((long[]) value, (long[]) lastValue);
        }
        if (value instanceof int[] && lastValue instanceof int[]) {
            return Arrays.equals((int[]) value, (int[]) lastValue);
        }
        if (value instanceof long[] && lastValue instanceof int[]) {
            long[] left = (long[]) value;
            int[] right = (int[]) lastValue;
            if (left.length != right.length) {
                return false;
            }
            for (int i = 0; i < left.length; i++) {
                if (left[i] != right[i]) {
                    return false;
                }
            }
            return true;
        }
        if (value instanceof int[] && lastValue instanceof long[]) {
            int[] left = (int[]) value;
            long[] right = (long[]) lastValue;
            if (left.length != right.length) {
                return false;
            }
            for (int i = 0; i < left.length; i++) {
                if (left[i] != right[i]) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    static long[] toLongArray(int[] values) {
        if (values == null) {
            return new long[0];
        }
        long[] out = new long[values.length];
        for (int i = 0; i < values.length; i++) {
            out[i] = values[i];
        }
        return out;
    }

    public record TelemetryConfig(
            int defaultPeriodMs,
            boolean diskEnabled,
            boolean networkTablesEnabled,
            String diskPrefix,
            String networkTablePrefix,
            String dataLogDirectory,
            int dataLogRetentionCount) {

        public static TelemetryConfig defaults() {
            return new TelemetryConfig(
                    DEFAULT_PERIOD_MS,
                    true,
                    true,
                    DEFAULT_PREFIX,
                    "Telemetry",
                    DEFAULT_DATALOG_DIRECTORY,
                    DEFAULT_DATALOG_RETENTION_COUNT);
        }

        public TelemetryConfig defaultPeriodMs(int defaultPeriodMs) {
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    networkTablesEnabled,
                    diskPrefix,
                    networkTablePrefix,
                    dataLogDirectory,
                    dataLogRetentionCount);
        }

        public TelemetryConfig diskEnabled(boolean enabled) {
            return new TelemetryConfig(
                    defaultPeriodMs,
                    enabled,
                    networkTablesEnabled,
                    diskPrefix,
                    networkTablePrefix,
                    dataLogDirectory,
                    dataLogRetentionCount);
        }

        public TelemetryConfig networkTablesEnabled(boolean enabled) {
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    enabled,
                    diskPrefix,
                    networkTablePrefix,
                    dataLogDirectory,
                    dataLogRetentionCount);
        }

        public TelemetryConfig diskPrefix(String prefix) {
            String resolved = prefix != null && !prefix.isBlank() ? prefix : DEFAULT_PREFIX;
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    networkTablesEnabled,
                    resolved,
                    networkTablePrefix,
                    dataLogDirectory,
                    dataLogRetentionCount);
        }

        public TelemetryConfig networkTablePrefix(String prefix) {
            String resolved = prefix != null && !prefix.isBlank() ? prefix : "Telemetry";
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    networkTablesEnabled,
                    diskPrefix,
                    resolved,
                    dataLogDirectory,
                    dataLogRetentionCount);
        }

        public TelemetryConfig dataLogDirectory(String directory) {
            String resolved = directory != null ? directory.trim() : DEFAULT_DATALOG_DIRECTORY;
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    networkTablesEnabled,
                    diskPrefix,
                    networkTablePrefix,
                    resolved,
                    dataLogRetentionCount);
        }

        public TelemetryConfig dataLogRetentionCount(int count) {
            int resolved = count > 0 ? count : DEFAULT_DATALOG_RETENTION_COUNT;
            return new TelemetryConfig(
                    defaultPeriodMs,
                    diskEnabled,
                    networkTablesEnabled,
                    diskPrefix,
                    networkTablePrefix,
                    dataLogDirectory,
                    resolved);
        }

        public boolean isDiskEnabled() {
            return diskEnabled;
        }

        public boolean isNetworkTablesEnabled() {
            return networkTablesEnabled;
        }
    }
}
