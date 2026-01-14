package ca.frc6390.athena.logging;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.wpilibj.DriverStation;

public final class TelemetryRegistry {
    private static final String DEFAULT_PREFIX = "Athena";
    private final TelemetrySink diskSink;
    private final TelemetrySink shuffleboardSink;
    private final List<Entry> entries;

    private static final class Entry {
        private final String key;
        private final TelemetryValueType type;
        private final TelemetryOutput diskOutput;
        private final TelemetryOutput shuffleboardOutput;
        private final ValueSupplier supplier;
        private final int periodMs;
        private final double epsilon;
        private long lastPublishMs;
        private Object lastValue;

        private Entry(String key,
                      TelemetryValueType type,
                      TelemetryOutput diskOutput,
                      TelemetryOutput shuffleboardOutput,
                      ValueSupplier supplier,
                      int periodMs,
                      double epsilon,
                      Object initialValue) {
            this.key = key;
            this.type = type;
            this.diskOutput = diskOutput;
            this.shuffleboardOutput = shuffleboardOutput;
            this.supplier = supplier;
            this.periodMs = periodMs;
            this.epsilon = epsilon;
            this.lastPublishMs = 0L;
            this.lastValue = snapshotValue(type, initialValue);
        }
    }

    @FunctionalInterface
    private interface ValueSupplier {
        Object get() throws Exception;
    }

    public TelemetryRegistry(TelemetrySink diskSink, TelemetrySink shuffleboardSink) {
        this.diskSink = diskSink;
        this.shuffleboardSink = shuffleboardSink;
        this.entries = new ArrayList<>();
    }

    public static TelemetryRegistry createDefault(String shuffleboardTab) {
        TelemetrySink shuffleboardSink = new MirrorTelemetrySink(
                new ShuffleboardTelemetrySink(shuffleboardTab),
                new NetworkTableTelemetrySink("Telemetry"));
        return new TelemetryRegistry(new DataLogTelemetrySink(DEFAULT_PREFIX), shuffleboardSink);
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
        tick(System.currentTimeMillis());
    }

    public void tick(long nowMs) {
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
            if (entry.shuffleboardOutput != null) {
                entry.shuffleboardOutput.write(value);
            }
            entry.lastValue = snapshotValue(entry.type, value);
            entry.lastPublishMs = nowMs;
        }
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
        TelemetryOutput shuffleboardOutput = shouldLogToShuffleboard(telemetry)
                ? shuffleboardSink.create(key, valueType, initialValue)
                : null;
        entries.add(new Entry(key, valueType, diskOutput, shuffleboardOutput, supplier,
                Math.max(telemetry.periodMs(), 0), telemetry.epsilon(), initialValue));
    }

    private boolean shouldLogToDisk(Telemetry telemetry) {
        if (diskSink == null) {
            return false;
        }
        return telemetry.destination() == TelemetryDestination.DISK
                || telemetry.destination() == TelemetryDestination.BOTH;
    }

    private boolean shouldLogToShuffleboard(Telemetry telemetry) {
        if (shuffleboardSink == null) {
            return false;
        }
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

    private static Object snapshotValue(TelemetryValueType type, Object value) {
        if (value == null) {
            return null;
        }
        return switch (type) {
            case DOUBLE_ARRAY -> ((double[]) value).clone();
            case BOOLEAN_ARRAY -> ((boolean[]) value).clone();
            case INTEGER_ARRAY -> value instanceof long[]
                    ? ((long[]) value).clone()
                    : ((int[]) value).clone();
            case STRING_ARRAY -> ((String[]) value).clone();
            default -> value;
        };
    }

    private static boolean integerArrayEquals(Object value, Object lastValue) {
        if (value instanceof long[] && lastValue instanceof long[]) {
            return Arrays.equals((long[]) value, (long[]) lastValue);
        }
        if (value instanceof int[] && lastValue instanceof int[]) {
            return Arrays.equals((int[]) value, (int[]) lastValue);
        }
        if (value instanceof long[] && lastValue instanceof int[]) {
            return Arrays.equals((long[]) value, toLongArray((int[]) lastValue));
        }
        if (value instanceof int[] && lastValue instanceof long[]) {
            return Arrays.equals(toLongArray((int[]) value), (long[]) lastValue);
        }
        return false;
    }

    static long[] toLongArray(int[] values) {
        return Arrays.stream(values).asLongStream().toArray();
    }
}
