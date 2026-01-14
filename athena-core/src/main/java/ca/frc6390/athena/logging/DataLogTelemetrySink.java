package ca.frc6390.athena.logging;

import java.util.Arrays;

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

public final class DataLogTelemetrySink implements TelemetrySink {
    private final DataLog log;
    private final String prefix;

    public DataLogTelemetrySink(String prefix) {
        DataLogManager.start();
        this.log = DataLogManager.getLog();
        this.prefix = prefix == null || prefix.isBlank() ? "" : prefix.trim();
    }

    @Override
    public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
        String fullKey = prefix.isEmpty() ? key : prefix + "/" + key;
        return switch (type) {
            case DOUBLE -> {
                DoubleLogEntry entry = new DoubleLogEntry(log, fullKey);
                if (initialValue instanceof Number) {
                    entry.append(((Number) initialValue).doubleValue());
                }
                yield value -> entry.append(((Number) value).doubleValue());
            }
            case BOOLEAN -> {
                BooleanLogEntry entry = new BooleanLogEntry(log, fullKey);
                if (initialValue instanceof Boolean) {
                    entry.append((Boolean) initialValue);
                }
                yield value -> entry.append((Boolean) value);
            }
            case INTEGER -> {
                IntegerLogEntry entry = new IntegerLogEntry(log, fullKey);
                if (initialValue instanceof Number) {
                    entry.append(((Number) initialValue).longValue());
                }
                yield value -> entry.append(((Number) value).longValue());
            }
            case STRING -> {
                StringLogEntry entry = new StringLogEntry(log, fullKey);
                if (initialValue instanceof String) {
                    entry.append((String) initialValue);
                }
                yield value -> entry.append((String) value);
            }
            case DOUBLE_ARRAY -> {
                DoubleArrayLogEntry entry = new DoubleArrayLogEntry(log, fullKey);
                if (initialValue instanceof double[]) {
                    entry.append((double[]) initialValue);
                }
                yield value -> entry.append((double[]) value);
            }
            case BOOLEAN_ARRAY -> {
                BooleanArrayLogEntry entry = new BooleanArrayLogEntry(log, fullKey);
                if (initialValue instanceof boolean[]) {
                    entry.append((boolean[]) initialValue);
                }
                yield value -> entry.append((boolean[]) value);
            }
            case INTEGER_ARRAY -> {
                IntegerArrayLogEntry entry = new IntegerArrayLogEntry(log, fullKey);
                if (initialValue instanceof long[]) {
                    entry.append((long[]) initialValue);
                } else if (initialValue instanceof int[]) {
                    entry.append(toLongArray((int[]) initialValue));
                }
                yield value -> {
                    if (value instanceof long[]) {
                        entry.append((long[]) value);
                    } else {
                        entry.append(toLongArray((int[]) value));
                    }
                };
            }
            case STRING_ARRAY -> {
                StringArrayLogEntry entry = new StringArrayLogEntry(log, fullKey);
                if (initialValue instanceof String[]) {
                    entry.append((String[]) initialValue);
                }
                yield value -> entry.append((String[]) value);
            }
        };
    }

    private static long[] toLongArray(int[] values) {
        return Arrays.stream(values).asLongStream().toArray();
    }
}
