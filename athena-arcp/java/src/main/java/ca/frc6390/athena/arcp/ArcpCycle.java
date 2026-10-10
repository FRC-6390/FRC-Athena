package ca.frc6390.athena.arcp;

import java.util.Arrays;
import java.util.Objects;

/**
 * Values of one robot cycle. Reuse one instance per cycle by calling {@link #clear()} first.
 *
 * <p>Numbers, booleans and integers are stored as primitives, so setting them does not allocate.
 * Strings, arrays and structs are kept as objects. The buffer grows once and is reused.</p>
 */
public final class ArcpCycle {

    private long[] ids = new long[64];
    private byte[] kinds = new byte[64];
    private long[] bits = new long[64];
    private Object[] objects = new Object[64];
    private int size;

    /** Adds a double value. Allocation free. */
    public ArcpCycle setNumber(long signalId, double value) {
        put(signalId, ArcpMarshal.KIND_F64, Double.doubleToRawLongBits(value), null);
        return this;
    }

    /** Adds a boolean value. Allocation free. */
    public ArcpCycle setBoolean(long signalId, boolean value) {
        put(signalId, ArcpMarshal.KIND_BOOL, value ? 1L : 0L, null);
        return this;
    }

    /** Adds an integer value. Allocation free. */
    public ArcpCycle setLong(long signalId, long value) {
        put(signalId, ArcpMarshal.KIND_I64, value, null);
        return this;
    }

    /**
     * Adds a value of any supported type. Prefer the typed setters on hot paths.
     *
     * @param signalId dictionary signal id
     * @param value Boolean, Long, Integer, Double, String, boolean[], long[], double[], String[] or {@link ArcpStruct}
     * @return this cycle
     */
    public ArcpCycle set(long signalId, Object value) {
        Objects.requireNonNull(value, "value for signal " + signalId);
        if (value instanceof Double d) {
            return setNumber(signalId, d);
        }
        if (value instanceof Boolean b) {
            return setBoolean(signalId, b);
        }
        if (value instanceof Long l) {
            return setLong(signalId, l);
        }
        if (value instanceof Integer i) {
            return setLong(signalId, i);
        }
        if (value instanceof String) {
            put(signalId, ArcpMarshal.KIND_STRING, 0L, value);
        } else if (value instanceof boolean[]) {
            put(signalId, ArcpMarshal.KIND_BOOL_ARRAY, 0L, value);
        } else if (value instanceof long[]) {
            put(signalId, ArcpMarshal.KIND_I64_ARRAY, 0L, value);
        } else if (value instanceof double[]) {
            put(signalId, ArcpMarshal.KIND_F64_ARRAY, 0L, value);
        } else if (value instanceof String[]) {
            put(signalId, ArcpMarshal.KIND_STRING_ARRAY, 0L, value);
        } else if (value instanceof ArcpStruct) {
            put(signalId, ArcpMarshal.KIND_STRUCT, 0L, value);
        } else {
            throw new IllegalArgumentException("unsupported ARCP value type: " + value.getClass().getName());
        }
        return this;
    }

    /** Removes every value. Keeps the buffers. */
    public ArcpCycle clear() {
        Arrays.fill(objects, 0, size, null);
        size = 0;
        return this;
    }

    /** Number of values in this cycle. */
    public int size() {
        return size;
    }

    /** Signal id of value {@code index}. */
    public long signalId(int index) {
        return ids[index];
    }

    /** Marshalling kind of value {@code index} (one of the {@code ArcpMarshal.KIND_*} values). */
    int kind(int index) {
        return kinds[index];
    }

    /** Raw bits of a primitive value (double bits, 0/1 for booleans, the integer). */
    long bits(int index) {
        return bits[index];
    }

    /** Object value of a string, array or struct. */
    Object object(int index) {
        return objects[index];
    }

    /** Boxed value of {@code index}. Allocates for primitives; use on cold paths only. */
    public Object value(int index) {
        return switch (kinds[index]) {
            case ArcpMarshal.KIND_F64 -> Double.longBitsToDouble(bits[index]);
            case ArcpMarshal.KIND_BOOL -> bits[index] != 0;
            case ArcpMarshal.KIND_I64 -> bits[index];
            default -> objects[index];
        };
    }

    private void put(long signalId, int kind, long rawBits, Object object) {
        if (size == ids.length) {
            int capacity = size * 2;
            ids = Arrays.copyOf(ids, capacity);
            kinds = Arrays.copyOf(kinds, capacity);
            bits = Arrays.copyOf(bits, capacity);
            objects = Arrays.copyOf(objects, capacity);
        }
        ids[size] = signalId;
        kinds[size] = (byte) kind;
        bits[size] = rawBits;
        objects[size] = object;
        size++;
    }
}
