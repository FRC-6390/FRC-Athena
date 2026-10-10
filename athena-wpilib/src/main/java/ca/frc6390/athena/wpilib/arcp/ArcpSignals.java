package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.arcp.WriteResult;
import ca.frc6390.athena.mechanism.core.TelemetryAction;
import ca.frc6390.athena.mechanism.core.TelemetryNode;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Predicate;

/**
 * The ARCP signal table for one robot: the dictionary built from the mechanism telemetry schema, and the
 * per-cycle sampling and write/action routing that goes with it.
 *
 * <p>Signals, in dictionary order and numbered from 1:</p>
 * <ul>
 *   <li>every published {@link TelemetryValue} under its telemetry path (numbers, booleans, strings and
 *       number arrays; points and geometry are not sent over ARCP);</li>
 *   <li>every {@link TelemetryAction} under its path, carrying its running state, role 5;</li>
 *   <li>one lock-state boolean per lock group in use, at {@code athena/locks/<group class name>}, role 7.</li>
 * </ul>
 *
 * <p>Writes go through {@link TelemetryValue#tryWrite}, so the robot's lock rules and value ranges decide.
 * Actions go through {@link TelemetryAction#request()} and {@link TelemetryAction#cancel()}.</p>
 *
 * <p>All sampling methods run on the robot thread and allocate nothing for numbers and booleans once the
 * table is built.</p>
 */
final class ArcpSignals {

    private static final int TYPE_BOOL = 1;
    private static final int TYPE_F64 = 3;
    private static final int TYPE_STRING = 4;
    private static final int TYPE_F64_ARRAY = 7;

    /** A telemetry value with its last published state. */
    private static final class ValueChannel {
        final long signalId;
        final String path;
        final TelemetryValue value;
        final String lockGroupName;
        boolean published;
        long lastBits;
        String lastText;
        double[] lastArray;

        ValueChannel(long signalId, String path, TelemetryValue value) {
            this.signalId = signalId;
            this.path = path;
            this.value = value;
            this.lockGroupName = value.lockGroup() == null ? "" : value.lockGroup().getName();
        }
    }

    private static final class ActionChannel {
        final long signalId;
        final TelemetryAction action;
        boolean published;
        boolean lastRunning;

        ActionChannel(long signalId, TelemetryAction action) {
            this.signalId = signalId;
            this.action = action;
        }
    }

    private static final class LockChannel {
        final long signalId;
        final Class<?> group;
        boolean published;
        boolean lastLocked;

        LockChannel(long signalId, Class<?> group) {
            this.signalId = signalId;
            this.group = group;
        }
    }

    private final List<ArcpDictEntry> dictionary = new ArrayList<>();
    private final List<ValueChannel> values = new ArrayList<>();
    private final List<ActionChannel> actions = new ArrayList<>();
    private final List<LockChannel> locks = new ArrayList<>();
    private final Map<Long, ValueChannel> valuesById = new HashMap<>();
    private final Map<Long, ActionChannel> actionsById = new HashMap<>();
    private final Predicate<Class<?>> locked;
    private long nextId = 1;

    /**
     * Builds the table from a schema.
     *
     * @param schema mechanism telemetry schema
     * @param locked lock-group condition, true while writes to the group are rejected
     */
    ArcpSignals(TelemetrySchema schema, Predicate<Class<?>> locked) {
        this.locked = locked;
        Map<Class<?>, Boolean> groups = new LinkedHashMap<>();
        walk(schema.root(), "", groups);
        for (Class<?> group : groups.keySet()) {
            long id = nextId++;
            locks.add(new LockChannel(id, group));
            dictionary.add(new ArcpDictEntry(id, "athena/locks/" + group.getName(), TYPE_BOOL, "",
                    ArcpDictEntry.ROLE_LOCK, 0, "", 0));
        }
    }

    /** The dictionary entries, in signal id order. */
    List<ArcpDictEntry> dictionary() {
        return List.copyOf(dictionary);
    }

    /** Number of published values (not counting actions and lock signals). */
    int valueCount() {
        return values.size();
    }

    /**
     * Writes the cycle's changed values into {@code cycle}. Clears it first. Numbers, booleans and integers are
     * stored without allocation.
     */
    void sample(ArcpCycle cycle) {
        cycle.clear();
        for (int i = 0, n = values.size(); i < n; i++) {
            sampleValue(values.get(i), cycle);
        }
        for (int i = 0, n = actions.size(); i < n; i++) {
            ActionChannel channel = actions.get(i);
            boolean running = channel.action.running();
            if (!channel.published || running != channel.lastRunning) {
                cycle.setBoolean(channel.signalId, running);
                channel.lastRunning = running;
                channel.published = true;
            }
        }
        for (int i = 0, n = locks.size(); i < n; i++) {
            LockChannel channel = locks.get(i);
            boolean isLocked = locked.test(channel.group);
            if (!channel.published || isLocked != channel.lastLocked) {
                cycle.setBoolean(channel.signalId, isLocked);
                channel.lastLocked = isLocked;
                channel.published = true;
            }
        }
    }

    private void sampleValue(ValueChannel channel, ArcpCycle cycle) {
        TelemetryValue value = channel.value;
        switch (value.type()) {
            case NUMBER -> {
                double current = value.number();
                long bits = Double.doubleToRawLongBits(current);
                if (!channel.published || bits != channel.lastBits) {
                    cycle.setNumber(channel.signalId, current);
                    channel.lastBits = bits;
                    channel.published = true;
                }
            }
            case BOOLEAN -> {
                boolean current = value.bool();
                long bits = current ? 1L : 0L;
                if (!channel.published || bits != channel.lastBits) {
                    cycle.setBoolean(channel.signalId, current);
                    channel.lastBits = bits;
                    channel.published = true;
                }
            }
            case STRING -> {
                String current = String.valueOf(value.value());
                if (!current.equals(channel.lastText)) {
                    cycle.set(channel.signalId, current);
                    channel.lastText = current;
                    channel.published = true;
                }
            }
            case NUMBER_ARRAY -> {
                double[] current = (double[]) value.value();
                if (!channel.published || !Arrays.equals(current, channel.lastArray)) {
                    cycle.set(channel.signalId, current);
                    channel.lastArray = current;
                    channel.published = true;
                }
            }
            default -> {
                // POINTS and GEOMETRY are not part of the ARCP v2 value set; they are never added.
            }
        }
    }

    /**
     * Handles a client write. The value is converted to the declared type, then applied with
     * {@link TelemetryValue#tryWrite}, which enforces read-only, lock and range rules.
     *
     * @param signalId target signal
     * @param requested requested value
     * @return accepted with the held value, or rejected with the robot's reason and the held value
     */
    WriteResult write(long signalId, Object requested) {
        ValueChannel channel = valuesById.get(signalId);
        if (channel == null) {
            return WriteResult.reject("rejected: unknown signal", Boolean.FALSE);
        }
        TelemetryValue value = channel.value;
        if (!value.writable()) {
            return WriteResult.reject("rejected: read-only", currentValue(channel));
        }
        Object converted;
        try {
            converted = convert(value.type(), requested);
        } catch (IllegalArgumentException mismatch) {
            return WriteResult.reject(mismatch.getMessage(), currentValue(channel));
        }
        TelemetryValue.WriteResult result = value.tryWrite(converted);
        Object actual = currentValue(channel);
        return result.accepted()
                ? WriteResult.accept(actual)
                : WriteResult.reject(result.reason(), actual);
    }

    /**
     * Runs or cancels an action.
     *
     * @param signalId action signal
     * @param run true to run, false to cancel
     * @return accepted with the running state, or rejected for an unknown signal
     */
    WriteResult action(long signalId, boolean run) {
        ActionChannel channel = actionsById.get(signalId);
        if (channel == null) {
            return WriteResult.reject("rejected: unknown action", Boolean.FALSE);
        }
        if (run) {
            channel.action.request();
        } else {
            channel.action.cancel();
        }
        return WriteResult.accept(channel.action.running());
    }

    /** Current values of persisted writable numbers, booleans and strings, keyed by telemetry path. */
    Map<String, Object> persistedValues() {
        Map<String, Object> out = new TreeMap<>();
        for (ValueChannel channel : values) {
            if (channel.value.isPersisted() && channel.value.writable()
                    && channel.value.type() != TelemetryValue.Type.NUMBER_ARRAY) {
                out.put(channel.path, channel.value.value());
            }
        }
        return out;
    }

    private Object currentValue(ValueChannel channel) {
        return switch (channel.value.type()) {
            case NUMBER -> channel.value.number();
            case BOOLEAN -> channel.value.bool();
            case STRING -> String.valueOf(channel.value.value());
            default -> channel.value.value();
        };
    }

    private static Object convert(TelemetryValue.Type type, Object requested) {
        switch (type) {
            case NUMBER -> {
                if (requested instanceof Double d) return d;
                if (requested instanceof Long l) return l.doubleValue();
                if (requested instanceof Integer i) return i.doubleValue();
                throw new IllegalArgumentException("rejected: type mismatch (f64 expected)");
            }
            case BOOLEAN -> {
                if (requested instanceof Boolean b) return b;
                throw new IllegalArgumentException("rejected: type mismatch (bool expected)");
            }
            case STRING -> {
                if (requested instanceof String s) return s;
                throw new IllegalArgumentException("rejected: type mismatch (string expected)");
            }
            default -> throw new IllegalArgumentException("rejected: read-only");
        }
    }

    private void walk(TelemetryNode node, String prefix, Map<Class<?>, Boolean> groups) {
        for (Map.Entry<String, TelemetryValue> entry : node.values().entrySet()) {
            TelemetryValue value = entry.getValue();
            if (!value.isPublished() || !supported(value)) {
                continue;
            }
            String path = join(prefix, entry.getKey());
            long id = nextId++;
            ValueChannel channel = new ValueChannel(id, path, value);
            values.add(channel);
            valuesById.put(id, channel);
            int typeTag = typeTag(value.type());
            int role = value.writable() ? ArcpDictEntry.ROLE_CONTROL : roleOf(path);
            int flags = (value.writable() ? ArcpDictEntry.FLAG_WRITABLE : 0)
                    | (value.isPersisted() ? ArcpDictEntry.FLAG_PERSISTED : 0);
            dictionary.add(new ArcpDictEntry(id, path, typeTag, "", role, flags, channel.lockGroupName, 0));
            if (value.lockGroup() != null) {
                groups.putIfAbsent(value.lockGroup(), Boolean.TRUE);
            }
        }
        for (Map.Entry<String, TelemetryAction> entry : node.actions().entrySet()) {
            String path = join(prefix, entry.getKey());
            long id = nextId++;
            ActionChannel channel = new ActionChannel(id, entry.getValue());
            actions.add(channel);
            actionsById.put(id, channel);
            dictionary.add(new ArcpDictEntry(id, path, TYPE_BOOL, "", ArcpDictEntry.ROLE_ACTION, 0, "", 0));
        }
        for (Map.Entry<String, TelemetryNode> child : node.children().entrySet()) {
            walk(child.getValue(), join(prefix, child.getKey()), groups);
        }
    }

    private static boolean supported(TelemetryValue value) {
        return switch (value.type()) {
            case NUMBER, BOOLEAN, STRING, NUMBER_ARRAY -> true;
            default -> false;
        };
    }

    private static int typeTag(TelemetryValue.Type type) {
        return switch (type) {
            case BOOLEAN -> TYPE_BOOL;
            case STRING -> TYPE_STRING;
            case NUMBER_ARRAY -> TYPE_F64_ARRAY;
            default -> TYPE_F64;
        };
    }

    /** Role from the telemetry group a value sits in (State, Devices, Hooks, ...). */
    static int roleOf(String path) {
        for (String segment : path.split("/")) {
            switch (segment) {
                case "State":
                    return ArcpDictEntry.ROLE_STATE;
                case "Devices":
                    return ArcpDictEntry.ROLE_DEVICE;
                case "Controls":
                    return ArcpDictEntry.ROLE_CONTROL;
                case "Hooks":
                    return ArcpDictEntry.ROLE_HOOK;
                case "Actions":
                    return ArcpDictEntry.ROLE_ACTION;
                default:
                    break;
            }
        }
        return ArcpDictEntry.ROLE_VALUE;
    }

    private static String join(String left, String right) {
        if (right == null || right.isBlank()) return left;
        return left == null || left.isBlank() ? right : left + "/" + right;
    }
}
