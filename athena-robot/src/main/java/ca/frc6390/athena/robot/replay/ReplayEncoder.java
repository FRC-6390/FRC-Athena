package ca.frc6390.athena.robot.replay;

import edu.wpi.first.units.measure.Voltage;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.mechanism.core.Output;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.robot.RuntimeCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Converts runtime cycles into ARCP v2 / replay records (appendix E1-W).
 *
 * <p>Every value gets a stable integer id the first time its path is seen. Frames carry only the
 * values that changed since the previous frame, except keyframes, which carry every known signal.
 * Keyframes are written on the first frame, when forced, and at least once per second. After
 * warm-up the encoder allocates nothing per cycle beyond growth of its arrays and writers.</p>
 *
 * <p>Command outputs are recorded as {@code command/<motor>/mode} and {@code command/<motor>/value}.
 * They are only written for cycles that ran mechanisms; a physics-only cycle leaves them unchanged.</p>
 */
final class ReplayEncoder {
    static final long KEYFRAME_INTERVAL_MICROS = 1_000_000L;
    private static final long NO_KEYFRAME = Long.MIN_VALUE;

    /** Field layout of one entity. The index of a field is its position in the array. */
    record Field(String name, int typeTag, String unit) {
    }

    static final Field[] LIFECYCLE = {
            new Field("mode", Wire.TYPE_STRING, ""),
            new Field("phase", Wire.TYPE_STRING, ""),
            new Field("enabled", Wire.TYPE_BOOL, ""),
            new Field("autonomous", Wire.TYPE_BOOL, ""),
            new Field("simulation", Wire.TYPE_BOOL, ""),
            new Field("mechanismsRan", Wire.TYPE_BOOL, ""),
            new Field("timestampSeconds", Wire.TYPE_F64, "s"),
            new Field("dtSeconds", Wire.TYPE_F64, "s"),
    };
    static final int L_MODE = 0;
    static final int L_PHASE = 1;
    static final int L_ENABLED = 2;
    static final int L_AUTONOMOUS = 3;
    static final int L_SIMULATION = 4;
    static final int L_MECHANISMS_RAN = 5;
    static final int L_TIMESTAMP = 6;
    static final int L_DT = 7;

    static final Field[] MOTOR = {
            new Field("positionRotations", Wire.TYPE_F64, "rot"),
            new Field("velocityRotationsPerSecond", Wire.TYPE_F64, "rot/s"),
            new Field("appliedVoltage", Wire.TYPE_F64, "V"),
            new Field("supplyCurrentAmps", Wire.TYPE_F64, "A"),
            new Field("statorCurrentAmps", Wire.TYPE_F64, "A"),
            new Field("connected", Wire.TYPE_BOOL, ""),
            new Field("failure", Wire.TYPE_STRING, ""),
    };
    static final Field[] ENCODER = {
            new Field("positionRotations", Wire.TYPE_F64, "rot"),
            new Field("absolutePositionRotations", Wire.TYPE_F64, "rot"),
            new Field("velocityRotationsPerSecond", Wire.TYPE_F64, "rot/s"),
            new Field("connected", Wire.TYPE_BOOL, ""),
            new Field("failure", Wire.TYPE_STRING, ""),
    };
    static final Field[] IMU = {
            new Field("yawDegrees", Wire.TYPE_F64, "deg"),
            new Field("pitchDegrees", Wire.TYPE_F64, "deg"),
            new Field("rollDegrees", Wire.TYPE_F64, "deg"),
            new Field("angleDegrees", Wire.TYPE_F64, "deg"),
            new Field("yawRateDegreesPerSecond", Wire.TYPE_F64, "deg/s"),
            new Field("pitchRateDegreesPerSecond", Wire.TYPE_F64, "deg/s"),
            new Field("rollRateDegreesPerSecond", Wire.TYPE_F64, "deg/s"),
            new Field("linearAccelerationXG", Wire.TYPE_F64, "g"),
            new Field("linearAccelerationYG", Wire.TYPE_F64, "g"),
            new Field("linearAccelerationZG", Wire.TYPE_F64, "g"),
            new Field("connected", Wire.TYPE_BOOL, ""),
            new Field("calibrating", Wire.TYPE_BOOL, ""),
            new Field("lastUpdateSeconds", Wire.TYPE_F64, "s"),
            new Field("failure", Wire.TYPE_STRING, ""),
    };
    static final Field[] DIGITAL = {
            new Field("raw", Wire.TYPE_BOOL, ""),
            new Field("active", Wire.TYPE_BOOL, ""),
    };
    static final Field[] COMMAND = {
            new Field("mode", Wire.TYPE_STRING, ""),
            new Field("value", Wire.TYPE_F64, ""),
    };

    private final WireWriter body = new WireWriter(4096);
    private final WireWriter framePayload = new WireWriter(4096);
    private final WireWriter recordPayload = new WireWriter(1024);

    private final java.util.ArrayList<ReplaySignal> signals = new java.util.ArrayList<>();
    private int capacity = 256;
    private int[] changed = new int[capacity];
    private int[] touched = new int[capacity];
    private int touchedCount;
    private long[] stamp = new long[capacity];
    private long[] lastBits = new long[capacity];
    private Object[] lastObjects = new Object[capacity];
    private long[] pendingBits = new long[capacity];
    private Object[] pendingObjects = new Object[capacity];
    private boolean[] known = new boolean[capacity];
    private long cycle;
    private int dictionaryFrom;

    private final int[] lifecycleIds;
    private final int[][] joystickIds = new int[RuntimeInputSnapshot.JOYSTICK_PORTS][];
    private final int[] driverStationIds;
    private final Map<Integer, int[]> localizationIds = new HashMap<>();
    private final Map<HardwareIdentity, int[]> motorIds = new HashMap<>();
    private final Map<String, int[]> encoderIds = new HashMap<>();
    private final Map<String, int[]> imuIds = new HashMap<>();
    private final Map<String, int[]> digitalIds = new HashMap<>();
    private final Map<String, Integer> commandSlots = new HashMap<>();
    private int[][] commandIds = new int[8][];
    private String[] commandModes = new String[8];
    private double[] commandValues = new double[8];
    private int commandCount;

    private boolean forceKeyframe = true;
    private long lastKeyframeMicros = NO_KEYFRAME;

    ReplayEncoder() {
        lifecycleIds = entity("lifecycle", LIFECYCLE, ReplaySignal.ROLE_LIFECYCLE);
        // Input signals are declared up front so the dictionary is the same in every file from this recorder.
        for (int port = 0; port < joystickIds.length; port++) {
            joystickIds[port] = entity("input/joystick/" + port, ReplayInputFields.JOYSTICK_FIELDS,
                    ReplaySignal.ROLE_INPUT);
        }
        driverStationIds = entity("input/driverStation", ReplayInputFields.DRIVER_STATION_FIELDS,
                ReplaySignal.ROLE_INPUT);
    }

    /** Forces the next frame to be a keyframe carrying every known signal. */
    void requestKeyframe() {
        forceKeyframe = true;
    }

    /** Writes a DICTIONARY record containing every entry, and marks all entries as written. */
    void writeDictionary(WireWriter out) {
        writeDictionaryRange(out, 0, signals.size());
        dictionaryFrom = signals.size();
    }

    /**
     * Encodes one cycle.
     *
     * @param snapshot runtime cycle; its lifecycle and digital inputs are used
     * @param mechanismInputs hardware inputs the mechanisms used this cycle, or null to use the snapshot's
     * @param outputs resolved mechanism outputs, or null when no mechanisms ran this cycle
     * @param dictionaryOut receives DICTIONARY records for signals first seen this cycle
     * @param frameOut receives one FRAME record
     */
    void encode(
            RuntimeCycleSnapshot snapshot,
            HardwareCycleSnapshot mechanismInputs,
            List<ResolvedOutput> outputs,
            WireWriter dictionaryOut,
            WireWriter frameOut) {
        cycle++;
        touchedCount = 0;
        HardwareCycleSnapshot hardware = mechanismInputs != null ? mechanismInputs : snapshot.hardware();
        stageLifecycle(snapshot, outputs != null);
        stageHardware(hardware);
        stageDigital(snapshot.digitalInputs());
        stageInputs(snapshot.inputs());
        stageLocalizations(snapshot.localizations());
        if (outputs != null) {
            stageCommands(hardware, outputs);
        }
        if (dictionaryFrom < signals.size()) {
            writeDictionaryRange(dictionaryOut, dictionaryFrom, signals.size());
            dictionaryFrom = signals.size();
        }

        long micros = micros(snapshot.timestampSeconds());
        int changedCount = commit();
        boolean keyframe = forceKeyframe
                || lastKeyframeMicros == NO_KEYFRAME
                || micros - lastKeyframeMicros >= KEYFRAME_INTERVAL_MICROS;
        body.clear();
        int entries = 0;
        if (keyframe) {
            for (int id = 1; id <= signals.size(); id++) {
                if (known[id]) {
                    writeEntry(body, id);
                    entries++;
                }
            }
            lastKeyframeMicros = micros;
            forceKeyframe = false;
        } else {
            for (int index = 0; index < changedCount; index++) {
                writeEntry(body, changed[index]);
                entries++;
            }
        }
        framePayload.clear();
        Wire.frameHeader(framePayload, keyframe, micros, snapshot.sequence(), entries);
        framePayload.bytes(body.buffer(), 0, body.size());
        frameOut.record(Wire.RECORD_FRAME, framePayload);
    }

    /** Writes a MARKER record. */
    void writeMarker(WireWriter out, double timestampSeconds, int kind, String message, int signalId) {
        recordPayload.clear();
        Wire.marker(recordPayload, micros(timestampSeconds), kind, message, signalId);
        out.record(Wire.RECORD_MARKER, recordPayload);
    }

    /** Writes a METADATA record with the given key/value pairs. */
    static void writeMetadata(WireWriter out, Map<String, String> metadata) {
        WireWriter payload = new WireWriter(256);
        payload.varint(metadata.size());
        for (Map.Entry<String, String> entry : metadata.entrySet()) {
            payload.string(entry.getKey());
            payload.string(entry.getValue());
        }
        out.record(Wire.RECORD_METADATA, payload);
    }

    static long micros(double seconds) {
        return Double.isFinite(seconds) ? Math.round(seconds * 1_000_000.0) : 0L;
    }

    private void writeDictionaryRange(WireWriter out, int from, int to) {
        if (from >= to) {
            return;
        }
        recordPayload.clear();
        recordPayload.varint(to - from);
        for (int index = from; index < to; index++) {
            ReplaySignal signal = signals.get(index);
            recordPayload.varint(signal.id())
                    .string(signal.path())
                    .u8(signal.typeTag())
                    .string(signal.unit())
                    .u8(signal.role())
                    .u8(signal.flags())
                    .string(signal.lockGroup())
                    .varint(signal.structSchemaId());
        }
        out.record(Wire.RECORD_DICTIONARY, recordPayload);
    }

    private void writeEntry(WireWriter out, int id) {
        int type = signals.get(id - 1).typeTag();
        out.varint(id).u8(type);
        switch (type) {
            case Wire.TYPE_BOOL -> out.u8(lastBits[id] != 0 ? 1 : 0);
            case Wire.TYPE_I64 -> out.zigzag(lastBits[id]);
            case Wire.TYPE_F64 -> out.u64(lastBits[id]);
            case Wire.TYPE_STRING -> out.string((String) lastObjects[id]);
            default -> throw new IllegalStateException("Unsupported replay type " + type + ".");
        }
    }

    /** Commits staged values and returns the number of changed ids, stored in {@code changed}. */
    private int commit() {
        int changedCount = 0;
        for (int index = 0; index < touchedCount; index++) {
            int id = touched[index];
            boolean differs;
            if (!known[id]) {
                differs = true;
            } else if (signals.get(id - 1).typeTag() == Wire.TYPE_STRING) {
                differs = !Objects.equals(lastObjects[id], pendingObjects[id]);
            } else {
                differs = lastBits[id] != pendingBits[id];
            }
            known[id] = true;
            lastBits[id] = pendingBits[id];
            lastObjects[id] = pendingObjects[id];
            if (differs) {
                changed[changedCount++] = id;
            }
        }
        return changedCount;
    }

    private void stageLifecycle(RuntimeCycleSnapshot snapshot, boolean mechanismsRan) {
        int[] ids = lifecycleIds;
        putString(ids[L_MODE], snapshot.mode().name());
        putString(ids[L_PHASE], snapshot.phase().name());
        putBool(ids[L_ENABLED], snapshot.enabled());
        putBool(ids[L_AUTONOMOUS], snapshot.autonomous());
        putBool(ids[L_SIMULATION], snapshot.simulation());
        putBool(ids[L_MECHANISMS_RAN], mechanismsRan);
        putF64(ids[L_TIMESTAMP], snapshot.timestampSeconds());
        putF64(ids[L_DT], snapshot.dtSeconds());
    }

    private void stageHardware(HardwareCycleSnapshot hardware) {
        for (Map.Entry<HardwareIdentity, HardwareCycleSnapshot.MotorInput> entry : hardware.motors().entrySet()) {
            HardwareCycleSnapshot.MotorInput motor = entry.getValue();
            int[] ids = motorIds.get(entry.getKey());
            if (ids == null) {
                ids = entity("hardware/motor/" + motor.name(), MOTOR, ReplaySignal.ROLE_HARDWARE);
                motorIds.put(entry.getKey(), ids);
                commandSlot(motor.name());
            }
            putF64(ids[0], motor.positionRotations());
            putF64(ids[1], motor.velocityRotationsPerSecond());
            putF64(ids[2], motor.appliedVoltage());
            putF64(ids[3], motor.supplyCurrentAmps());
            putF64(ids[4], motor.statorCurrentAmps());
            putBool(ids[5], motor.connected());
            putString(ids[6], motor.failure());
        }
        for (Map.Entry<HardwareIdentity, HardwareCycleSnapshot.EncoderInput> entry : hardware.encoders().entrySet()) {
            HardwareCycleSnapshot.EncoderInput encoder = entry.getValue();
            int[] ids = encoderIds.computeIfAbsent(
                    encoder.name(), name -> entity("hardware/encoder/" + name, ENCODER, ReplaySignal.ROLE_HARDWARE));
            putF64(ids[0], encoder.positionRotations());
            putF64(ids[1], encoder.absolutePositionRotations());
            putF64(ids[2], encoder.velocityRotationsPerSecond());
            putBool(ids[3], encoder.connected());
            putString(ids[4], encoder.failure());
        }
        for (HardwareCycleSnapshot.ImuInput imu : hardware.imus().values()) {
            int[] ids = imuIds.computeIfAbsent(
                    imu.name(), name -> entity("hardware/imu/" + name, IMU, ReplaySignal.ROLE_HARDWARE));
            putF64(ids[0], imu.yawDegrees());
            putF64(ids[1], imu.pitchDegrees());
            putF64(ids[2], imu.rollDegrees());
            putF64(ids[3], imu.angleDegrees());
            putF64(ids[4], imu.yawRateDegreesPerSecond());
            putF64(ids[5], imu.pitchRateDegreesPerSecond());
            putF64(ids[6], imu.rollRateDegreesPerSecond());
            putF64(ids[7], imu.linearAccelerationXG());
            putF64(ids[8], imu.linearAccelerationYG());
            putF64(ids[9], imu.linearAccelerationZG());
            putBool(ids[10], imu.connected());
            putBool(ids[11], imu.calibrating());
            putF64(ids[12], imu.lastUpdateSeconds());
            putString(ids[13], imu.failure());
        }
    }

    private void stageDigital(List<RuntimeCycleSnapshot.DigitalInput> inputs) {
        for (RuntimeCycleSnapshot.DigitalInput input : inputs) {
            int[] ids = digitalIds.computeIfAbsent(
                    input.name(), name -> entity("digital/" + name, DIGITAL, ReplaySignal.ROLE_DIGITAL_INPUT));
            putBool(ids[0], input.raw());
            putBool(ids[1], input.active());
        }
    }

    private void stageInputs(RuntimeInputSnapshot inputs) {
        for (int port = 0; port < joystickIds.length; port++) {
            RuntimeInputSnapshot.Joystick joystick = inputs.joystick(port);
            int[] ids = joystickIds[port];
            putBool(ids[ReplayInputFields.J_CONNECTED], joystick.connected());
            putString(ids[ReplayInputFields.J_NAME], joystick.name());
            putI64(ids[ReplayInputFields.J_TYPE], joystick.type());
            putI64(ids[ReplayInputFields.J_BUTTONS], joystick.buttons());
            for (int axis = 0; axis < RuntimeInputSnapshot.AXES; axis++) {
                putF64(ids[ReplayInputFields.J_AXIS_0 + axis], joystick.axis(axis));
            }
            for (int pov = 0; pov < RuntimeInputSnapshot.POVS; pov++) {
                putI64(ids[ReplayInputFields.J_POV_0 + pov], joystick.pov(pov));
            }
        }
        RuntimeInputSnapshot.DriverStationState state = inputs.driverStation();
        int[] ds = driverStationIds;
        putBool(ds[ReplayInputFields.DS_ENABLED], state.enabled());
        putBool(ds[ReplayInputFields.DS_AUTONOMOUS], state.autonomous());
        putBool(ds[ReplayInputFields.DS_TEST], state.test());
        putBool(ds[ReplayInputFields.DS_FMS], state.fmsAttached());
        putBool(ds[ReplayInputFields.DS_DS_ATTACHED], state.dsAttached());
        putString(ds[ReplayInputFields.DS_ALLIANCE], state.alliance());
        putF64(ds[ReplayInputFields.DS_MATCH_TIME], state.matchTime());
        putString(ds[ReplayInputFields.DS_EVENT], state.eventName());
        putString(ds[ReplayInputFields.DS_MATCH_TYPE], state.matchType());
        putI64(ds[ReplayInputFields.DS_MATCH_NUMBER], state.matchNumber());
        putString(ds[ReplayInputFields.DS_GAME_MESSAGE], state.gameMessage());
    }

    /** Stages a summary of each localization: the newest measurement and, when present, the newest pose. */
    private void stageLocalizations(List<RuntimeCycleSnapshot.LocalizationInput> localizations) {
        for (RuntimeCycleSnapshot.LocalizationInput input : localizations) {
            int[] ids = localizationIds.computeIfAbsent(input.index(),
                    index -> entity("localization/" + index, ReplayInputFields.LOCALIZATION_FIELDS,
                            ReplaySignal.ROLE_LOCALIZATION));
            Measurement newest = null;
            PoseMeasurementSample pose = null;
            for (Measurement measurement : input.measurements()) {
                if (newest == null || measurement.timestampSeconds() >= newest.timestampSeconds()) {
                    newest = measurement;
                }
                if (measurement instanceof PoseMeasurementSample sample
                        && (pose == null || sample.timestampSeconds() >= pose.timestampSeconds())) {
                    pose = sample;
                }
            }
            putString(ids[0], input.type());
            putI64(ids[1], input.measurements().size());
            putString(ids[2], newest == null ? "" : newest.getClass().getSimpleName());
            putF64(ids[3], newest == null ? Double.NaN : newest.timestampSeconds());
            putF64(ids[4], newest == null ? Double.NaN : newest.latencySeconds());
            PoseSnapshot value = pose == null ? null : pose.pose();
            putF64(ids[5], value == null ? Double.NaN : value.xMeters());
            putF64(ids[6], value == null ? Double.NaN : value.yMeters());
            putF64(ids[7], value == null ? Double.NaN : value.headingRadians());
        }
    }

    private void stageCommands(HardwareCycleSnapshot hardware, List<ResolvedOutput> outputs) {
        for (int slot = 0; slot < commandCount; slot++) {
            commandModes[slot] = "neutral";
            commandValues[slot] = 0.0;
        }
        for (ResolvedOutput resolved : outputs) {
            for (MotorDevice motor : resolved.request().motors()) {
                int slot = commandSlot(motor.defaultName());
                commandModes[slot] = outputMode(resolved.output());
                commandValues[slot] = outputValue(resolved.output());
            }
        }
        for (int slot = 0; slot < commandCount; slot++) {
            putString(commandIds[slot][0], commandModes[slot]);
            putF64(commandIds[slot][1], commandValues[slot]);
        }
    }

    private int commandSlot(String name) {
        Integer existing = commandSlots.get(name);
        if (existing != null) {
            return existing;
        }
        int slot = commandCount++;
        if (slot == commandIds.length) {
            int size = slot * 2;
            commandIds = java.util.Arrays.copyOf(commandIds, size);
            commandModes = java.util.Arrays.copyOf(commandModes, size);
            commandValues = java.util.Arrays.copyOf(commandValues, size);
        }
        commandIds[slot] = entity("command/" + name, COMMAND, ReplaySignal.ROLE_COMMAND);
        commandModes[slot] = "neutral";
        commandValues[slot] = 0.0;
        commandSlots.put(name, slot);
        return slot;
    }

    private int[] entity(String prefix, Field[] fields, int role) {
        int[] ids = new int[fields.length];
        for (int index = 0; index < fields.length; index++) {
            ids[index] = addSignal(prefix + "/" + fields[index].name(), fields[index].typeTag(), fields[index].unit(), role);
        }
        return ids;
    }

    private int addSignal(String path, int typeTag, String unit, int role) {
        int id = signals.size() + 1;
        signals.add(new ReplaySignal(id, path, typeTag, unit, role, 0, "", 0L));
        ensureCapacity(id + 1);
        return id;
    }

    private void ensureCapacity(int required) {
        if (required <= capacity) {
            return;
        }
        int size = Math.max(required, capacity * 2);
        stamp = java.util.Arrays.copyOf(stamp, size);
        lastBits = java.util.Arrays.copyOf(lastBits, size);
        lastObjects = java.util.Arrays.copyOf(lastObjects, size);
        pendingBits = java.util.Arrays.copyOf(pendingBits, size);
        pendingObjects = java.util.Arrays.copyOf(pendingObjects, size);
        known = java.util.Arrays.copyOf(known, size);
        touched = java.util.Arrays.copyOf(touched, size);
        changed = java.util.Arrays.copyOf(changed, size);
        capacity = size;
    }

    private void stage(int id) {
        if (stamp[id] != cycle) {
            stamp[id] = cycle;
            touched[touchedCount++] = id;
        }
    }

    private void putF64(int id, double value) {
        stage(id);
        pendingBits[id] = Double.doubleToRawLongBits(value);
    }

    private void putBool(int id, boolean value) {
        stage(id);
        pendingBits[id] = value ? 1L : 0L;
    }

    private void putI64(int id, long value) {
        stage(id);
        pendingBits[id] = value;
    }

    private void putString(int id, String value) {
        stage(id);
        pendingObjects[id] = value == null ? "" : value;
    }

    static String outputMode(Output output) {
        if (output instanceof Output.Percent) return "percent";
        if (output instanceof Output.Voltage) return "voltage";
        if (output instanceof Output.Position) return "position";
        if (output instanceof Output.Velocity) return "velocity";
        if (output instanceof Output.Fault) return "fault";
        return "neutral";
    }

    static double outputValue(Output output) {
        if (output instanceof Output.Percent percent) return percent.percent();
        if (output instanceof Output.Voltage voltage) return voltage.volts();
        if (output instanceof Output.Position position) return position.position();
        if (output instanceof Output.Velocity velocity) return velocity.velocity();
        return 0.0;
    }
}
