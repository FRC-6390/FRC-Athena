package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import ca.frc6390.athena.mechanism.core.MechanismTraceLevel;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Publishes compact, AdvantageScope-compatible mechanism execution traces over NT4. */
public final class MechanismTracePublisher implements AutoCloseable {
    private static final double PERIOD_SECONDS = 0.02;
    private final NetworkTableInstance networkTables;
    private final StringSubscriber profileOverride;
    private final Map<String, Channel> channels = new LinkedHashMap<>();
    private Profile profile = Profile.SUMMARY;
    private String lastProfileOverride = "";
    private Profile resolvedProfile = Profile.SUMMARY;

    /** Runtime publishing profile. */
    public enum Profile {
        /** Publishes nothing. */
        OFF,
        /** Publishes mechanism state and readable action labels. */
        SUMMARY,
        /** Publishes action arbitration, controls, motors, and hooks at capture rate. */
        CAPTURE
    }

    /** Creates a publisher on the default NT4 instance. */
    public MechanismTracePublisher() {
        this(NetworkTableInstance.getDefault());
    }

    /** Creates a publisher on an explicit NT4 instance. */
    public MechanismTracePublisher(NetworkTableInstance networkTables) {
        this.networkTables = java.util.Objects.requireNonNull(networkTables, "networkTables");
        profileOverride = networkTables.getStringTopic("/Athena/Telemetry/Profile").subscribe("");
    }

    /** Publishes traces captured by the runtime. This performs no hardware reads. */
    public void publish(List<MechanismTraceSnapshot> traces) {
        Profile activeProfile = activeProfile();
        if (traces == null || activeProfile == Profile.OFF) {
            return;
        }
        for (MechanismTraceSnapshot trace : traces) {
            if (trace != null) {
                channels.computeIfAbsent(trace.mechanism(), this::channel).publish(trace, activeProfile);
            }
        }
    }

    /** Selects the amount of live NT4 telemetry. */
    public MechanismTracePublisher profile(Profile profile) {
        this.profile = profile == null ? Profile.SUMMARY : profile;
        resolvedProfile = parseProfile(lastProfileOverride, this.profile);
        return this;
    }

    /** Returns the live runtime trace level, including the NT profile override. */
    public MechanismTraceLevel traceLevel() {
        return switch (activeProfile()) {
            case OFF -> MechanismTraceLevel.OFF;
            case SUMMARY -> MechanismTraceLevel.SUMMARY;
            case CAPTURE -> MechanismTraceLevel.CAPTURE;
        };
    }

    /** Returns the runtime capture period needed by the active publication profile. */
    public double runtimePeriodSeconds() {
        return activeProfile() == Profile.CAPTURE ? PERIOD_SECONDS : 0.10;
    }

    @Override
    public void close() {
        channels.values().forEach(Channel::close);
        channels.clear();
        profileOverride.close();
    }

    private Profile activeProfile() {
        String override = profileOverride.get();
        if (!lastProfileOverride.equals(override)) {
            lastProfileOverride = override;
            resolvedProfile = parseProfile(override, profile);
        }
        return resolvedProfile;
    }

    static Profile parseProfile(String value, Profile fallback) {
        Profile safeFallback = fallback == null ? Profile.SUMMARY : fallback;
        if (value == null || value.isBlank()) {
            return safeFallback;
        }
        try {
            return Profile.valueOf(value.trim().toUpperCase(java.util.Locale.ROOT));
        } catch (IllegalArgumentException ignored) {
            return safeFallback;
        }
    }

    private Channel channel(String mechanism) {
        String root = "/Athena/Mechanisms/" + topicPath(mechanism) + "/Trace";
        return new Channel(networkTables, root);
    }

    private static String topicPath(String value) {
        if (value == null || value.isBlank()) {
            return "mechanism";
        }
        StringBuilder result = new StringBuilder(value.length());
        int segmentStart = 0;
        for (int index = 0; index <= value.length(); index++) {
            if (index == value.length() || value.charAt(index) == '/') {
                if (result.length() > 0) result.append('/');
                if (index == segmentStart) {
                    result.append("mechanism");
                } else {
                    for (int character = segmentStart; character < index; character++) {
                        result.append(topicCharacter(value.charAt(character)));
                    }
                }
                segmentStart = index + 1;
            }
        }
        return result.toString();
    }

    private static char topicCharacter(char value) {
        return Character.isLetterOrDigit(value) || value == '_' || value == '.' || value == '-'
                ? value : '_';
    }

    private static int code(String value) {
        return value == null ? 0 : value.hashCode();
    }

    private static byte outputMode(String value) {
        return switch (value == null ? "" : value) {
            case "percent" -> 1;
            case "voltage" -> 2;
            case "position" -> 3;
            case "velocity" -> 4;
            case "fault" -> 5;
            default -> 0;
        };
    }

    static StateFrame stateFrame(MechanismTraceSnapshot trace) {
        return new StateFrame(
                trace.timestampSeconds(), trace.timeInStateSeconds(), trace.schedulerStep(),
                trace.activeLeaseCount(), trace.enabled(), trace.schedulerComplete());
    }

    static CandidateFrame candidateFrame(MechanismTraceSnapshot.ActionCandidate candidate) {
        return new CandidateFrame(
                code(candidate.actionType()),
                candidate.motors().isEmpty() ? 0 : code(candidate.motors().get(0)),
                candidate.recency(), candidate.order(), candidate.requestedValue(),
                (byte) ("lease".equals(candidate.source()) ? 1 : 0),
                outputMode(candidate.outputMode()), candidate.selected(), code(candidate.decision()));
    }

    static ControlFrame controlFrame(MechanismTraceSnapshot.Control control) {
        return new ControlFrame(
                code(control.name()), outputMode(control.requestedMode()), outputMode(control.appliedMode()),
                (byte) Channel.flags(control.constrained(), control.blocked(), control.saturated()),
                code(control.route()), control.requestedValue(), control.transformedValue(), control.goal(),
                control.referencePosition(), control.referenceVelocity(), control.referenceAcceleration(),
                control.measuredPosition(), control.measuredVelocity(), control.error(),
                control.proportionalVolts(), control.integralVolts(), control.derivativeVolts(),
                control.staticFeedforwardVolts(), control.velocityFeedforwardVolts(),
                control.accelerationFeedforwardVolts(), control.gravityFeedforwardVolts(), control.appliedValue());
    }

    static MotorFrame motorFrame(MechanismTraceSnapshot.Motor motor) {
        return new MotorFrame(
                code(motor.name()), outputMode(motor.commandMode()), motor.commandValue(),
                motor.positionRotations(), motor.velocityRotationsPerSecond(), motor.appliedVoltage(),
                motor.supplyCurrentAmps(), motor.statorCurrentAmps(),
                (byte) Channel.flags(motor.disabled(), !motor.connected(), false), code(motor.failure()));
    }

    static HookFrame hookFrame(MechanismTraceSnapshot.Hook hook) {
        return new HookFrame(code(hook.name()), hook.sourceActive(), hook.active(), hook.triggeredThisCycle());
    }

    private static final class Channel implements AutoCloseable {
        private final StructPublisher<StateFrame> state;
        private final StructArrayPublisher<CandidateFrame> candidates;
        private final StructArrayPublisher<ControlFrame> controls;
        private final StructArrayPublisher<MotorFrame> motors;
        private final StructArrayPublisher<HookFrame> hooks;
        private final StringPublisher requestedAction;
        private final StringPublisher requestedActionType;
        private final StringPublisher scheduledActionType;
        private final StringArrayPublisher metadata;
        private final Map<Integer, String> names = new LinkedHashMap<>();
        private String lastRequestedAction = "";
        private String lastRequestedActionType = "";
        private String lastScheduledActionType = "";
        private int publishedMetadataSize;
        private CandidateFrame[] candidateFrames = new CandidateFrame[0];
        private ControlFrame[] controlFrames = new ControlFrame[0];
        private MotorFrame[] motorFrames = new MotorFrame[0];
        private HookFrame[] hookFrames = new HookFrame[0];
        private double lastPublishSeconds = Double.NEGATIVE_INFINITY;

        private Channel(NetworkTableInstance nt, String root) {
            PubSubOption period = PubSubOption.periodic(PERIOD_SECONDS);
            state = nt.getStructTopic(root + "/State", StateFrame.STRUCT).publish(period);
            candidates = nt.getStructArrayTopic(root + "/Candidates", CandidateFrame.STRUCT).publish(period);
            controls = nt.getStructArrayTopic(root + "/Controls", ControlFrame.STRUCT).publish(period);
            motors = nt.getStructArrayTopic(root + "/Motors", MotorFrame.STRUCT).publish(period);
            hooks = nt.getStructArrayTopic(root + "/Hooks", HookFrame.STRUCT).publish(period);
            requestedAction = nt.getStringTopic(root + "/RequestedAction").publish();
            requestedActionType = nt.getStringTopic(root + "/RequestedActionType").publish();
            scheduledActionType = nt.getStringTopic(root + "/ScheduledActionType").publish();
            metadata = nt.getStringArrayTopic(root + "/Metadata").publish();
        }

        private void publish(MechanismTraceSnapshot trace, Profile profile) {
            double periodSeconds = profile == Profile.CAPTURE ? PERIOD_SECONDS : 0.10;
            double elapsed = trace.timestampSeconds() - lastPublishSeconds;
            if (elapsed >= 0.0 && elapsed < periodSeconds) return;
            lastPublishSeconds = trace.timestampSeconds();
            publishLabels(trace);
            state.set(stateFrame(trace));
            if (profile != Profile.CAPTURE) {
                publishMetadataIfChanged();
                return;
            }
            if (candidateFrames.length != trace.candidates().size()) {
                candidateFrames = new CandidateFrame[trace.candidates().size()];
            }
            for (int index = 0; index < candidateFrames.length; index++) {
                MechanismTraceSnapshot.ActionCandidate candidate = trace.candidates().get(index);
                register(candidate.actionType());
                for (String motor : candidate.motors()) register(motor);
                candidateFrames[index] = candidateFrame(candidate);
            }
            candidates.set(candidateFrames);
            if (controlFrames.length != trace.controls().size()) {
                controlFrames = new ControlFrame[trace.controls().size()];
            }
            for (int index = 0; index < controlFrames.length; index++) {
                MechanismTraceSnapshot.Control control = trace.controls().get(index);
                register(control.name());
                register(control.route());
                controlFrames[index] = controlFrame(control);
            }
            controls.set(controlFrames);
            if (motorFrames.length != trace.motors().size()) {
                motorFrames = new MotorFrame[trace.motors().size()];
            }
            for (int index = 0; index < motorFrames.length; index++) {
                MechanismTraceSnapshot.Motor motor = trace.motors().get(index);
                register(motor.name());
                motorFrames[index] = motorFrame(motor);
            }
            motors.set(motorFrames);
            if (hookFrames.length != trace.hooks().size()) {
                hookFrames = new HookFrame[trace.hooks().size()];
            }
            for (int index = 0; index < hookFrames.length; index++) {
                MechanismTraceSnapshot.Hook hook = trace.hooks().get(index);
                register(hook.name());
                hookFrames[index] = hookFrame(hook);
            }
            hooks.set(hookFrames);
            publishMetadataIfChanged();
        }

        private void publishLabels(MechanismTraceSnapshot trace) {
            if (!lastRequestedAction.equals(trace.requestedAction())) {
                lastRequestedAction = trace.requestedAction();
                requestedAction.set(lastRequestedAction);
            }
            if (!lastRequestedActionType.equals(trace.requestedActionType())) {
                lastRequestedActionType = trace.requestedActionType();
                requestedActionType.set(lastRequestedActionType);
            }
            if (!lastScheduledActionType.equals(trace.scheduledActionType())) {
                lastScheduledActionType = trace.scheduledActionType();
                scheduledActionType.set(lastScheduledActionType);
            }
            register(trace.requestedAction());
            register(trace.requestedActionType());
            register(trace.scheduledActionType());
        }

        private void register(String value) {
            if (value != null && !value.isBlank()) {
                names.putIfAbsent(code(value), value);
            }
        }

        private void publishMetadataIfChanged() {
            if (publishedMetadataSize == names.size()) {
                return;
            }
            List<String> values = new ArrayList<>(names.size());
            names.forEach((key, value) -> values.add(key + "=" + value));
            metadata.set(values.toArray(String[]::new));
            publishedMetadataSize = names.size();
        }

        private static int flags(boolean constrained, boolean blocked, boolean saturated) {
            return (constrained ? 1 : 0) | (blocked ? 2 : 0) | (saturated ? 4 : 0);
        }

        @Override
        public void close() {
            state.close();
            candidates.close();
            controls.close();
            motors.close();
            hooks.close();
            requestedAction.close();
            requestedActionType.close();
            scheduledActionType.close();
            metadata.close();
        }
    }

    /** Compact mechanism-level state. */
    public record StateFrame(
            double timestampSeconds,
            double timeInStateSeconds,
            int schedulerStep,
            int activeLeaseCount,
            boolean enabled,
            boolean schedulerComplete) {
        public static final Struct<StateFrame> STRUCT = new FixedStruct<>(
                StateFrame.class,
                "AthenaMechanismState",
                26,
                "double timestampSeconds;double timeInStateSeconds;int32 schedulerStep;int32 activeLeaseCount;bool enabled;bool schedulerComplete",
                buffer -> new StateFrame(
                        buffer.getDouble(), buffer.getDouble(), buffer.getInt(), buffer.getInt(),
                        getBool(buffer), getBool(buffer)),
                (buffer, value) -> {
                    buffer.putDouble(value.timestampSeconds).putDouble(value.timeInStateSeconds)
                            .putInt(value.schedulerStep).putInt(value.activeLeaseCount);
                    putBool(buffer, value.enabled);
                    putBool(buffer, value.schedulerComplete);
                });
    }

    /** Compact arbitration candidate. */
    public record CandidateFrame(
            int actionType,
            int motor,
            long recency,
            int order,
            double requestedValue,
            byte source,
            byte outputMode,
            boolean selected,
            int decision) {
        public CandidateFrame(
                int actionType, int motor, long recency, int order, double requestedValue,
                byte source, byte outputMode, boolean selected) {
            this(actionType, motor, recency, order, requestedValue, source, outputMode, selected, 0);
        }

        public static final Struct<CandidateFrame> STRUCT = new FixedStruct<>(
                CandidateFrame.class,
                "AthenaActionCandidate",
                35,
                "int32 actionType;int32 motor;int64 recency;int32 order;double requestedValue;int8 source;int8 outputMode;bool selected;int32 decision",
                buffer -> new CandidateFrame(buffer.getInt(), buffer.getInt(), buffer.getLong(), buffer.getInt(),
                        buffer.getDouble(), buffer.get(), buffer.get(), getBool(buffer), buffer.getInt()),
                (buffer, value) -> {
                    buffer.putInt(value.actionType).putInt(value.motor).putLong(value.recency).putInt(value.order)
                            .putDouble(value.requestedValue).put(value.source).put(value.outputMode);
                    putBool(buffer, value.selected);
                    buffer.putInt(value.decision);
                });
    }

    /** Compact control-pipeline result. */
    public record ControlFrame(
            int name,
            byte requestedMode,
            byte appliedMode,
            byte flags,
            int route,
            double requestedValue,
            double transformedValue,
            double goal,
            double referencePosition,
            double referenceVelocity,
            double referenceAcceleration,
            double measuredPosition,
            double measuredVelocity,
            double error,
            double proportionalVolts,
            double integralVolts,
            double derivativeVolts,
            double staticFeedforwardVolts,
            double velocityFeedforwardVolts,
            double accelerationFeedforwardVolts,
            double gravityFeedforwardVolts,
            double appliedValue) {
        public static final Struct<ControlFrame> STRUCT = new FixedStruct<>(
                ControlFrame.class,
                "AthenaControlTrace",
                147,
                "int32 name;int8 requestedMode;int8 appliedMode;int8 flags;int32 route;double requestedValue;double transformedValue;double goal;double referencePosition;double referenceVelocity;double referenceAcceleration;double measuredPosition;double measuredVelocity;double error;double proportionalVolts;double integralVolts;double derivativeVolts;double staticFeedforwardVolts;double velocityFeedforwardVolts;double accelerationFeedforwardVolts;double gravityFeedforwardVolts;double appliedValue",
                buffer -> new ControlFrame(
                        buffer.getInt(), buffer.get(), buffer.get(), buffer.get(), buffer.getInt(),
                        buffer.getDouble(), buffer.getDouble(), buffer.getDouble(), buffer.getDouble(),
                        buffer.getDouble(), buffer.getDouble(), buffer.getDouble(), buffer.getDouble(),
                        buffer.getDouble(), buffer.getDouble(), buffer.getDouble(), buffer.getDouble(),
                        buffer.getDouble(), buffer.getDouble(), buffer.getDouble(), buffer.getDouble(),
                        buffer.getDouble()),
                (buffer, value) -> buffer.putInt(value.name).put(value.requestedMode).put(value.appliedMode)
                        .put(value.flags).putInt(value.route).putDouble(value.requestedValue)
                        .putDouble(value.transformedValue).putDouble(value.goal)
                        .putDouble(value.referencePosition).putDouble(value.referenceVelocity)
                        .putDouble(value.referenceAcceleration).putDouble(value.measuredPosition)
                        .putDouble(value.measuredVelocity).putDouble(value.error)
                        .putDouble(value.proportionalVolts).putDouble(value.integralVolts)
                        .putDouble(value.derivativeVolts).putDouble(value.staticFeedforwardVolts)
                        .putDouble(value.velocityFeedforwardVolts).putDouble(value.accelerationFeedforwardVolts)
                        .putDouble(value.gravityFeedforwardVolts).putDouble(value.appliedValue));
    }

    /** Compact cached motor state. */
    public record MotorFrame(
            int name,
            byte commandMode,
            double commandValue,
            double positionRotations,
            double velocityRotationsPerSecond,
            double appliedVoltage,
            double supplyCurrentAmps,
            double statorCurrentAmps,
            byte flags,
            int failure) {
        public MotorFrame(
                int name, byte commandMode, double commandValue, double positionRotations,
                double velocityRotationsPerSecond, double appliedVoltage,
                double supplyCurrentAmps, double statorCurrentAmps) {
            this(name, commandMode, commandValue, positionRotations, velocityRotationsPerSecond,
                    appliedVoltage, supplyCurrentAmps, statorCurrentAmps, (byte) 0, 0);
        }

        public static final Struct<MotorFrame> STRUCT = new FixedStruct<>(
                MotorFrame.class,
                "AthenaMotorTrace",
                58,
                "int32 name;int8 commandMode;double commandValue;double positionRotations;double velocityRotationsPerSecond;double appliedVoltage;double supplyCurrentAmps;double statorCurrentAmps;int8 flags;int32 failure",
                buffer -> new MotorFrame(buffer.getInt(), buffer.get(), buffer.getDouble(), buffer.getDouble(),
                        buffer.getDouble(), buffer.getDouble(), buffer.getDouble(), buffer.getDouble(),
                        buffer.get(), buffer.getInt()),
                (buffer, value) -> buffer.putInt(value.name).put(value.commandMode).putDouble(value.commandValue)
                        .putDouble(value.positionRotations).putDouble(value.velocityRotationsPerSecond)
                        .putDouble(value.appliedVoltage).putDouble(value.supplyCurrentAmps)
                        .putDouble(value.statorCurrentAmps).put(value.flags).putInt(value.failure));
    }

    /** Compact event-hook state. */
    public record HookFrame(int name, boolean sourceActive, boolean active, boolean triggeredThisCycle) {
        public static final Struct<HookFrame> STRUCT = new FixedStruct<>(
                HookFrame.class,
                "AthenaHookTrace",
                7,
                "int32 name;bool sourceActive;bool active;bool triggeredThisCycle",
                buffer -> new HookFrame(buffer.getInt(), getBool(buffer), getBool(buffer), getBool(buffer)),
                (buffer, value) -> {
                    buffer.putInt(value.name);
                    putBool(buffer, value.sourceActive);
                    putBool(buffer, value.active);
                    putBool(buffer, value.triggeredThisCycle);
                });
    }

    private interface Unpacker<T> {
        T unpack(ByteBuffer buffer);
    }

    private interface Packer<T> {
        void pack(ByteBuffer buffer, T value);
    }

    private record FixedStruct<T>(
            Class<T> getTypeClass,
            String getTypeName,
            int getSize,
            String getSchema,
            Unpacker<T> unpacker,
            Packer<T> packer) implements Struct<T> {
        @Override
        public T unpack(ByteBuffer buffer) {
            return unpacker.unpack(buffer);
        }

        @Override
        public void pack(ByteBuffer buffer, T value) {
            packer.pack(buffer, value);
        }
    }

    private static boolean getBool(ByteBuffer buffer) {
        return buffer.get() != 0;
    }

    private static void putBool(ByteBuffer buffer, boolean value) {
        buffer.put((byte) (value ? 1 : 0));
    }
}
