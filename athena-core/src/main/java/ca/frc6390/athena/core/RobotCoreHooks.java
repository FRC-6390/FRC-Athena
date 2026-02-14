package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.input.TypedInputRegistration;
import ca.frc6390.athena.mechanisms.OutputType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Immutable hook/input registry for {@link RobotCore} lifecycle phases.
 *
 * <p>Use {@link #hooks(Consumer)} and {@link #inputs(Consumer)} from
 * {@link RobotCore.RobotCoreConfig} to register callbacks and external values.
 *
 * @param <T> drivetrain type
 */
public final class RobotCoreHooks<T extends RobotDrivetrain<T>> {

    public enum Phase {
        ROBOT_INIT,
        ROBOT_PERIODIC,
        DISABLED_INIT,
        DISABLED_PERIODIC,
        DISABLED_EXIT,
        TELEOP_INIT,
        TELEOP_PERIODIC,
        TELEOP_EXIT,
        AUTONOMOUS_INIT,
        AUTONOMOUS_PERIODIC,
        AUTONOMOUS_EXIT,
        TEST_INIT,
        TEST_PERIODIC,
        TEST_EXIT,
    }

    @FunctionalInterface
    public interface Binding<T extends RobotDrivetrain<T>> {
        void apply(RobotCoreContext<T> context);
    }

    @FunctionalInterface
    public interface ControlLoop<T extends RobotDrivetrain<T>> {
        double calculate(RobotCoreContext<T> context);
    }

    public record PeriodicHookBinding<T extends RobotDrivetrain<T>>(
            Binding<T> hook,
            double periodMs) {
        public PeriodicHookBinding {
            Objects.requireNonNull(hook, "hook");
            if (!Double.isFinite(periodMs) || periodMs < 0.0) {
                throw new IllegalArgumentException("periodMs must be finite and >= 0");
            }
        }
    }

    public record ControlLoopBinding<T extends RobotDrivetrain<T>>(
            String name,
            double periodSeconds,
            ControlLoop<T> loop) {
        public ControlLoopBinding {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(loop, "loop");
            if (name.isBlank()) {
                throw new IllegalArgumentException("control loop name cannot be blank");
            }
            if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
                throw new IllegalArgumentException("control loop periodSeconds must be finite and > 0");
            }
        }
    }

    public record PidProfile(OutputType outputType, double kP, double kI, double kD, double iZone, double tolerance) {}

    public record FeedforwardProfile(OutputType outputType, SimpleMotorFeedforward feedforward) {
        public FeedforwardProfile {
            Objects.requireNonNull(feedforward, "feedforward");
        }
    }

    private final Map<String, BooleanSupplier> inputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;
    private final List<ControlLoopBinding<T>> controlLoopBindings;
    private final Map<String, PidProfile> pidProfiles;
    private final Map<String, FeedforwardProfile> feedforwardProfiles;

    private final List<Binding<T>> initBindings;
    private final List<Binding<T>> periodicBindings;
    private final List<PeriodicHookBinding<T>> periodicLoopBindings;
    private final List<Binding<T>> exitBindings;

    private final List<Binding<T>> disabledInitBindings;
    private final List<Binding<T>> disabledPeriodicBindings;
    private final List<PeriodicHookBinding<T>> disabledPeriodicLoopBindings;
    private final List<Binding<T>> disabledExitBindings;

    private final List<Binding<T>> teleopInitBindings;
    private final List<Binding<T>> teleopPeriodicBindings;
    private final List<PeriodicHookBinding<T>> teleopPeriodicLoopBindings;
    private final List<Binding<T>> teleopExitBindings;

    private final List<Binding<T>> autonomousInitBindings;
    private final List<Binding<T>> autonomousPeriodicBindings;
    private final List<PeriodicHookBinding<T>> autonomousPeriodicLoopBindings;
    private final List<Binding<T>> autonomousExitBindings;

    private final List<Binding<T>> testInitBindings;
    private final List<Binding<T>> testPeriodicBindings;
    private final List<PeriodicHookBinding<T>> testPeriodicLoopBindings;
    private final List<Binding<T>> testExitBindings;

    private RobotCoreHooks(Builder<T> builder) {
        this.inputs = Map.copyOf(builder.inputs);
        this.doubleInputs = Map.copyOf(builder.doubleInputs);
        this.intInputs = Map.copyOf(builder.intInputs);
        this.stringInputs = Map.copyOf(builder.stringInputs);
        this.pose2dInputs = Map.copyOf(builder.pose2dInputs);
        this.pose3dInputs = Map.copyOf(builder.pose3dInputs);
        this.objectInputs = Map.copyOf(builder.objectInputs);
        this.controlLoopBindings = List.copyOf(builder.controlLoopBindings);
        this.pidProfiles = Map.copyOf(builder.pidProfiles);
        this.feedforwardProfiles = Map.copyOf(builder.feedforwardProfiles);

        this.initBindings = List.copyOf(builder.initBindings);
        this.periodicBindings = List.copyOf(builder.periodicBindings);
        this.periodicLoopBindings = List.copyOf(builder.periodicLoopBindings);
        this.exitBindings = List.copyOf(builder.exitBindings);

        this.disabledInitBindings = List.copyOf(builder.disabledInitBindings);
        this.disabledPeriodicBindings = List.copyOf(builder.disabledPeriodicBindings);
        this.disabledPeriodicLoopBindings = List.copyOf(builder.disabledPeriodicLoopBindings);
        this.disabledExitBindings = List.copyOf(builder.disabledExitBindings);

        this.teleopInitBindings = List.copyOf(builder.teleopInitBindings);
        this.teleopPeriodicBindings = List.copyOf(builder.teleopPeriodicBindings);
        this.teleopPeriodicLoopBindings = List.copyOf(builder.teleopPeriodicLoopBindings);
        this.teleopExitBindings = List.copyOf(builder.teleopExitBindings);

        this.autonomousInitBindings = List.copyOf(builder.autonomousInitBindings);
        this.autonomousPeriodicBindings = List.copyOf(builder.autonomousPeriodicBindings);
        this.autonomousPeriodicLoopBindings = List.copyOf(builder.autonomousPeriodicLoopBindings);
        this.autonomousExitBindings = List.copyOf(builder.autonomousExitBindings);

        this.testInitBindings = List.copyOf(builder.testInitBindings);
        this.testPeriodicBindings = List.copyOf(builder.testPeriodicBindings);
        this.testPeriodicLoopBindings = List.copyOf(builder.testPeriodicLoopBindings);
        this.testExitBindings = List.copyOf(builder.testExitBindings);
    }

    public static <T extends RobotDrivetrain<T>> RobotCoreHooks<T> empty() {
        return new Builder<T>().build();
    }

    public RobotCoreHooks<T> hooks(Consumer<HooksSection<T>> section) {
        if (section == null) {
            return this;
        }
        Builder<T> copy = toBuilder();
        section.accept(new HooksSection<>(copy));
        return copy.build();
    }

    public RobotCoreHooks<T> inputs(Consumer<InputsSection<T>> section) {
        if (section == null) {
            return this;
        }
        Builder<T> copy = toBuilder();
        section.accept(new InputsSection<>(copy));
        return copy.build();
    }

    private Builder<T> toBuilder() {
        Builder<T> builder = new Builder<>();
        builder.inputs.putAll(inputs);
        builder.doubleInputs.putAll(doubleInputs);
        builder.intInputs.putAll(intInputs);
        builder.stringInputs.putAll(stringInputs);
        builder.pose2dInputs.putAll(pose2dInputs);
        builder.pose3dInputs.putAll(pose3dInputs);
        builder.objectInputs.putAll(objectInputs);
        builder.controlLoopBindings.addAll(controlLoopBindings);
        builder.pidProfiles.putAll(pidProfiles);
        builder.feedforwardProfiles.putAll(feedforwardProfiles);

        builder.initBindings.addAll(initBindings);
        builder.periodicBindings.addAll(periodicBindings);
        builder.periodicLoopBindings.addAll(periodicLoopBindings);
        builder.exitBindings.addAll(exitBindings);

        builder.disabledInitBindings.addAll(disabledInitBindings);
        builder.disabledPeriodicBindings.addAll(disabledPeriodicBindings);
        builder.disabledPeriodicLoopBindings.addAll(disabledPeriodicLoopBindings);
        builder.disabledExitBindings.addAll(disabledExitBindings);

        builder.teleopInitBindings.addAll(teleopInitBindings);
        builder.teleopPeriodicBindings.addAll(teleopPeriodicBindings);
        builder.teleopPeriodicLoopBindings.addAll(teleopPeriodicLoopBindings);
        builder.teleopExitBindings.addAll(teleopExitBindings);

        builder.autonomousInitBindings.addAll(autonomousInitBindings);
        builder.autonomousPeriodicBindings.addAll(autonomousPeriodicBindings);
        builder.autonomousPeriodicLoopBindings.addAll(autonomousPeriodicLoopBindings);
        builder.autonomousExitBindings.addAll(autonomousExitBindings);

        builder.testInitBindings.addAll(testInitBindings);
        builder.testPeriodicBindings.addAll(testPeriodicBindings);
        builder.testPeriodicLoopBindings.addAll(testPeriodicLoopBindings);
        builder.testExitBindings.addAll(testExitBindings);
        return builder;
    }

    public Map<String, BooleanSupplier> inputs() {
        return inputs;
    }

    public Map<String, DoubleSupplier> doubleInputs() {
        return doubleInputs;
    }

    public Map<String, IntSupplier> intInputs() {
        return intInputs;
    }

    public Map<String, Supplier<String>> stringInputs() {
        return stringInputs;
    }

    public Map<String, Supplier<Pose2d>> pose2dInputs() {
        return pose2dInputs;
    }

    public Map<String, Supplier<Pose3d>> pose3dInputs() {
        return pose3dInputs;
    }

    public Map<String, Supplier<?>> objectInputs() {
        return objectInputs;
    }

    public List<ControlLoopBinding<T>> controlLoopBindings() {
        return controlLoopBindings;
    }

    public Map<String, PidProfile> pidProfiles() {
        return pidProfiles;
    }

    public Map<String, FeedforwardProfile> feedforwardProfiles() {
        return feedforwardProfiles;
    }

    public List<Binding<T>> initBindings() {
        return initBindings;
    }

    public List<Binding<T>> periodicBindings() {
        return periodicBindings;
    }

    public List<PeriodicHookBinding<T>> periodicLoopBindings() {
        return periodicLoopBindings;
    }

    public List<Binding<T>> exitBindings() {
        return exitBindings;
    }

    public List<Binding<T>> disabledInitBindings() {
        return disabledInitBindings;
    }

    public List<Binding<T>> disabledPeriodicBindings() {
        return disabledPeriodicBindings;
    }

    public List<PeriodicHookBinding<T>> disabledPeriodicLoopBindings() {
        return disabledPeriodicLoopBindings;
    }

    public List<Binding<T>> disabledExitBindings() {
        return disabledExitBindings;
    }

    public List<Binding<T>> teleopInitBindings() {
        return teleopInitBindings;
    }

    public List<Binding<T>> teleopPeriodicBindings() {
        return teleopPeriodicBindings;
    }

    public List<PeriodicHookBinding<T>> teleopPeriodicLoopBindings() {
        return teleopPeriodicLoopBindings;
    }

    public List<Binding<T>> teleopExitBindings() {
        return teleopExitBindings;
    }

    public List<Binding<T>> autonomousInitBindings() {
        return autonomousInitBindings;
    }

    public List<Binding<T>> autonomousPeriodicBindings() {
        return autonomousPeriodicBindings;
    }

    public List<PeriodicHookBinding<T>> autonomousPeriodicLoopBindings() {
        return autonomousPeriodicLoopBindings;
    }

    public List<Binding<T>> autonomousExitBindings() {
        return autonomousExitBindings;
    }

    public List<Binding<T>> testInitBindings() {
        return testInitBindings;
    }

    public List<Binding<T>> testPeriodicBindings() {
        return testPeriodicBindings;
    }

    public List<PeriodicHookBinding<T>> testPeriodicLoopBindings() {
        return testPeriodicLoopBindings;
    }

    public List<Binding<T>> testExitBindings() {
        return testExitBindings;
    }

    public static final class HooksSection<T extends RobotDrivetrain<T>> {
        private final Builder<T> owner;

        private HooksSection(Builder<T> owner) {
            this.owner = owner;
        }

        private HooksSection<T> onPhase(Phase phase, Binding<T> binding) {
            Objects.requireNonNull(phase, "phase");
            Objects.requireNonNull(binding, "binding");
            switch (phase) {
                case ROBOT_INIT -> owner.initBindings.add(binding);
                case ROBOT_PERIODIC -> owner.periodicBindings.add(binding);
                case DISABLED_INIT -> owner.disabledInitBindings.add(binding);
                case DISABLED_PERIODIC -> owner.disabledPeriodicBindings.add(binding);
                case DISABLED_EXIT -> owner.disabledExitBindings.add(binding);
                case TELEOP_INIT -> owner.teleopInitBindings.add(binding);
                case TELEOP_PERIODIC -> owner.teleopPeriodicBindings.add(binding);
                case TELEOP_EXIT -> owner.teleopExitBindings.add(binding);
                case AUTONOMOUS_INIT -> owner.autonomousInitBindings.add(binding);
                case AUTONOMOUS_PERIODIC -> owner.autonomousPeriodicBindings.add(binding);
                case AUTONOMOUS_EXIT -> owner.autonomousExitBindings.add(binding);
                case TEST_INIT -> owner.testInitBindings.add(binding);
                case TEST_PERIODIC -> owner.testPeriodicBindings.add(binding);
                case TEST_EXIT -> owner.testExitBindings.add(binding);
            }
            return this;
        }

        private HooksSection<T> onPhasePeriodic(Phase phase, Binding<T> binding, double periodMs) {
            Objects.requireNonNull(phase, "phase");
            Objects.requireNonNull(binding, "binding");
            switch (phase) {
                case ROBOT_PERIODIC -> owner.periodicLoopBindings.add(new PeriodicHookBinding<>(binding, periodMs));
                case DISABLED_PERIODIC ->
                        owner.disabledPeriodicLoopBindings.add(new PeriodicHookBinding<>(binding, periodMs));
                case TELEOP_PERIODIC -> owner.teleopPeriodicLoopBindings.add(new PeriodicHookBinding<>(binding, periodMs));
                case AUTONOMOUS_PERIODIC ->
                        owner.autonomousPeriodicLoopBindings.add(new PeriodicHookBinding<>(binding, periodMs));
                case TEST_PERIODIC -> owner.testPeriodicLoopBindings.add(new PeriodicHookBinding<>(binding, periodMs));
                default -> throw new IllegalArgumentException("Phase does not support periodic-loop bindings: " + phase);
            }
            return this;
        }

        public HooksSection<T> onInit(Binding<T> binding) {
            return onPhase(Phase.ROBOT_INIT, binding);
        }

        public HooksSection<T> onPeriodic(Binding<T> binding) {
            return onPhase(Phase.ROBOT_PERIODIC, binding);
        }

        public HooksSection<T> onPeriodic(Binding<T> binding, double periodMs) {
            return onPhasePeriodic(Phase.ROBOT_PERIODIC, binding, periodMs);
        }

        public HooksSection<T> onExit(Binding<T> binding) {
            owner.exitBindings.add(Objects.requireNonNull(binding, "binding"));
            return this;
        }

        public HooksSection<T> onDisabledInit(Binding<T> binding) {
            return onPhase(Phase.DISABLED_INIT, binding);
        }

        public HooksSection<T> onDisabledPeriodic(Binding<T> binding) {
            return onPhase(Phase.DISABLED_PERIODIC, binding);
        }

        public HooksSection<T> onDisabledPeriodic(Binding<T> binding, double periodMs) {
            return onPhasePeriodic(Phase.DISABLED_PERIODIC, binding, periodMs);
        }

        public HooksSection<T> onDisabledExit(Binding<T> binding) {
            return onPhase(Phase.DISABLED_EXIT, binding);
        }

        public HooksSection<T> onTeleopInit(Binding<T> binding) {
            return onPhase(Phase.TELEOP_INIT, binding);
        }

        public HooksSection<T> onTeleInit(Binding<T> binding) {
            return onTeleopInit(binding);
        }

        public HooksSection<T> onTeleopPeriodic(Binding<T> binding) {
            return onPhase(Phase.TELEOP_PERIODIC, binding);
        }

        public HooksSection<T> onTelePeriodic(Binding<T> binding) {
            return onTeleopPeriodic(binding);
        }

        public HooksSection<T> onTeleopPeriodic(Binding<T> binding, double periodMs) {
            return onPhasePeriodic(Phase.TELEOP_PERIODIC, binding, periodMs);
        }

        public HooksSection<T> onTelePeriodic(Binding<T> binding, double periodMs) {
            return onTeleopPeriodic(binding, periodMs);
        }

        public HooksSection<T> onTeleopExit(Binding<T> binding) {
            return onPhase(Phase.TELEOP_EXIT, binding);
        }

        public HooksSection<T> onTeleExit(Binding<T> binding) {
            return onTeleopExit(binding);
        }

        public HooksSection<T> onAutonomousInit(Binding<T> binding) {
            return onPhase(Phase.AUTONOMOUS_INIT, binding);
        }

        public HooksSection<T> onAutoInit(Binding<T> binding) {
            return onAutonomousInit(binding);
        }

        public HooksSection<T> onAutonomousPeriodic(Binding<T> binding) {
            return onPhase(Phase.AUTONOMOUS_PERIODIC, binding);
        }

        public HooksSection<T> onAutoPeriodic(Binding<T> binding) {
            return onAutonomousPeriodic(binding);
        }

        public HooksSection<T> onAutonomousPeriodic(Binding<T> binding, double periodMs) {
            return onPhasePeriodic(Phase.AUTONOMOUS_PERIODIC, binding, periodMs);
        }

        public HooksSection<T> onAutoPeriodic(Binding<T> binding, double periodMs) {
            return onAutonomousPeriodic(binding, periodMs);
        }

        public HooksSection<T> onAutonomousExit(Binding<T> binding) {
            return onPhase(Phase.AUTONOMOUS_EXIT, binding);
        }

        public HooksSection<T> onAutoExit(Binding<T> binding) {
            return onAutonomousExit(binding);
        }

        public HooksSection<T> onTestInit(Binding<T> binding) {
            return onPhase(Phase.TEST_INIT, binding);
        }

        public HooksSection<T> onTestPeriodic(Binding<T> binding) {
            return onPhase(Phase.TEST_PERIODIC, binding);
        }

        public HooksSection<T> onTestPeriodic(Binding<T> binding, double periodMs) {
            return onPhasePeriodic(Phase.TEST_PERIODIC, binding, periodMs);
        }

        public HooksSection<T> onTestExit(Binding<T> binding) {
            return onPhase(Phase.TEST_EXIT, binding);
        }

        public HooksSection<T> controlLoop(String name, double periodMs, ControlLoop<T> loop) {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(loop, "loop");
            if (name.isBlank()) {
                throw new IllegalArgumentException("control loop name cannot be blank");
            }
            if (!Double.isFinite(periodMs) || periodMs <= 0.0) {
                throw new IllegalArgumentException("control loop period must be finite and > 0");
            }
            for (ControlLoopBinding<T> binding : owner.controlLoopBindings) {
                if (binding != null && name.equals(binding.name())) {
                    throw new IllegalArgumentException("control loop name already registered: " + name);
                }
            }
            owner.controlLoopBindings.add(new ControlLoopBinding<>(name, periodMs / 1000.0, loop));
            return this;
        }

        public HooksSection<T> controlLoopSeconds(String name, double periodSeconds, ControlLoop<T> loop) {
            return controlLoop(name, periodSeconds * 1000.0, loop);
        }

        public HooksSection<T> pidProfile(String name, double kP, double kI, double kD) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, Double.NaN, Double.NaN);
        }

        public HooksSection<T> pidProfile(String name, OutputType outputType, double kP, double kI, double kD) {
            return pidProfile(name, outputType, kP, kI, kD, Double.NaN, Double.NaN);
        }

        public HooksSection<T> pidProfile(String name, double kP, double kI, double kD, double iZone) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, iZone, Double.NaN);
        }

        public HooksSection<T> pidProfile(
                String name,
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone) {
            return pidProfile(name, outputType, kP, kI, kD, iZone, Double.NaN);
        }

        public HooksSection<T> pidProfile(
                String name,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, iZone, tolerance);
        }

        public HooksSection<T> pidProfile(
                String name,
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("PID profile name cannot be blank");
            }
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.PERCENT;
            if (resolvedOutput != OutputType.PERCENT && resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("PID profile output type must be PERCENT or VOLTAGE");
            }
            owner.pidProfiles.put(name, new PidProfile(resolvedOutput, kP, kI, kD, iZone, tolerance));
            return this;
        }

        public HooksSection<T> feedforwardProfile(String name, double kS, double kV, double kA) {
            return feedforwardProfile(name, OutputType.VOLTAGE, kS, kV, kA);
        }

        public HooksSection<T> feedforwardProfile(String name, OutputType outputType, double kS, double kV, double kA) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("feedforward profile name cannot be blank");
            }
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.VOLTAGE;
            if (resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("feedforward profile output type must be VOLTAGE");
            }
            owner.feedforwardProfiles.put(
                    name,
                    new FeedforwardProfile(resolvedOutput, new SimpleMotorFeedforward(kS, kV, kA)));
            return this;
        }
    }

    public static final class InputsSection<T extends RobotDrivetrain<T>> {
        private final Builder<T> owner;

        private InputsSection(Builder<T> owner) {
            this.owner = owner;
        }

        public InputsSection<T> boolVal(String key, BooleanSupplier supplier) {
            TypedInputRegistration.put(owner.inputs, key, supplier);
            return this;
        }

        public InputsSection<T> doubleVal(String key, DoubleSupplier supplier) {
            TypedInputRegistration.put(owner.doubleInputs, key, supplier);
            return this;
        }

        public InputsSection<T> intVal(String key, IntSupplier supplier) {
            TypedInputRegistration.put(owner.intInputs, key, supplier);
            return this;
        }

        public InputsSection<T> stringVal(String key, Supplier<String> supplier) {
            TypedInputRegistration.put(owner.stringInputs, key, supplier);
            return this;
        }

        public InputsSection<T> pose2dVal(String key, Supplier<Pose2d> supplier) {
            TypedInputRegistration.put(owner.pose2dInputs, key, supplier);
            return this;
        }

        public InputsSection<T> pose3dVal(String key, Supplier<Pose3d> supplier) {
            TypedInputRegistration.put(owner.pose3dInputs, key, supplier);
            return this;
        }

        public <V> InputsSection<T> objectVal(String key, Supplier<V> supplier) {
            TypedInputRegistration.put(owner.objectInputs, key, supplier);
            return this;
        }
    }

    private static final class Builder<T extends RobotDrivetrain<T>> {
        private final Map<String, BooleanSupplier> inputs = new HashMap<>();
        private final Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
        private final Map<String, IntSupplier> intInputs = new HashMap<>();
        private final Map<String, Supplier<String>> stringInputs = new HashMap<>();
        private final Map<String, Supplier<Pose2d>> pose2dInputs = new HashMap<>();
        private final Map<String, Supplier<Pose3d>> pose3dInputs = new HashMap<>();
        private final Map<String, Supplier<?>> objectInputs = new HashMap<>();
        private final List<ControlLoopBinding<T>> controlLoopBindings = new ArrayList<>();
        private final Map<String, PidProfile> pidProfiles = new HashMap<>();
        private final Map<String, FeedforwardProfile> feedforwardProfiles = new HashMap<>();

        private final List<Binding<T>> initBindings = new ArrayList<>();
        private final List<Binding<T>> periodicBindings = new ArrayList<>();
        private final List<PeriodicHookBinding<T>> periodicLoopBindings = new ArrayList<>();
        private final List<Binding<T>> exitBindings = new ArrayList<>();

        private final List<Binding<T>> disabledInitBindings = new ArrayList<>();
        private final List<Binding<T>> disabledPeriodicBindings = new ArrayList<>();
        private final List<PeriodicHookBinding<T>> disabledPeriodicLoopBindings = new ArrayList<>();
        private final List<Binding<T>> disabledExitBindings = new ArrayList<>();

        private final List<Binding<T>> teleopInitBindings = new ArrayList<>();
        private final List<Binding<T>> teleopPeriodicBindings = new ArrayList<>();
        private final List<PeriodicHookBinding<T>> teleopPeriodicLoopBindings = new ArrayList<>();
        private final List<Binding<T>> teleopExitBindings = new ArrayList<>();

        private final List<Binding<T>> autonomousInitBindings = new ArrayList<>();
        private final List<Binding<T>> autonomousPeriodicBindings = new ArrayList<>();
        private final List<PeriodicHookBinding<T>> autonomousPeriodicLoopBindings = new ArrayList<>();
        private final List<Binding<T>> autonomousExitBindings = new ArrayList<>();

        private final List<Binding<T>> testInitBindings = new ArrayList<>();
        private final List<Binding<T>> testPeriodicBindings = new ArrayList<>();
        private final List<PeriodicHookBinding<T>> testPeriodicLoopBindings = new ArrayList<>();
        private final List<Binding<T>> testExitBindings = new ArrayList<>();

        private RobotCoreHooks<T> build() {
            return new RobotCoreHooks<>(this);
        }
    }
}
