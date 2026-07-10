package ca.frc6390.athena.hardware.sim;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;
import java.util.Set;

/**
 * One composable simulation declaration.
 *
 * <p>A model may contain provider-backed physics leaves, custom runtime rules, or other models.
 * The simulation session treats all three forms uniformly. Convenience model factories are leaves;
 * they are not separate runtime systems.</p>
 */
public final class SimModel {
    private final BuiltIn kind;
    private final OptionalDouble momentOfInertia;
    private final OptionalDouble lengthMeters;
    private final GearRatio gearRatio;
    private final boolean simulatesGravity;
    private final List<MotorDevice> directMotors;
    private final List<EncoderDevice> directEncoders;
    private final Range range;
    private final List<SimLimit> directLimits;
    private final List<DigitalInputDevice> directDigitalInputs;
    private final List<SimModel> children;
    private final RuntimeFactory runtimeFactory;

    private SimModel(
            BuiltIn kind,
            OptionalDouble momentOfInertia,
            OptionalDouble lengthMeters,
            GearRatio gearRatio,
            boolean simulatesGravity,
            List<MotorDevice> motors,
            List<EncoderDevice> encoders,
            Range range,
            List<SimLimit> limits,
            List<DigitalInputDevice> digitalInputs,
            List<SimModel> children,
            RuntimeFactory runtimeFactory) {
        this.kind = kind;
        this.momentOfInertia = momentOfInertia == null ? OptionalDouble.empty() : momentOfInertia;
        this.lengthMeters = lengthMeters == null ? OptionalDouble.empty() : lengthMeters;
        this.gearRatio = gearRatio;
        this.simulatesGravity = simulatesGravity;
        directMotors = motors == null ? List.of() : List.copyOf(motors);
        directEncoders = encoders == null ? List.of() : List.copyOf(encoders);
        this.range = range;
        directLimits = limits == null ? List.of() : List.copyOf(limits);
        directDigitalInputs = digitalInputs == null ? List.of() : List.copyOf(digitalInputs);
        this.children = children == null ? List.of() : List.copyOf(children);
        this.runtimeFactory = runtimeFactory;
    }

    /** Creates a provider-backed motor axis. */
    public static SimModel motor(MotorDevice... motors) {
        return leaf(BuiltIn.MOTOR, false).motors(motors);
    }

    /** Creates a provider-backed flywheel axis. */
    public static SimModel flywheel(MotorDevice... motors) {
        return leaf(BuiltIn.FLYWHEEL, false).motors(motors);
    }

    /** Creates a provider-backed arm axis. */
    public static SimModel arm(MotorDevice... motors) {
        return leaf(BuiltIn.ARM, true).motors(motors);
    }

    /** Creates a provider-backed elevator axis. */
    public static SimModel elevator(MotorDevice... motors) {
        return leaf(BuiltIn.ELEVATOR, true).motors(motors);
    }

    /**
     * Creates an empty unified-model builder for custom behavior and composition.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Composes models into one declaration. Null values are ignored.
     */
    public static SimModel compose(SimModel... models) {
        return compose(models == null ? List.of() : Arrays.asList(models));
    }

    /** Composes models into one declaration. */
    public static SimModel compose(Collection<? extends SimModel> models) {
        Builder builder = builder();
        if (models != null) {
            models.forEach(builder::include);
        }
        return builder.build();
    }

    /** Returns a composition containing this model followed by another model. */
    public SimModel and(SimModel other) {
        return compose(this, Objects.requireNonNull(other, "other"));
    }

    /**
     * Compatibility constructor for provider-backed leaves.
     */
    public static SimModel of(BuiltIn kind) {
        return leaf(kind == null ? BuiltIn.MOTOR : kind, false);
    }

    private static SimModel leaf(BuiltIn kind, boolean gravity) {
        return new SimModel(
                Objects.requireNonNull(kind, "kind"),
                OptionalDouble.empty(),
                OptionalDouble.empty(),
                null,
                gravity,
                List.of(),
                List.of(),
                null,
                List.of(),
                List.of(),
                List.of(),
                null);
    }

    public SimModel momentOfInertia(double value) {
        return copy(OptionalDouble.of(value), lengthMeters, gearRatio, simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel lengthMeters(double value) {
        return copy(momentOfInertia, OptionalDouble.of(value), gearRatio, simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel gearRatio(GearRatio value) {
        return copy(momentOfInertia, lengthMeters, Objects.requireNonNull(value, "gearRatio"), simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel gravity(boolean value) {
        return copy(momentOfInertia, lengthMeters, gearRatio, value,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel withMotor(MotorDevice motor) {
        List<MotorDevice> updated = new ArrayList<>(directMotors);
        updated.add(Objects.requireNonNull(motor, "motor"));
        return copy(momentOfInertia, lengthMeters, gearRatio, simulatesGravity,
                updated, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel motors(MotorDevice... motors) {
        SimModel updated = this;
        if (motors != null) {
            for (MotorDevice motor : motors) {
                updated = updated.withMotor(motor);
            }
        }
        return updated;
    }

    public SimModel encoder(EncoderDevice encoder) {
        List<EncoderDevice> updated = new ArrayList<>(directEncoders);
        updated.add(Objects.requireNonNull(encoder, "encoder"));
        return copy(momentOfInertia, lengthMeters, gearRatio, simulatesGravity,
                directMotors, updated, range, directLimits, directDigitalInputs);
    }

    public SimModel range(Range range) {
        return copy(momentOfInertia, lengthMeters, gearRatio, simulatesGravity,
                directMotors, directEncoders, Objects.requireNonNull(range, "range"), directLimits, directDigitalInputs);
    }

    public SimModel limit(DigitalInputDevice sensor, double position) {
        return limit(sensor, position, 0.25);
    }

    public SimModel limit(DigitalInputDevice sensor, double position, double tolerance) {
        List<SimLimit> updated = new ArrayList<>(directLimits);
        updated.add(new SimLimit(sensor, position, tolerance));
        return copy(momentOfInertia, lengthMeters, gearRatio, simulatesGravity,
                directMotors, directEncoders, range, updated, directDigitalInputs);
    }

    private SimModel copy(
            OptionalDouble momentOfInertia,
            OptionalDouble lengthMeters,
            GearRatio gearRatio,
            boolean gravity,
            List<MotorDevice> motors,
            List<EncoderDevice> encoders,
            Range range,
            List<SimLimit> limits,
            List<DigitalInputDevice> digitalInputs) {
        if (kind == null) {
            throw new IllegalStateException("Provider-backed physics options can only be applied to a leaf model.");
        }
        return new SimModel(kind, momentOfInertia, lengthMeters, gearRatio, gravity,
                motors, encoders, range, limits, digitalInputs, children, runtimeFactory);
    }

    /** Returns all motors claimed by this model and its children. */
    public List<MotorDevice> motors() {
        LinkedHashSet<MotorDevice> values = new LinkedHashSet<>(directMotors);
        children.forEach(child -> values.addAll(child.motors()));
        return List.copyOf(values);
    }

    /** Returns all encoders claimed by this model and its children. */
    public List<EncoderDevice> encoders() {
        LinkedHashSet<EncoderDevice> values = new LinkedHashSet<>(directEncoders);
        children.forEach(child -> values.addAll(child.encoders()));
        return List.copyOf(values);
    }

    /** Returns the motion range for a provider-backed leaf, when declared. */
    public Range range() {
        return range;
    }

    /** Returns typed simulated limits declared by this model and its children. */
    public List<SimLimit> limits() {
        List<SimLimit> values = new ArrayList<>(directLimits);
        children.forEach(child -> values.addAll(child.limits()));
        return List.copyOf(values);
    }

    /** Returns digital inputs claimed directly or through typed limit declarations. */
    public List<DigitalInputDevice> digitalInputs() {
        LinkedHashSet<DigitalInputDevice> values = new LinkedHashSet<>(directDigitalInputs);
        directLimits.forEach(limit -> values.add(limit.sensor()));
        children.forEach(child -> values.addAll(child.digitalInputs()));
        return List.copyOf(values);
    }

    /** Returns the provider-backed physics leaves contained by this model. */
    public List<SimModel> physicsLeaves() {
        List<SimModel> leaves = new ArrayList<>();
        collectPhysicsLeaves(leaves);
        return List.copyOf(leaves);
    }

    private void collectPhysicsLeaves(List<SimModel> leaves) {
        if (kind != null) {
            leaves.add(this);
        }
        children.forEach(child -> child.collectPhysicsLeaves(leaves));
    }

    /** Binds custom runtime rules for one simulation session. */
    public List<Runtime> bind(Context context) {
        Objects.requireNonNull(context, "context");
        List<Runtime> runtimes = new ArrayList<>();
        if (runtimeFactory != null) {
            Runtime runtime = runtimeFactory.bind(context);
            if (runtime != null) {
                runtimes.add(runtime);
            }
        }
        children.forEach(child -> runtimes.addAll(child.bind(context)));
        return List.copyOf(runtimes);
    }

    // Provider-facing leaf properties. Compositions do not expose a synthetic kind.
    public BuiltIn kind() {
        return kind;
    }

    /** Provider-backed leaf implementations available through this unified model API. */
    public enum BuiltIn {
        MOTOR,
        ARM,
        FLYWHEEL,
        ELEVATOR
    }

    public OptionalDouble momentOfInertia() {
        return momentOfInertia;
    }

    public OptionalDouble lengthMeters() {
        return lengthMeters;
    }

    public GearRatio gearRatio() {
        return gearRatio;
    }

    public boolean simulatesGravity() {
        return simulatesGravity;
    }

    /** A declaration that can contribute a model without requiring user-side sim setup. */
    public interface Source {
        SimModel simulationModel();
    }

    /** Per-session simulation access used by custom model runtimes. */
    public interface Context {
        MotorCommand command(MotorDevice motor);

        double motorPosition(MotorDevice motor);

        double motorVelocity(MotorDevice motor);

        void motorState(MotorDevice motor, double position, double velocity);

        double encoderPosition(EncoderDevice encoder);

        double encoderAbsolutePosition(EncoderDevice encoder);

        double encoderVelocity(EncoderDevice encoder);

        void encoderState(EncoderDevice encoder, double position, double absolutePosition, double velocity);

        void digitalInput(DigitalInputDevice input, boolean active);

        PoseSnapshot pose();

        void advancePose(double fieldXVelocity, double fieldYVelocity, double angularVelocity, double seconds);
    }

    /** Last command applied to a simulated motor. */
    public record MotorCommand(CommandMode mode, double value) {
        public MotorCommand {
            mode = mode == null ? CommandMode.NEUTRAL : mode;
            value = Double.isFinite(value) ? value : 0.0;
        }
    }

    public enum CommandMode {
        NEUTRAL,
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    @FunctionalInterface
    public interface RuntimeFactory {
        Runtime bind(Context context);
    }

    @FunctionalInterface
    public interface Runtime {
        void step(double seconds);
    }

    /** Builder for custom rules and composed models. */
    public static final class Builder {
        private final Set<MotorDevice> motors = new LinkedHashSet<>();
        private final Set<EncoderDevice> encoders = new LinkedHashSet<>();
        private final Set<DigitalInputDevice> digitalInputs = new LinkedHashSet<>();
        private final List<SimModel> children = new ArrayList<>();
        private RuntimeFactory runtimeFactory;

        public Builder motor(MotorDevice motor) {
            motors.add(Objects.requireNonNull(motor, "motor"));
            return this;
        }

        public Builder motors(MotorDevice... motors) {
            if (motors != null) {
                Arrays.stream(motors).forEach(this::motor);
            }
            return this;
        }

        public Builder encoder(EncoderDevice encoder) {
            encoders.add(Objects.requireNonNull(encoder, "encoder"));
            return this;
        }

        public Builder digitalInput(DigitalInputDevice input) {
            digitalInputs.add(Objects.requireNonNull(input, "input"));
            return this;
        }

        public Builder include(SimModel model) {
            if (model != null) {
                children.add(model);
            }
            return this;
        }

        public Builder runtime(RuntimeFactory runtimeFactory) {
            this.runtimeFactory = Objects.requireNonNull(runtimeFactory, "runtimeFactory");
            return this;
        }

        public SimModel build() {
            return new SimModel(
                    null,
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    null,
                    false,
                    List.copyOf(motors),
                    List.copyOf(encoders),
                    null,
                    List.of(),
                    List.copyOf(digitalInputs),
                    List.copyOf(children),
                    runtimeFactory);
        }
    }
}
