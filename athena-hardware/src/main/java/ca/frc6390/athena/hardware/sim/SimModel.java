package ca.frc6390.athena.hardware.sim;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
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
    private final Optional<MomentOfInertia> momentOfInertia;
    private final Optional<Distance> length;
    private final GearRatio gearRatio;
    private final boolean simulatesGravity;
    private final List<MotorDevice> directMotors;
    private final List<EncoderDevice> directEncoders;
    private final Range range;
    private final List<SimLimit> directLimits;
    private final List<DigitalInputDevice> directDigitalInputs;
    private final List<SimModel> children;
    private final RuntimeFactory runtimeFactory;
    private final SimPlant plant;
    // Aggregates over this immutable model and its children, built on first use. The simulation reads them every
    // step, so rebuilding them would allocate (and rehash every declaration) each cycle.
    private volatile List<MotorDevice> allMotors;
    private volatile List<EncoderDevice> allEncoders;
    private volatile List<SimLimit> allLimits;
    private volatile List<DigitalInputDevice> allDigitalInputs;
    private volatile List<SimModel> allPhysicsLeaves;

    private SimModel(
            BuiltIn kind,
            Optional<MomentOfInertia> momentOfInertia,
            Optional<Distance> length,
            GearRatio gearRatio,
            boolean simulatesGravity,
            List<MotorDevice> motors,
            List<EncoderDevice> encoders,
            Range range,
            List<SimLimit> limits,
            List<DigitalInputDevice> digitalInputs,
            List<SimModel> children,
            RuntimeFactory runtimeFactory,
            SimPlant plant) {
        this.kind = kind;
        this.momentOfInertia = momentOfInertia == null ? Optional.empty() : momentOfInertia;
        this.length = length == null ? Optional.empty() : length;
        this.gearRatio = gearRatio;
        this.simulatesGravity = simulatesGravity;
        directMotors = motors == null ? List.of() : List.copyOf(motors);
        directEncoders = encoders == null ? List.of() : List.copyOf(encoders);
        this.range = range;
        directLimits = limits == null ? List.of() : List.copyOf(limits);
        directDigitalInputs = digitalInputs == null ? List.of() : List.copyOf(digitalInputs);
        this.children = children == null ? List.of() : List.copyOf(children);
        this.runtimeFactory = runtimeFactory;
        this.plant = plant;
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

    private static SimModel leaf(BuiltIn kind, boolean gravity) {
        return new SimModel(
                Objects.requireNonNull(kind, "kind"),
                Optional.empty(),
                Optional.empty(),
                null,
                gravity,
                List.of(),
                List.of(),
                null,
                List.of(),
                List.of(),
                List.of(),
                null,
                null);
    }

    /** Declares the simulated mechanism's moment of inertia about its motion axis. */
    public SimModel momentOfInertia(MomentOfInertia value) {
        return copy(Optional.of(Objects.requireNonNull(value, "momentOfInertia")), length, gearRatio, simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    /**
     * Applies plant parameters from a source, such as an identified mechanism model. Only the
     * parameters the source provides are changed; the rest of the declaration is kept.
     */
    public SimModel model(SimDynamicsSource source) {
        Objects.requireNonNull(source, "source");
        Optional<MomentOfInertia> inertia = source.momentOfInertia();
        SimModel updated = inertia.isPresent() ? momentOfInertia(inertia.get()) : this;
        Optional<SimPlant> identified = source.plant();
        return identified.isPresent() ? updated.withPlant(identified.get()) : updated;
    }

    private SimModel withPlant(SimPlant value) {
        if (kind == null) {
            throw new IllegalStateException("Provider-backed physics options can only be applied to a leaf model.");
        }
        return new SimModel(kind, momentOfInertia, length, gearRatio, simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs, children, runtimeFactory,
                Objects.requireNonNull(value, "plant"));
    }

    /**
     * Returns the identified plant applied by {@link #model(SimDynamicsSource)}. When present, the physics
     * engine simulates this plant (friction, gravity, viscous and inertia from the model) instead of the
     * motor's own dynamics; the declared inertia and gravity flag are then not used.
     */
    public Optional<SimPlant> plant() {
        return Optional.ofNullable(plant);
    }

    /** Declares the simulated arm length, from the pivot to the center of mass. */
    public SimModel length(Distance value) {
        return copy(momentOfInertia, Optional.of(Objects.requireNonNull(value, "length")), gearRatio, simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel gearRatio(GearRatio value) {
        return copy(momentOfInertia, length, Objects.requireNonNull(value, "gearRatio"), simulatesGravity,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel gravity(boolean value) {
        return copy(momentOfInertia, length, gearRatio, value,
                directMotors, directEncoders, range, directLimits, directDigitalInputs);
    }

    public SimModel withMotor(MotorDevice motor) {
        List<MotorDevice> updated = new ArrayList<>(directMotors);
        updated.add(Objects.requireNonNull(motor, "motor"));
        return copy(momentOfInertia, length, gearRatio, simulatesGravity,
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
        return copy(momentOfInertia, length, gearRatio, simulatesGravity,
                directMotors, updated, range, directLimits, directDigitalInputs);
    }

    /**
     * Limits the simulated mechanism's travel. Plants work in mechanism rotations, so a range declared in
     * degrees is converted to rotations; other ranges are used as given.
     */
    public SimModel range(Range range) {
        Objects.requireNonNull(range, "range");
        Range plantRange = range.unit() == Units.Degrees
                ? Range.of(Units.Rotations.of(range.raw().minimumInUnit() / 360.0), Units.Rotations.of(range.raw().maximumInUnit() / 360.0))
                : range;
        return copy(momentOfInertia, length, gearRatio, simulatesGravity,
                directMotors, directEncoders, plantRange, directLimits, directDigitalInputs);
    }

    public SimModel limit(DigitalInputDevice sensor, double position) {
        return limit(sensor, position, 0.25);
    }

    public SimModel limit(DigitalInputDevice sensor, double position, double tolerance) {
        List<SimLimit> updated = new ArrayList<>(directLimits);
        updated.add(new SimLimit(sensor, position, tolerance));
        return copy(momentOfInertia, length, gearRatio, simulatesGravity,
                directMotors, directEncoders, range, updated, directDigitalInputs);
    }

    private SimModel copy(
            Optional<MomentOfInertia> momentOfInertia,
            Optional<Distance> length,
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
        return new SimModel(kind, momentOfInertia, length, gearRatio, gravity,
                motors, encoders, range, limits, digitalInputs, children, runtimeFactory, plant);
    }

    /** Returns all motors claimed by this model and its children. */
    public List<MotorDevice> motors() {
        List<MotorDevice> cached = allMotors;
        if (cached == null) {
            LinkedHashSet<MotorDevice> values = new LinkedHashSet<>(directMotors);
            children.forEach(child -> values.addAll(child.motors()));
            cached = List.copyOf(values);
            allMotors = cached;
        }
        return cached;
    }

    /** Returns all encoders claimed by this model and its children. */
    public List<EncoderDevice> encoders() {
        List<EncoderDevice> cached = allEncoders;
        if (cached == null) {
            LinkedHashSet<EncoderDevice> values = new LinkedHashSet<>(directEncoders);
            children.forEach(child -> values.addAll(child.encoders()));
            cached = List.copyOf(values);
            allEncoders = cached;
        }
        return cached;
    }

    /** Returns the motion range for a provider-backed leaf, when declared. */
    public Range range() {
        return range;
    }

    /** Returns typed simulated limits declared by this model and its children. */
    public List<SimLimit> limits() {
        List<SimLimit> cached = allLimits;
        if (cached == null) {
            List<SimLimit> values = new ArrayList<>(directLimits);
            children.forEach(child -> values.addAll(child.limits()));
            cached = List.copyOf(values);
            allLimits = cached;
        }
        return cached;
    }

    /** Returns digital inputs claimed directly or through typed limit declarations. */
    public List<DigitalInputDevice> digitalInputs() {
        List<DigitalInputDevice> cached = allDigitalInputs;
        if (cached == null) {
            LinkedHashSet<DigitalInputDevice> values = new LinkedHashSet<>(directDigitalInputs);
            directLimits.forEach(limit -> values.add(limit.sensor()));
            children.forEach(child -> values.addAll(child.digitalInputs()));
            cached = List.copyOf(values);
            allDigitalInputs = cached;
        }
        return cached;
    }

    /** Returns the provider-backed physics leaves contained by this model. */
    public List<SimModel> physicsLeaves() {
        List<SimModel> cached = allPhysicsLeaves;
        if (cached == null) {
            List<SimModel> leaves = new ArrayList<>();
            collectPhysicsLeaves(leaves);
            cached = List.copyOf(leaves);
            allPhysicsLeaves = cached;
        }
        return cached;
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

    /** Returns the declared moment of inertia, when present. */
    public Optional<MomentOfInertia> momentOfInertia() {
        return momentOfInertia;
    }

    /** Returns the declared arm length, when present. */
    public Optional<Distance> length() {
        return length;
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

        /** Integrated motor position, as an angle of the motor shaft. */
        Angle motorPosition(MotorDevice motor);

        /** Integrated motor velocity. */
        AngularVelocity motorVelocity(MotorDevice motor);

        void motorState(MotorDevice motor, Angle position, AngularVelocity velocity);

        /** Raw encoder position in sensor revolutions. */
        Angle encoderPosition(EncoderDevice encoder);

        /** Raw absolute encoder position in sensor revolutions. */
        Angle encoderAbsolutePosition(EncoderDevice encoder);

        /** Raw encoder velocity. */
        AngularVelocity encoderVelocity(EncoderDevice encoder);

        void encoderState(EncoderDevice encoder, Angle position, Angle absolutePosition, AngularVelocity velocity);

        void digitalInput(DigitalInputDevice input, boolean active);

        PoseSnapshot pose();

        void advancePose(LinearVelocity fieldXVelocity, LinearVelocity fieldYVelocity,
                AngularVelocity angularVelocity, Time seconds);
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
                    Optional.empty(),
                    Optional.empty(),
                    null,
                    false,
                    List.copyOf(motors),
                    List.copyOf(encoders),
                    null,
                    List.of(),
                    List.copyOf(digitalInputs),
                    List.copyOf(children),
                    runtimeFactory,
                    null);
        }
    }
}
