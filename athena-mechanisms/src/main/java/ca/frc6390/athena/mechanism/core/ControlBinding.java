package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.constraint.Constraint;
import ca.frc6390.athena.mechanism.constraint.UnitConstraint;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.motion.MotionPlanner;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;
import ca.frc6390.athena.mechanism.model.AccelerationEstimator;
import ca.frc6390.athena.mechanism.model.BindingCheck;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.model.Identification;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import ca.frc6390.athena.mechanism.model.ModelFeedforward;
import ca.frc6390.athena.mechanism.model.MonotonicClock;
import ca.frc6390.athena.mechanism.model.SlipCheck;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.mechanism.staging.StageModifier;
import ca.frc6390.athena.mechanism.effort.Effort;
import ca.frc6390.athena.mechanism.effort.EffortLimit;
import ca.frc6390.athena.mechanism.interpolation.CurveMapping;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.interpolation.InterpolationModel;
import ca.frc6390.athena.runtime.control.ControlSink;
import ca.frc6390.athena.hardware.device.Range;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Declarative control binding for one actuator group.
 *
 * <p>The runtime binds each control-loop declaration once per binding identity. Target
 * changes preserve loop and profile state. Neutral, cancellation, loss of arbitration,
 * and disable reset that state without reconstructing the bound loop.</p>
 *
 * <p>The type parameter {@code U} is the WPILib unit the binding's plain {@code double} values are
 * expressed in: {@link edu.wpi.first.units.AngleUnit} for rotary bindings and
 * {@link edu.wpi.first.units.DistanceUnit} for linear ones. Bindings created without a unit
 * ({@code Controls.position(motor)}) keep working with raw doubles and have a null {@link #unit()};
 * typed methods on them throw. Typed values are converted to base-unit doubles when declared, so the
 * per-cycle path never allocates measures.</p>
 */
public record ControlBinding<U extends Unit>(
        ControlMode mode,
        MotorDevice output,
        int slot,
        List<MotorDevice> followers,
        FeedbackBinding feedback,
        List<Object> dependencies,
        List<ControlLoop> loops,
        List<Constraint<Double>> constraints,
        MotionProfile profile,
        MotionPlanner planner,
        List<MotorDevice> motors,
        boolean isDisabled,
        ControlSink sink,
        Range continuousRange,
        U unit,
        Class<?> persistGroup,
        Retarget retarget,
        Release release) {
    /** Speed below which a mechanism counts as not moving for the checks. */
    private static final double STOPPED_SPEED = 0.01;

    public ControlBinding {
        Objects.requireNonNull(mode, "mode");
        slot = Math.max(0, slot);
        followers = followers == null ? List.of() : followers.stream()
                .filter(motor -> !motor.isDisabled())
                .toList();
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
        loops = loops == null ? List.of() : List.copyOf(loops);
        validateFeedbackUnit(unit, feedback);
        constraints = convertConstraints(constraints, unit);
        // Reject non-overlapping position ranges when the binding is declared, not in the robot loop.
        ca.frc6390.athena.mechanism.constraint.Constraints.positionRange(constraints);
        validateMotorTargets(output, followers);
        if (output != null && sink != null) {
            throw new IllegalArgumentException("A control binding cannot target both a motor and a scalar sink.");
        }
        motors = motorList(output, followers);
        retarget = retarget == null ? Retarget.SMOOTH : retarget;
        release = release == null ? Release.NEUTRAL : release;
    }

    /**
     * Source-compatible form of the canonical constructor from before tuning persistence existed.
     * The binding is not persisted.
     */
    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops,
            List<Constraint<Double>> constraints,
            MotionProfile profile,
            MotionPlanner planner,
            List<MotorDevice> motors,
            boolean isDisabled,
            ControlSink sink,
            Range continuousRange,
            U unit) {
        this(mode, output, slot, followers, feedback, dependencies, loops, constraints, profile, planner,
                motors, isDisabled, sink, continuousRange, unit, null, null, null);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, 0, followers, feedback, dependencies, loops,
                null, null, null, null, false, null, null, null, null, null, null);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, slot, followers, feedback, dependencies, loops,
                null, null, null, null, false, null, null, null, null, null, null);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops,
            List<Constraint<Double>> constraints,
            MotionProfile profile,
            MotionPlanner planner,
            List<MotorDevice> motors) {
        this(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, motors, false, null, null, null, null, null, null);
    }

    /** Source-compatible untyped form of the canonical constructor. */
    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops,
            List<Constraint<Double>> constraints,
            MotionProfile profile,
            MotionPlanner planner,
            List<MotorDevice> motors,
            boolean isDisabled,
            ControlSink sink,
            Range continuousRange) {
        this(mode, output, slot, followers, feedback, dependencies, loops, constraints, profile, planner,
                motors, isDisabled, sink, continuousRange, null, null, null, null);
    }

    /**
     * Persists this binding's tuning (PID and feedforward gains, and temporary constraint overrides)
     * under the telemetry store. Dashboard edits are saved and reloaded at the next registration. Writes
     * are gated by {@code lockGroup}; see {@code AthenaRobot.tuningLock}.
     *
     * @param lockGroup lock group class, such as {@code Locks.InMatch.class}
     * @return binding with tuning persistence enabled
     */
    public ControlBinding<U> persistTuning(Class<?> lockGroup) {
        Objects.requireNonNull(lockGroup, "lockGroup");
        return new ControlBinding<>(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled, sink, continuousRange, unit, lockGroup, retarget, release);
    }

    public ControlBinding<U> output(MotorDevice output) {
        MotorDevice safeOutput = Objects.requireNonNull(output, "output");
        if (followers.contains(safeOutput)) {
            throw new IllegalArgumentException("Control output cannot also be a follower.");
        }
        return new ControlBinding<>(mode, safeOutput, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled, null, continuousRange, unit, persistGroup, retarget, release);
    }

    /** Routes the calculated scalar result to a non-motor destination. */
    public ControlBinding<U> output(ControlSink sink) {
        return new ControlBinding<>(mode, null, slot, List.of(), feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled,
                Objects.requireNonNull(sink, "sink"), continuousRange, unit, persistGroup, retarget, release);
    }

    /** Configures wrapped error calculation for a continuous position input. */
    public ControlBinding<U> continuous(double minimum, double maximum) {
        requirePositionControl("Continuous input");
        if (!Double.isFinite(minimum) || !Double.isFinite(maximum) || maximum <= minimum) {
            throw new IllegalArgumentException("Continuous input bounds must be finite and ordered.");
        }
        return new ControlBinding<>(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled, sink, Range.of(minimum, maximum), unit, persistGroup, retarget, release);
    }

    /** Typed form of {@link #continuous(double, double)}; both bounds must measure this binding's unit. */
    public ControlBinding<U> continuous(Measure<?> minimum, Measure<?> maximum) {
        requireUnit("Typed continuous input");
        return continuous(UnitConversions.value(minimum, unit), UnitConversions.value(maximum, unit));
    }

    /** Returns the motion kind of this binding: rotary for angle units, linear for distance units. */
    public MotionKind motionKind() {
        return MotionKind.of(unit);
    }

    public ControlBinding<U> slot(int slot) {
        return copy(output, slot, followers, feedback, dependencies, loops, constraints, profile, planner);
    }

    public ControlBinding<U> follower(MotorDevice follower) {
        Objects.requireNonNull(follower, "follower");
        if (output == null) {
            throw new IllegalStateException("Motor followers require a motor output.");
        }
        if (follower.equals(output)) {
            throw new IllegalArgumentException("Control output cannot follow itself.");
        }
        if (followers.contains(follower)) {
            throw new IllegalArgumentException("Control follower is already configured: " + follower.defaultName());
        }
        List<MotorDevice> updated = new ArrayList<>(followers);
        updated.add(follower);
        return copy(output, slot, updated, feedback, dependencies, loops, constraints, profile, planner);
    }

    public ControlBinding<U> followers(MotorDevice... followers) {
        Objects.requireNonNull(followers, "followers");
        ControlBinding<U> updated = this;
        for (MotorDevice follower : followers) {
            updated = updated.follower(follower);
        }
        return updated;
    }

    public ControlBinding<U> feedback(EncoderDevice encoder) {
        Objects.requireNonNull(encoder, "encoder");
        return feedback(new FeedbackBinding(encoder, encoder));
    }

    /** Uses one position signal as feedback when no independent velocity signal is required. */
    public ControlBinding<U> feedback(PositionSignal position) {
        Objects.requireNonNull(position, "position");
        VelocitySignal stationaryVelocity = new VelocitySignal() {
            @Override
            public double velocity() {
                return 0.0;
            }

            @Override
            public List<?> dependencies() {
                return position.dependencies();
            }
        };
        return feedback(position, stationaryVelocity);
    }

    public ControlBinding<U> feedback(PositionSignal position, VelocitySignal velocity) {
        return feedback(new FeedbackBinding(position, velocity));
    }

    public ControlBinding<U> feedback(FeedbackBinding feedback) {
        return copy(output, slot, followers, Objects.requireNonNull(feedback, "feedback"), dependencies, loops,
                constraints, profile, planner);
    }

    public ControlBinding<U> dependency(Object dependency) {
        Objects.requireNonNull(dependency, "dependency");
        List<Object> updated = new ArrayList<>(dependencies);
        updated.add(dependency);
        return copy(output, slot, followers, feedback, updated, loops, constraints, profile, planner);
    }

    public ControlBinding<U> dependencies(Object... dependencies) {
        Objects.requireNonNull(dependencies, "dependencies");
        ControlBinding<U> updated = this;
        for (Object dependency : dependencies) {
            updated = updated.dependency(dependency);
        }
        return updated;
    }

    public ControlBinding<U> loop(ControlLoop loop) {
        Objects.requireNonNull(loop, "loop");
        List<ControlLoop> updated = new ArrayList<>(loops);
        updated.add(loop);
        return copy(output, slot, followers, feedback, dependencies, updated, constraints, profile, planner);
    }

    public ControlBinding<U> loops(ControlLoop... loops) {
        Objects.requireNonNull(loops, "loops");
        ControlBinding<U> updated = this;
        for (ControlLoop loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding<U> pid(double p, double i, double d) {
        return pid(PidGains.of(p, i, d));
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding<U> pid(PidGains pid) {
        return replaceLoop(PidGains.class, Objects.requireNonNull(pid, "pid"));
    }

    /** Adds static, velocity, and gravity feedforward contributions measured in volts. */
    public ControlBinding<U> ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardGains.of(staticGain, velocityGain, gravityGain));
    }

    /** Adds static, velocity, acceleration, and gravity feedforward contributions measured in volts. */
    public ControlBinding<U> ff(
            double staticGain,
            double velocityGain,
            double accelerationGain,
            double gravityGain) {
        return feedforward(FeedforwardGains.of(
                staticGain,
                velocityGain,
                accelerationGain,
                gravityGain));
    }

    /** Adds a feedforward contribution measured in volts. */
    public ControlBinding<U> feedforward(FeedforwardGains feedforward) {
        return replaceLoop(FeedforwardGains.class, Objects.requireNonNull(feedforward, "feedforward"));
    }

    public ControlBinding<U> constraint(Constraint<Double> constraint) {
        Objects.requireNonNull(constraint, "constraint");
        List<Constraint<Double>> updated = new ArrayList<>(constraints);
        updated.add(constraint);
        return copy(output, slot, followers, feedback, dependencies, loops, updated, profile, planner);
    }

    @SafeVarargs
    public final ControlBinding<U> constraints(Constraint<Double>... constraints) {
        Objects.requireNonNull(constraints, "constraints");
        ControlBinding<U> updated = this;
        for (Constraint<Double> constraint : constraints) {
            updated = updated.constraint(constraint);
        }
        return updated;
    }

    public ControlBinding<U> profile(MotionProfile profile) {
        requirePositionControl("Motion profiles");
        return copy(output, slot, followers, feedback, dependencies, loops, constraints,
                Objects.requireNonNull(profile, "profile"), planner);
    }

    public ControlBinding<U> planner(MotionPlanner planner) {
        requirePositionControl("Motion planners");
        return copy(output, slot, followers, feedback, dependencies, loops, constraints, profile,
                Objects.requireNonNull(planner, "planner"));
    }

    private void requirePositionControl(String feature) {
        if (mode != ControlMode.POSITION) {
            throw new IllegalStateException(feature + " require a position control binding.");
        }
    }

    public List<MotorDevice> motors() {
        return motors;
    }

    /**
     * Declares this binding's effort ceiling (B5): the largest torque, force or current it may drive.
     * Declare it before creating the Actions that use the binding, because torque and force need its
     * gearing. The ceiling is enforced as a voltage clamp on the software output path (B6), so it works
     * with any controller. Load responses ({@link Effort#onLoad}) need a model on this binding
     * ({@link #model(Characterization)} or {@link #model(MechanismModel)}) declared before this call. Without
     * one, the responses are ignored and only the ceiling applies.
     *
     * @param effort ceiling and optional load responses
     * @return binding with the ceiling
     * @throws IllegalArgumentException when the units, gearing or motor constants do not suit this binding
     */
    public ControlBinding<U> effort(Effort effort) {
        Objects.requireNonNull(effort, "effort");
        Supplier<MechanismModel> model = null;
        if (!effort.responses().isEmpty()) {
            for (ControlLoop loop : loops) {
                if (loop instanceof ModelFeedforward feedforward) {
                    model = feedforward::currentModel;
                }
            }
        }
        double factor = Identification.baseFactor(unit);
        return replaceLoop(EffortLimit.class, EffortLimit.declare(effort, motionKind(), motors(), model, factor));
    }

    /**
     * Stage cap that scales voltage and percent output of this binding while a staging stage is active.
     *
     * @param fraction multiplier in {@code [0, 1]}
     * @return modifier for {@link ca.frc6390.athena.mechanism.staging.Staging#stage}
     */
    public StageModifier limitEffort(double fraction) {
        return new StageModifier(this, StageModifier.Kind.EFFORT, fraction);
    }

    /**
     * Stage cap that scales maximum velocity (and velocity targets) of this binding.
     *
     * @param fraction multiplier in {@code [0, 1]}
     * @return modifier for {@link ca.frc6390.athena.mechanism.staging.Staging#stage}
     */
    public StageModifier limitSpeed(double fraction) {
        return new StageModifier(this, StageModifier.Kind.SPEED, fraction);
    }

    /**
     * Stage cap that scales maximum acceleration of this binding.
     *
     * @param fraction multiplier in {@code [0, 1]}
     * @return modifier for {@link ca.frc6390.athena.mechanism.staging.Staging#stage}
     */
    public StageModifier limitAccel(double fraction) {
        return new StageModifier(this, StageModifier.Kind.ACCELERATION, fraction);
    }

    /**
     * Binding default for how a new target re-plans a running motion profile. Per-Action overrides
     * use {@code action.retarget(...)}. Defaults to {@link Retarget#SMOOTH}.
     */
    public ControlBinding<U> retarget(Retarget retarget) {
        return new ControlBinding<>(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled, sink, continuousRange, unit, persistGroup,
                Objects.requireNonNull(retarget, "retarget"), release);
    }

    /**
     * Binding default for what happens when the last Action driving this binding ends and nothing
     * replaces it. Per-Action overrides use {@code action.release(...)}. Defaults to
     * {@link Release#NEUTRAL}, the behaviour before the release policy existed.
     */
    public ControlBinding<U> release(Release release) {
        return new ControlBinding<>(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, isDisabled, sink, continuousRange, unit, persistGroup,
                retarget, Objects.requireNonNull(release, "release"));
    }

    public ControlBinding<U> disabled() {
        return disabled(true);
    }

    public ControlBinding<U> disabled(boolean disabled) {
        return new ControlBinding<>(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, disabled, sink, continuousRange, unit, persistGroup, retarget, release);
    }

    /** Returns the latest configured position feedback. */
    public double position() {
        return requireFeedback().position().position();
    }

    /** Returns the latest configured velocity feedback. */
    public double velocity() {
        return requireFeedback().velocity().velocity();
    }

    /** Returns the feedback channel controlled by this binding. */
    public double measurement() {
        return mode == ControlMode.VELOCITY ? velocity() : position();
    }

    /** Returns signed target error in this control's configured feedback units. */
    public double error(double target) {
        requireFinite(target, "Control target");
        return target - measurement();
    }

    /** Returns whether this control is currently within tolerance of a target. */
    public boolean isAt(double target, double tolerance) {
        validateToleranceTarget(target, tolerance);
        return Math.abs(target - measurement()) <= tolerance;
    }

    /** Returns a reusable condition that tracks whether this control is at a target. */
    public BooleanSupplier at(double target, double tolerance) {
        validateToleranceTarget(target, tolerance);
        return () -> isAt(target, tolerance);
    }

    /** Returns a reusable condition for a dynamically supplied target. */
    public BooleanSupplier at(DoubleSupplier target, double tolerance) {
        Objects.requireNonNull(target, "target");
        validateTolerance(tolerance);
        return () -> isAt(target.getAsDouble(), tolerance);
    }

    /** Creates a WPILib-compatible SysId routine for this control and its feedback. */
    public ControlSysId sysId() {
        return ControlSysId.forControl(this);
    }

    /**
     * Creates the characterization declaration for this binding. Declare it as a field to expose its
     * test Actions, and pass it to {@link #model(Characterization)} to drive feedforward from it.
     */
    public Characterization characterization() {
        return Characterization.of(this);
    }

    /**
     * Uses a fixed identified model for feedforward. Replaces {@code ff(...)} and any previous model.
     * {@code ff(...)} called later replaces the model again.
     */
    public ControlBinding<U> model(MechanismModel model) {
        return replaceModelLoop(new ModelFeedforward(() -> Objects.requireNonNull(model, "model"), null));
    }

    /**
     * Uses the characterization's active model for feedforward. The loop follows later fits, adopted
     * estimates and rebaselines. When the characterization learns continuously, the loop also feeds
     * normal operation to it.
     */
    public ControlBinding<U> model(Characterization characterization) {
        Objects.requireNonNull(characterization, "characterization");
        return replaceModelLoop(new ModelFeedforward(characterization::model, characterization));
    }

    private ControlBinding<U> replaceModelLoop(ModelFeedforward model) {
        List<ControlLoop> updated = new ArrayList<>(loops.size() + 1);
        for (ControlLoop existing : loops) {
            if (!(existing instanceof FeedforwardGains) && !(existing instanceof ModelFeedforward)) {
                updated.add(existing);
            }
        }
        updated.add(model);
        return copy(output, slot, followers, feedback, dependencies, updated, constraints, profile, planner);
    }

    /**
     * Estimated external load on the mechanism. The load is the applied voltage minus the model's
     * prediction (gravity, friction, velocity term) minus {@code kA} times the measured acceleration.
     * It is in N m at the mechanism when the motor constants are known, and in volts otherwise.
     * Requires a model ({@link #model(MechanismModel)} or {@link #model(Characterization)}).
     */
    public DoubleSupplier load() {
        ModelFeedforward model = requireModelLoop("load()");
        MotorDevice motor = requireOutput("load()");
        FeedbackBinding feedback = requireFeedback();
        double factor = Identification.baseFactor(unit);
        AccelerationEstimator acceleration = new AccelerationEstimator(MonotonicClock.SYSTEM);
        return () -> {
            MechanismModel current = model.currentModel();
            double velocity = feedback.velocity().velocity() * factor;
            double accel = acceleration.update(velocity);
            double applied = appliedVoltage(motor);
            double predicted = current.feedforwardVolts(feedback.position().position() * factor, velocity, 0.0, 0.0);
            double loadVolts = (applied - predicted) - current.kA() * accel;
            if (current.motorTorqueConstant() <= 0.0 || current.motorResistance() <= 0.0) {
                return loadVolts;
            }
            double motors = Math.max(1, current.motorCount()) * current.gearRatio();
            return loadVolts * motors * current.motorTorqueConstant() / current.motorResistance();
        };
    }

    /**
     * True while the output is commanded to move but is not moving, with the drive voltage applied
     * (default threshold 1 V for 0.2 s). Requires a model.
     */
    public BindingCheck blocked() {
        ModelFeedforward model = requireModelLoop("blocked()");
        MotorDevice motor = requireOutput("blocked()");
        FeedbackBinding feedback = requireFeedback();
        double factor = Identification.baseFactor(unit);
        return new BindingCheck(() -> {
            double velocity = feedback.velocity().velocity() * factor;
            return Math.abs(velocity) < STOPPED_SPEED ? Math.abs(appliedVoltage(motor)) : 0.0;
        }, 1.0, true, 0.2, MonotonicClock.SYSTEM);
    }

    /**
     * True while the mechanism moves against the drive direction (default threshold 0.05 units per
     * second for 0.1 s). Requires a model.
     */
    public BindingCheck backdriven() {
        ModelFeedforward model = requireModelLoop("backdriven()");
        MotorDevice motor = requireOutput("backdriven()");
        FeedbackBinding feedback = requireFeedback();
        return new BindingCheck(() -> {
            double applied = appliedVoltage(motor);
            if (applied == 0.0 || !Double.isFinite(applied)) {
                return 0.0;
            }
            return -feedback.velocity().velocity() * Math.signum(applied);
        }, 0.05, true, 0.1, MonotonicClock.SYSTEM);
    }

    /**
     * True while the output draws current and the feedback does not move (default threshold 2 A for
     * 0.1 s). Needs no model.
     */
    public BindingCheck feedbackStalled() {
        MotorDevice motor = requireOutput("feedbackStalled()");
        FeedbackBinding feedback = requireFeedback();
        return new BindingCheck(() -> {
            double applied = appliedVoltage(motor);
            if (!(Math.abs(applied) > 0.5) || Math.abs(feedback.velocity().velocity()) >= STOPPED_SPEED) {
                return 0.0;
            }
            try {
                return Math.abs(motor.statorCurrentAmps());
            } catch (RuntimeException unavailable) {
                return Double.NaN;
            }
        }, 2.0, true, 0.1, MonotonicClock.SYSTEM);
    }

    /**
     * Compares this binding's feedback with a second encoder on the same mechanism. The second
     * encoder's reading is multiplied by {@code ratio} (sensor to mechanism). Both must be in this
     * binding's unit. Requires a unit-typed binding.
     */
    public SlipCheck slipCheck(EncoderDevice second, GearRatio ratio, Measure<?> tolerance) {
        requireUnit("Typed slip tolerance");
        return slipCheck(second, ratio, UnitConversions.value(tolerance, unit));
    }

    /** Raw form of {@link #slipCheck(EncoderDevice, GearRatio, Measure)}; the tolerance is in binding units. */
    public SlipCheck slipCheck(EncoderDevice second, GearRatio ratio, double tolerance) {
        Objects.requireNonNull(second, "second");
        Objects.requireNonNull(ratio, "ratio");
        FeedbackBinding feedback = requireFeedback();
        return new SlipCheck(() -> feedback.position().position(),
                () -> second.position() * ratio.ratio(), tolerance);
    }

    private ModelFeedforward requireModelLoop(String feature) {
        for (ControlLoop loop : loops) {
            if (loop instanceof ModelFeedforward model) {
                return model;
            }
        }
        throw new IllegalStateException(feature + " requires a model: call model(...) first.");
    }

    private MotorDevice requireOutput(String feature) {
        if (output == null) {
            throw new IllegalStateException(feature + " requires a motor output.");
        }
        return output;
    }

    private static double appliedVoltage(MotorDevice motor) {
        try {
            return motor.appliedVoltage();
        } catch (RuntimeException unavailable) {
            return Double.NaN;
        }
    }

    /**
     * Rejects a typed binding whose encoder feedback does not measure the binding's unit. Checked on
     * every construction, so it fires when the robot declares the mechanism, not in the robot loop.
     */
    private static void validateFeedbackUnit(Unit unit, FeedbackBinding feedback) {
        if (unit == null || feedback == null) {
            return;
        }
        checkEncoderUnit(unit, feedback.position(), "position");
        checkEncoderUnit(unit, feedback.velocity(), "velocity");
    }

    private static void checkEncoderUnit(Unit unit, Object signal, String channel) {
        if (!(signal instanceof EncoderDevice encoder)) {
            return;
        }
        Unit measured = encoder.measuresUnit();
        if (measured == null) {
            throw new IllegalArgumentException("Control unit " + unit.name() + " requires encoder "
                    + encoder.defaultName() + " to declare measures(...) for its " + channel + " feedback.");
        }
        if (!unit.equals(measured)) {
            throw new IllegalArgumentException("Control unit " + unit.name() + " does not match encoder "
                    + encoder.defaultName() + " which measures " + measured.name() + " (" + channel
                    + " feedback).");
        }
    }

    /** Converts typed constraints to this binding's unit; plain constraints pass through unchanged. */
    private static List<Constraint<Double>> convertConstraints(List<Constraint<Double>> constraints, Unit unit) {
        if (constraints == null) {
            return List.of();
        }
        List<Constraint<Double>> converted = new ArrayList<>(constraints.size());
        for (Constraint<Double> constraint : constraints) {
            Objects.requireNonNull(constraint, "constraint");
            if (constraint instanceof UnitConstraint typed) {
                if (unit == null) {
                    throw new IllegalStateException("Typed constraints require a unit-typed control binding "
                            + "(Controls.position(motor, unit)).");
                }
                converted.add(typed.expressedIn(unit));
            } else {
                converted.add(constraint);
            }
        }
        return List.copyOf(converted);
    }

    private void requireUnit(String feature) {
        if (unit == null) {
            throw new IllegalStateException(feature + " require a unit-typed control binding "
                    + "(Controls.position(motor, unit) or Controls.velocity(motor, unit)).");
        }
    }

    /**
     * Converts a measure into this binding's plain value for the current mode: a position value for
     * position bindings, a rate value (unit per second) for velocity bindings.
     */
    double modeValue(Measure<?> measure) {
        requireUnit("Typed targets");
        return mode == ControlMode.VELOCITY
                ? UnitConversions.rate(measure, unit)
                : UnitConversions.value(measure, unit);
    }

    private static List<MotorDevice> motorList(MotorDevice output, List<MotorDevice> followers) {
        List<MotorDevice> motors = new ArrayList<>();
        if (output != null) {
            motors.add(output);
        }
        motors.addAll(followers == null ? List.of() : followers);
        return List.copyOf(motors);
    }

    private static void validateMotorTargets(MotorDevice output, List<MotorDevice> followers) {
        List<MotorDevice> seen = new ArrayList<>();
        for (MotorDevice follower : followers) {
            Objects.requireNonNull(follower, "follower");
            if (follower.equals(output)) {
                throw new IllegalArgumentException("Control output cannot follow itself.");
            }
            if (seen.contains(follower)) {
                throw new IllegalArgumentException("Control followers must be unique.");
            }
            seen.add(follower);
        }
    }

    private ControlBinding<U> replaceLoop(Class<? extends ControlLoop> type, ControlLoop replacement) {
        List<ControlLoop> updated = new ArrayList<>(loops.size() + 1);
        for (ControlLoop existing : loops) {
            boolean replacedByGains = type == FeedforwardGains.class && existing instanceof ModelFeedforward;
            if (!type.isInstance(existing) && !replacedByGains) {
                updated.add(existing);
            }
        }
        updated.add(replacement);
        return copy(output, slot, followers, feedback, dependencies, updated, constraints, profile, planner);
    }

    private ControlBinding<U> copy(
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops,
            List<Constraint<Double>> constraints,
            MotionProfile profile,
            MotionPlanner planner) {
        return new ControlBinding<>(
                mode,
                output,
                slot,
                followers,
                feedback,
                dependencies,
                loops,
                constraints,
                profile,
                planner,
                null,
                isDisabled,
                sink,
                continuousRange,
                unit,
                persistGroup,
                retarget,
                release);
    }

    public Action set(double target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public Action set(DoubleSupplier target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    /** Creates an interpolated position or velocity action for this binding. */
    public InterpolatedControlAction interpolate(InterpolationModel model, DoubleSupplier input) {
        requireOutput();
        return new InterpolatedControlAction(
                this,
                Objects.requireNonNull(model, "model"),
                Objects.requireNonNull(input, "input"));
    }

    /** Creates interpolation whose segments use the same mapping contract as controller axes. */
    public InterpolatedControlAction interpolate(CurveMapping mapping, DoubleSupplier input) {
        return interpolate(InterpolationKinds.curved(
                Objects.requireNonNull(mapping, "mapping")), input);
    }

    /**
     * Typed position target. The measure must measure this binding's unit (for example
     * {@code Degrees.of(90)} for a binding created with {@code Controls.position(motor, Degrees)}).
     * Conversion happens here, at declaration.
     */
    public Action position(Measure<?> position) {
        requireUnit("Typed position targets");
        return position(UnitConversions.value(position, unit));
    }

    /** Typed velocity target, expressed as a rate of this binding's unit per second. */
    public Action velocity(Measure<?> velocity) {
        requireUnit("Typed velocity targets");
        return velocity(UnitConversions.rate(velocity, unit));
    }

    /**
     * Dynamic position target supplied in {@code positionUnit}. The value is converted to this
     * binding's unit with a precomputed factor, so the robot loop does not allocate.
     */
    public Action position(DoubleSupplier position, U positionUnit) {
        requireUnit("Typed position targets");
        double factor = UnitConversions.factor(positionUnit, unit);
        Objects.requireNonNull(position, "position");
        return position(() -> position.getAsDouble() * factor);
    }

    /** Dynamic velocity target supplied as unit-per-second values in {@code velocityUnit}. */
    public Action velocity(DoubleSupplier velocity, U velocityUnit) {
        requireUnit("Typed velocity targets");
        double factor = UnitConversions.factor(velocityUnit, unit);
        Objects.requireNonNull(velocity, "velocity");
        return velocity(() -> velocity.getAsDouble() * factor);
    }

    /** Returns a reusable condition that tracks whether this control is at a typed target. */
    public BooleanSupplier at(Measure<?> target, Measure<?> tolerance) {
        return at(modeValue(target), modeValue(tolerance));
    }

    /** Typed form of {@link #interpolate(InterpolationModel, DoubleSupplier)} with a unit on the input. */
    public InterpolatedControlAction interpolate(InterpolationModel model, DoubleSupplier input, Unit inputUnit) {
        requireOutput();
        return new InterpolatedControlAction(
                this,
                Objects.requireNonNull(model, "model"),
                Objects.requireNonNull(input, "input"),
                Objects.requireNonNull(inputUnit, "inputUnit"));
    }

    public Action percent(double percent) {
        requireOutput();
        requireFinite(percent, "Control percent");
        return Actions.percent(this, percent);
    }

    public Action percent(DoubleSupplier percent) {
        requireOutput();
        return Actions.percent(this, finiteSupplier(percent, "Control percent", false));
    }

    public Action voltage(double volts) {
        requireOutput();
        requireFinite(volts, "Control voltage");
        return Actions.voltage(this, volts);
    }

    public Action voltage(DoubleSupplier volts) {
        requireOutput();
        return Actions.voltage(this, finiteSupplier(volts, "Control voltage", false));
    }

    /** Stops driving this control and applies the motor's configured neutral mode. */
    public Action neutral() {
        requireOutput();
        return Actions.neutral(this);
    }

    public Action position(double position) {
        requireOutput();
        requireMode(ControlMode.POSITION, "Position targets");
        requireFinite(position, "Control position");
        return Actions.position(this, position);
    }

    public Action position(DoubleSupplier position) {
        requireOutput();
        requireMode(ControlMode.POSITION, "Position targets");
        return Actions.position(this, finiteSupplier(position, "Control position", true));
    }

    public Action velocity(double velocity) {
        requireOutput();
        requireMode(ControlMode.VELOCITY, "Velocity targets");
        requireFinite(velocity, "Control velocity");
        return Actions.velocity(this, velocity);
    }

    public Action velocity(DoubleSupplier velocity) {
        requireOutput();
        requireMode(ControlMode.VELOCITY, "Velocity targets");
        return Actions.velocity(this, finiteSupplier(velocity, "Control velocity", true));
    }

    private FeedbackBinding requireFeedback() {
        if (feedback == null) {
            throw new IllegalStateException(
                    "Control measurements require an explicit feedback(...) binding.");
        }
        return feedback;
    }

    private void requireOutput() {
        if (output == null && sink == null) {
            throw new IllegalStateException("Control actions require an output destination.");
        }
    }

    private static void validateToleranceTarget(double target, double tolerance) {
        requireFinite(target, "Control target");
        validateTolerance(tolerance);
    }

    private static void validateTolerance(double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            throw new IllegalArgumentException("Control tolerance must be finite and non-negative.");
        }
    }

    private static void requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
    }

    private void requireMode(ControlMode required, String description) {
        if (mode != required) {
            throw new IllegalStateException(description + " require a "
                    + required.name().toLowerCase(java.util.Locale.ROOT) + " control binding.");
        }
    }

    /**
     * Wraps a robot-code supplier so a non-finite (or throwing) sample never throws in the robot
     * loop. Targets hold the last valid sample; open-loop efforts return NaN, which the runtime turns
     * into neutral output. Either way a rate-limited warning is reported.
     */
    private static DoubleSupplier finiteSupplier(DoubleSupplier supplier, String description, boolean holdLast) {
        Objects.requireNonNull(supplier, "supplier");
        return new DoubleSupplier() {
            private double lastValid = Double.NaN;

            @Override
            public double getAsDouble() {
                double value;
                try {
                    value = supplier.getAsDouble();
                } catch (RuntimeException exception) {
                    value = Double.NaN;
                    FaultIsolation.warn(this, description + " supplier threw " + exception
                            + (holdLast && Double.isFinite(lastValid) ? "; holding last valid target" : "; output neutral"));
                    return holdLast ? lastValid : Double.NaN;
                }
                if (Double.isFinite(value)) {
                    if (holdLast) lastValid = value;
                    return value;
                }
                FaultIsolation.warn(this, description + " supplier returned " + value
                        + (holdLast && Double.isFinite(lastValid) ? "; holding last valid target " + lastValid
                                : "; output neutral"));
                return holdLast ? lastValid : Double.NaN;
            }
        };
    }
}
