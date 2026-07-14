package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.constraint.Constraint;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.motion.MotionPlanner;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Declarative control binding for one actuator group.
 */
public record ControlBinding(
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
        boolean isDisabled) {
    public ControlBinding {
        Objects.requireNonNull(mode, "mode");
        slot = Math.max(0, slot);
        followers = followers == null ? List.of() : followers.stream()
                .filter(motor -> !motor.isDisabled())
                .toList();
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
        loops = loops == null ? List.of() : List.copyOf(loops);
        constraints = constraints == null ? List.of() : List.copyOf(constraints);
        validateMotorTargets(output, followers);
        motors = motorList(output, followers);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, 0, followers, feedback, dependencies, loops, null, null, null, null, false);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, slot, followers, feedback, dependencies, loops, null, null, null, null, false);
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
                constraints, profile, planner, motors, false);
    }

    public ControlBinding output(MotorDevice output) {
        MotorDevice safeOutput = Objects.requireNonNull(output, "output");
        if (followers.contains(safeOutput)) {
            throw new IllegalArgumentException("Control output cannot also be a follower.");
        }
        return copy(safeOutput, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner);
    }

    public ControlBinding slot(int slot) {
        return copy(output, slot, followers, feedback, dependencies, loops, constraints, profile, planner);
    }

    public ControlBinding follower(MotorDevice follower) {
        Objects.requireNonNull(follower, "follower");
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

    public ControlBinding followers(MotorDevice... followers) {
        Objects.requireNonNull(followers, "followers");
        ControlBinding updated = this;
        for (MotorDevice follower : followers) {
            updated = updated.follower(follower);
        }
        return updated;
    }

    public ControlBinding feedback(EncoderDevice encoder) {
        Objects.requireNonNull(encoder, "encoder");
        return feedback(new FeedbackBinding(encoder, encoder));
    }

    public ControlBinding feedback(PositionSignal position, VelocitySignal velocity) {
        return feedback(new FeedbackBinding(position, velocity));
    }

    public ControlBinding feedback(FeedbackBinding feedback) {
        return copy(output, slot, followers, Objects.requireNonNull(feedback, "feedback"), dependencies, loops,
                constraints, profile, planner);
    }

    public ControlBinding dependency(Object dependency) {
        Objects.requireNonNull(dependency, "dependency");
        List<Object> updated = new ArrayList<>(dependencies);
        updated.add(dependency);
        return copy(output, slot, followers, feedback, updated, loops, constraints, profile, planner);
    }

    public ControlBinding dependencies(Object... dependencies) {
        Objects.requireNonNull(dependencies, "dependencies");
        ControlBinding updated = this;
        for (Object dependency : dependencies) {
            updated = updated.dependency(dependency);
        }
        return updated;
    }

    public ControlBinding loop(ControlLoop loop) {
        Objects.requireNonNull(loop, "loop");
        List<ControlLoop> updated = new ArrayList<>(loops);
        updated.add(loop);
        return copy(output, slot, followers, feedback, dependencies, updated, constraints, profile, planner);
    }

    public ControlBinding loops(ControlLoop... loops) {
        Objects.requireNonNull(loops, "loops");
        ControlBinding updated = this;
        for (ControlLoop loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding pid(double p, double i, double d) {
        return pid(PidGains.of(p, i, d));
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding pid(PidGains pid) {
        return replaceLoop(PidGains.class, Objects.requireNonNull(pid, "pid"));
    }

    /** Adds static, velocity, and gravity feedforward contributions measured in volts. */
    public ControlBinding ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardGains.of(staticGain, velocityGain, gravityGain));
    }

    /** Adds static, velocity, acceleration, and gravity feedforward contributions measured in volts. */
    public ControlBinding ff(
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
    public ControlBinding feedforward(FeedforwardGains feedforward) {
        return replaceLoop(FeedforwardGains.class, Objects.requireNonNull(feedforward, "feedforward"));
    }

    public ControlBinding constraint(Constraint<Double> constraint) {
        Objects.requireNonNull(constraint, "constraint");
        List<Constraint<Double>> updated = new ArrayList<>(constraints);
        updated.add(constraint);
        return copy(output, slot, followers, feedback, dependencies, loops, updated, profile, planner);
    }

    @SafeVarargs
    public final ControlBinding constraints(Constraint<Double>... constraints) {
        Objects.requireNonNull(constraints, "constraints");
        ControlBinding updated = this;
        for (Constraint<Double> constraint : constraints) {
            updated = updated.constraint(constraint);
        }
        return updated;
    }

    public ControlBinding profile(MotionProfile profile) {
        requirePositionControl("Motion profiles");
        return copy(output, slot, followers, feedback, dependencies, loops, constraints,
                Objects.requireNonNull(profile, "profile"), planner);
    }

    public ControlBinding planner(MotionPlanner planner) {
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

    public ControlBinding disabled() {
        return disabled(true);
    }

    public ControlBinding disabled(boolean disabled) {
        return new ControlBinding(mode, output, slot, followers, feedback, dependencies, loops,
                constraints, profile, planner, null, disabled);
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

    private ControlBinding replaceLoop(Class<? extends ControlLoop> type, ControlLoop replacement) {
        List<ControlLoop> updated = new ArrayList<>(loops.size() + 1);
        for (ControlLoop existing : loops) {
            if (!type.isInstance(existing)) {
                updated.add(existing);
            }
        }
        updated.add(replacement);
        return copy(output, slot, followers, feedback, dependencies, updated, constraints, profile, planner);
    }

    private ControlBinding copy(
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops,
            List<Constraint<Double>> constraints,
            MotionProfile profile,
            MotionPlanner planner) {
        return new ControlBinding(
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
                isDisabled);
    }

    public Action set(double target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public Action set(DoubleSupplier target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public Action percent(double percent) {
        requireOutput();
        requireFinite(percent, "Control percent");
        return Actions.percent(this, percent);
    }

    public Action percent(DoubleSupplier percent) {
        requireOutput();
        return Actions.percent(this, finiteSupplier(percent, "Control percent"));
    }

    public Action voltage(double volts) {
        requireOutput();
        requireFinite(volts, "Control voltage");
        return Actions.voltage(this, volts);
    }

    public Action voltage(DoubleSupplier volts) {
        requireOutput();
        return Actions.voltage(this, finiteSupplier(volts, "Control voltage"));
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
        return Actions.position(this, finiteSupplier(position, "Control position"));
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
        return Actions.velocity(this, finiteSupplier(velocity, "Control velocity"));
    }

    private FeedbackBinding requireFeedback() {
        if (feedback == null) {
            throw new IllegalStateException(
                    "Control measurements require an explicit feedback(...) binding.");
        }
        return feedback;
    }

    private void requireOutput() {
        if (output == null) {
            throw new IllegalStateException("Control actions require an output motor.");
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

    private static DoubleSupplier finiteSupplier(DoubleSupplier supplier, String description) {
        Objects.requireNonNull(supplier, "supplier");
        return () -> {
            double value = supplier.getAsDouble();
            requireFinite(value, description);
            return value;
        };
    }
}
