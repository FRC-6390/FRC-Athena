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
        List<MotorDevice> motors) {
    public ControlBinding {
        mode = mode == null ? ControlMode.POSITION : mode;
        slot = Math.max(0, slot);
        followers = followers == null ? List.of() : List.copyOf(followers);
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
        loops = loops == null ? List.of() : List.copyOf(loops);
        constraints = constraints == null ? List.of() : List.copyOf(constraints);
        motors = motors == null ? motorList(output, followers) : List.copyOf(motors);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, 0, followers, feedback, dependencies, loops, null, null, null, null);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            int slot,
            List<MotorDevice> followers,
            FeedbackBinding feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, slot, followers, feedback, dependencies, loops, null, null, null, null);
    }

    public ControlBinding output(MotorDevice output) {
        return copy(Objects.requireNonNull(output, "output"), slot, followers, feedback, dependencies, loops,
                constraints, profile, planner);
    }

    public ControlBinding slot(int slot) {
        return copy(output, slot, followers, feedback, dependencies, loops, constraints, profile, planner);
    }

    public ControlBinding follower(MotorDevice follower) {
        Objects.requireNonNull(follower, "follower");
        List<MotorDevice> updated = new ArrayList<>(followers);
        updated.add(follower);
        return copy(output, slot, updated, feedback, dependencies, loops, constraints, profile, planner);
    }

    public ControlBinding followers(MotorDevice... followers) {
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
        ControlBinding updated = this;
        for (ControlLoop loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding pid(double p, double i, double d) {
        return loop(PidGains.of(p, i, d));
    }

    /** Adds PID gains whose resulting control effort is measured in volts. */
    public ControlBinding pid(PidGains pid) {
        return loop(pid);
    }

    /** Adds static, velocity, and gravity feedforward contributions measured in volts. */
    public ControlBinding ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardGains.of(staticGain, velocityGain, gravityGain));
    }

    /** Adds a feedforward contribution measured in volts. */
    public ControlBinding feedforward(FeedforwardGains feedforward) {
        return loop(feedforward);
    }

    public ControlBinding constraint(Constraint<Double> constraint) {
        Objects.requireNonNull(constraint, "constraint");
        List<Constraint<Double>> updated = new ArrayList<>(constraints);
        updated.add(constraint);
        return copy(output, slot, followers, feedback, dependencies, loops, updated, profile, planner);
    }

    @SafeVarargs
    public final ControlBinding constraints(Constraint<Double>... constraints) {
        ControlBinding updated = this;
        if (constraints != null) {
            for (Constraint<Double> constraint : constraints) {
                updated = updated.constraint(constraint);
            }
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

    private static List<MotorDevice> motorList(MotorDevice output, List<MotorDevice> followers) {
        List<MotorDevice> motors = new ArrayList<>();
        if (output != null) {
            motors.add(output);
        }
        motors.addAll(followers == null ? List.of() : followers);
        return List.copyOf(motors);
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
                null);
    }

    public Action set(double target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public Action set(DoubleSupplier target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public Action percent(double percent) {
        return Actions.percent(this, percent);
    }

    public Action percent(DoubleSupplier percent) {
        return Actions.percent(this, percent);
    }

    public Action voltage(double volts) {
        return Actions.voltage(this, volts);
    }

    public Action voltage(DoubleSupplier volts) {
        return Actions.voltage(this, volts);
    }

    /** Stops driving this control and applies the motor's configured neutral mode. */
    public Action neutral() {
        return Actions.neutral(this);
    }

    public Action position(double position) {
        return Actions.position(this, position);
    }

    public Action position(DoubleSupplier position) {
        return Actions.position(this, position);
    }

    public Action velocity(double velocity) {
        return Actions.velocity(this, velocity);
    }

    public Action velocity(DoubleSupplier velocity) {
        return Actions.velocity(this, velocity);
    }
}
