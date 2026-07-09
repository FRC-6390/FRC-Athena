package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.ref.FeedforwardGains;
import ca.frc6390.athena.mechanism.ref.PidGains;
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
        List<MotorDevice> followers,
        List<EncoderDevice> feedback,
        List<Object> dependencies,
        List<ControlLoop> loops,
        List<MotorDevice> motors) {
    public ControlBinding {
        mode = mode == null ? ControlMode.POSITION : mode;
        followers = followers == null ? List.of() : List.copyOf(followers);
        feedback = feedback == null ? List.of() : List.copyOf(feedback);
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
        loops = loops == null ? List.of() : List.copyOf(loops);
        motors = motors == null ? motorList(output, followers) : List.copyOf(motors);
    }

    public ControlBinding(
            ControlMode mode,
            MotorDevice output,
            List<MotorDevice> followers,
            List<EncoderDevice> feedback,
            List<Object> dependencies,
            List<ControlLoop> loops) {
        this(mode, output, followers, feedback, dependencies, loops, null);
    }

    public ControlBinding output(MotorDevice output) {
        return new ControlBinding(mode, Objects.requireNonNull(output, "output"), followers, feedback, dependencies, loops);
    }

    public ControlBinding follower(MotorDevice follower) {
        Objects.requireNonNull(follower, "follower");
        List<MotorDevice> updated = new ArrayList<>(followers);
        updated.add(follower);
        return new ControlBinding(mode, output, updated, feedback, dependencies, loops);
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
        List<EncoderDevice> updated = new ArrayList<>(feedback);
        updated.add(encoder);
        return new ControlBinding(mode, output, followers, updated, dependencies, loops);
    }

    public ControlBinding feedback(EncoderDevice... encoders) {
        ControlBinding updated = this;
        for (EncoderDevice encoder : encoders) {
            updated = updated.feedback(encoder);
        }
        return updated;
    }

    public ControlBinding dependency(Object dependency) {
        Objects.requireNonNull(dependency, "dependency");
        List<Object> updated = new ArrayList<>(dependencies);
        updated.add(dependency);
        return new ControlBinding(mode, output, followers, feedback, updated, loops);
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
        return new ControlBinding(mode, output, followers, feedback, dependencies, updated);
    }

    public ControlBinding loops(ControlLoop... loops) {
        ControlBinding updated = this;
        for (ControlLoop loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    public ControlBinding pid(double p, double i, double d) {
        return loop(PidGains.of(p, i, d));
    }

    public ControlBinding pid(PidGains pid) {
        return loop(pid);
    }

    public ControlBinding ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardGains.of(staticGain, velocityGain, gravityGain));
    }

    public ControlBinding feedforward(FeedforwardGains feedforward) {
        return loop(feedforward);
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
