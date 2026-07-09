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
        List<ControlLoop> loops) {
    public ControlBinding {
        mode = mode == null ? ControlMode.POSITION : mode;
        followers = followers == null ? List.of() : List.copyOf(followers);
        feedback = feedback == null ? List.of() : List.copyOf(feedback);
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
        loops = loops == null ? List.of() : List.copyOf(loops);
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
        if (output == null) {
            return followers;
        }
        List<MotorDevice> motors = new ArrayList<>();
        motors.add(output);
        motors.addAll(followers);
        return List.copyOf(motors);
    }

    public State set(double target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public State set(DoubleSupplier target) {
        return mode == ControlMode.VELOCITY ? velocity(target) : position(target);
    }

    public State percent(double percent) {
        return States.percent(this, percent);
    }

    public State percent(DoubleSupplier percent) {
        return States.percent(this, percent);
    }

    public State voltage(double volts) {
        return States.voltage(this, volts);
    }

    public State voltage(DoubleSupplier volts) {
        return States.voltage(this, volts);
    }

    public State position(double position) {
        return States.position(this, position);
    }

    public State position(DoubleSupplier position) {
        return States.position(this, position);
    }

    public State velocity(double velocity) {
        return States.velocity(this, velocity);
    }

    public State velocity(DoubleSupplier velocity) {
        return States.velocity(this, velocity);
    }
}
