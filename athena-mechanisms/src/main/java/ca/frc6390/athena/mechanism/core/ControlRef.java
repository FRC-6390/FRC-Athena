package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.mechanism.ref.CrtRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Command binding for one controller mode over one actuator group.
 */
public record ControlRef(
        ControlMode mode,
        MotorRef output,
        List<MotorRef> followers,
        List<EncoderRef> feedback,
        List<Object> refs,
        List<ControlLoopRef> loops,
        List<RuleRef> rules,
        ConstraintRef constraints) {
    public ControlRef {
        mode = mode == null ? ControlMode.POSITION : mode;
        followers = followers == null ? List.of() : List.copyOf(followers);
        feedback = feedback == null ? List.of() : List.copyOf(feedback);
        refs = refs == null ? List.of() : List.copyOf(refs);
        loops = loops == null ? List.of() : List.copyOf(loops);
        rules = rules == null ? List.of() : List.copyOf(rules);
        constraints = constraints == null ? ConstraintRef.none() : constraints;
    }

    public ControlRef output(MotorRef output) {
        return new ControlRef(mode, Objects.requireNonNull(output, "output"), followers, feedback, refs, loops, rules, constraints);
    }

    public ControlRef follower(MotorRef follower) {
        Objects.requireNonNull(follower, "follower");
        List<MotorRef> updated = new ArrayList<>(followers);
        updated.add(follower);
        return new ControlRef(mode, output, updated, feedback, refs, loops, rules, constraints);
    }

    public ControlRef followers(MotorRef... followers) {
        ControlRef updated = this;
        for (MotorRef follower : followers) {
            updated = updated.follower(follower);
        }
        return updated;
    }

    public ControlRef feedback(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        List<EncoderRef> updated = new ArrayList<>(feedback);
        updated.add(encoder);
        return new ControlRef(mode, output, followers, updated, refs, loops, rules, constraints);
    }

    public ControlRef feedback(EncoderRef... encoders) {
        ControlRef updated = this;
        for (EncoderRef encoder : encoders) {
            updated = updated.feedback(encoder);
        }
        return updated;
    }

    public ControlRef ref(Object ref) {
        Objects.requireNonNull(ref, "ref");
        List<Object> updated = new ArrayList<>(refs);
        updated.add(ref);
        return new ControlRef(mode, output, followers, feedback, updated, loops, rules, constraints);
    }

    public ControlRef refs(Object... refs) {
        ControlRef updated = this;
        for (Object ref : refs) {
            updated = updated.ref(ref);
        }
        return updated;
    }

    public ControlRef loops(ControlLoopRef... loops) {
        ControlRef updated = this;
        for (ControlLoopRef loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    public ControlRef loop(ControlLoopRef loop) {
        Objects.requireNonNull(loop, "loop");
        List<ControlLoopRef> updated = new ArrayList<>(loops);
        updated.add(loop);
        return new ControlRef(mode, output, followers, feedback, refs, updated, rules, constraints);
    }

    public ControlRef rule(RuleRef rule) {
        Objects.requireNonNull(rule, "rule");
        List<RuleRef> updated = new ArrayList<>(rules);
        updated.add(rule);
        return new ControlRef(mode, output, followers, feedback, refs, loops, updated, constraints);
    }

    public ControlRef rules(RuleRef... rules) {
        ControlRef updated = this;
        for (RuleRef rule : rules) {
            updated = updated.rule(rule);
        }
        return updated;
    }

    public ControlRef pid(double p, double i, double d) {
        return loop(PidRef.of(p, i, d));
    }

    public ControlRef pid(PidRef pid) {
        return loop(pid);
    }

    public ControlRef ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardRef.of(staticGain, velocityGain, gravityGain));
    }

    public ControlRef feedforward(FeedforwardRef feedforward) {
        return loop(feedforward);
    }

    public ControlRef crt(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) {
        return loop(new CrtRef(coarse, fine, ratio));
    }

    public ControlRef constraints(ConstraintRef constraints) {
        return new ControlRef(mode, output, followers, feedback, refs, loops, rules, Objects.requireNonNull(constraints, "constraints"));
    }

    public List<MotorRef> motors() {
        if (output == null) {
            return followers;
        }
        List<MotorRef> motors = new ArrayList<>();
        motors.add(output);
        motors.addAll(followers);
        return List.copyOf(motors);
    }

    public MechanismState set(double target) {
        if (mode == ControlMode.VELOCITY) {
            return velocity(target);
        }
        return position(target);
    }

    public MechanismState set(DoubleSupplier target) {
        if (mode == ControlMode.VELOCITY) {
            return velocity(target);
        }
        return position(target);
    }

    public MechanismState percent(double percent) {
        return States.percent(this, percent);
    }

    public MechanismState percent(DoubleSupplier percent) {
        return States.percent(this, percent);
    }

    public MechanismState voltage(double volts) {
        return States.voltage(this, volts);
    }

    public MechanismState voltage(DoubleSupplier volts) {
        return States.voltage(this, volts);
    }

    public MechanismState position(double position) {
        return States.position(this, position);
    }

    public MechanismState position(DoubleSupplier position) {
        return States.position(this, position);
    }

    public MechanismState velocity(double velocity) {
        return States.velocity(this, velocity);
    }

    public MechanismState velocity(DoubleSupplier velocity) {
        return States.velocity(this, velocity);
    }
}
