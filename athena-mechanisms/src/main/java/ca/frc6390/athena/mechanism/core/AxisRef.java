package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.mechanism.ref.CrtRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Explicit commandable axis binding between hardware refs, encoder refs, and control loops.
 */
public record AxisRef(
        AxisKind kind,
        List<MotorRef> motors,
        List<EncoderRef> encoders,
        List<DigitalInputRef> sensors,
        List<Object> refs,
        RangeRef range,
        GearRatioRef gearRatio,
        List<ControlLoopRef> loops,
        List<RuleRef> rules,
        BlockPolicy blockPolicy) {
    public AxisRef {
        kind = kind == null ? AxisKind.CUSTOM : kind;
        motors = motors == null ? List.of() : List.copyOf(motors);
        encoders = encoders == null ? List.of() : List.copyOf(encoders);
        sensors = sensors == null ? List.of() : List.copyOf(sensors);
        refs = refs == null ? List.of() : List.copyOf(refs);
        loops = loops == null ? List.of() : List.copyOf(loops);
        rules = rules == null ? List.of() : List.copyOf(rules);
        blockPolicy = blockPolicy == null ? BlockPolicy.NEUTRAL : blockPolicy;
    }

    public static AxisRef of(AxisKind kind) {
        return new AxisRef(kind, List.of(), List.of(), List.of(), List.of(), null, null, List.of(), List.of(), BlockPolicy.NEUTRAL);
    }

    public AxisRef motor(MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        List<MotorRef> updated = new ArrayList<>(motors);
        updated.add(motor);
        return withMotors(updated);
    }

    public AxisRef motors(MotorRef... motors) {
        AxisRef updated = this;
        for (MotorRef motor : motors) {
            updated = updated.motor(motor);
        }
        return updated;
    }

    public AxisRef encoder(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        List<EncoderRef> updated = new ArrayList<>(encoders);
        updated.add(encoder);
        return withEncoders(updated);
    }

    public AxisRef encoders(EncoderRef... encoders) {
        AxisRef updated = this;
        for (EncoderRef encoder : encoders) {
            updated = updated.encoder(encoder);
        }
        return updated;
    }

    public AxisRef sensor(DigitalInputRef sensor) {
        Objects.requireNonNull(sensor, "sensor");
        List<DigitalInputRef> updated = new ArrayList<>(sensors);
        updated.add(sensor);
        return withSensors(updated);
    }

    public AxisRef ref(Object ref) {
        Objects.requireNonNull(ref, "ref");
        List<Object> updated = new ArrayList<>(refs);
        updated.add(ref);
        return withRefs(updated);
    }

    public AxisRef refs(Object... refs) {
        AxisRef updated = this;
        for (Object ref : refs) {
            updated = updated.ref(ref);
        }
        return updated;
    }

    public AxisRef range(RangeRef range) {
        return new AxisRef(kind, motors, encoders, sensors, refs, Objects.requireNonNull(range, "range"), gearRatio, loops, rules, blockPolicy);
    }

    public AxisRef gearRatio(GearRatioRef gearRatio) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, Objects.requireNonNull(gearRatio, "gearRatio"), loops, rules, blockPolicy);
    }

    public AxisRef loops(ControlLoopRef... loops) {
        AxisRef updated = this;
        for (ControlLoopRef loop : loops) {
            updated = updated.loop(loop);
        }
        return updated;
    }

    public AxisRef loop(ControlLoopRef loop) {
        Objects.requireNonNull(loop, "loop");
        List<ControlLoopRef> updated = new ArrayList<>(loops);
        updated.add(loop);
        return withLoops(updated);
    }

    public AxisRef rule(RuleRef rule) {
        Objects.requireNonNull(rule, "rule");
        List<RuleRef> updated = new ArrayList<>(rules);
        updated.add(rule);
        return withRules(updated);
    }

    public AxisRef rules(RuleRef... rules) {
        AxisRef updated = this;
        for (RuleRef rule : rules) {
            updated = updated.rule(rule);
        }
        return updated;
    }

    public AxisRef onBlocked(BlockPolicy blockPolicy) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, Objects.requireNonNull(blockPolicy, "blockPolicy"));
    }

    public AxisRef pid(double p, double i, double d) {
        return loop(PidRef.of(p, i, d));
    }

    public AxisRef pid(PidRef pid) {
        return loop(pid);
    }

    public AxisRef ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardRef.of(staticGain, velocityGain, gravityGain));
    }

    public AxisRef feedforward(FeedforwardRef feedforward) {
        return loop(feedforward);
    }

    public AxisRef crt(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) {
        return loop(new CrtRef(coarse, fine, ratio));
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

    private AxisRef withMotors(List<MotorRef> motors) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }

    private AxisRef withEncoders(List<EncoderRef> encoders) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }

    private AxisRef withSensors(List<DigitalInputRef> sensors) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }

    private AxisRef withRefs(List<Object> refs) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }

    private AxisRef withLoops(List<ControlLoopRef> loops) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }

    private AxisRef withRules(List<RuleRef> rules) {
        return new AxisRef(kind, motors, encoders, sensors, refs, range, gearRatio, loops, rules, blockPolicy);
    }
}
