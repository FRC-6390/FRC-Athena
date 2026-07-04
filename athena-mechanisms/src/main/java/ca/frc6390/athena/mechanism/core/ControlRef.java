package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Explicit commandable binding between hardware refs, feedback refs, and a control method.
 */
public record ControlRef(
        ControlKind kind,
        List<MotorRef> motors,
        List<EncoderRef> feedback,
        List<DigitalInputRef> sensors,
        List<Object> refs,
        RangeRef range,
        GearRatioRef gearRatio,
        List<ControlMethod> methods) {
    public ControlRef {
        kind = kind == null ? ControlKind.CUSTOM : kind;
        motors = motors == null ? List.of() : List.copyOf(motors);
        feedback = feedback == null ? List.of() : List.copyOf(feedback);
        sensors = sensors == null ? List.of() : List.copyOf(sensors);
        refs = refs == null ? List.of() : List.copyOf(refs);
        methods = methods == null ? List.of() : List.copyOf(methods);
    }

    public static ControlRef of(ControlKind kind) {
        return new ControlRef(kind, List.of(), List.of(), List.of(), List.of(), null, null, List.of());
    }

    public ControlRef motor(MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        List<MotorRef> updated = new ArrayList<>(motors);
        updated.add(motor);
        return withMotors(updated);
    }

    public ControlRef motors(MotorRef... motors) {
        ControlRef updated = this;
        for (MotorRef motor : motors) {
            updated = updated.motor(motor);
        }
        return updated;
    }

    public ControlRef feedback(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        List<EncoderRef> updated = new ArrayList<>(feedback);
        updated.add(encoder);
        return withFeedback(updated);
    }

    public ControlRef sensor(DigitalInputRef sensor) {
        Objects.requireNonNull(sensor, "sensor");
        List<DigitalInputRef> updated = new ArrayList<>(sensors);
        updated.add(sensor);
        return withSensors(updated);
    }

    public ControlRef ref(Object ref) {
        Objects.requireNonNull(ref, "ref");
        List<Object> updated = new ArrayList<>(refs);
        updated.add(ref);
        return withRefs(updated);
    }

    public ControlRef refs(Object... refs) {
        ControlRef updated = this;
        for (Object ref : refs) {
            updated = updated.ref(ref);
        }
        return updated;
    }

    public ControlRef range(RangeRef range) {
        return new ControlRef(kind, motors, feedback, sensors, refs, Objects.requireNonNull(range, "range"), gearRatio, methods);
    }

    public ControlRef gearRatio(GearRatioRef gearRatio) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, Objects.requireNonNull(gearRatio, "gearRatio"), methods);
    }

    public ControlRef using(ControlMethod... methods) {
        ControlRef updated = this;
        for (ControlMethod method : methods) {
            updated = updated.method(method);
        }
        return updated;
    }

    public ControlRef method(ControlMethod method) {
        Objects.requireNonNull(method, "method");
        List<ControlMethod> updated = new ArrayList<>(methods);
        updated.add(method);
        return withMethods(updated);
    }

    public ControlRef pid(double p, double i, double d) {
        return method(ControlMethods.pid(p, i, d));
    }

    public ControlRef pid(PidRef pid) {
        return method(ControlMethods.pid(pid));
    }

    public ControlRef ff(double staticGain, double velocityGain, double gravityGain) {
        return method(ControlMethods.ff(staticGain, velocityGain, gravityGain));
    }

    public ControlRef feedforward(FeedforwardRef feedforward) {
        return method(ControlMethods.feedforward(feedforward));
    }

    public ControlRef crt(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) {
        return method(ControlMethods.crt(coarse, fine, ratio));
    }

    public ControlRef custom(String kind, Object... refs) {
        return method(ControlMethods.custom(kind, Arrays.copyOf(refs, refs.length)));
    }

    private ControlRef withMotors(List<MotorRef> motors) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, gearRatio, methods);
    }

    private ControlRef withFeedback(List<EncoderRef> feedback) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, gearRatio, methods);
    }

    private ControlRef withSensors(List<DigitalInputRef> sensors) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, gearRatio, methods);
    }

    private ControlRef withRefs(List<Object> refs) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, gearRatio, methods);
    }

    private ControlRef withMethods(List<ControlMethod> methods) {
        return new ControlRef(kind, motors, feedback, sensors, refs, range, gearRatio, methods);
    }
}
