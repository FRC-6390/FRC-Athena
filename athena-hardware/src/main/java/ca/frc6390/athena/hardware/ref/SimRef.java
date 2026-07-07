package ca.frc6390.athena.hardware.ref;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;

/**
 * Shared simulation model binding for one mechanism.
 */
public record SimRef(
        SimProfile.Kind kind,
        OptionalDouble momentOfInertia,
        OptionalDouble lengthMeters,
        GearRatioRef gearRatio,
        boolean simulatesGravity,
        List<MotorRef> motors,
        List<EncoderRef> encoders,
        List<Object> refs) {
    public SimRef {
        kind = kind == null ? SimProfile.Kind.MOTOR : kind;
        momentOfInertia = momentOfInertia == null ? OptionalDouble.empty() : momentOfInertia;
        lengthMeters = lengthMeters == null ? OptionalDouble.empty() : lengthMeters;
        motors = motors == null ? List.of() : List.copyOf(motors);
        encoders = encoders == null ? List.of() : List.copyOf(encoders);
        refs = refs == null ? List.of() : List.copyOf(refs);
    }

    public static SimRef of(SimProfile.Kind kind) {
        return new SimRef(kind, OptionalDouble.empty(), OptionalDouble.empty(), null, false, List.of(), List.of(), List.of());
    }

    public SimRef momentOfInertia(double momentOfInertia) {
        return new SimRef(kind, OptionalDouble.of(momentOfInertia), lengthMeters, gearRatio, simulatesGravity, motors, encoders, refs);
    }

    public SimRef lengthMeters(double lengthMeters) {
        return new SimRef(kind, momentOfInertia, OptionalDouble.of(lengthMeters), gearRatio, simulatesGravity, motors, encoders, refs);
    }

    public SimRef gearRatio(GearRatioRef gearRatio) {
        return new SimRef(kind, momentOfInertia, lengthMeters, Objects.requireNonNull(gearRatio, "gearRatio"), simulatesGravity, motors, encoders, refs);
    }

    public SimRef gravity(boolean simulatesGravity) {
        return new SimRef(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, encoders, refs);
    }

    public SimRef motor(MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        List<MotorRef> updated = new ArrayList<>(motors);
        updated.add(motor);
        return new SimRef(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, updated, encoders, refs);
    }

    public SimRef motors(MotorRef... motors) {
        SimRef updated = this;
        for (MotorRef motor : motors) {
            updated = updated.motor(motor);
        }
        return updated;
    }

    public SimRef encoder(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        List<EncoderRef> updated = new ArrayList<>(encoders);
        updated.add(encoder);
        return new SimRef(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, updated, refs);
    }

    public SimRef ref(Object ref) {
        Objects.requireNonNull(ref, "ref");
        List<Object> updated = new ArrayList<>(refs);
        updated.add(ref);
        return new SimRef(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, encoders, updated);
    }

    public SimRef refs(Object... refs) {
        SimRef updated = this;
        for (Object ref : Arrays.asList(refs)) {
            updated = updated.ref(ref);
        }
        return updated;
    }

    public SimRef range(RangeRef range) {
        return ref(range);
    }

    public SimRef limit(DigitalInputRef sensor, double position) {
        return limit(sensor, position, 0.25);
    }

    public SimRef limit(DigitalInputRef sensor, double position, double tolerance) {
        return ref(new SimLimitRef(sensor, position, tolerance));
    }
}
