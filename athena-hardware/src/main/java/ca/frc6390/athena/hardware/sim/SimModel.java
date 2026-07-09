package ca.frc6390.athena.hardware.sim;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;

/**
 * Shared simulation model binding for one mechanism.
 */
public record SimModel(
        SimProfile.Kind kind,
        OptionalDouble momentOfInertia,
        OptionalDouble lengthMeters,
        GearRatio gearRatio,
        boolean simulatesGravity,
        List<MotorDevice> motors,
        List<EncoderDevice> encoders,
        List<Object> dependencies) {
    public SimModel {
        kind = kind == null ? SimProfile.Kind.MOTOR : kind;
        momentOfInertia = momentOfInertia == null ? OptionalDouble.empty() : momentOfInertia;
        lengthMeters = lengthMeters == null ? OptionalDouble.empty() : lengthMeters;
        motors = motors == null ? List.of() : List.copyOf(motors);
        encoders = encoders == null ? List.of() : List.copyOf(encoders);
        dependencies = dependencies == null ? List.of() : List.copyOf(dependencies);
    }

    public static SimModel of(SimProfile.Kind kind) {
        return new SimModel(kind, OptionalDouble.empty(), OptionalDouble.empty(), null, false, List.of(), List.of(), List.of());
    }

    public SimModel momentOfInertia(double momentOfInertia) {
        return new SimModel(kind, OptionalDouble.of(momentOfInertia), lengthMeters, gearRatio, simulatesGravity, motors, encoders, dependencies);
    }

    public SimModel lengthMeters(double lengthMeters) {
        return new SimModel(kind, momentOfInertia, OptionalDouble.of(lengthMeters), gearRatio, simulatesGravity, motors, encoders, dependencies);
    }

    public SimModel gearRatio(GearRatio gearRatio) {
        return new SimModel(kind, momentOfInertia, lengthMeters, Objects.requireNonNull(gearRatio, "gearRatio"), simulatesGravity, motors, encoders, dependencies);
    }

    public SimModel gravity(boolean simulatesGravity) {
        return new SimModel(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, encoders, dependencies);
    }

    public SimModel motor(MotorDevice motor) {
        Objects.requireNonNull(motor, "motor");
        List<MotorDevice> updated = new ArrayList<>(motors);
        updated.add(motor);
        return new SimModel(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, updated, encoders, dependencies);
    }

    public SimModel motors(MotorDevice... motors) {
        SimModel updated = this;
        for (MotorDevice motor : motors) {
            updated = updated.motor(motor);
        }
        return updated;
    }

    public SimModel encoder(EncoderDevice encoder) {
        Objects.requireNonNull(encoder, "encoder");
        List<EncoderDevice> updated = new ArrayList<>(encoders);
        updated.add(encoder);
        return new SimModel(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, updated, dependencies);
    }

    public SimModel dependency(Object dependency) {
        Objects.requireNonNull(dependency, "dependency");
        List<Object> updated = new ArrayList<>(dependencies);
        updated.add(dependency);
        return new SimModel(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity, motors, encoders, updated);
    }

    public SimModel dependencies(Object... dependencies) {
        SimModel updated = this;
        for (Object dependency : Arrays.asList(dependencies)) {
            updated = updated.dependency(dependency);
        }
        return updated;
    }

    public SimModel range(Range range) {
        return dependency(range);
    }

    public SimModel limit(DigitalInputDevice sensor, double position) {
        return limit(sensor, position, 0.25);
    }

    public SimModel limit(DigitalInputDevice sensor, double position, double tolerance) {
        return dependency(new SimLimit(sensor, position, tolerance));
    }
}
