package ca.frc6390.athena.hardware.sim;

import ca.frc6390.athena.hardware.device.GearRatio;
import java.util.OptionalDouble;

/**
 * Hardware-linked simulation metadata.
 *
 * @param kind simulation model kind
 * @param momentOfInertia moment of inertia, when applicable
 * @param lengthMeters mechanism length, when applicable
 * @param gearRatio gear ratio, when applicable
 * @param simulatesGravity true when gravity should be modeled
 */
public record SimProfile(
        Kind kind,
        OptionalDouble momentOfInertia,
        OptionalDouble lengthMeters,
        GearRatio gearRatio,
        boolean simulatesGravity) {
    public SimProfile {
        kind = kind == null ? Kind.MOTOR : kind;
        momentOfInertia = momentOfInertia == null ? OptionalDouble.empty() : momentOfInertia;
        lengthMeters = lengthMeters == null ? OptionalDouble.empty() : lengthMeters;
    }

    /**
     * Simulation model kind.
     */
    public enum Kind {
        MOTOR,
        ARM,
        FLYWHEEL
    }

    /**
     * Returns a copy with moment of inertia.
     *
     * @param momentOfInertia moment of inertia
     * @return updated profile
     */
    public SimProfile momentOfInertia(double momentOfInertia) {
        return new SimProfile(kind, OptionalDouble.of(momentOfInertia), lengthMeters, gearRatio, simulatesGravity);
    }

    /**
     * Returns a copy with mechanism length.
     *
     * @param lengthMeters length in meters
     * @return updated profile
     */
    public SimProfile lengthMeters(double lengthMeters) {
        return new SimProfile(kind, momentOfInertia, OptionalDouble.of(lengthMeters), gearRatio, simulatesGravity);
    }

    /**
     * Returns a copy with gear ratio.
     *
     * @param gearRatio gear ratio
     * @return updated profile
     */
    public SimProfile gearRatio(GearRatio gearRatio) {
        return new SimProfile(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity);
    }

    /**
     * Returns a copy with gravity simulation enabled or disabled.
     *
     * @param simulatesGravity true to model gravity
     * @return updated profile
     */
    public SimProfile gravity(boolean simulatesGravity) {
        return new SimProfile(kind, momentOfInertia, lengthMeters, gearRatio, simulatesGravity);
    }
}
