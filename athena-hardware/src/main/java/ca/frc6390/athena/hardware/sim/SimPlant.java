package ca.frc6390.athena.hardware.sim;

/**
 * Identified plant of a mechanism, in the same units as the feedforward model it came from.
 *
 * <p>The simulated mechanism obeys the feedforward equation
 * {@code V = S * sign(v) + kV * v + kA * a + G(p)}, where {@code S} is the kinetic friction while
 * moving (the static friction holds the mechanism at rest until its breakaway), {@code G(p)} is gravity
 * ({@code kG cos(2 pi p + offset)} for rotary mechanisms, {@code kG} for linear ones). Motor constants
 * convert the volts into mechanism-side torque (rotary) or force (linear), so the simulated inertia is
 * {@code J = kA * n * G * Kt / (2 pi R)}, the same relation the characterization uses to report kA.</p>
 *
 * @param rotary true for rotary mechanisms (positions in rotations), false for linear (meters)
 * @param staticFrictionVolts breakaway friction voltage (stiction)
 * @param kineticFrictionVolts friction voltage while moving
 * @param kVVoltsPerUnitPerSecond viscous voltage per unit per second
 * @param kAVoltsPerUnitPerSecondSquared voltage per unit per second squared
 * @param gravityVolts kG: gravity voltage amplitude (rotary) or constant gravity voltage (linear)
 * @param gravityOffsetRadians phase of the rotary gravity term
 * @param torqueConstantNmPerAmp motor torque constant per motor, N m per A
 * @param resistanceOhms motor winding resistance per motor
 * @param motorCount motors driving the mechanism
 * @param gearRatio motor turns per mechanism unit
 */
public record SimPlant(
        boolean rotary,
        double staticFrictionVolts,
        double kineticFrictionVolts,
        double kVVoltsPerUnitPerSecond,
        double kAVoltsPerUnitPerSecondSquared,
        double gravityVolts,
        double gravityOffsetRadians,
        double torqueConstantNmPerAmp,
        double resistanceOhms,
        int motorCount,
        double gearRatio) {

    public SimPlant {
        requireFinite(staticFrictionVolts, "staticFrictionVolts");
        requireFinite(kineticFrictionVolts, "kineticFrictionVolts");
        requireFinite(kVVoltsPerUnitPerSecond, "kV");
        requireFinite(gravityVolts, "gravityVolts");
        requireFinite(gravityOffsetRadians, "gravityOffsetRadians");
        if (!(kAVoltsPerUnitPerSecondSquared > 0.0) || !Double.isFinite(kAVoltsPerUnitPerSecondSquared)) {
            throw new IllegalArgumentException("kA must be positive and finite.");
        }
        if (!(torqueConstantNmPerAmp > 0.0) || !Double.isFinite(torqueConstantNmPerAmp)) {
            throw new IllegalArgumentException("Motor torque constant must be positive and finite.");
        }
        if (!(resistanceOhms > 0.0) || !Double.isFinite(resistanceOhms)) {
            throw new IllegalArgumentException("Motor resistance must be positive and finite.");
        }
        if (motorCount <= 0) {
            throw new IllegalArgumentException("Motor count must be positive.");
        }
        if (!(gearRatio > 0.0) || !Double.isFinite(gearRatio)) {
            throw new IllegalArgumentException("Gear ratio must be positive and finite.");
        }
        if (staticFrictionVolts < 0.0 || kineticFrictionVolts < 0.0) {
            throw new IllegalArgumentException("Friction voltages cannot be negative.");
        }
    }

    /**
     * Mechanism-side torque (rotary, N m) or force (linear, N) per volt applied to the motors. Multiplying
     * by {@code kA / (2 pi)} gives the rotary moment of inertia, and by {@code kA} divided by the drum radius
     * gives the linear mass.
     */
    public double outputPerVolt() {
        return motorCount * gearRatio * torqueConstantNmPerAmp / resistanceOhms;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite.");
        }
    }
}
