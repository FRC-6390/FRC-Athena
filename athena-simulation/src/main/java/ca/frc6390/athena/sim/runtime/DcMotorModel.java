package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;

/**
 * Linear DC motor model derived from the same catalogue values WPILib's {@code DCMotor} uses for the
 * physical motor kinds: nominal 12 V, stall torque, stall current, free current and free speed.
 *
 * <p>Per motor, with motor speed {@code w} (rad/s), applied voltage {@code V} and winding resistance
 * {@code R}:</p>
 * <ul>
 *   <li>current {@code I = (V - Ke w) / R}, torque {@code Kt I - b w};</li>
 *   <li>{@code Kt = stallTorque / stallCurrent}, {@code R = nominal / stallCurrent},
 *       {@code Ke = (nominal - freeCurrent R) / freeSpeed} (so the no-load speed is the free speed),
 *       and {@code b = Kt freeCurrent / freeSpeed} is the friction that makes the free-spinning motor draw
 *       its free current.</li>
 * </ul>
 */
record DcMotorModel(
        double nominalVoltage,
        double torqueConstantNmPerAmp,
        double backEmfVoltsPerRadPerSecond,
        double resistanceOhms,
        double viscousNmPerRadPerSecond,
        double freeSpeedRadPerSecond) {

    static final double NOMINAL_VOLTAGE = 12.0;

    /** Model used for motor kinds without a catalogue entry (a generic brushless motor). */
    static final DcMotorModel GENERIC = of(MotorKinds.KRAKEN_X60);

    static DcMotorModel of(MotorKind kind) {
        if (kind instanceof MotorKinds motorKind) {
            return switch (motorKind) {
                case FALCON_500 -> from(4.69, 257.0, 1.5, 668.112);
                case KRAKEN_X60 -> from(7.09, 366.0, 2.0, 628.319);
                case KRAKEN_X44 -> from(4.11, 279.0, 2.0, 812.416);
                case MINION -> from(3.17, 211.0, 2.0, 806.761);
                case NEO -> from(2.60, 105.0, 1.8, 594.389);
                case NEO_550 -> from(0.97, 100.0, 1.4, 1151.917);
                case NEO_VORTEX -> from(3.60, 211.0, 3.6, 710.419);
                case CIM -> from(2.42, 133.0, 2.7, 556.062);
                case MINI_CIM -> from(1.41, 89.0, 3.0, 611.563);
                case BAG -> from(0.43, 53.0, 1.8, 1380.206);
                case VEX_775_PRO -> from(0.71, 134.0, 0.7, 1961.401);
                case ANDYMARK_9015 -> from(0.36, 71.0, 3.7, 1494.351);
                case ANDYMARK_RS775_125 -> from(0.28, 18.0, 1.6, 607.375);
                case BANEBOTS_RS550 -> from(0.38, 84.0, 0.4, 1989.675);
                case BANEBOTS_RS775 -> from(0.72, 97.0, 2.7, 1366.593);
            };
        }
        return GENERIC;
    }

    private static DcMotorModel from(double stallTorque, double stallCurrent, double freeCurrent, double freeSpeed) {
        double resistance = NOMINAL_VOLTAGE / stallCurrent;
        double torqueConstant = stallTorque / stallCurrent;
        double backEmf = (NOMINAL_VOLTAGE - freeCurrent * resistance) / freeSpeed;
        double viscous = torqueConstant * freeCurrent / freeSpeed;
        return new DcMotorModel(NOMINAL_VOLTAGE, torqueConstant, backEmf, resistance, viscous, freeSpeed);
    }

    /** Motor current for an applied voltage at a motor speed, before any current limit. */
    double current(double volts, double motorSpeedRadPerSecond) {
        return (volts - backEmfVoltsPerRadPerSecond * motorSpeedRadPerSecond) / resistanceOhms;
    }

    /** Shaft torque for a motor current and speed. */
    double torque(double amps, double motorSpeedRadPerSecond) {
        return torqueConstantNmPerAmp * amps - viscousNmPerRadPerSecond * motorSpeedRadPerSecond;
    }

    /**
     * Motor-side damping coefficient of a shorted winding (N m s/rad). Together with the axis inertia it sets
     * the fastest time constant the integrator must resolve.
     */
    double motorSideDampingNmSPerRad() {
        return (torqueConstantNmPerAmp * backEmfVoltsPerRadPerSecond / resistanceOhms) + viscousNmPerRadPerSecond;
    }
}
