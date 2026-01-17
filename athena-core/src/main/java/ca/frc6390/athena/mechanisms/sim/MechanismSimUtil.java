package ca.frc6390.athena.mechanisms.sim;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility helpers for converting between simulation units and the library's {@link Encoder} units.
 */
public final class MechanismSimUtil {

    private MechanismSimUtil() {
        /* util class */ 
    }

    /**
     * Apply the supplied mechanism-space position and velocity to the given encoder. Mechanism-space
     * units must match the encoder conversion configured via {@code MechanismConfig#setEncoderConversion}.
     *
     * @param encoder mechanism encoder (may be {@code null})
     * @param position mechanism-space position (e.g. inches, degrees)
     * @param velocity mechanism-space velocity (units per second)
     */
    public static void applyEncoderState(Encoder encoder, double position, double velocity) {
        if (encoder == null) {
            return;
        }

        SimEncoderState state = computeSimEncoderState(encoder, position, velocity);
        encoder.setSimulatedState(state.rawPosition, state.rawVelocity);
    }

    public static void applyEncoderState(Mechanism mechanism, double position, double velocity) {
        if (mechanism == null) {
            return;
        }
        Encoder encoder = mechanism.getEncoder();
        if (encoder == null) {
            if (RobotBase.isSimulation()) {
                mechanism.setSimulatedEncoderState(position, velocity);
            }
            return;
        }

        SimEncoderState state = computeSimEncoderState(encoder, position, velocity);
        encoder.setSimulatedState(state.rawPosition, state.rawVelocity);

        if (RobotBase.isSimulation() && !encoder.supportsSimulation()) {
            mechanism.setSimulatedEncoderState(position, velocity);
        } else {
            mechanism.clearSimulatedEncoderState();
        }
    }

    static SimEncoderState computeSimEncoderState(Encoder encoder, double position, double velocity) {
        double conversion = encoder.getConversion();
        double gearRatio = encoder.getGearRatio();
        double conversionOffset = encoder.getConversionOffset();
        double offset = encoder.getOffset();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;

        double rawPosition = ((position - conversionOffset) / safeConversion) / safeGearRatio - offset;
        double rawVelocity = (velocity / safeConversion) / safeGearRatio;

        if (!Double.isFinite(rawPosition)) {
            rawPosition = 0.0;
        }

        if (!Double.isFinite(rawVelocity)) {
            rawVelocity = 0.0;
        }

        return new SimEncoderState(rawPosition, rawVelocity);
    }

    static final class SimEncoderState {
        final double rawPosition;
        final double rawVelocity;

        SimEncoderState(double rawPosition, double rawVelocity) {
            this.rawPosition = rawPosition;
            this.rawVelocity = rawVelocity;
        }
    }
}
