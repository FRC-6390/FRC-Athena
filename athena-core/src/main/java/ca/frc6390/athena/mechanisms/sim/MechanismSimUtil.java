package ca.frc6390.athena.mechanisms.sim;

import ca.frc6390.athena.hardware.encoder.Encoder;

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

        double conversion = encoder.getConversion();
        double gearRatio = encoder.getGearRatio();
        double conversionOffset = encoder.getConversionOffset();
        double offset = encoder.getOffset();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;

        double rawPosition = ((position + conversionOffset) / safeConversion + offset) / safeGearRatio;
        double rawVelocity = (velocity / safeConversion) / safeGearRatio;

        if (!Double.isFinite(rawPosition)) {
            rawPosition = 0.0;
        }

        if (!Double.isFinite(rawVelocity)) {
            rawVelocity = 0.0;
        }

        encoder.setSimulatedState(rawPosition, rawVelocity);
    }
}
