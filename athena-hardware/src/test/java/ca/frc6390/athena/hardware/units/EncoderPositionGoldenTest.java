package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/**
 * Golden raw-rotation values for the position inverse, from the documented formula
 * {@code raw = (position / (gearRatio * conversion) + offset)}, negated when inverted. Pins the numbers that
 * the typed unit conversion must keep.
 */
class EncoderPositionGoldenTest {
    private static final double TOLERANCE = 1.0e-12;

    @Test
    void defaultRotationEncoderIsOneRawTurnPerRotation() {
        assertEquals(-1.5, EncoderDevice.of(EncoderKinds.CANCODER, 2).rotationsFromPosition(Units.Rotations.of(-1.5)), TOLERANCE);
    }

    @Test
    void degreesWithReductionMatchesTheInverse() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees).reduction(100.0);
        assertEquals(25.0, encoder.rotationsFromPosition(Units.Degrees.of(90.0)), TOLERANCE);
        assertEquals(0.0, encoder.rotationsFromPosition(Units.Degrees.of(0.0)), TOLERANCE);
    }

    @Test
    void offsetAndInversionApplyAfterTheScale() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees).reduction(100.0)
                .offset(0.25).inverted();
        assertEquals(-25.25, encoder.rotationsFromPosition(Units.Degrees.of(90.0)), TOLERANCE);
    }

    @Test
    void rotationsWithGearingMatchTheInverse() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Rotations).reduction(4.0);
        assertEquals(8.0, encoder.rotationsFromPosition(Units.Rotations.of(2.0)), TOLERANCE);
    }

    @Test
    void radiansMatchTheInverse() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Radians);
        assertEquals(0.5, encoder.rotationsFromPosition(Units.Radians.of(Math.PI)), TOLERANCE);
    }

    @Test
    void wheelTravelMatchesTheInverse() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Meters).reduction(6.75)
                .travelPerRevolution(Units.Meters.of(0.3048));
        assertEquals(6.75, encoder.rotationsFromPosition(Units.Meters.of(0.3048)), 1.0e-9);
    }
}
