package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class ImusTest {
    @Test
    void relativeSourceMaintainsIndependentHeadingOffset() {
        double[] yaw = {35.0};
        ImuSource physical = Imus.yaw(() -> Units.Degrees.of(yaw[0]));
        ImuSource relative = physical.relative();

        assertEquals(0.0, relative.raw().yawDegrees(), 1.0e-9);
        yaw[0] = 50.0;
        assertEquals(15.0, relative.raw().yawDegrees(), 1.0e-9);

        relative.raw().applyYawDegrees(ActionContext.empty(), -10.0);
        assertEquals(-10.0, relative.raw().yawDegrees(), 1.0e-9);
        assertEquals(50.0, physical.raw().yawDegrees(), 1.0e-9);
    }

    @Test
    void yawOnlySourceRejectsMeasurementsItDoesNotProvide() {
        ImuSource source = Imus.yaw(() -> Units.Degrees.of(12.0));

        assertThrows(UnsupportedOperationException.class, source.raw()::pitchDegrees);
        assertThrows(UnsupportedOperationException.class, source.raw()::linearAccelerationXG);
    }

    @Test
    void invertedSourceNegatesReadingsAndTranslatesYawMutations() {
        RecordingSource physical = new RecordingSource();
        ImuSource inverted = physical.inverted();

        assertEquals(-30.0, inverted.raw().yawDegrees(), 1.0e-9);
        assertEquals(12.0, inverted.raw().pitchDegrees(), 1.0e-9);
        assertEquals(-8.0, inverted.raw().rollDegrees(), 1.0e-9);
        assertEquals(-390.0, inverted.raw().angleDegrees(), 1.0e-9);
        assertEquals(-45.0, inverted.raw().yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-0.1, inverted.raw().linearAccelerationXG(), 1.0e-9);
        assertEquals(0.2, inverted.raw().linearAccelerationYG(), 1.0e-9);
        assertEquals(-0.3, inverted.raw().linearAccelerationZG(), 1.0e-9);
        assertEquals(physical.dependencies(), inverted.dependencies());

        inverted.raw().applyYawDegrees(ActionContext.empty(), 25.0);
        assertEquals(-25.0, physical.requestedYaw, 1.0e-9);
        assertSame(physical, inverted.inverted());
        assertSame(physical, physical.inverted(false));
    }

    @Test
    void transformCanInvertOrRemapIndividualAxes() {
        RecordingSource physical = new RecordingSource();
        ImuSource transformed = physical.transform(ImuTransform.identity()
                .yawInverted()
                .swapRollAndPitch());

        assertEquals(-12.0, transformed.raw().rollDegrees(), 1.0e-9);
        assertEquals(8.0, transformed.raw().pitchDegrees(), 1.0e-9);
        assertEquals(-30.0, transformed.raw().yawDegrees(), 1.0e-9);
        assertEquals(2.0, transformed.raw().rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(1.0, transformed.raw().pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-45.0, transformed.raw().yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-0.2, transformed.raw().linearAccelerationXG(), 1.0e-9);
        assertEquals(0.1, transformed.raw().linearAccelerationYG(), 1.0e-9);
        assertEquals(-0.3, transformed.raw().linearAccelerationZG(), 1.0e-9);

        transformed.raw().applyYawDegrees(ActionContext.empty(), 25.0);
        assertEquals(-25.0, physical.requestedYaw, 1.0e-9);
    }

    @Test
    void mountingConvertsPhysicalOrientationAndYawMutationsToRobotFrame() {
        MutableSource physical = new MutableSource();
        physical.yaw = 120.0;
        physical.angle = 840.0;
        physical.rollRate = 1.0;
        physical.accelerationX = 1.0;
        ImuSource mounted = physical.mounting(ImuMount.yaw(Units.Degrees.of(90.0)));

        assertEquals(30.0, mounted.raw().yawDegrees(), 1.0e-9);
        assertEquals(750.0, mounted.raw().angleDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.raw().pitchDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.raw().rollDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.raw().rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(1.0, mounted.raw().pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(0.0, mounted.raw().linearAccelerationXG(), 1.0e-9);
        assertEquals(1.0, mounted.raw().linearAccelerationYG(), 1.0e-9);

        mounted.raw().applyYawDegrees(ActionContext.empty(), 20.0);
        assertEquals(110.0, physical.requestedYaw, 1.0e-9);
    }

    @Test
    void cardinalMountMatchesEquivalentYawRotation() {
        MutableSource physical = new MutableSource();
        physical.yaw = 120.0;
        ImuSource mounted = physical.mounting(ImuMount.forward(ImuDirection.NEGATIVE_Y)
                .up(ImuDirection.POSITIVE_Z));

        assertEquals(30.0, mounted.raw().yawDegrees(), 1.0e-9);
    }

    @Test
    void derivedSourcesPreserveHealthAndFreshness() {
        MutableSource physical = new MutableSource();
        physical.connected = true;
        physical.calibrating = false;
        physical.updated = System.nanoTime() * 1.0e-9;
        ImuSource derived = physical.relative().inverted()
                .transform(ImuTransform.identity().yawInverted())
                .mounting(ImuMount.identity());

        assertEquals(true, derived.isConnected());
        assertEquals(false, derived.isCalibrating());
        assertEquals(physical.updated, derived.raw().lastUpdateSeconds(), 1.0e-9);
        assertEquals(true, derived.isFresh(Units.Seconds.of(0.1)));

        physical.calibrating = true;
        assertEquals(false, derived.isFresh(Units.Seconds.of(0.1)));
        physical.calibrating = false;
        physical.updated -= 1.0;
        assertEquals(false, derived.isFresh(Units.Seconds.of(0.1)));
        assertThrows(IllegalArgumentException.class, () -> derived.isFresh(Units.Seconds.of(-0.1)));
    }

    private static final class RecordingSource implements ImuSource {
        private final Object dependency = new Object();
        private double requestedYaw;

        @Override public double readYawDegrees() { return 30.0; }
        @Override public double readPitchDegrees() { return -12.0; }
        @Override public double readRollDegrees() { return 8.0; }
        @Override public double readAngleDegrees() { return 390.0; }
        @Override public double readYawRateDegreesPerSecond() { return 45.0; }
        @Override public double readPitchRateDegreesPerSecond() { return 2.0; }
        @Override public double readRollRateDegreesPerSecond() { return 1.0; }
        @Override public double readLinearAccelerationXG() { return 0.1; }
        @Override public double readLinearAccelerationYG() { return -0.2; }
        @Override public double readLinearAccelerationZG() { return 0.3; }
        @Override public List<?> dependencies() { return List.of(dependency); }
        @Override public void writeYawDegrees(ActionContext context, double yawDegrees) { requestedYaw = yawDegrees; }
    }

    private static final class MutableSource implements ImuSource {
        private double yaw;
        private double angle = Double.NaN;
        private double requestedYaw;
        private double rollRate;
        private double pitchRate;
        private double yawRate;
        private double accelerationX;
        private double accelerationY;
        private double accelerationZ;
        private boolean connected = true;
        private boolean calibrating;
        private double updated = System.nanoTime() * 1.0e-9;

        @Override public double readYawDegrees() { return yaw; }
        @Override public double readPitchDegrees() { return 0.0; }
        @Override public double readRollDegrees() { return 0.0; }
        @Override public double readAngleDegrees() { return Double.isFinite(angle) ? angle : yaw; }
        @Override public double readYawRateDegreesPerSecond() { return yawRate; }
        @Override public double readPitchRateDegreesPerSecond() { return pitchRate; }
        @Override public double readRollRateDegreesPerSecond() { return rollRate; }
        @Override public double readLinearAccelerationXG() { return accelerationX; }
        @Override public double readLinearAccelerationYG() { return accelerationY; }
        @Override public double readLinearAccelerationZG() { return accelerationZ; }
        @Override public boolean isConnected() { return connected; }
        @Override public boolean isCalibrating() { return calibrating; }
        @Override public double readLastUpdateSeconds() { return updated; }
        @Override public void writeYawDegrees(ActionContext context, double yawDegrees) { requestedYaw = yawDegrees; }
    }
}
