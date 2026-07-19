package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.List;
import org.junit.jupiter.api.Test;

class ImusTest {
    @Test
    void relativeSourceMaintainsIndependentHeadingOffset() {
        double[] yaw = {35.0};
        ImuSource physical = Imus.yaw(() -> yaw[0]);
        ImuSource relative = physical.relative();

        assertEquals(0.0, relative.yawDegrees(), 1.0e-9);
        yaw[0] = 50.0;
        assertEquals(15.0, relative.yawDegrees(), 1.0e-9);

        relative.applyYaw(ActionContext.empty(), -10.0);
        assertEquals(-10.0, relative.yawDegrees(), 1.0e-9);
        assertEquals(50.0, physical.yawDegrees(), 1.0e-9);
    }

    @Test
    void yawOnlySourceRejectsMeasurementsItDoesNotProvide() {
        ImuSource source = Imus.yaw(() -> 12.0);

        assertThrows(UnsupportedOperationException.class, source::pitchDegrees);
        assertThrows(UnsupportedOperationException.class, source::linearAccelerationXG);
    }

    @Test
    void invertedSourceNegatesReadingsAndTranslatesYawMutations() {
        RecordingSource physical = new RecordingSource();
        ImuSource inverted = physical.inverted();

        assertEquals(-30.0, inverted.yawDegrees(), 1.0e-9);
        assertEquals(12.0, inverted.pitchDegrees(), 1.0e-9);
        assertEquals(-8.0, inverted.rollDegrees(), 1.0e-9);
        assertEquals(-390.0, inverted.angleDegrees(), 1.0e-9);
        assertEquals(-45.0, inverted.yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-0.1, inverted.linearAccelerationXG(), 1.0e-9);
        assertEquals(0.2, inverted.linearAccelerationYG(), 1.0e-9);
        assertEquals(-0.3, inverted.linearAccelerationZG(), 1.0e-9);
        assertEquals(physical.dependencies(), inverted.dependencies());

        inverted.applyYaw(ActionContext.empty(), 25.0);
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

        assertEquals(-12.0, transformed.rollDegrees(), 1.0e-9);
        assertEquals(8.0, transformed.pitchDegrees(), 1.0e-9);
        assertEquals(-30.0, transformed.yawDegrees(), 1.0e-9);
        assertEquals(2.0, transformed.rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(1.0, transformed.pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-45.0, transformed.yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(-0.2, transformed.linearAccelerationXG(), 1.0e-9);
        assertEquals(0.1, transformed.linearAccelerationYG(), 1.0e-9);
        assertEquals(-0.3, transformed.linearAccelerationZG(), 1.0e-9);

        transformed.applyYaw(ActionContext.empty(), 25.0);
        assertEquals(-25.0, physical.requestedYaw, 1.0e-9);
    }

    @Test
    void mountingConvertsPhysicalOrientationAndYawMutationsToRobotFrame() {
        MutableSource physical = new MutableSource();
        physical.yaw = 120.0;
        physical.angle = 840.0;
        physical.rollRate = 1.0;
        physical.accelerationX = 1.0;
        ImuSource mounted = physical.mounting(ImuMount.yawDegrees(90.0));

        assertEquals(30.0, mounted.yawDegrees(), 1.0e-9);
        assertEquals(750.0, mounted.angleDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.pitchDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.rollDegrees(), 1.0e-9);
        assertEquals(0.0, mounted.rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(1.0, mounted.pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(0.0, mounted.linearAccelerationXG(), 1.0e-9);
        assertEquals(1.0, mounted.linearAccelerationYG(), 1.0e-9);

        mounted.applyYaw(ActionContext.empty(), 20.0);
        assertEquals(110.0, physical.requestedYaw, 1.0e-9);
    }

    @Test
    void cardinalMountMatchesEquivalentYawRotation() {
        MutableSource physical = new MutableSource();
        physical.yaw = 120.0;
        ImuSource mounted = physical.mounting(ImuMount.forward(ImuDirection.NEGATIVE_Y)
                .up(ImuDirection.POSITIVE_Z));

        assertEquals(30.0, mounted.yawDegrees(), 1.0e-9);
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
        assertEquals(physical.updated, derived.lastUpdateSeconds(), 1.0e-9);
        assertEquals(true, derived.isFresh(0.1));

        physical.calibrating = true;
        assertEquals(false, derived.isFresh(0.1));
        physical.calibrating = false;
        physical.updated -= 1.0;
        assertEquals(false, derived.isFresh(0.1));
        assertThrows(IllegalArgumentException.class, () -> derived.isFresh(-0.1));
    }

    private static final class RecordingSource implements ImuSource {
        private final Object dependency = new Object();
        private double requestedYaw;

        @Override public double yawDegrees() { return 30.0; }
        @Override public double pitchDegrees() { return -12.0; }
        @Override public double rollDegrees() { return 8.0; }
        @Override public double angleDegrees() { return 390.0; }
        @Override public double yawRateDegreesPerSecond() { return 45.0; }
        @Override public double pitchRateDegreesPerSecond() { return 2.0; }
        @Override public double rollRateDegreesPerSecond() { return 1.0; }
        @Override public double linearAccelerationXG() { return 0.1; }
        @Override public double linearAccelerationYG() { return -0.2; }
        @Override public double linearAccelerationZG() { return 0.3; }
        @Override public List<?> dependencies() { return List.of(dependency); }
        @Override public void applyYaw(ActionContext context, double yawDegrees) { requestedYaw = yawDegrees; }
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

        @Override public double yawDegrees() { return yaw; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double angleDegrees() { return Double.isFinite(angle) ? angle : yaw; }
        @Override public double yawRateDegreesPerSecond() { return yawRate; }
        @Override public double pitchRateDegreesPerSecond() { return pitchRate; }
        @Override public double rollRateDegreesPerSecond() { return rollRate; }
        @Override public double linearAccelerationXG() { return accelerationX; }
        @Override public double linearAccelerationYG() { return accelerationY; }
        @Override public double linearAccelerationZG() { return accelerationZ; }
        @Override public boolean isConnected() { return connected; }
        @Override public boolean isCalibrating() { return calibrating; }
        @Override public double lastUpdateSeconds() { return updated; }
        @Override public void applyYaw(ActionContext context, double yawDegrees) { requestedYaw = yawDegrees; }
    }
}
