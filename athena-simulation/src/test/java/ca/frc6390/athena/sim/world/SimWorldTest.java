package ca.frc6390.athena.sim.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;

class SimWorldTest {
    @Test
    void motorStateClampsOutputAndIntegratesPosition() {
        SimWorld world = new SimWorld();

        SimMotorState motor = world.motor("drive.left")
                .percentOutput(2.0)
                .velocityPerSecond(3.0);
        world.step(0.5);

        assertEquals(1.0, motor.percentOutput());
        assertEquals(1.5, motor.position());
    }

    @Test
    void imuStateIntegratesYawRate() {
        SimWorld world = new SimWorld();

        SimImuState imu = world.imu("gyro")
                .yawDegrees(10.0)
                .yawRateDegreesPerSecond(90.0);
        world.step(0.25);

        assertEquals(32.5, imu.yawDegrees());
    }

    @Test
    void nonFiniteValuesNormalizeToSafeDefaults() {
        SimMotorState motor = new SimMotorState("motor")
                .percentOutput(Double.NaN)
                .velocityPerSecond(Double.POSITIVE_INFINITY)
                .position(Double.NaN);
        SimImuState imu = new SimImuState("imu")
                .yawDegrees(Double.NaN)
                .yawRateDegreesPerSecond(Double.NEGATIVE_INFINITY);

        assertEquals(0.0, motor.percentOutput());
        assertEquals(0.0, motor.velocityPerSecond());
        assertEquals(0.0, motor.position());
        assertEquals(0.0, imu.yawDegrees());
        assertEquals(0.0, imu.yawRateDegreesPerSecond());
    }

    @Test
    void cameraStoresVisionFramesBySpecPath() {
        SimWorld world = new SimWorld();
        var spec = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.SIM, "frontCam"));
        SimVisionCamera camera = world.camera(spec);

        camera.frame(VisionFrame.of(VisionObservation.tag(4, 3.0, 1.0, 2.0, 0.9)));

        assertSame(camera, world.findCamera("vision.front").orElseThrow());
        assertTrue(camera.frame().hasValidTarget());
        assertEquals(4, camera.frame().tagId().orElseThrow());
    }
}
