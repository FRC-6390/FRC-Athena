package ca.frc6390.athena.examples;

import ca.frc6390.athena.hardware.sensor.BlockDirection;
import ca.frc6390.athena.hardware.sensor.SensorSpec;
import ca.frc6390.athena.hardware.sensor.Sensors;
import ca.frc6390.athena.vision.spec.CameraTargetView;

/**
 * Example sensor wrappers for digital sensors and camera targeting.
 */
public final class SensorExample {
    private SensorExample() {
    }

    /**
     * Creates an arm lower-limit switch with hardstop metadata.
     *
     * @return sensor spec
     */
    public static SensorSpec armLowerLimit() {
        return Sensors.sensor("arm", "lowerLimit", sensor -> sensor
                .limitSwitch(2)
                .hardstop(BlockDirection.NEGATIVE, -12.5));
    }

    /**
     * Creates an intake beam-break sensor.
     *
     * @return sensor spec
     */
    public static SensorSpec intakeLoaded() {
        return Sensors.sensor("intake", "loaded", sensor -> sensor.beamBreak(0));
    }

    /**
     * Creates a camera target view from the example vision frame.
     *
     * @return target view
     */
    public static CameraTargetView cameraTarget() {
        return new CameraTargetView(VisionExample.sampleFrame());
    }
}
