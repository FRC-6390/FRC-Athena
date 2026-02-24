package ca.frc6390.athena.sensors.examples;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import ca.frc6390.athena.sensors.button.GenericButton;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera.CameraSoftware;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;

/**
 * Examples for common sensor wrapper setup patterns.
 */
public final class SensorWrapperExamples {
    private SensorWrapperExamples() {}

    public static GenericLimitSwitch createHardstopSwitch(
            int port,
            BlockDirection blockDirection,
            double position,
            double delaySeconds) {
        GenericLimitSwitch.GenericLimitSwitchConfig config = GenericLimitSwitch.GenericLimitSwitchConfig
                .create(port)
                .position(position)
                .hardstop(true, blockDirection)
                .delay(delaySeconds);
        return GenericLimitSwitch.fromConfig(config);
    }

    public static GenericButton createButton(int port) {
        return new GenericButton(port);
    }

    public static IRBeamBreak createBeamBreak(int port) {
        return new IRBeamBreak(port);
    }

    public static VisionCamera createTargetingCamera(String table, DoubleSupplier targetYawDegrees) {
        VisionCameraConfig config = VisionCameraConfig.create(table, CameraSoftware.PhotonVision);
        config.config(section -> section
                .hasTargetsSupplier(() -> Double.isFinite(targetYawDegrees.getAsDouble()))
                .targetYawSupplier(targetYawDegrees));
        return new VisionCamera(config);
    }

    public static OptionalDouble readYaw(VisionCamera camera) {
        return camera.getTargetYawDegrees();
    }
}
