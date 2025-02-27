package ca.frc6390.athena.sensors.camera;

import edu.wpi.first.math.geometry.Rotation2d;


public interface ConfigurableCamera {

    public static enum CameraSoftware{
        LimeLight,
        PhotonVision,
    }

    Rotation2d getYawRelativeToForwards();

    default double getYawCos() {
        return Math.cos(getYawRelativeToForwards().getRadians());
    }

    default double getYawSin() {
        return Math.sin(getYawRelativeToForwards().getRadians());
    }

    CameraSoftware getSoftware();

    String getTable();
}
