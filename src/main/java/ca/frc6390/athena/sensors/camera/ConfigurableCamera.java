package ca.frc6390.athena.sensors.camera;

public interface ConfigurableCamera {

    enum CameraSoftware {
        LimeLight,
        PhotonVision,
    }

    CameraSoftware getSoftware();

    String getTable();
}
