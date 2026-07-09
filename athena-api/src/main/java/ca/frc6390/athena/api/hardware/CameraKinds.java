package ca.frc6390.athena.api.hardware;

/**
 * Common built-in camera kinds known to Athena.
 */
public enum CameraKinds implements CameraKind {
    /** PhotonVision camera pipeline. */
    PHOTONVISION("photonvision:camera"),

    /** Limelight camera pipeline. */
    LIMELIGHT("limelight:camera"),

    /** HeliOS camera pipeline. */
    HELIOS("helios:camera"),

    /** Simulation-only camera source. */
    SIM("sim:camera");

    private final String key;

    CameraKinds(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }
}
