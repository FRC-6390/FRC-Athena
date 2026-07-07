package ca.frc6390.athena.vision.ref;

/**
 * Limelight camera entry points.
 */
public final class Limelight {
    private Limelight() {
    }

    /**
     * Creates a Limelight camera ref.
     *
     * @param name Limelight network name
     * @return Limelight ref
     */
    public static LimelightRef camera(String name) {
        return new LimelightRef(name);
    }
}
