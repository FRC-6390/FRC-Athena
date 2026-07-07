package ca.frc6390.athena.vision.ref;

/**
 * HeliOS camera entry points.
 */
public final class HeliOS {
    private HeliOS() {
    }

    /**
     * Creates a HeliOS camera ref.
     *
     * @param address camera name or address
     * @return HeliOS ref
     */
    public static HeliOSRef camera(String address) {
        return new HeliOSRef(address);
    }
}
