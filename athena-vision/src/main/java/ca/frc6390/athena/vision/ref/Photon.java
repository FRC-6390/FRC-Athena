package ca.frc6390.athena.vision.ref;

/**
 * PhotonVision camera entry points.
 */
public final class Photon {
    private Photon() {
    }

    /**
     * Creates a PhotonVision camera ref.
     *
     * @param name camera name
     * @return Photon ref
     */
    public static PhotonRef camera(String name) {
        return new PhotonRef(name);
    }
}
