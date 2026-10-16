package ca.frc6390.athena.vision.device;

/**
 * PhotonVision pose solve selected on a camera declaration.
 *
 * <p>Single-tag fallback behavior is decided by the adapter: a coprocessor multi-tag choice falls
 * back to lowest ambiguity for one-tag frames, unless single-tag heading is enabled.</p>
 */
public enum PhotonVisionPoseStrategy {
    /** Coprocessor multi-tag solve, with a lowest-ambiguity fallback for single-tag frames. */
    MULTI_TAG_ON_COPROCESSOR,
    /** Lowest-ambiguity single-tag solve for every frame. */
    LOWEST_AMBIGUITY
}
