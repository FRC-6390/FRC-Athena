package ca.frc6390.athena.vision.device;

/**
 * A camera that uses the robot's heading in its own solve (for example PhotonVision's heading-constrained
 * single-tag solve). A Kalman {@code Localization} reports its estimated heading every cycle to each such
 * camera upstream of it, so robot code does not have to wire a heading source.
 */
public interface RobotHeadingAware {
    /**
     * Reports the robot's field heading.
     *
     * @param timestampSeconds robot time the heading belongs to
     * @param headingRadians field-relative heading, counter-clockwise positive
     */
    void reportRobotHeading(double timestampSeconds, double headingRadians);
}
