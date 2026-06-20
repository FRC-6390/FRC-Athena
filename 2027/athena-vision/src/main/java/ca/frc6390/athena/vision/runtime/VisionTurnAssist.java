package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.vision.spec.VisionFrame;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Runtime helper that converts camera yaw observations into robot-speed feedback.
 */
public final class VisionTurnAssist {
    private final RobotSpeeds speeds;
    private final Supplier<VisionFrame> frames;
    private final double proportionalGain;
    private final double toleranceRadians;

    /**
     * Creates a vision turn-assist helper.
     *
     * @param speeds robot speed blender
     * @param frames camera frame supplier
     * @param proportionalGain yaw-to-angular velocity gain
     * @param toleranceRadians aligned tolerance
     */
    public VisionTurnAssist(
            RobotSpeeds speeds,
            Supplier<VisionFrame> frames,
            double proportionalGain,
            double toleranceRadians) {
        this.speeds = Objects.requireNonNull(speeds, "speeds");
        this.frames = Objects.requireNonNull(frames, "frames");
        this.proportionalGain = finiteOrZero(proportionalGain);
        this.toleranceRadians = Math.abs(finiteOrZero(toleranceRadians));
    }

    /**
     * Reads the latest frame and writes angular feedback.
     */
    public void execute() {
        VisionFrame frame = latestFrame();
        double angular = frame.yawDegrees().isPresent()
                ? Math.toRadians(frame.yawDegrees().orElseThrow()) * proportionalGain
                : 0.0;
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, angular);
    }

    /**
     * Returns whether the latest visible target is inside tolerance.
     *
     * @return true when aligned
     */
    public boolean isAligned() {
        VisionFrame frame = latestFrame();
        return frame.yawDegrees().isPresent()
                && Math.abs(Math.toRadians(frame.yawDegrees().orElseThrow())) <= toleranceRadians;
    }

    /**
     * Clears feedback output.
     */
    public void stop() {
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.0);
    }

    private VisionFrame latestFrame() {
        VisionFrame frame = frames.get();
        return frame == null ? VisionFrame.noTarget() : frame;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
