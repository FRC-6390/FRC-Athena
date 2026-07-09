package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Objects;

/**
 * Field-relative target used by vendor-backed vision simulation.
 *
 * @param id target id, or {@code -1} when unavailable
 * @param pose field-relative target pose
 * @param heightMeters target center height
 * @param confidence target confidence where higher is better
 */
public record VisionSimulationTarget(int id, PoseSnapshot pose, double heightMeters, double confidence) {
    public VisionSimulationTarget {
        pose = Objects.requireNonNull(pose, "pose");
        heightMeters = finiteOrZero(heightMeters);
        confidence = finiteOrZero(confidence);
    }

    /**
     * Creates an AprilTag-style field target.
     *
     * @param id target id
     * @param xMeters field x
     * @param yMeters field y
     * @param zMeters target center height
     * @param headingRadians target heading
     * @return target
     */
    public static VisionSimulationTarget aprilTag(
            int id,
            double xMeters,
            double yMeters,
            double zMeters,
            double headingRadians) {
        return new VisionSimulationTarget(id, new PoseSnapshot(xMeters, yMeters, headingRadians), zMeters, 1.0);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
