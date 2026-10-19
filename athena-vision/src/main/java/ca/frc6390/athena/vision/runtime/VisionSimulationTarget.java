package ca.frc6390.athena.vision.runtime;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

/**
 * Field-relative target used by vendor-backed vision simulation.
 *
 * @param id target id, or {@code -1} when unavailable
 * @param pose field-relative target pose
 * @param height target center height
 * @param confidence target confidence where higher is better
 */
public record VisionSimulationTarget(int id, PoseSnapshot pose, Distance height, double confidence) {
    public VisionSimulationTarget {
        pose = Objects.requireNonNull(pose, "pose");
        height = Objects.requireNonNull(height, "height");
        confidence = finiteOrZero(confidence);
    }

    /**
     * Creates an AprilTag-style field target.
     *
     * @param id target id
     * @param x field x
     * @param y field y
     * @param z target center height
     * @param heading target heading
     * @return target
     */
    public static VisionSimulationTarget aprilTag(
            int id,
            Distance x,
            Distance y,
            Distance z,
            Angle heading) {
        return new VisionSimulationTarget(id,
                new PoseSnapshot(x.in(Meters), y.in(Meters), heading.in(Radians)),
                z, 1.0);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
