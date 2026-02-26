package ca.frc6390.athena.core.localization;

import java.util.EnumSet;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public record PoseConfig(
        String name,
        PoseFrame frame,
        EnumSet<PoseInput> continuousInputs,
        EnumSet<PoseInput> onDemandInputs,
        PoseConstraints constraints,
        RobotLocalizationConfig.BackendConfig backendOverride,
        boolean active,
        boolean publishToNetworkTables,
        Pose2d startPose2d,
        Pose3d startPose3d) {

    public PoseConfig {
        Objects.requireNonNull(name, "name");
        if (name.isBlank()) {
            throw new IllegalArgumentException("Pose name cannot be blank.");
        }
        frame = frame != null ? frame : PoseFrame.FIELD;
        continuousInputs = continuousInputs != null
                ? EnumSet.copyOf(continuousInputs)
                : EnumSet.noneOf(PoseInput.class);
        onDemandInputs = onDemandInputs != null
                ? EnumSet.copyOf(onDemandInputs)
                : EnumSet.noneOf(PoseInput.class);
        constraints = constraints != null ? constraints : PoseConstraints.defaults();
    }
}
