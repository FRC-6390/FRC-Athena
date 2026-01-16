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

    public static PoseConfig custom(String name) {
        return custom(name, (Pose2d) null);
    }

    public static PoseConfig custom(String name, Pose2d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.noneOf(PoseInput.class),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                startPose,
                null);
    }

    public static PoseConfig custom(String name, Pose3d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.noneOf(PoseInput.class),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                null,
                startPose);
    }

    public static PoseConfig field(String name) {
        return field(name, (Pose2d) null);
    }

    public static PoseConfig field(String name, Pose2d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.noneOf(PoseInput.class),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                startPose,
                null);
    }

    public static PoseConfig field(String name, Pose3d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.noneOf(PoseInput.class),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                null,
                startPose);
    }

    public PoseConfig withFrame(PoseFrame frame) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withContinuousInputs(EnumSet<PoseInput> inputs) {
        return new PoseConfig(
                name,
                frame,
                inputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withOnDemandInputs(EnumSet<PoseInput> inputs) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                inputs,
                constraints,
                backendOverride,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withConstraints(PoseConstraints constraints) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withBackendOverride(RobotLocalizationConfig.BackendConfig backend) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backend,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withActive(boolean active) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withStartPose(Pose2d startPose) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                startPose,
                null);
    }

    public PoseConfig withStartPose(Pose3d startPose) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                null,
                startPose);
    }
}
