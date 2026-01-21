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
        boolean publishToShuffleboard,
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
                false,
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
                false,
                null,
                startPose);
    }

    public static PoseConfig defaults(String name) {
        return defaults(name, (Pose2d) null);
    }

    public static PoseConfig defaults(String name, Pose2d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.ODOMETRY, PoseInput.IMU_YAW, PoseInput.VISION),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
                startPose,
                null);
    }

    public static PoseConfig defaults(String name, Pose3d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.ODOMETRY, PoseInput.IMU_YAW, PoseInput.VISION),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
                null,
                startPose);
    }

    public static PoseConfig vision(String name) {
        return vision(name, (Pose2d) null);
    }

    public static PoseConfig vision(String name, Pose2d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.VISION),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
                startPose,
                null);
    }

    public static PoseConfig vision(String name, Pose3d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.VISION),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
                null,
                startPose);
    }

    public static PoseConfig odometry(String name) {
        return odometry(name, (Pose2d) null);
    }

    public static PoseConfig odometry(String name, Pose2d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.ODOMETRY),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
                startPose,
                null);
    }

    public static PoseConfig odometry(String name, Pose3d startPose) {
        return new PoseConfig(
                name,
                PoseFrame.FIELD,
                EnumSet.of(PoseInput.ODOMETRY),
                EnumSet.noneOf(PoseInput.class),
                PoseConstraints.defaults(),
                null,
                true,
                false,
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
                publishToShuffleboard,
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
                publishToShuffleboard,
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
                publishToShuffleboard,
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
                publishToShuffleboard,
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
                publishToShuffleboard,
                startPose2d,
                startPose3d);
    }

    public PoseConfig setActive(boolean active) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                publishToShuffleboard,
                startPose2d,
                startPose3d);
    }

    public PoseConfig withShuffleboardPublishing(boolean publishToShuffleboard) {
        return new PoseConfig(
                name,
                frame,
                continuousInputs,
                onDemandInputs,
                constraints,
                backendOverride,
                active,
                publishToShuffleboard,
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
                publishToShuffleboard,
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
                publishToShuffleboard,
                null,
                startPose);
    }
}
