package ca.frc6390.athena.core.auto;

import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Inputs required to configure a vendor-specific holonomic auto builder.
 */
public record HolonomicDriveBinding(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> poseResetter,
        Supplier<ChassisSpeeds> robotRelativeSpeeds,
        BiConsumer<ChassisSpeeds, HolonomicFeedforward> output,
        HolonomicPidConstants translationPid,
        HolonomicPidConstants rotationPid,
        Supplier<Boolean> isRedAllianceSupplier) {

    public HolonomicDriveBinding {
        Objects.requireNonNull(poseSupplier, "poseSupplier");
        Objects.requireNonNull(poseResetter, "poseResetter");
        Objects.requireNonNull(robotRelativeSpeeds, "robotRelativeSpeeds");
        Objects.requireNonNull(output, "output");
        Objects.requireNonNull(translationPid, "translationPid");
        Objects.requireNonNull(rotationPid, "rotationPid");
        isRedAllianceSupplier = isRedAllianceSupplier != null ? isRedAllianceSupplier : () -> false;
    }
}
