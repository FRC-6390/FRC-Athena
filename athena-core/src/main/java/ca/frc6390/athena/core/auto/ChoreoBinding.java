package ca.frc6390.athena.core.auto;

import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Inputs needed to build Choreo-style trajectory factories without pulling vendor dependencies
 * into the core.
 */
public record ChoreoBinding(
        HolonomicPidConstants translationPid,
        HolonomicPidConstants rotationPid,
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> poseResetter,
        BiConsumer<Pose2d, ChassisSpeeds> follower,
        boolean mirrorForAlliance,
        Subsystem drivetrain) {

    public ChoreoBinding {
        Objects.requireNonNull(translationPid, "translationPid");
        Objects.requireNonNull(rotationPid, "rotationPid");
        Objects.requireNonNull(poseSupplier, "poseSupplier");
        Objects.requireNonNull(poseResetter, "poseResetter");
        Objects.requireNonNull(follower, "follower");
        Objects.requireNonNull(drivetrain, "drivetrain");
    }
}
