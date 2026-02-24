package ca.frc6390.athena.core.localization;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class RobotLocalizationSpeedsView {
    private final RobotLocalization<?> owner;

    RobotLocalizationSpeedsView(RobotLocalization<?> owner) {
        this.owner = owner;
    }

    public ChassisSpeeds robotRelative() {
        return owner.getRobotRelativeSpeeds();
    }

    public ChassisSpeeds fieldRelative() {
        return owner.getFieldRelativeSpeeds();
    }

    public double xMetersPerSecond() {
        return owner.getXSpeedMetersPerSecond();
    }

    public double yMetersPerSecond() {
        return owner.getYSpeedMetersPerSecond();
    }

    public double thetaRadiansPerSecond() {
        return owner.getThetaSpeedRadiansPerSecond();
    }

    public double movementMetersPerSecond() {
        return owner.getMovementSpeedMetersPerSecond();
    }

    public double normalizedMovement() {
        return owner.getNormalizedMovementSpeed();
    }
}
