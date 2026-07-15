package ca.frc6390.athena.drivetrain.swerve;

/** Trajectory source used by a kinematics-owned path follower. */
public enum FollowerBackend {
    CHOREO("choreo"),
    PATHPLANNER("pathplanner");

    private final String source;

    FollowerBackend(String source) {
        this.source = source;
    }

    public String source() {
        return source;
    }
}
