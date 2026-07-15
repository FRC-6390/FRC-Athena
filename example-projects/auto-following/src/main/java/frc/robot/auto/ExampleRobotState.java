package frc.robot.auto;

/** Mutable sensor/operator inputs used to make branching examples concrete. */
public final class ExampleRobotState {
    private boolean hasGamePiece = true;

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public void setHasGamePiece(boolean value) {
        hasGamePiece = value;
    }
}
