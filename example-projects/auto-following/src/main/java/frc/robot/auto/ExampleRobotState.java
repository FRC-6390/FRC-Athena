package frc.robot.auto;

import java.util.concurrent.atomic.AtomicBoolean;

/** Mutable sensor/operator inputs used to make branching examples concrete. */
public final class ExampleRobotState {
    private final AtomicBoolean hasGamePiece = new AtomicBoolean(true);
    private final AtomicBoolean centerLaneClear = new AtomicBoolean(true);
    private final AtomicBoolean visionTargetVisible = new AtomicBoolean(false);

    public boolean hasGamePiece() {
        return hasGamePiece.get();
    }

    public void setHasGamePiece(boolean value) {
        hasGamePiece.set(value);
    }

    public boolean centerLaneClear() {
        return centerLaneClear.get();
    }

    public void setCenterLaneClear(boolean value) {
        centerLaneClear.set(value);
    }

    public boolean visionTargetVisible() {
        return visionTargetVisible.get();
    }

    public void setVisionTargetVisible(boolean value) {
        visionTargetVisible.set(value);
    }
}
