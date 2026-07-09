package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;

public final class DriveTrain implements Mechanism {
    private double forward;
    private double turn;

    public final MotorDevice leftLeader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1);
    public final MotorDevice leftFollower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2);
    public final MotorDevice rightLeader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 3).inverted();
    public final MotorDevice rightFollower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 4).inverted();

    public final Action drive = Actions.parallel(
            leftLeader.percent(this::leftPercent),
            leftFollower.percent(this::leftPercent),
            rightLeader.percent(this::rightPercent),
            rightFollower.percent(this::rightPercent));

    public void arcade(double forward, double turn) {
        this.forward = clamp(forward);
        this.turn = clamp(turn);
    }

    private double leftPercent() {
        return clamp(forward + turn);
    }

    private double rightPercent() {
        return clamp(forward - turn);
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
