package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;

public final class DriveTrain implements Mechanism {
    private double forward;
    private double turn;

    private final MotorDevice left = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1);
    private final MotorDevice right = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2).inverted();

    public final Action drive = Actions.parallel(
            left.percent(this::leftOutput),
            right.percent(this::rightOutput));
    public final Action stop = Actions.parallel(left.percent(0.0), right.percent(0.0));

    public void arcade(double forward, double turn) {
        this.forward = clamp(forward);
        this.turn = clamp(turn);
    }

    private double leftOutput() {
        return clamp(forward + turn);
    }

    private double rightOutput() {
        return clamp(forward - turn);
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
