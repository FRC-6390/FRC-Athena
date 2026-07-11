package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class SplitWheelShooter implements Mechanism {
    private final MotorDevice topMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 8).coast();
    private final MotorDevice bottomMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 9).inverted().coast();
    private final ControlBinding top = Controls.velocity(topMotor).feedback(topMotor.encoder()).pid(0.08, 0.0, 0.0);
    private final ControlBinding bottom = Controls.velocity(bottomMotor).feedback(bottomMotor.encoder()).pid(0.08, 0.0, 0.0);

    public final Action stop = Actions.parallel(topMotor.neutral(), bottomMotor.neutral());
    public final Action straightShot = velocity(70.0, 70.0);
    public final Action spinShot = velocity(78.0, 58.0);

    private Action velocity(double topVelocity, double bottomVelocity) {
        return Actions.parallel(
                top.velocity(topVelocity).untilWithin(2.0),
                bottom.velocity(bottomVelocity).untilWithin(2.0));
    }
}
