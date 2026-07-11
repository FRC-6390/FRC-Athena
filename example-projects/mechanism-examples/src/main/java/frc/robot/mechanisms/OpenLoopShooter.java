package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class OpenLoopShooter implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 7).coast().currentLimit(40);

    public final Action stop = motor.neutral();
    public final Action warmup = motor.voltage(6.0);
    public final Action shoot = motor.percent(0.9);
}
