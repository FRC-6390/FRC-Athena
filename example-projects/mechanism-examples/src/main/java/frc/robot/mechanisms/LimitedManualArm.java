package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.DigitalInputs;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class LimitedManualArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 11).brake();
    private final DigitalInputDevice lowerLimit = DigitalInputs.rio(1).inverted();
    private final DigitalInputDevice upperLimit = DigitalInputs.rio(2).inverted();

    public final Action stop = motor.percent(0.0);
    public final Action lower = motor.percent(() -> lowerLimit.active() ? 0.0 : -0.25);
    public final Action raise = motor.percent(() -> upperLimit.active() ? 0.0 : 0.25);
}
