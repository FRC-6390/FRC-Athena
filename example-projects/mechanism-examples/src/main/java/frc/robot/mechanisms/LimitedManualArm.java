package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class LimitedManualArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 11).brake();
    private final DigitalInputDevice lowerLimit = Constants.RIO.dio(1).digitalInput().inverted();
    private final DigitalInputDevice upperLimit = Constants.RIO.dio(2).digitalInput().inverted();
    private final SimModel simulation = SimModel.arm(motor)
            .encoder(motor.encoder())
            .limit(lowerLimit, -45.0)
            .limit(upperLimit, 90.0);

    public final Action stop = motor.neutral();
    public final Action lower = motor.percent(() -> lowerLimit.active() ? 0.0 : -0.25);
    public final Action raise = motor.percent(() -> upperLimit.active() ? 0.0 : 0.25);
}
