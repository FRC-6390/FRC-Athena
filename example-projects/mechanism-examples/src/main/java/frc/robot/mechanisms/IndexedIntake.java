package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class IndexedIntake implements Mechanism {
    private final MotorDevice roller = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 12).coast();
    private final DigitalInputDevice beamBreak = Constants.RIO.dio(3).digitalInput().inverted();

    public final Action stop = roller.neutral();
    public final Action collect = Actions.sequence()
            .until(beamBreak::active, roller.percent(0.7))
            .forTime(0.15, roller.percent(0.2))
            .then(stop);
    public final Action reject = roller.percent(-0.6);
}
