package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;

public final class Intake implements Mechanism {
    private final MotorDevice roller = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 3);

    public final Action stop = roller.percent(0.0);
    public final Action collect = roller.percent(0.8);
    public final Action eject = roller.percent(-0.6);
}
