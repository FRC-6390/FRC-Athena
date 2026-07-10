package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.DigitalInputs;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class Conveyor implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 11);
    private final DigitalInputDevice beamBreak = DigitalInputs.rio(3).inverted();
    private boolean operatorOverride;

    @SuppressWarnings("unused")
    public final HookBinding latchOperatorOverride = Events.testInit().onStart(() -> operatorOverride = true);

    public final Action stop = motor.percent(0.0);
    public final Action feedWhenLoaded = Actions.when(() -> beamBreak.active() || operatorOverride)
            .then(motor.percent(0.7))
            .otherwise(stop);
    public final Action indexOneBall = Actions.sequence()
            .until(beamBreak::active, motor.percent(0.35))
            .forTime(0.18, motor.percent(0.2))
            .then(stop);
}
