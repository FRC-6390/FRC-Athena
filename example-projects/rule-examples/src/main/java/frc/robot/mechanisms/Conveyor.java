package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;
import java.util.Objects;
import java.util.function.BooleanSupplier;

public final class Conveyor implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 11).coast();
    private final DigitalInputDevice beamBreak = Constants.RIO.dio(3).digitalInput().inverted();
    private final SimModel simulation = SimModel.motor(motor).encoder(motor.encoder());
    private final BooleanSupplier armAtScore;
    private boolean operatorOverride;

    public Conveyor(BooleanSupplier armAtScore) {
        this.armAtScore = Objects.requireNonNull(armAtScore, "armAtScore");
    }

    @SuppressWarnings("unused")
    public final HookBinding enableTestOverride = Events.testInit().onStart(() -> operatorOverride = true);
    public final HookBinding disableTestOverride = Events.testExit().onStart(() -> operatorOverride = false);

    public final Action stop = motor.neutral();
    public final Action feedWhenLoaded = Actions.when(
                    () -> armAtScore() && (beamBreak.active() || operatorOverride))
            .then(motor.percent(0.7))
            .otherwise(stop);
    public final Action indexOneBall = Actions.sequence()
            .until(beamBreak::active, motor.percent(0.35))
            .forTime(0.18, motor.percent(0.2))
            .then(stop);

    private boolean armAtScore() {
        return armAtScore.getAsBoolean();
    }
}
