package frc.robot.mechanisms;

import ca.frc6390.athena.hardware.device.EncoderDevice;

import static edu.wpi.first.units.Units.Seconds;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public final class GuardedArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 10).brake().currentLimit(edu.wpi.first.units.Units.Amps.of(30));
    private final DigitalInputDevice home = Constants.RIO.dio(1).digitalInput().inverted();
    private final DigitalInputDevice hardStop = Constants.RIO.dio(2).digitalInput().inverted();
    private final Range travel = Range.of(edu.wpi.first.units.Units.Degrees.of(0.0), edu.wpi.first.units.Units.Degrees.of(110.0));
    private final EncoderDevice encoder = motor.encoder().measures(edu.wpi.first.units.Units.Degrees);
    private final SimModel simulation = SimModel.arm(motor)
            .encoder(encoder)
            .range(travel)
            .limit(home, travel.minimum().magnitude())
            .limit(hardStop, travel.maximum().magnitude());
    private final ControlBinding position = Controls.position(motor, edu.wpi.first.units.Units.Degrees)
            .feedback(encoder)
            .pid(0.72, 0.0, 0.0)
            .constraints(
                    Constraints.range(travel),
                    Constraints.lower(home),
                    Constraints.upper(hardStop),
                    Constraints.motion(edu.wpi.first.units.Units.DegreesPerSecond.of(70.0), edu.wpi.first.units.Units.DegreesPerSecondPerSecond.of(180.0)));

    @SuppressWarnings("unused")
    public final HookBinding zeroWhenHomed = Events.when(home).rising().onStart(encoder.setPosition(0.0));

    public final Action stop = position.neutral();
    public final Action homeSlowly = Actions.sequence()
            .until(home::active, position.percent(-0.12))
            .then(position.position(edu.wpi.first.units.Units.Degrees.of(0.0)));
    public final BooleanSupplier atScore = position.at(edu.wpi.first.units.Units.Degrees.of(90.0), edu.wpi.first.units.Units.Degrees.of(2.0));
    public final Action score = position.position(edu.wpi.first.units.Units.Degrees.of(90.0)).untilWithin(edu.wpi.first.units.Units.Degrees.of(2.0));
    public final Action nudgeUp = position.percent(0.2).until(hardStop::active);
    public final Action timedManualEscape = Actions.timeout(position.percent(-0.15), Seconds.of(0.4)).then(stop);
}
