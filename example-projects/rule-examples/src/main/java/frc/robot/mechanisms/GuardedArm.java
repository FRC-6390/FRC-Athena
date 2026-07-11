package frc.robot.mechanisms;

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
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public final class GuardedArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 10).brake().currentLimit(30);
    private final DigitalInputDevice home = Constants.RIO.dio(1).digitalInput().inverted();
    private final DigitalInputDevice hardStop = Constants.RIO.dio(2).digitalInput().inverted();
    private final Range travel = Range.degrees(0.0, 110.0);
    private final SimModel simulation = SimModel.arm(motor)
            .encoder(motor.encoder())
            .range(travel)
            .limit(home, travel.minimum())
            .limit(hardStop, travel.maximum());
    private final ControlBinding position = Controls.position(motor)
            .feedback(motor.encoder())
            .pid(0.72, 0.0, 0.0)
            .constraints(Constraints.range(travel), Constraints.lower(home), Constraints.upper(hardStop))
            .profile(MotionProfiles.trapezoid(70.0, 180.0));

    @SuppressWarnings("unused")
    public final HookBinding zeroWhenHomed = Events.when(home).rising().onStart(motor.encoder().setPosition(0.0));

    public final Action stop = position.neutral();
    public final Action homeSlowly = Actions.sequence()
            .until(home::active, position.percent(-0.12))
            .then(position.position(0.0));
    public final BooleanSupplier atScore = position.at(90.0, 2.0);
    public final Action score = position.position(90.0).untilWithin(2.0);
    public final Action nudgeUp = position.percent(0.2).until(hardStop::active);
    public final Action timedManualEscape = Actions.timeout(position.percent(-0.15), 0.4).then(stop);
}
