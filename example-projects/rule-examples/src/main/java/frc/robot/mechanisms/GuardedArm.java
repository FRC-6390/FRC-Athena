package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.DigitalInputs;
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

public final class GuardedArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 10);
    private final DigitalInputDevice home = DigitalInputs.rio(1).inverted();
    private final DigitalInputDevice hardStop = DigitalInputs.rio(2).inverted();
    private final Range travel = Range.degrees(0.0, 110.0);
    private final ControlBinding position = Controls.position(motor)
            .feedback(motor.encoder())
            .pid(0.06, 0.0, 0.0)
            .constraints(Constraints.range(travel), Constraints.lower(home), Constraints.upper(hardStop))
            .profile(MotionProfiles.trapezoid(70.0, 180.0));

    @SuppressWarnings("unused")
    public final HookBinding zeroWhenHomed = Events.when(home).rising().onStart(motor.encoder().setPosition(0.0));

    public final Action hold = position.percent(0.0);
    public final Action homeSlowly = Actions.sequence()
            .until(home::active, position.percent(-0.12))
            .then(position.position(0.0));
    public final Action score = position.position(90.0)
            .until(hardStop::active)
            .then(hold);
    public final Action nudgeUp = position.percent(0.2).until(hardStop::active);
    public final Action timedManualEscape = Actions.timeout(position.percent(-0.15), 0.4).then(hold);
}
