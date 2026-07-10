package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import frc.robot.Constants;

public final class FollowerElevator implements Mechanism {
    private final MotorDevice leader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 13).brake().currentLimit(40);
    private final MotorDevice follower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 14)
            .follow(leader)
            .inverted()
            .brake();
    private final Range travel = Range.of(0.0, 1.7);
    private final ControlBinding lift = Controls.position(leader)
            .feedback(leader.encoder())
            .pid(0.5, 0.0, 0.0)
            .constraint(Constraints.range(travel))
            .profile(MotionProfiles.trapezoid(1.0, 2.0));

    public final Action bottom = lift.position(0.0).untilWithin(0.03);
    public final Action middle = lift.position(0.8).untilWithin(0.03);
    public final Action top = lift.position(1.6).untilWithin(0.03);
}
