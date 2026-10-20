package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import frc.robot.Constants;

public final class FollowerElevator implements Mechanism {
    private final MotorDevice leader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 13).brake().currentLimit(edu.wpi.first.units.Units.Amps.of(40));
    private final MotorDevice follower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 14)
            .follow(leader)
            .inverted()
            .brake();
    private final Range travel = Range.of(edu.wpi.first.units.Units.Meters.of(0.0), edu.wpi.first.units.Units.Meters.of(1.7));
    private final ControlBinding lift = Controls.position(leader, edu.wpi.first.units.Units.Rotations)
            .feedback(leader.encoder())
            .pid(6.0, 0.0, 0.0)
            .constraints(Constraints.range(travel), Constraints.motion(edu.wpi.first.units.Units.RotationsPerSecond.of(1.0), edu.wpi.first.units.Units.RotationsPerSecondPerSecond.of(2.0)));

    public final Action bottom = lift.position(edu.wpi.first.units.Units.Rotations.of(0.0)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
    public final Action middle = lift.position(edu.wpi.first.units.Units.Rotations.of(0.8)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
    public final Action top = lift.position(edu.wpi.first.units.Units.Rotations.of(1.6)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
}
