package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import frc.robot.Constants;

public final class PositionElevator implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 6).brake().currentLimit(edu.wpi.first.units.Units.Amps.of(40));
    private final EncoderDevice height = motor.encoder();
    private final Range travel = Range.of(edu.wpi.first.units.Units.Meters.of(0.0), edu.wpi.first.units.Units.Meters.of(1.4));
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModel.elevator(motor).encoder(height).range(travel);
    private final ControlBinding lift = Controls.position(motor, edu.wpi.first.units.Units.Rotations)
            .feedback(height)
            .pid(6.0, 0.0, 0.0)
            .constraints(Constraints.range(travel), Constraints.motion(edu.wpi.first.units.Units.RotationsPerSecond.of(0.8), edu.wpi.first.units.Units.RotationsPerSecondPerSecond.of(1.8)));

    public final Action home = lift.position(edu.wpi.first.units.Units.Rotations.of(0.0)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
    public final Action low = lift.position(edu.wpi.first.units.Units.Rotations.of(0.35)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
    public final Action high = lift.position(edu.wpi.first.units.Units.Rotations.of(1.25)).untilWithin(edu.wpi.first.units.Units.Rotations.of(0.03));
}
