package frc.robot.mechanisms;

import ca.frc6390.athena.hardware.device.EncoderDevice;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import frc.robot.Constants;

public final class ExperimentalArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 20)
            .brake()
            .currentLimit(edu.wpi.first.units.Units.Amps.of(40));
    private final EncoderDevice encoder = motor.encoder().measures(edu.wpi.first.units.Units.Degrees);
    private final SimModel simulation = SimModel.arm(motor)
            .encoder(encoder)
            .gearRatio(GearRatio.reduction(100.0, 1.0))
            .momentOfInertia(edu.wpi.first.units.Units.KilogramSquareMeters.of(0.18))
            .length(edu.wpi.first.units.Units.Meters.of(0.55));
    private final ControlBinding arm = Controls.position(motor, edu.wpi.first.units.Units.Degrees)
            .feedback(encoder)
            .pid(0.96, 0.0, 0.024)
            .constraint(Constraints.motion(edu.wpi.first.units.Units.DegreesPerSecond.of(75.0), edu.wpi.first.units.Units.DegreesPerSecondPerSecond.of(180.0)))
            .loop(ControlLoops.arbitraryFeedforward(binding -> context ->
                    ControlOutput.voltage(0.45 * Math.cos(Math.toRadians(context.position())))));

    public final Action home = arm.position(edu.wpi.first.units.Units.Degrees.of(0.0));
    public final Action score = arm.position(edu.wpi.first.units.Units.Degrees.of(75.0));
    public final Action holdOperatorSetpoint = arm.position(this::operatorSetpointDegrees, edu.wpi.first.units.Units.Degrees);

    private double operatorSetpointDegrees() {
        return 35.0;
    }
}
