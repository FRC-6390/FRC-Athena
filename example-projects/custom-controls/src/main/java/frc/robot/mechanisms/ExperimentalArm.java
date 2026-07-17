package frc.robot.mechanisms;

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
            .currentLimit(40);
    private final SimModel simulation = SimModel.arm(motor)
            .encoder(motor.encoder())
            .gearRatio(GearRatio.reduction(100.0, 1.0))
            .momentOfInertia(0.18)
            .lengthMeters(0.55);
    private final ControlBinding arm = Controls.position(motor)
            .feedback(motor.encoder())
            .pid(0.96, 0.0, 0.024)
            .constraint(Constraints.motion(75.0, 180.0))
            .loop(ControlLoops.arbitraryFeedforward(binding -> context ->
                    ControlOutput.voltage(0.45 * Math.cos(Math.toRadians(context.position())))));

    public final Action home = arm.position(0.0);
    public final Action score = arm.position(75.0);
    public final Action holdOperatorSetpoint = arm.position(this::operatorSetpointDegrees);

    private double operatorSetpointDegrees() {
        return 35.0;
    }
}
