package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;
import frc.robot.controls.SlewLimitedPositionLoop;

public final class ExperimentalArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 20);
    private final ControlBinding arm = Controls.position(motor)
            .feedback(motor.encoder())
            .loop(ControlLoops.software(binding -> new SlewLimitedPositionLoop(0.45, 0.08)));

    public final Action home = arm.position(0.0);
    public final Action motionProfiled = arm.position(75.0);
    public final Action holdOperatorSetpoint = arm.position(this::operatorSetpointDegrees);

    private double operatorSetpointDegrees() {
        return 35.0;
    }
}
