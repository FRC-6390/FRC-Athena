package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class Flywheel implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 21);
    private final ControlBinding wheel = Controls.velocity(motor)
            .feedback(motor.encoder())
            .slot(1)
            .pid(0.08, 0.0, 0.0)
            .loop(ControlLoops.arbitraryFeedforward(binding -> context ->
                    ControlOutput.voltage(0.4 + 0.02 * Math.abs(context.target()))));

    public final Action off = motor.percent(0.0);
    public final Action withDevicePid = wheel.velocity(55.0);
    public final Action withArbitraryFeedforward = wheel.velocity(75.0);
}
