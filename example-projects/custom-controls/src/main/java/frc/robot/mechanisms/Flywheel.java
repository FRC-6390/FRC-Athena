package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class Flywheel implements Mechanism {
    private static final double KP_VOLTS_PER_ROTATION_PER_SECOND = 0.08;

    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 21);
    private final SimModel simulation = SimModel.flywheel(motor)
            .encoder(motor.encoder())
            .momentOfInertia(0.006);
    private final ControlBinding devicePid = Controls.velocity(motor)
            .feedback(motor.encoder())
            .slot(1)
            .pid(KP_VOLTS_PER_ROTATION_PER_SECOND, 0.0, 0.0);
    private final ControlBinding assisted = devicePid
            .loop(ControlLoops.arbitraryFeedforward(binding -> context ->
                    ControlOutput.voltage(0.4 + 0.02 * Math.abs(context.target()))));

    public final Action off = devicePid.neutral();
    public final Action withDevicePid = devicePid.velocity(55.0);
    public final Action withArbitraryFeedforward = assisted.velocity(75.0);
}
