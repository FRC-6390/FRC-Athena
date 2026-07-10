package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class PositionElevator implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 6).brake().currentLimit(40);
    private final EncoderDevice height = motor.encoder();
    private final Range travel = Range.of(0.0, 1.4);
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModels.elevator(motor).encoder(height).range(travel);
    private final ControlBinding lift = Controls.position(motor).feedback(height).pid(0.5, 0.0, 0.0);

    public final Action home = lift.position(0.0).clamp(travel);
    public final Action low = lift.position(0.35).clamp(travel);
    public final Action high = lift.position(1.25).clamp(travel);
}
