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
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import frc.robot.Constants;

public final class PositionElevator implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 6).brake().currentLimit(40);
    private final EncoderDevice height = motor.encoder();
    private final Range travel = Range.of(0.0, 1.4);
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModel.elevator(motor).encoder(height).range(travel);
    private final ControlBinding lift = Controls.position(motor)
            .feedback(height)
            .pid(6.0, 0.0, 0.0)
            .constraint(Constraints.range(travel))
            .profile(MotionProfiles.trapezoid(0.8, 1.8));

    public final Action home = lift.position(0.0).untilWithin(0.03);
    public final Action low = lift.position(0.35).untilWithin(0.03);
    public final Action high = lift.position(1.25).untilWithin(0.03);
}
