package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class SingleJointArm implements Mechanism {
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 10).brake().currentLimit(30);
    private final EncoderDevice encoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 10);
    private final Range travel = Range.degrees(-10.0, 105.0);
    private final ControlBinding position = Controls.position(motor).feedback(encoder).pid(0.07, 0.0, 0.002);

    public final Action stow = position.position(0.0).clamp(travel);
    public final Action pickup = position.position(35.0).clamp(travel);
    public final Action score = position.position(92.0).clamp(travel);
}
