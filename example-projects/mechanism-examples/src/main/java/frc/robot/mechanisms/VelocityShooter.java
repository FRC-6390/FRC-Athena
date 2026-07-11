package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;

public final class VelocityShooter implements Mechanism {
    private final MotorDevice leader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1).coast();
    private final MotorDevice follower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2)
            .follow(leader)
            .inverted()
            .coast();
    private final EncoderDevice velocity = leader.encoder();
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModel.flywheel(leader, follower)
            .encoder(velocity)
            .momentOfInertia(0.006);
    private final ControlBinding wheel = Controls.velocity(leader)
            .feedback(velocity)
            .slot(0)
            .pid(PidGains.of(0.08, 0.0, 0.0))
            .feedforward(FeedforwardGains.simple(0.2, 0.12, 0.0));

    public final Action stop = wheel.neutral();
    public final Action idle = wheel.velocity(35.0);
    public final Action podium = wheel.velocity(78.0).untilWithin(2.0);
    public final Action reverse = wheel.velocity(-25.0);
}
