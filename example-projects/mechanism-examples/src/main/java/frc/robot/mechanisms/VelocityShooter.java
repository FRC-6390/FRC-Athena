package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Sag;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.InterpolatedControlAction;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.persist.Locks;
import frc.robot.Constants;

public final class VelocityShooter implements Mechanism {
    /**
     * Shared power profile. Both motors draw from it; the endgame Action below reduces the share without
     * redeclaring the motors.
     */
    public static final PowerProfile SHOOT = PowerProfile.of()
            .sustained(Amps.of(60))
            .burst(Amps.of(90), Seconds.of(0.5))
            .rampLimit(Amps.of(10), Seconds.of(0.1))
            .share(3.0)
            .onSag(Sag.shed(0.5).below(Volts.of(9)));

    /** Persisted across reboots and deploys. Writes are accepted unless the Locks.InMatch condition is active. */
    @Telemetry(value = "shotDistanceMeters", writable = true, min = 1.0, max = 8.0, persist = true)
    public double shotDistanceMeters = 3.0;

    private final MotorDevice leader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1).coast().power(SHOOT);
    private final MotorDevice follower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2)
            .follow(leader)
            .inverted()
            .coast()
            .power(SHOOT);
    private final EncoderDevice velocity = leader.encoder();
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModel.flywheel(leader, follower)
            .encoder(velocity)
            .momentOfInertia(edu.wpi.first.units.Units.KilogramSquareMeters.of(0.006));
    private final ControlBinding wheel = Controls.velocity(leader)
            .feedback(velocity)
            .slot(0)
            .pid(PidGains.of(0.08, 0.0, 0.0))
            .feedforward(FeedforwardGains.simple(0.2, 0.12, 0.0))
            .persistTuning(Locks.InMatch.class);

    public final Action stop = wheel.neutral();
    public final Action idle = wheel.velocity(35.0);
    public final Action podium = wheel.velocity(78.0).untilWithin(2.0);

    /** Interpolated shot table. `persisted()` keeps the points across reboots. */
    public final InterpolatedControlAction distanceShot = wheel
            .interpolate(InterpolationKinds.LINEAR, () -> shotDistanceMeters)
            .at(1.0, 45.0)
            .at(3.0, 68.0)
            .at(5.0, 82.0)
            .at(8.0, 96.0)
            .persisted();

    /** Records the current distance and wheel speed as a new table point (the field-tuning workflow). */
    public final Action recordShot = distanceShot.record();

    public final Action reverse = wheel.velocity(-25.0);

    /** Ends the match with a smaller share of the battery budget so the drivetrain keeps power. */
    public final Action endgame = SHOOT.update(profile -> profile.share(0.5));
    public final Action normalPower = SHOOT.restore();

    @Telemetry("measuredVelocityRps")
    public double measuredVelocity() {
        return velocity.velocity();
    }
}
