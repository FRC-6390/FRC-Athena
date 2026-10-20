package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Sag;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.control.TorqueVelocityGains;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.InterpolatedControlAction;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.model.Characterization;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;


public final class Shooter implements Mechanism {
    /** Four Falcons spinning up together is the robot's biggest shock load: ramp it in and shed on sag. */
    public static final PowerProfile SHOOTER = PowerProfile.of()
            .sustained(Amps.of(40))
            .burst(Amps.of(60), Seconds.of(0.5))
            .rampLimit(Amps.of(15), Seconds.of(0.1))
            .share(2.0)
            .onSag(Sag.shed(0.5).below(Volts.of(8.0)));

    private static final double MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND = 200.0; // telemetry bounds
    private static final AngularVelocity MAXIMUM_VELOCITY = RotationsPerSecond.of(MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND);
    private static final AngularAcceleration MAXIMUM_ACCELERATION = RotationsPerSecondPerSecond.of(200.0);

    // Devices
    private final MotorDevice leader = Constants.CAN.motor(MotorKinds.FALCON_500, 31).power(SHOOTER);

    private final MotorDevice follower = Constants.CAN.motor(MotorKinds.FALCON_500, 32)
            .follow(leader).power(SHOOTER);

    private final MotorDevice follower1 = Constants.RIO.motor(MotorKinds.FALCON_500, 33)
            .follow(leader).inverted().power(SHOOTER);

    private final MotorDevice follower2 = Constants.RIO.motor(MotorKinds.FALCON_500, 34)
            .follow(leader).inverted().power(SHOOTER);

    private final EncoderDevice encoder = leader.encoder()
            .reduction(0.6)
            .measures(Rotations);

    // Bindings and controls
    private final ControlBinding<?> velocity = Controls.velocity(leader, Rotations)
            .feedback(encoder)
            // Holds speed under load with motor current, not voltage: back-EMF is cancelled from the
            // measured motor speed and the load estimate adds the current a ball costs right away.
            // Followers mirror the leader in hardware, so these amps are per motor.
            // kP 2.1 A/rps matches the old 0.1 V/rps PID (Falcon R ~0.047 ohm); kS is the Falcon free current.
            .torqueVelocity(TorqueVelocityGains.of(2.1, 0.0, 0.0)
                    .staticCurrent(Amps.of(1.5))
                    .loadEstimate(Seconds.of(0.02)))
            .constraints(
                    Constraints.range(MAXIMUM_VELOCITY.unaryMinus(), MAXIMUM_VELOCITY),
                    Constraints.motion(MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION),
                    Constraints.maxVoltage(Volts.of(12.0)));
    // Flywheel: no gravity term. Run the routine with the shooter clear; it spins up to 6 V.
    public final Characterization characterization = velocity.characterization()
            .logName("shooter")
            .maxVoltage(Volts.of(6.0))
            .timeout(Seconds.of(8.0));

    // Shot table values are saved on the robot and survive reboots and deploys; dashboard writes
    // are refused while the FMS is attached.
    @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND, persist = true)
    private double nearShotVelocityRps = Constants.ShotCalibration.NEAR_SHOOTER_RPS;
//     @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND)
//     private double nearMiddleShotVelocityRps = Constants.ShotCalibration.NEAR_MIDDLE_SHOOTER_RPS;
    @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND, persist = true)
    private double middleShotVelocityRps = Constants.ShotCalibration.MIDDLE_SHOOTER_RPS;
//     @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND)
//     private double middleFarShotVelocityRps = Constants.ShotCalibration.MIDDLE_FAR_DISTANCE_METERS;
    @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND, persist = true)
    private double farShotVelocityRps = Constants.ShotCalibration.FAR_SHOOTER_RPS;
    
    @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND, persist = true)
    private double idleVelocityRotationsPerSecond = 20;
    @Telemetry(writable = true, min = 0.0, max = MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND, persist = true)
    private double reverseVelocityRotationsPerSecond = 20;
     @Telemetry(writable = true, min = -1.0, max = 1.0)
    private double shooterPercentOut = 0.5;
     @Telemetry(writable = true, min = -15.0, max = 15.0)
    private double shooterRps = 0.0;

    // Actions
    /**
     * Distance -> wheel speed. The three tuned points are the base table; {@link #recordShot} adds a point
     * at the current distance and speed (trim included). Recorded points survive reboots and deploys.
     */
    public final InterpolatedControlAction fire;
    public final Action recordShot;
    public final Action idle = velocity.velocity(() -> idleVelocityRotationsPerSecond, Rotations);
    public final Action reverse = velocity.velocity(() -> -reverseVelocityRotationsPerSecond, Rotations);
    public final Action neutral = velocity.neutral();
    public final Action percentFire = leader.percent(() -> shooterPercentOut);
    public final Action speedUp = Actions.doOnce(() -> shooterRps += 2.0);
    public final Action speedDown = Actions.doOnce(() -> shooterRps -= 2.0);
    public final Action speedReset = Actions.doOnce(() -> shooterRps = 0.0);

    public Shooter(Localization localization) {
        fire = velocity.interpolate(InterpolationKinds.LINEAR, localization::distanceToHubMeters, Meters)
                .at(Constants.ShotCalibration.NEAR_DISTANCE,
                        () -> Math.max(0, nearShotVelocityRps + shooterRps), Rotations)
                .at(Constants.ShotCalibration.MIDDLE_DISTANCE,
                        () -> Math.max( 0, middleShotVelocityRps + shooterRps), Rotations)
                .at(Constants.ShotCalibration.FAR_DISTANCE,
                        () -> Math.max( 0, farShotVelocityRps + shooterRps), Rotations)
                .persisted();
        recordShot = fire.record();
    }
    public final Action shooterPercent = leader.percent(
            () -> shooterPercentOut + shooterRps
        );
}
