package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.InterpolatedControlAction;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.model.Characterization;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.concurrent.atomic.AtomicBoolean;

public final class Hood implements Mechanism {
    private static final PowerProfile ACTUATOR = PowerProfile.of()
            .sustained(Amps.of(25))
            .burst(Amps.of(40), Seconds.of(0.25))
            .share(1.0);
    /** Old NEOs: the current limit slides from full at 60 C to 30 percent at 90 C (it never brakes or locks out). */
    private static final Thermal NEO_DERATE = Thermal.derate(Celsius.of(60.0), Celsius.of(90.0), 0.3);

    private static final double MINIMUM_ANGLE_DEGREES = 13.0;
    private static final double MAXIMUM_ANGLE_DEGREES = 49.0;
    private static final Angle MINIMUM_ANGLE = Degrees.of(MINIMUM_ANGLE_DEGREES);
    private static final Angle MAXIMUM_ANGLE = Degrees.of(MAXIMUM_ANGLE_DEGREES);
    private static final double MAXIMUM_VELOCITY_DEGREES_PER_SECOND = 60.0;
    private static final double MAXIMUM_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 120.0;

    // Devices
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.NEO, 30).brake().inverted().currentLimit(Amps.of(30))
            .power(ACTUATOR)
            .thermal(NEO_DERATE);
    private final EncoderDevice encoder = motor.encoder()
            .reduction(97.5)
            .measures(Degrees);

    // Bindings and controls
    private final Range travel = Range.of(MINIMUM_ANGLE, MAXIMUM_ANGLE);
    private final ControlBinding<?> angle;
    public final Characterization characterization;

    @Telemetry(writable = true, min = 0.0, max = 0.25)
    private double homingOutputPercent = 0.08;
    // Hooks and miscellaneous state
    private final AtomicBoolean homed = new AtomicBoolean(false);
    // Zero once on the first enable in any mode (assumes the mechanism starts at its stop);
    // later enables keep the existing zero. homeAll still re-homes.
    public final HookBinding initializeEncoderOnFirstEnable = Events.when(DriverStation::isEnabled)
            .rising()
            .onStart(context -> {
                if (homed.compareAndSet(false, true)) {
                    encoder.setPosition(MINIMUM_ANGLE_DEGREES).apply(context);
                }
            });

    @Telemetry
    private boolean homed() {
        return homed.get();
    }
    // Saved on the robot (survive reboots and deploys); dashboard writes are refused while the FMS is attached.
    @Telemetry(writable = true, min = MINIMUM_ANGLE_DEGREES, max = MAXIMUM_ANGLE_DEGREES, persist = true)
    private double nearShotAngleDegrees = Constants.ShotCalibration.NEAR_HOOD_DEGREES;
//      @Telemetry(writable = true, min = MINIMUM_ANGLE_DEGREES, max = MAXIMUM_ANGLE_DEGREES)
//     private double nearMiddleShotAngleDegrees = Constants.ShotCalibration.NEAR_MIDDLE_HOOD_DEGREES;
    @Telemetry(writable = true, min = MINIMUM_ANGLE_DEGREES, max = MAXIMUM_ANGLE_DEGREES, persist = true)
    private double middleShotAngleDegrees = Constants.ShotCalibration.MIDDLE_HOOD_DEGREES;
//      @Telemetry(writable = true, min = MINIMUM_ANGLE_DEGREES, max = MAXIMUM_ANGLE_DEGREES)
//     private double middleFarShotAngleDegrees = Constants.ShotCalibration.MIDDLE_FAR_DISTANCE_METERS;
    @Telemetry(writable = true, min = MINIMUM_ANGLE_DEGREES, max = MAXIMUM_ANGLE_DEGREES, persist = true)
    private double farShotAngleDegrees = Constants.ShotCalibration.FAR_HOOD_DEGREES;
    private double manualHood = 0.0;

    // Actions
    // Drive into the hard stop until the motor stalls (current up, no movement), then zero. The
    // 1.5 s timeout keeps the old timed homing as a fallback if the stall is never seen.
    private final MotorStallSignal homingStall = motor.stall()
            .atCurrentLimit(0.2) // 6 A of the current limit
            .velocityBelow(RotationsPerSecond.of(1.0)) // motor speed
            .forTime(Seconds.of(0.1));
    public final Action home = Actions.sequence()
            .run(Actions.untilStalled(homingStall, motor.percent(() -> -homingOutputPercent)).timeout(Seconds.of(1.5)))
            .doOnce(() -> homed.set(true))
            .doOnce(encoder.setPosition(MINIMUM_ANGLE_DEGREES));
    public final Action stow;
    /** Distance -> hood angle; {@link #recordShot} adds a point like the shooter's table. */
    public final InterpolatedControlAction aim;
    public final Action recordShot;
    public final Action neutral;
    public final Action hoodUp = motor.percent(0.20);
    public final Action hoodDown = motor.percent(-0.20);
    public final Action mHoodUp = Actions.doOnce(() -> manualHood += 2.5);
    public final Action mHoodDown = Actions.doOnce(() -> manualHood -= 2.5);
    public final Action mHoodReset = Actions.doOnce(() -> manualHood = 0.0);

    public Hood(Localization localization) {
        angle = Controls.position(motor, Degrees)
                .feedback(encoder)
                .pid(0.2, 0.0, 0.0)
                .constraints(
                        Constraints.override(localization::inTrench, MINIMUM_ANGLE_DEGREES),
                        Constraints.range(travel),
                        Constraints.maxVoltage(Volts.of(12.0)));
        // Hood gravity is small; Athena's default for a rotary position binding fits a cosine term anyway.
        characterization = angle.characterization()
                .logName("hood")
                .maxVoltage(Volts.of(3.0))
                .range(MINIMUM_ANGLE, MAXIMUM_ANGLE);
        stow = angle.position(MINIMUM_ANGLE);
        aim = angle.interpolate(InterpolationKinds.LINEAR, localization::distanceToHubMeters, Meters)
                .at(Constants.ShotCalibration.NEAR_DISTANCE,
                        () -> nearShotAngleDegrees + manualHood, Degrees)
                .at(Constants.ShotCalibration.MIDDLE_DISTANCE,
                        () -> middleShotAngleDegrees + manualHood, Degrees)
                .at(Constants.ShotCalibration.FAR_DISTANCE,
                        () -> farShotAngleDegrees + manualHood, Degrees)
                .persisted();
        recordShot = aim.record();
        neutral = angle.neutral();
    }
}
