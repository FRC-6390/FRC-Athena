package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

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
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.model.BindingCheck;
import ca.frc6390.athena.mechanism.staging.Staging;
import ca.frc6390.athena.mechanism.model.Characterization;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.concurrent.atomic.AtomicBoolean;

public final class IntakeArm implements Mechanism {
    private static final PowerProfile ACTUATOR = PowerProfile.of()
            .sustained(Amps.of(35))
            .burst(Amps.of(40), Seconds.of(0.5))
            .share(1.0);
    /** Old NEOs: the current limit slides from full at 60 C to 30 percent at 90 C (it never brakes or locks out). */
    private static final Thermal NEO_DERATE = Thermal.derate(Celsius.of(60.0), Celsius.of(90.0), 0.3);

    private static final Angle STOWED_ANGLE = Degrees.of(0.0);
    private static final Angle DEPLOYED_ANGLE = Degrees.of(123.6);

    // Devices
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.NEO, 40).brake().inverted().currentLimit(Amps.of(40))
            .power(ACTUATOR)
            .thermal(NEO_DERATE);
    private final EncoderDevice encoder = motor.encoder()
            .reduction(29.63)
            .measures(Degrees);

    // Bindings and controls
    private final Range travel = Range.of(STOWED_ANGLE, DEPLOYED_ANGLE);
    private final ControlBinding<?> angle = Controls.position(motor, Degrees)
            .feedback(encoder)
            .pid(0.1, 0.0, 0.005)
            .ff(0.0, 0.0, 1.0)
            .constraints(
                    Constraints.range(travel),
                    Constraints.motion(DegreesPerSecond.of(60.0), DegreesPerSecondPerSecond.of(40.0)),
                    Constraints.maxVoltage(Volts.of(12.0)));
    // Gravity defaults to a cosine term for a rotary arm; its angle reference is the encoder zero (stowed),
    // so treat the fitted kG as approximate until the arm's level angle is measured.
    public final Characterization characterization = angle.characterization()
            .logName("intake-arm")
            .maxVoltage(Volts.of(4.0))
            .range(STOWED_ANGLE, DEPLOYED_ANGLE);

    @Telemetry(writable = true, min = 0.0, max = 0.25)
    private double homingOutputPercent = 0.16;
    // Hooks and miscellaneous state
    private final AtomicBoolean homed = new AtomicBoolean(false);
    // Zero once on the first enable in any mode (assumes the mechanism starts at its stop);
    // later enables keep the existing zero. homeAll still re-homes.
    public final HookBinding initializeEncoderOnFirstEnable = Events.when(DriverStation::isEnabled)
            .rising()
            .onStart(context -> {
                if (homed.compareAndSet(false, true)) {
                    encoder.setPosition(STOWED_ANGLE.in(Degrees)).apply(context);
                }
            });

    @Telemetry
    private boolean homed() {
        return homed.get();
    }

    private boolean isHomed() {
        return homed.get();
    }

    // Actions
    // Drive into the hard stop until the motor stalls (current up, no movement), then zero. The
    // 1.5 s timeout keeps the old timed homing as a fallback if the stall is never seen.
    private final MotorStallSignal homingStall = motor.stall()
            .atCurrentLimit(0.3) // 12 A of the current limit
            .velocityBelow(RotationsPerSecond.of(1.0)) // motor speed
            .forTime(Seconds.of(0.1));
    public final Action home = Actions.sequence()
            .run(Actions.untilStalled(homingStall, motor.percent(() -> -homingOutputPercent)).timeout(Seconds.of(1.5)))
            .doOnce(() -> homed.set(true))
            .doOnce(encoder.setPosition(STOWED_ANGLE.in(Degrees)));
    public final Action neutral = angle.neutral();

    /**
     * Health stages for the arm: once it has been pushing a stalled arm for a second (blocked by a
     * game piece or the field) output drops to half, a hot motor drops further, and an overheated one
     * lets go. A stage is left
     * only after its condition has stayed false for a second.
     */
    public enum ArmHealth { NORMAL, HOLDING, HOT, SHUTDOWN }
    private final BindingCheck pushingStop = angle.feedbackStalled().forTime(Seconds.of(1.0));
    public final Staging<ArmHealth> health = Staging.of(ArmHealth.NORMAL)
            .stage(ArmHealth.HOLDING, pushingStop, angle.limitEffort(0.5))
            .stage(ArmHealth.HOT, () -> motor.thermalLoad() > 0.6, angle.limitEffort(0.3))
            .stage(ArmHealth.SHUTDOWN, () -> motor.thermalLoad() >= 1.0, neutral)
            .exitAfter(Seconds.of(1.0));

    /**
     * Torque-limited moves instead of timed percent pushes. The effort cap is a current (torque) limit on
     * the motor, enforced every cycle, so the profile can ask for whatever it wants without slamming:
     * stow lifts against gravity with a lot of torque available; deploy goes down with very little, rests
     * on the stop lightly, and if something hits the intake the motor cannot fight back past the cap, so
     * it stalls and gives way instead of breaking. Tune the two caps on the robot.
     */
    private static final Current STOW_CURRENT = Amps.of(30.0);
    private static final Current DEPLOY_CURRENT = Amps.of(6.0);

    public final Action stow = angle.position(STOWED_ANGLE)
            .effort(STOW_CURRENT)
            .staged(health);
    public final Action deploy = angle.position(DEPLOYED_ANGLE)
            .effort(DEPLOY_CURRENT)
            .staged(health);

    @Telemetry
    private String healthStage() {
        return health.current().get().name();
    }
    public final Action cycle = Actions.cycle()
            .forTime(Seconds.of(0.5), stow)
            .forTime(Seconds.of(0.5), deploy);
   
}
