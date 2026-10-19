package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.model.BindingCheck;
import ca.frc6390.athena.mechanism.staging.Staging;
import edu.wpi.first.units.AngleUnit;
import frc.robot.Calibration;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * An arm that backs off as its motor heats up or strains. Thermal derating scales the current limit on its own;
 * the staging enum below adds the stronger responses, and `lift.staged(health)` applies them only while the lift
 * runs.
 */
public final class StagedArm implements Mechanism {
    /** Team-defined health stages. The worst true stage (highest ordinal) applies. */
    public enum ArmHealth { NORMAL, STRAINED, HOT, SAFE_SHUTDOWN }

    /** Derates the limit from 100 C (full) down to 40 percent at 130 C. Thermal alone never brakes the motor. */
    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 20)
            .brake()
            .currentLimit(edu.wpi.first.units.Units.Amps.of(60))
            .thermal(Thermal.derate(Celsius.of(100.0), Celsius.of(130.0), 0.4));
    private final EncoderDevice encoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 20).measures(Degrees);

    private final ControlBinding<AngleUnit> angle = Controls.position(motor, Degrees)
            .feedback(encoder)
            .pid(0.84, 0.0, 0.024)
            .constraints(
                    Constraints.range(Degrees.of(-10.0), Degrees.of(105.0)),
                    Constraints.motion(DegreesPerSecond.of(90.0), DegreesPerSecondPerSecond.of(220.0)))
            .persistTuning(Calibration.class);

    /** Stalled without a model: a check that needs only the current and the commanded output. */
    private final BindingCheck stalled = angle.feedbackStalled().forTime(edu.wpi.first.units.Units.Seconds.of(0.5));

    public final Action stow = angle.position(Degrees.of(0.0)).untilWithin(Degrees.of(2.0));
    public final Action lift = angle.position(Degrees.of(80.0)).untilWithin(Degrees.of(2.0));

    /**
     * Stages: STRAINED caps the effort; HOT also caps speed and acceleration; SAFE_SHUTDOWN requests stow.
     * Leaving a stage requires its condition to stay false for two seconds.
     */
    public final Staging<ArmHealth> health = Staging.of(ArmHealth.NORMAL)
            .stage(ArmHealth.STRAINED,
                    () -> motor.thermalLoad() > 0.25 || stalled.getAsBoolean(),
                    angle.limitEffort(0.6))
            .stage(ArmHealth.HOT,
                    () -> motor.thermalLoad() > 0.6,
                    angle.limitEffort(0.4), angle.limitSpeed(0.5), angle.limitAccel(0.5))
            .stage(ArmHealth.SAFE_SHUTDOWN,
                    () -> motor.thermalLoad() >= 1.0,
                    stow)
            .exitAfter(Seconds.of(2.0));

    /** The lift runs under the staging: its effort, speed, and acceleration caps follow the current stage. */
    public final Action stagedLift = lift.staged(health);

    /** Read-only stage for the dashboard. The Staging also publishes its own transitions as replay markers. */
    public final Supplier<ArmHealth> stage = health.current();

    @Telemetry("thermalLoad")
    public double thermalLoad() {
        return motor.thermalLoad();
    }

    /** Persisted flag in the Calibration lock group: writable only while the robot is disabled (see Robot). */
    @Telemetry(persist = true, writable = true, lock = Calibration.class)
    public boolean calibrated = false;
}
