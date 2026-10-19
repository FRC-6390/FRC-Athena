package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Interrupt;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Release;
import ca.frc6390.athena.mechanism.core.Retarget;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.model.BindingCheck;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import ca.frc6390.athena.hardware.units.MotionKind;
import edu.wpi.first.units.AngleUnit;
import frc.robot.Constants;
import java.time.Duration;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Single-joint arm declared with typed units. The encoder reports degrees, the binding takes and returns degrees,
 * and every tolerance and limit is a measure. The characterization identifies the arm at runtime; the simulation
 * uses the code default model until the characterization is fitted on the robot.
 */
public final class SingleJointArm implements Mechanism {
    /** Starting model used until the characterization fits real data (kA sets the inertia in simulation). */
    private static final MechanismModel DEFAULT_MODEL = new MechanismModel(
            MotionKind.ROTARY, 0.25, 0.20, 0.12, 0.02, 0.0, 0.0, FrictionTable.EMPTY,
            0.0, 0.0, 0.0,
            0.0195, 0.12, 1, List.of("KRAKEN_X60"), 10.0, Map.of(), 0.0, 0);

    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 10).brake().currentLimit(30);
    private final EncoderDevice encoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 10).measures(Degrees);

    /** The base declaration: typed feedback, gains, constraints, and motion policies. */
    private final ControlBinding<AngleUnit> position = Controls.position(motor, Degrees)
            .feedback(encoder)
            .pid(0.84, 0.0, 0.024)
            .constraints(
                    Constraints.range(Degrees.of(-10.0), Degrees.of(105.0)),
                    Constraints.motion(DegreesPerSecond.of(90.0), DegreesPerSecondPerSecond.of(220.0)))
            .retarget(Retarget.SMOOTH)   // default: a new target re-plans from the current reference
            .release(Release.HOLD);      // keep the stow angle when no Action drives the arm

    /** Identifies the arm from motion tests. The dashboard gets QuasistaticForward, DynamicForward, and so on. */
    public final Characterization characterization = position.characterization()
            .maxVoltage(Volts.of(4.0))
            .rampRate(Volts.per(Seconds).of(0.5))
            .learnContinuously(true)
            .driftTolerance(0.25)
            .gearRatio(10.0)
            .defaults(DEFAULT_MODEL);

    /** The active model: the code default until a fit, then the fitted values. */
    public final MechanismModel model = characterization.model();

    /** The same declaration, now feedforward and load estimates come from the characterization. */
    private final ControlBinding<AngleUnit> control = position.model(characterization);

    /** Simulation plant from the model (`SimModel.model`), so the physics follows the identified arm. */
    @SuppressWarnings("unused")
    private final SimModel simulation = SimModel.arm(motor).encoder(encoder).model(model);

    /** Checks exist because the binding has a model. Thresholds and debounce are overridable. */
    public final BindingCheck blocked = control.blocked().threshold(1.5).forSeconds(0.25);
    public final DoubleSupplier load = control.load();

    /** Stow completes before a newer request can take the arm (bounded by atMost). */
    public final Action stow = control.position(Degrees.of(0.0))
            .untilWithin(Degrees.of(2.0))
            .atMost(Duration.ofSeconds(3))
            .interruptible(Interrupt.AFTER_COMPLETE);
    public final Action pickup = control.position(Degrees.of(35.0)).untilWithin(Degrees.of(2.0));
    public final Action score = control.position(Degrees.of(92.0)).untilWithin(Degrees.of(2.0));

    /**
     * Fallback chain: try a tight tolerance for two seconds, then accept a looser one for two seconds, and report a
     * fault if neither reaches the pickup angle.
     */
    public final Action pickupOrFault = Actions.attempt(control.position(Degrees.of(35.0)).untilWithin(Degrees.of(1.0)))
            .within(2.0)
            .orTry(control.position(Degrees.of(35.0)).untilWithin(Degrees.of(4.0)))
            .within(2.0)
            .orElse(Actions.fault("arm did not reach the pickup angle"));

    /** Back off to stow when the arm meets an obstruction and the check stays true. */
    public final HookBinding stowWhenBlocked = Events.when(blocked).rising().onStart(stow);

    public final Action exercise = Actions.cycle()
            .run(control.position(Degrees.of(20.0)).untilWithin(Degrees.of(2.0)))
            .run(control.position(Degrees.of(85.0)).untilWithin(Degrees.of(2.0)));

    @Telemetry("loadVolts")
    public double loadTelemetry() {
        return load.getAsDouble();
    }
}
