package ca.frc6390.athena.mechanism.effort;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.hardware.units.UnitConversions;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Binding-level effort ceiling (B5), stored on a control binding as a loop declaration.
 *
 * <p>It is a {@link ControlLoop} with the default software-output role, so a binding with an effort
 * ceiling always runs its loops in Athena. Device-side closed loops could not be clamped to a current
 * limit. The declaration adds no output of its own. Its {@link EffortRuntime} clamps the final voltage
 * (B6) and runs the load responses.</p>
 */
public final class EffortLimit implements ControlLoop, TelemetrySource {
    private final Effort effort;
    private final EffortUnits.Kind kind;
    private final MotionKind motion;
    private final Supplier<MechanismModel> model;
    private final double baseFactor;
    private final double threshold;
    private final double ceilingAmpsMax;
    private volatile double activeAmps = Double.POSITIVE_INFINITY;
    private volatile double activeMechanism = Double.NaN;
    private volatile double loadValue = Double.NaN;
    private volatile double speedScale = 1.0;
    private volatile boolean yielded;
    private final Map<String, TelemetryValue> telemetry;

    private EffortLimit(Effort effort, EffortUnits.Kind kind, MotionKind motion, Supplier<MechanismModel> model,
            double baseFactor, double threshold, double ceilingAmpsMax) {
        this.effort = effort;
        this.kind = kind;
        this.motion = motion;
        this.model = model;
        this.baseFactor = baseFactor;
        this.threshold = threshold;
        this.ceilingAmpsMax = ceilingAmpsMax;
        this.telemetry = Map.of(
                "CeilingUnit", TelemetryValue.constant(effort.ceiling().unit().name()),
                "Ceiling", TelemetryValue.constant(effort.ceiling().magnitude()),
                "CeilingAmps", TelemetryValue.constant(ceilingAmpsMax),
                "ActiveAmps", TelemetryValue.number(() -> activeAmps),
                "ActiveEffort", TelemetryValue.number(() -> activeMechanism),
                "Load", TelemetryValue.number(() -> loadValue),
                "LoadThreshold", TelemetryValue.constant(Double.isFinite(threshold) ? threshold : 0.0),
                "Yielded", TelemetryValue.bool(() -> yielded),
                "SpeedScale", TelemetryValue.number(() -> speedScale));
    }

    /**
     * Validates an effort ceiling against a control and creates its declaration.
     *
     * @param effort declared ceiling
     * @param motion motion kind of the binding
     * @param motors motors of the binding
     * @param model the binding's model supplier (B3 feedforward for load estimates), or null without a model
     * @param baseFactor multiplier from the binding unit to the model's base units (rotations or meters)
     * @return declaration
     * @throws IllegalArgumentException when the units, gearing or motor constants do not suit the binding
     */
    public static EffortLimit declare(Effort effort, MotionKind motion, List<MotorDevice> motors,
            Supplier<MechanismModel> model, double baseFactor) {
        Objects.requireNonNull(effort, "effort");
        Objects.requireNonNull(motion, "motion");
        Objects.requireNonNull(motors, "motors");
        EffortUnits.Kind kind = EffortUnits.kind(effort.ceiling(), motion);
        double magnitude = effort.ceiling().magnitude();
        if (!Double.isFinite(magnitude) || magnitude < 0.0) {
            throw new IllegalArgumentException("Effort ceiling must be finite and non-negative.");
        }
        validateMeasure(effort.radius(), "Effort radius", Units.Meters, true);
        if (motors.isEmpty()) {
            throw new IllegalArgumentException("Effort needs a control with at least one motor.");
        }
        double[] ceiling = EffortUnits.perMotorAmps(effort.ceiling(), kind, motors, effort.gearing(),
                effort.radius());
        double ceilingMax = 0.0;
        for (double amps : ceiling) {
            ceilingMax = Math.max(ceilingMax, amps);
        }
        validateResponses(effort, motion, motors);
        boolean responding = !effort.responses().isEmpty() && model != null;
        double threshold = Double.NaN;
        if (responding) {
            threshold = EffortUnits.mechanismTorque(ceiling, motors, effort.gearing());
            if (!Double.isFinite(threshold)) {
                throw new IllegalArgumentException("onLoad responses compare the load with the ceiling in "
                        + "mechanism torque — declare gearing with Effort.gearing(...) and use motors with "
                        + "known constants, or drop the responses.");
            }
        }
        return new EffortLimit(effort, kind, motion, responding ? model : null, baseFactor, threshold, ceilingMax);
    }

    private static void validateResponses(Effort effort, MotionKind motion, List<MotorDevice> motors) {
        int yields = 0;
        int holds = 0;
        int pushes = 0;
        int slows = 0;
        for (LoadResponse response : effort.responses()) {
            if (response instanceof LoadResponse.Yield yieldResponse) {
                yields++;
                validateMeasure(yieldResponse.recovery(), "YIELD recoverAfter", Units.Seconds, true);
            } else if (response instanceof LoadResponse.Hold) {
                holds++;
            } else if (response instanceof LoadResponse.PushThrough push) {
                pushes++;
                validateMeasure(push.duration(), "pushThrough duration", Units.Seconds, false);
                EffortUnits.Kind burstKind = EffortUnits.kind(push.burst(), motion);
                EffortUnits.perMotorAmps(push.burst(), burstKind, motors, effort.gearing(), effort.radius());
            } else if (response instanceof LoadResponse.Slow) {
                slows++;
            }
        }
        if (yields > 1 || holds > 1 || pushes > 1 || slows > 1) {
            throw new IllegalArgumentException("onLoad accepts at most one YIELD, HOLD, pushThrough and slow.");
        }
    }

    private static void validateMeasure(Measure<?> measure, String name, Unit dimension, boolean optional) {
        if (measure == null) {
            if (optional) {
                return;
            }
            throw new IllegalArgumentException(name + " is required.");
        }
        if (!UnitConversions.sameDimension(measure.unit(), dimension)) {
            throw new IllegalArgumentException(name + " must measure " + dimension.name() + ", not "
                    + measure.unit().name() + ".");
        }
    }

    /** Returns the declared effort ceiling. */
    public Effort effort() {
        return effort;
    }

    /** Returns the motion kind of the binding that declared this ceiling. */
    public MotionKind motion() {
        return motion;
    }

    /** Returns the kind of the declared ceiling (torque, force or current). */
    public EffortUnits.Kind kind() {
        return kind;
    }

    /** Returns the model supplier for load estimates, or null when no load response applies. */
    public Supplier<MechanismModel> model() {
        return model;
    }

    /** Returns the multiplier from the binding unit to the model's base units. */
    public double baseFactor() {
        return baseFactor;
    }

    /** Returns the load threshold in mechanism torque (N m), NaN when no response applies. */
    public double threshold() {
        return threshold;
    }

    /** Returns the largest per-motor ceiling at declaration. */
    public edu.wpi.first.units.measure.Current ceilingMax() {
        return edu.wpi.first.units.Units.Amps.of(ceilingAmpsMax);
    }

    /** Creates the per-binding runtime. The runtime is bound once for each binding identity. */
    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new EffortRuntime(this, binding.control().motors());
    }

    /**
     * Returns whether the binding is yielding to a load, as of the last cycle it ran.
     *
     * @return true while position authority is dropped
     */
    public boolean yielded() {
        return yielded;
    }

    /** Publishes the latest runtime state for telemetry. */
    void publish(double activeAmps, double activeMechanism, boolean yielded, double load, double speedScale) {
        this.activeAmps = activeAmps;
        this.activeMechanism = activeMechanism;
        this.yielded = yielded;
        this.loadValue = load;
        this.speedScale = speedScale;
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }
}
