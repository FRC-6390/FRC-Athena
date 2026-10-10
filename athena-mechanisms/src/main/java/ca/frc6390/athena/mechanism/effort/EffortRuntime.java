package ca.frc6390.athena.mechanism.effort;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.units.UnitConversions;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import edu.wpi.first.units.Units;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * Per-binding effort state (B5) and current-limit enforcement (B6).
 *
 * <p>Each applied cycle the output path calls {@link #advance(double, double, double, double, double)}
 * once, then {@link #clampVolts(ActionContext, double)} on the voltage it is about to command.</p>
 *
 * <p><b>Current limit (B6).</b> For motor {@code i} with winding resistance {@code R_i}, speed constant
 * {@code Kv_i} (rad/s per volt) and measured speed {@code omega_i}, a voltage {@code V} drives the
 * current {@code I_i = (V - omega_i / Kv_i) / R_i}. The clamp keeps {@code |I_i| <= limit_i} by bounding
 * {@code V} to {@code [omega_i/Kv_i - limit_i R_i, omega_i/Kv_i + limit_i R_i]}, intersected over the
 * motors. It bounds the voltage the controller is asked to apply, so it works with any controller.
 * Motors without known constants are not clamped and keep the hardware limit only. When no voltage
 * satisfies every motor, the midpoint of the empty interval is used. The velocity sign is assumed to
 * follow the applied voltage.</p>
 *
 * <p><b>Load (B3 formula).</b> The mechanism torque beyond the model's prediction is
 * {@code g * sum_i Kt_i (V - predicted - kA a) / R_i}, with {@code predicted} the model's feedforward
 * voltage. Detection uses the controller's requested voltage, before the clamp, because the clamp caps the
 * applied current at the ceiling. Clearance while yielded uses the voltage applied in the previous cycle.
 * With the current at zero, that reads only the terms the model predicts for gravity and friction.</p>
 *
 * <p><b>Load responses.</b> {@code YIELD} sets the limit to zero (compliant) when the requested load
 * reaches the ceiling. It returns to the ceiling after the applied load has been clear for the recovery
 * time, ramping in over the same time. Detection is suspended during the ramp so the controller can move
 * back toward its target. {@code pushThrough} allows the burst limit from the moment the load appears
 * until its duration ends. {@code slow} sets a speed scale while loaded, which the profile path reads.</p>
 */
public final class EffortRuntime implements ControlLoopRuntime {
    private static final double TWO_PI = 2.0 * Math.PI;

    private final EffortLimit declaration;
    private final List<MotorDevice> motors;
    private final MotorConstants[] constants;
    private final double[] ceilingAmps;
    private final double[] burstAmps;
    private final double burstSeconds;
    private final boolean yieldDeclared;
    private final double recoverSeconds;
    private final boolean slowDeclared;
    private final double slowFraction;
    private final boolean burstDeclared;
    private final double threshold;
    private final Supplier<MechanismModel> model;
    private final double baseFactor;
    private final double[] activeAmps;
    private boolean anyLimit;
    private boolean yielded;
    private boolean ramping;
    private double loadedSeconds;
    private double clearSeconds;
    private double rampElapsed;
    private double speedScale = 1.0;
    private double demandLoad = Double.NaN;
    private double appliedLoad = Double.NaN;
    private double lastVelocityBase = Double.NaN;
    private double lastAppliedVolts = Double.NaN;
    private double predictedVolts;
    private double accelerationBase;
    private double kA;
    private boolean kinematicsValid;

    /**
     * Creates the runtime for a binding's motors.
     *
     * @param declaration binding ceiling, or null when the binding has none (action limits still apply)
     * @param motors motors of the binding
     */
    EffortRuntime(EffortLimit declaration, List<MotorDevice> motors) {
        this.declaration = declaration;
        this.motors = List.copyOf(motors);
        int count = this.motors.size();
        constants = new MotorConstants[count];
        for (int i = 0; i < count; i++) {
            constants[i] = EffortUnits.constants(this.motors.get(i)).orElse(null);
        }
        activeAmps = new double[count];
        if (declaration == null || count == 0) {
            ceilingAmps = new double[count];
            Arrays.fill(ceilingAmps, Double.POSITIVE_INFINITY);
            burstAmps = null;
            burstSeconds = 0.0;
            yieldDeclared = false;
            recoverSeconds = 0.0;
            slowDeclared = false;
            slowFraction = 1.0;
            burstDeclared = false;
            threshold = Double.NaN;
            model = null;
            baseFactor = 1.0;
        } else {
            Effort effort = declaration.effort();
            ceilingAmps = EffortUnits.perMotorAmps(effort.ceiling(), declaration.kind(), this.motors,
                    effort.gearing(), effort.radius());
            LoadResponse.Yield yieldResponse = null;
            LoadResponse.PushThrough push = null;
            LoadResponse.Slow slow = null;
            for (LoadResponse response : effort.responses()) {
                if (response instanceof LoadResponse.Yield found) {
                    yieldResponse = found;
                } else if (response instanceof LoadResponse.PushThrough found) {
                    push = found;
                } else if (response instanceof LoadResponse.Slow found) {
                    slow = found;
                }
            }
            model = declaration.model();
            baseFactor = declaration.baseFactor();
            yieldDeclared = yieldResponse != null && model != null;
            recoverSeconds = yieldResponse == null || yieldResponse.recovery() == null ? 0.0
                    : UnitConversions.value(yieldResponse.recovery(), Units.Seconds);
            burstDeclared = push != null && model != null;
            burstSeconds = push == null ? 0.0 : UnitConversions.value(push.duration(), Units.Seconds);
            burstAmps = push == null ? null : EffortUnits.perMotorAmps(push.burst(),
                    EffortUnits.kind(push.burst(), declaration.motion()), this.motors,
                    effort.gearing(), effort.radius());
            slowDeclared = slow != null && model != null;
            slowFraction = slow == null ? 1.0 : slow.fraction();
            threshold = declaration.threshold();
        }
    }

    /**
     * Creates a runtime for a binding without an effort ceiling. It still applies Action limits.
     *
     * @param motors motors of the binding
     * @return limit-free runtime
     */
    public static EffortRuntime unlimited(List<MotorDevice> motors) {
        return new EffortRuntime(null, motors);
    }

    @Override
    public ControlOutput calculate(ControlLoopContext context) {
        return null;
    }

    @Override
    public void reset(ControlLoopContext context) {
        reset();
    }

    /** Clears load state and the speed scale. Called on neutral, cancellation and disable. */
    public void reset() {
        yielded = false;
        ramping = false;
        loadedSeconds = 0.0;
        clearSeconds = 0.0;
        rampElapsed = 0.0;
        speedScale = 1.0;
        demandLoad = Double.NaN;
        appliedLoad = Double.NaN;
        lastVelocityBase = Double.NaN;
        lastAppliedVolts = Double.NaN;
        anyLimit = false;
        if (declaration != null) {
            declaration.publish(Double.POSITIVE_INFINITY, Double.NaN, false, Double.NaN, 1.0);
        }
    }

    /**
     * Returns the profile speed multiplier from {@code slow(...)}, 1 when not loaded.
     *
     * @return speed scale in {@code (0, 1]}
     */
    public double speedScale() {
        return speedScale;
    }

    /**
     * Returns whether the binding is currently yielding to a load.
     *
     * @return true while the position authority is dropped
     */
    public boolean yielded() {
        return yielded;
    }

    /**
     * Advances load state and computes the active per-motor limits for this cycle. Call exactly once
     * per applied cycle, before {@link #clampVolts(ActionContext, double)}.
     *
     * @param actionLimitAmps per-motor limit from the running Action (infinite for none)
     * @param dtSeconds cycle duration in seconds
     * @param requestedVolts controller voltage before the clamp, NaN when the output is not a voltage
     * @param position feedback position in the binding's unit
     * @param velocity feedback velocity in the binding's unit per second
     */
    public void advance(double actionLimitAmps, double dtSeconds, double requestedVolts, double position,
            double velocity) {
        double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.0;
        boolean responding = declaration != null && model != null
                && (yieldDeclared || burstDeclared || slowDeclared);
        boolean detected = false;
        if (responding) {
            computeKinematics(position, velocity, dt);
            demandLoad = torqueFor(requestedVolts);
            appliedLoad = torqueFor(lastAppliedVolts);
            detected = Double.isFinite(demandLoad) && threshold > 0.0 && Math.abs(demandLoad) >= threshold;
            if (yieldDeclared) {
                advanceYield(dt, detected);
            }
            loadedSeconds = detected ? loadedSeconds + dt : 0.0;
        }
        boolean burst = burstDeclared && detected && !yielded && loadedSeconds <= burstSeconds;
        speedScale = slowDeclared && detected ? slowFraction : 1.0;
        double fraction = rampFraction();
        double minimum = Double.POSITIVE_INFINITY;
        boolean limited = false;
        for (int i = 0; i < activeAmps.length; i++) {
            double base;
            if (yielded) {
                base = 0.0;
            } else if (burst && burstAmps != null) {
                base = burstAmps[i];
            } else {
                double ceiling = ceilingAmps[i];
                base = Double.isInfinite(ceiling) ? ceiling : ceiling * fraction;
            }
            double active = Math.min(base, actionLimitAmps);
            activeAmps[i] = active;
            minimum = Math.min(minimum, active);
            limited |= Double.isFinite(active);
        }
        anyLimit = limited;
        if (declaration != null) {
            declaration.publish(minimum, mechanismEffort(minimum), yielded,
                    responding ? demandLoad : Double.NaN, speedScale);
        }
    }

    private void advanceYield(double dt, boolean detected) {
        if (yielded) {
            boolean stillLoaded = Double.isFinite(appliedLoad) && threshold > 0.0
                    && Math.abs(appliedLoad) >= threshold;
            clearSeconds = stillLoaded ? 0.0 : clearSeconds + dt;
            if (clearSeconds >= recoverSeconds) {
                yielded = false;
                ramping = true;
                rampElapsed = 0.0;
            }
        } else if (ramping) {
            rampElapsed += dt;
            if (rampElapsed >= recoverSeconds) {
                ramping = false;
            }
        } else if (detected) {
            yielded = true;
            clearSeconds = 0.0;
        }
    }

    /**
     * Clamps a commanded voltage so the predicted motor current stays within the active limits.
     *
     * @param context hardware access for speed reads
     * @param volts commanded voltage
     * @return the clamped voltage, or {@code volts} itself when no limit applies
     */
    public double clampVolts(ActionContext context, double volts) {
        if (!anyLimit || !Double.isFinite(volts)) {
            lastAppliedVolts = volts;
            return volts;
        }
        double lower = Double.NEGATIVE_INFINITY;
        double upper = Double.POSITIVE_INFINITY;
        boolean enforced = false;
        for (int i = 0; i < activeAmps.length; i++) {
            MotorConstants motor = constants[i];
            double limit = activeAmps[i];
            if (motor == null || !Double.isFinite(limit)) {
                continue;
            }
            MotorHandle handle = context.motor(motors.get(i));
            double omega = finiteOrZero(handle.integratedVelocityRotationsPerSecond()) * TWO_PI;
            double backEmf = omega / motor.speedConstant();
            double span = limit * motor.resistanceOhms();
            lower = Math.max(lower, backEmf - span);
            upper = Math.min(upper, backEmf + span);
            enforced = true;
        }
        double clamped = volts;
        if (enforced) {
            clamped = lower > upper ? 0.5 * (lower + upper) : Math.min(Math.max(volts, lower), upper);
        }
        lastAppliedVolts = clamped;
        return clamped;
    }

    private double rampFraction() {
        if (!ramping || recoverSeconds <= 0.0) {
            return 1.0;
        }
        return Math.min(1.0, rampElapsed / recoverSeconds);
    }

    /** Computes the model's predicted voltage and the acceleration (base units) for this cycle. */
    private void computeKinematics(double position, double velocity, double dt) {
        double velocityBase = velocity * baseFactor;
        kinematicsValid = Double.isFinite(velocityBase) && Double.isFinite(position);
        if (!kinematicsValid) {
            return;
        }
        accelerationBase = Double.isNaN(lastVelocityBase) || dt <= 0.0
                ? 0.0 : (velocityBase - lastVelocityBase) / dt;
        lastVelocityBase = velocityBase;
        MechanismModel current = model.get();
        predictedVolts = current.feedforwardVolts(position * baseFactor, velocityBase, 0.0, 0.0);
        kA = current.kA();
    }

    /** Converts a voltage into the mechanism torque (N m) it adds beyond the model's prediction. */
    private double torqueFor(double volts) {
        if (!kinematicsValid || !Double.isFinite(volts)) {
            return Double.NaN;
        }
        double loadVolts = volts - predictedVolts - kA * accelerationBase;
        double torque = 0.0;
        for (MotorConstants motor : constants) {
            if (motor == null) {
                return Double.NaN;
            }
            torque += loadVolts * motor.torqueConstant() / motor.resistanceOhms();
        }
        return torque * declaration.effort().gearing();
    }

    /** Converts the active per-motor amps to the ceiling's mechanism unit (torque, force or amps). */
    private double mechanismEffort(double minimumAmps) {
        if (!Double.isFinite(minimumAmps) || declaration.kind() == EffortUnits.Kind.CURRENT) {
            return minimumAmps;
        }
        double torque = 0.0;
        for (int i = 0; i < activeAmps.length; i++) {
            MotorConstants motor = constants[i];
            if (motor == null || !Double.isFinite(activeAmps[i])) {
                return Double.NaN;
            }
            torque += activeAmps[i] * motor.torqueConstant() * declaration.effort().gearing();
        }
        if (declaration.kind() == EffortUnits.Kind.FORCE) {
            return torque / UnitConversions.value(declaration.effort().radius(), Units.Meters);
        }
        return torque;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
