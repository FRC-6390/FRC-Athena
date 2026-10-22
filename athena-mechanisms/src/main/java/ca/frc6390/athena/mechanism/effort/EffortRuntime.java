package ca.frc6390.athena.mechanism.effort;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
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
import java.util.Optional;
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
 * motors, after mapping each interval from its handle's measurement frame into the shared command frame.
 * It bounds the voltage the controller is asked to apply, so it works with any controller.
 * When no voltage satisfies every motor, the command is rejected so the output path can neutralize it. The velocity sign is
 * assumed to follow the applied voltage in the handle's measurement frame.</p>
 *
 * <p>The per-motor limit is the smallest of the binding ceiling, the Action effort and the power/thermal cap
 * from the power pass ({@code PowerRuntime}). The output path passes the caps in with
 * {@link #advance(double, double[], double, double, double, double)}, so every voltage command is bounded,
 * closed-loop or open-loop. A motor without known constants (no {@code MotorSpec} row) has no model to invert.
 * Its command is scaled instead: when its measured supply current exceeds its limit, the voltage is multiplied by
 * {@code limit / measured}. That is an estimate, not a bound, and it settles over several cycles.</p>
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
    private static final double[] NO_LIMITS = new double[0];

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
                    : Units.Seconds.fromBaseUnits(yieldResponse.recovery().baseUnitMagnitude());
            burstDeclared = push != null && model != null;
            burstSeconds = push == null ? 0.0 : Units.Seconds.fromBaseUnits(push.duration().baseUnitMagnitude());
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

    /** Local binding and Action winding limit before the robot-wide power allocation. */
    public double activeLimitAmps(int index) {
        return activeAmps[index];
    }

    /** Records the final command after the robot-wide allocation, without advancing load state again. */
    public void appliedVolts(double volts) {
        lastAppliedVolts = volts;
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
        advance(actionLimitAmps, null, dtSeconds, requestedVolts, position, velocity);
    }

    /**
     * Advances load state and computes the active per-motor limits for this cycle, including the power caps.
     *
     * @param actionLimitAmps per-motor limit from the running Action (infinite for none)
     * @param powerCapsAmps per-motor power/thermal caps from the power pass, in the binding's motor order, or null
     * @param dtSeconds cycle duration in seconds
     * @param requestedVolts controller voltage before the clamp, NaN when the output is not a voltage
     * @param position feedback position in the binding's unit
     * @param velocity feedback velocity in the binding's unit per second
     */
    public void advance(double actionLimitAmps, double[] powerCapsAmps, double dtSeconds, double requestedVolts,
            double position, double velocity) {
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
            if (powerCapsAmps != null && i < powerCapsAmps.length) {
                active = Math.min(active, powerCapsAmps[i]);
            }
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
        return clampVolts(context, volts, List.of(), NO_LIMITS);
    }

    /**
     * Clamps a commanded voltage to the active limits of this binding and to the limits of extra motors that
     * receive the same command (software followers). The bound is one intersection, so no motor exceeds its limit.
     *
     * @param context hardware access for speed and current reads
     * @param volts commanded voltage
     * @param extraMotors additional motors that receive the command, may be empty
     * @param extraLimitsAmps per-motor limits of {@code extraMotors}, same order
     * @return the clamped voltage
     */
    public double clampVolts(ActionContext context, double volts, List<MotorDevice> extraMotors,
            double[] extraLimitsAmps) {
        return clampVolts(context, volts, extraMotors, extraLimitsAmps, null);
    }

    /**
     * Clamps with the actual extra command handles. A failover group may replace a software follower's wrapper
     * while retaining the original handle for device telemetry, so the output path passes the command handles.
     *
     * @param context hardware access for the binding motors
     * @param volts shared commanded voltage
     * @param extraMotors additional motor declarations
     * @param extraLimitsAmps their current limits
     * @param extraHandles their active command handles, or null to resolve through the context
     * @return clamped voltage in the shared command frame
     */
    public double clampVolts(ActionContext context, double volts, List<MotorDevice> extraMotors,
            double[] extraLimitsAmps, MotorHandle[] extraHandles) {
        if (!anyLimit && extraMotors.isEmpty() || !Double.isFinite(volts)) {
            lastAppliedVolts = volts;
            return volts;
        }
        double clamped = clampVolts(context, volts, motors, activeAmps, null, extraMotors, extraLimitsAmps, extraHandles);
        lastAppliedVolts = clamped;
        return clamped;
    }

    /**
     * Clamps a voltage to the per-motor current limits of a set of motors that share the command. Used by
     * the output path for open-loop motor outputs, which have no binding. Limits are not tied to a binding,
     * so the caller passes them in.
     *
     * @param context hardware access
     * @param volts commanded voltage
     * @param motors motors that receive the command
     * @param limitsAmps per-motor current limits, same order, infinite for none
     * @return the clamped voltage
     */
    public static double clampToLimits(ActionContext context, double volts, List<MotorDevice> motors,
            double[] limitsAmps) {
        return clampToLimits(context, volts, motors, limitsAmps, null);
    }

    /**
     * Clamps shared open-loop voltage with explicit command handles; see
     * {@link #clampVolts(ActionContext, double, List, double[], MotorHandle[])}.
     *
     * @param context fallback hardware access
     * @param volts shared commanded voltage
     * @param motors motor declarations
     * @param limitsAmps their current limits
     * @param handles active command handles, or null to resolve through the context
     * @return clamped voltage in the shared command frame
     */
    public static double clampToLimits(ActionContext context, double volts, List<MotorDevice> motors,
            double[] limitsAmps, MotorHandle[] handles) {
        return clampVolts(context, volts, motors, limitsAmps, handles, List.of(), NO_LIMITS, null);
    }

    private static double clampVolts(ActionContext context, double volts, List<MotorDevice> motors,
            double[] limitsAmps, MotorHandle[] handles, List<MotorDevice> extraMotors, double[] extraLimitsAmps,
            MotorHandle[] extraHandles) {
        if (!Double.isFinite(volts)) {
            return volts;
        }
        Bounds bounds = new Bounds();
        for (int i = 0; i < motors.size(); i++) {
            if (!Double.isFinite(limitsAmps[i])) continue;
            bounds.add(motors.get(i), handles == null ? context.motor(motors.get(i)) : handles[i], limitsAmps[i]);
        }
        for (int i = 0; i < extraMotors.size(); i++) {
            if (!Double.isFinite(extraLimitsAmps[i])) continue;
            bounds.add(extraMotors.get(i),
                    extraHandles == null ? context.motor(extraMotors.get(i)) : extraHandles[i], extraLimitsAmps[i]);
        }
        return bounds.apply(volts);
    }

    /** Voltage interval (model-based) and scale (estimate-based) accumulated over the motors of one command. */
    private static final class Bounds {
        private double lower = Double.NEGATIVE_INFINITY;
        private double upper = Double.POSITIVE_INFINITY;
        private double scale = 1.0;
        private boolean enforced;

        void add(MotorDevice device, MotorHandle handle, double limitAmps) {
            if (!Double.isFinite(limitAmps)) {
                return;
            }
            double commandScale = handle.voltageCommandScale();
            // A suppressed command cannot be adjusted by this clamp. Other motors still constrain the command.
            if (commandScale == 0.0) return;
            EffortUnits.Electrical motor = EffortUnits.electrical(device);
            if (motor != null) {
                double omega = finiteOrZero(handle.integratedVelocityRotationsPerSecond()) * TWO_PI;
                double backEmf = omega / motor.speedConstantRadPerSecPerVolt();
                double span = limitAmps * motor.resistanceOhms();
                double first = (backEmf - span) / commandScale;
                double second = (backEmf + span) / commandScale;
                lower = Math.max(lower, Math.min(first, second));
                upper = Math.min(upper, Math.max(first, second));
            } else {
                // No constants: scale the command by the limit over the measured supply current.
                if (limitAmps <= 0.0) {
                    scale = 0.0;
                    enforced = true;
                    return;
                }
                double measured;
                try {
                    measured = Math.abs(handle.supplyCurrentAmps());
                } catch (RuntimeException exception) {
                    measured = Double.NaN;
                }
                if (Double.isFinite(measured) && measured > limitAmps) {
                    scale = Math.min(scale, Math.max(0.0, limitAmps) / measured);
                }
            }
            enforced = true;
        }

        double apply(double volts) {
            double minimum = lower;
            double maximum = upper;
            if (scale < 1.0) {
                // An estimated unknown-motor ceiling is another voltage constraint. Scaling after
                // projecting the known-motor interval can move outside it when back-EMF is nonzero.
                double magnitude = Math.abs(volts) * scale;
                minimum = Math.max(minimum, -magnitude);
                maximum = Math.min(maximum, magnitude);
            }
            if (minimum > maximum) {
                throw new IllegalStateException("No shared voltage satisfies the motors' current limits.");
            }
            return enforced ? Math.min(Math.max(volts, minimum), maximum) : volts;
        }
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
            torque += loadVolts * motor.raw().torqueConstantNmPerAmp() / motor.resistance().in(Units.Ohms);
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
            torque += activeAmps[i] * motor.raw().torqueConstantNmPerAmp() * declaration.effort().gearing();
        }
        if (declaration.kind() == EffortUnits.Kind.FORCE) {
            return torque / Units.Meters.fromBaseUnits(declaration.effort().radius().baseUnitMagnitude());
        }
        return torque;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
