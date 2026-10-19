package ca.frc6390.athena.mechanism.control;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlLoopTrace;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Output;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.effort.EffortUnits;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Torque-based velocity loop: the velocity error becomes a motor <i>current</i> command, and the current is
 * turned into a voltage with the motor model, {@code V = I R + omega / Kv}, using the measured motor speed.
 *
 * <p>Gains are in amps of <b>total</b> motor current, summed over every motor of the binding (leader and
 * followers). Each motor carries an equal share, {@code I / N}. Every cycle:</p>
 *
 * <pre>
 * I = kP e + kI integral(e) - kD d(measured)/dt + kS sign(target) + kA a_ref + d_hat
 * V_i = (I / N) R_i + omega_i / Kv_i        (averaged over the motors, clamped to +-12 V)
 * </pre>
 *
 * <p>The back-EMF term is cancelled with the measured speed, so the current follows the command on any
 * controller that accepts a voltage. Unlike {@link PidGains} and {@link FeedforwardGains}, which produce volts
 * from a velocity error, no velocity feedforward gain is needed: the model's {@code Kv} and {@code R} do it.
 * Voltage saturation, the binding's effort ceiling and the power caps still bound the voltage after this loop,
 * and the integral is re-seeded from the applied voltage so it does not wind up.</p>
 *
 * <p>The optional load estimate {@code d_hat} (see {@link #loadEstimate(double)}) is a disturbance observer
 * that uses only the motor's measured current and speed. It adds the load current that the measured
 * acceleration does not explain, so a load step is compensated within about one control cycle.</p>
 *
 * <p>This loop runs in software. Declaring it on a binding keeps the binding's loops on the software output
 * path, the same as an effort ceiling.</p>
 */
public final class TorqueVelocityGains implements ControlLoop, TelemetrySource {
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double MAX_VOLTS = EffortUnits.NOMINAL_VOLTS;

    private volatile double p;
    private volatile double i;
    private volatile double d;
    private volatile double staticAmps;
    private volatile double inertia;
    private volatile double loadTimeConstant;
    private volatile boolean disabled;

    // Runtime state, written by the runtime every cycle for telemetry.
    private volatile double targetValue = Double.NaN;
    private volatile double measuredValue = Double.NaN;
    private volatile double currentCommand = Double.NaN;
    private volatile double voltageCommand = Double.NaN;
    private volatile double loadEstimate = Double.NaN;

    private final Map<String, TelemetryValue> telemetry = Map.ofEntries(
            Map.entry("p", TelemetryValue.writableNumber(this::p, this::setP)),
            Map.entry("i", TelemetryValue.writableNumber(this::i, this::setI)),
            Map.entry("d", TelemetryValue.writableNumber(this::d, this::setD)),
            Map.entry("static", TelemetryValue.writableNumber(this::staticAmps, this::setStaticAmps)),
            Map.entry("inertia", TelemetryValue.writableNumber(this::inertia, this::setInertia)),
            Map.entry("loadTimeConstant",
                    TelemetryValue.writableNumber(this::loadTimeConstant, this::setLoadTimeConstant)),
            Map.entry("disabled", TelemetryValue.writableBoolean(this::isDisabled, value -> disabled = value)),
            Map.entry("Target", TelemetryValue.number(() -> targetValue)),
            Map.entry("Measured", TelemetryValue.number(() -> measuredValue)),
            Map.entry("CurrentCommandAmps", TelemetryValue.number(() -> currentCommand)),
            Map.entry("VoltageCommand", TelemetryValue.number(() -> voltageCommand)),
            Map.entry("LoadEstimateAmps", TelemetryValue.number(() -> loadEstimate)));

    /**
     * Creates torque-velocity gains with no static term, inertia term or load estimate.
     *
     * @param p proportional gain in amps of total motor current per velocity unit
     * @param i integral gain in amps per velocity unit per second
     * @param d derivative gain in amps per velocity unit per second, on the measurement
     * @param staticAmps static friction current in amps of total motor current
     * @param inertia inertia gain in amps per velocity unit per second squared
     * @param loadTimeConstant load-estimate time constant in seconds, 0 to disable the estimate
     */
    public TorqueVelocityGains(double p, double i, double d, double staticAmps, double inertia,
            double loadTimeConstant) {
        setP(p);
        setI(i);
        setD(d);
        setStaticAmps(staticAmps);
        setInertia(inertia);
        setLoadTimeConstant(loadTimeConstant);
    }

    /**
     * Creates torque-velocity gains from PID terms only.
     *
     * @param p proportional gain in amps of total motor current per velocity unit
     * @param i integral gain in amps per velocity unit per second
     * @param d derivative gain in amps per velocity unit per second, on the measurement
     * @return gains
     */
    public static TorqueVelocityGains of(double p, double i, double d) {
        return new TorqueVelocityGains(p, i, d, 0.0, 0.0, 0.0);
    }

    /**
     * Returns these gains with a static friction current. The current is applied with the sign of the target,
     * so it is the current needed to overcome friction in the direction of motion.
     *
     * @param amps friction current in amps of total motor current, non-negative
     * @return gains with the static term
     */
    public TorqueVelocityGains staticAmps(double amps) {
        return new TorqueVelocityGains(p, i, d, amps, inertia, loadTimeConstant);
    }

    /**
     * Returns these gains with an inertia term. The current {@code kA * a_ref} accelerates the mechanism along
     * the reference, where {@code a_ref} is the acceleration of the reference the control follows.
     *
     * @param ampsPerUnitPerSecondSquared inertia gain in amps of total motor current per velocity unit per second squared
     * @return gains with the inertia term
     */
    public TorqueVelocityGains inertia(double ampsPerUnitPerSecondSquared) {
        return new TorqueVelocityGains(p, i, d, staticAmps, ampsPerUnitPerSecondSquared, loadTimeConstant);
    }

    /**
     * Returns these gains with the load estimate (a disturbance observer) enabled.
     *
     * <p>Each cycle the loop computes {@code raw = I_measured - kA * a_measured - kS sign(v)}, where {@code I_measured}
     * is the stator current summed over the motors. The raw value is low-pass filtered with this time constant and
     * added to the current command. A load that slows the mechanism then draws current at once, instead of waiting
     * for the speed error to build up. Use a time constant of about one control period (0.01 to 0.02 s at the
     * 20 ms loop): the estimate then follows a load step within a cycle, and the speed dip falls to about the
     * one-cycle latency of the loop. Larger values make the estimate smoother, which helps with noisy current
     * readings, but slow the compensation roughly in proportion. Use 0 to turn the estimate off. The estimate
     * needs a signed stator current ({@code MotorHandle#statorCurrentAmps}); a magnitude-only reading, such as
     * a SPARK output current, makes it wrong in the reverse direction.</p>
     *
     * @param timeConstantSeconds filter time constant in seconds, non-negative; 0 disables the estimate
     * @return gains with the load estimate
     */
    public TorqueVelocityGains loadEstimate(double timeConstantSeconds) {
        return new TorqueVelocityGains(p, i, d, staticAmps, inertia, timeConstantSeconds);
    }

    public double p() { return p; }
    public double i() { return i; }
    public double d() { return d; }
    public double staticAmps() { return staticAmps; }
    public double inertia() { return inertia; }
    /** Load-estimate time constant in seconds; 0 when the estimate is off. */
    public double loadTimeConstant() { return loadTimeConstant; }
    /** Whether the load estimate is on. */
    public boolean loadEstimateEnabled() { return loadTimeConstant > 0.0; }
    public boolean isDisabled() { return disabled; }

    private void setP(double value) { p = finite(value, "Torque velocity proportional gain"); }
    private void setI(double value) { i = finite(value, "Torque velocity integral gain"); }
    private void setD(double value) { d = finite(value, "Torque velocity derivative gain"); }
    private void setStaticAmps(double value) {
        staticAmps = nonNegative(value, "Torque velocity static current");
    }
    private void setInertia(double value) { inertia = finite(value, "Torque velocity inertia gain"); }
    private void setLoadTimeConstant(double value) {
        loadTimeConstant = nonNegative(value, "Torque velocity load-estimate time constant");
    }

    private static double finite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
        return value;
    }

    private static double nonNegative(double value, String description) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(description + " must be finite and non-negative.");
        }
        return value;
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(binding.control().motors(), binding.actionContext());
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof TorqueVelocityGains gains
                && Double.compare(p, gains.p) == 0 && Double.compare(i, gains.i) == 0
                && Double.compare(d, gains.d) == 0 && Double.compare(staticAmps, gains.staticAmps) == 0
                && Double.compare(inertia, gains.inertia) == 0
                && Double.compare(loadTimeConstant, gains.loadTimeConstant) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(p, i, d, staticAmps, inertia, loadTimeConstant);
    }

    @Override
    public String toString() {
        return "TorqueVelocityGains[p=" + p + ", i=" + i + ", d=" + d + ", static=" + staticAmps
                + ", inertia=" + inertia + ", loadTimeConstant=" + loadTimeConstant + "]";
    }

    /** Per-binding state: integral, derivative history, load estimate and the last cycle's voltage terms. */
    private final class Runtime implements ControlLoopRuntime {
        private final MotorDevice[] devices;
        private final MotorConstants[] constants;
        private final ActionContext actions;
        private final double count;

        private double integral;
        private double previousMeasurement = Double.NaN;
        private double filteredAcceleration;
        private double loadCurrent;
        private double lastNonIntegral;
        private double lastResistance = Double.NaN;
        private double lastEmfVolts = Double.NaN;
        private double lastUnclampedVolts = Double.NaN;

        private double seenP = p;
        private double seenI = i;
        private double seenD = d;
        private double seenStatic = staticAmps;
        private double seenInertia = inertia;
        private double seenLoad = loadTimeConstant;

        private Runtime(List<MotorDevice> motors, ActionContext actions) {
            this.actions = actions;
            this.devices = motors.toArray(new MotorDevice[0]);
            this.constants = new MotorConstants[devices.length];
            for (int index = 0; index < devices.length; index++) {
                MotorDevice device = devices[index];
                constants[index] = EffortUnits.constants(device).orElseThrow(() -> new IllegalStateException(
                        "Torque velocity control needs the motor constants of " + device.defaultName() + "."));
            }
            this.count = devices.length;
        }

        @Override
        public void reset(ControlLoopContext context) {
            integral = 0.0;
            previousMeasurement = Double.NaN;
            filteredAcceleration = 0.0;
            loadCurrent = 0.0;
            lastNonIntegral = 0.0;
            lastUnclampedVolts = Double.NaN;
            publish(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }

        @Override
        public boolean consumeGainChange() {
            boolean changed = p != seenP || i != seenI || d != seenD || staticAmps != seenStatic
                    || inertia != seenInertia || loadTimeConstant != seenLoad;
            if (changed) {
                seenP = p;
                seenI = i;
                seenD = d;
                seenStatic = staticAmps;
                seenInertia = inertia;
                seenLoad = loadTimeConstant;
                previousMeasurement = Double.NaN;
            }
            return changed;
        }

        @Override
        public boolean bumpTo(double targetVolts, double currentVolts) {
            if (i == 0.0 || !Double.isFinite(targetVolts) || !Double.isFinite(currentVolts)
                    || !Double.isFinite(lastResistance) || lastResistance <= 0.0) {
                return false;
            }
            // dVolts / dTotalAmps = R / N, so a voltage shift becomes an integral shift of shift * N / R.
            integral += (targetVolts - currentVolts) * count / lastResistance;
            lastUnclampedVolts = targetVolts;
            return true;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double target = context.target();
            double measured = context.velocity();
            if (!Double.isFinite(target) || !Double.isFinite(measured)) {
                return ControlOutput.neutral();
            }
            if (disabled) {
                lastUnclampedVolts = 0.0;
                return ControlOutput.voltage(0.0);
            }
            double[] omega = new double[devices.length];
            for (int index = 0; index < devices.length; index++) {
                omega[index] = motorSpeed(index);
            }
            if (!Double.isFinite(omega[0])) {
                return ControlOutput.neutral();
            }
            double emf = 0.0;
            double resistance = 0.0;
            for (int index = 0; index < devices.length; index++) {
                double speed = Double.isFinite(omega[index]) ? omega[index] : omega[0];
                emf += speed / constants[index].speedConstantRadPerSecPerVolt();
                resistance += constants[index].resistance().in(edu.wpi.first.units.Units.Ohms);
            }
            emf /= count;
            resistance /= count;

            double dt = context.dtSeconds();
            double step = Double.isFinite(dt) && dt > 0.0 ? dt : 0.0;
            double error = target - measured;
            boolean first = Double.isNaN(previousMeasurement) || step == 0.0;
            double acceleration = first ? 0.0 : (measured - previousMeasurement) / step;
            double derivative = first ? 0.0 : -acceleration;
            if (loadTimeConstant > 0.0 && step > 0.0) {
                updateLoadEstimate(measured, acceleration, step);
            }
            if (i != 0.0 && step > 0.0) {
                integral += i * error * step;
            }
            double referenceAcceleration = context.reference().acceleration();
            if (!Double.isFinite(referenceAcceleration)) {
                referenceAcceleration = 0.0;
            }
            double staticTerm = staticAmps * Math.signum(target);
            double nonIntegral = p * error + d * derivative + staticTerm
                    + inertia * referenceAcceleration + loadCurrent;
            double total = nonIntegral + integral;
            double volts = total / count * resistance + emf;
            double clamped = Math.max(-MAX_VOLTS, Math.min(MAX_VOLTS, volts));

            lastNonIntegral = nonIntegral;
            lastResistance = resistance;
            lastEmfVolts = emf;
            lastUnclampedVolts = volts;
            previousMeasurement = measured;
            publish(target, measured, total, clamped, loadCurrent);
            return ControlOutput.voltage(clamped);
        }

        @Override
        public void applied(ControlLoopContext context, Output requested, Output applied) {
            if (!(applied instanceof Output.Voltage appliedVoltage)) {
                return;
            }
            double appliedVolts = appliedVoltage.volts();
            // The loop's own clamp to the bus voltage and any clamp after it (saturation, effort, power caps) all
            // show up as a difference from the unclamped voltage this loop computed.
            if (Double.compare(appliedVolts, lastUnclampedVolts) == 0 || i == 0.0
                    || !Double.isFinite(appliedVolts) || !Double.isFinite(lastResistance)
                    || !Double.isFinite(lastEmfVolts)) {
                return;
            }
            // Anti-windup: re-seed the integral so the total current matches what the applied voltage can drive.
            double appliedTotal = (appliedVolts - lastEmfVolts) * count / lastResistance;
            integral = appliedTotal - lastNonIntegral;
        }

        @Override
        public ControlLoopTrace trace() {
            return ControlLoopTrace.ZERO;
        }

        /**
         * Disturbance observer on the motor's measured current. The current the measured acceleration does not
         * explain (after the static friction term) is the load, low-pass filtered with the configured time constant.
         */
        private void updateLoadEstimate(double measured, double acceleration, double step) {
            double gain = step / (loadTimeConstant + step);
            filteredAcceleration += gain * (acceleration - filteredAcceleration);
            double measuredCurrent = measuredTotalCurrent();
            if (!Double.isFinite(measuredCurrent)) {
                return;
            }
            double friction = staticAmps * Math.signum(measured);
            double raw = measuredCurrent - inertia * filteredAcceleration - friction;
            loadCurrent += gain * (raw - loadCurrent);
        }

        /** Total stator current over the motors (per-motor reading times the motor count), or NaN. */
        private double measuredTotalCurrent() {
            double sum = 0.0;
            int readings = 0;
            for (MotorDevice device : devices) {
                MotorHandle handle = actions.motor(device);
                if (handle == null) {
                    continue;
                }
                try {
                    double amps = handle.statorCurrentAmps();
                    if (Double.isFinite(amps)) {
                        sum += amps;
                        readings++;
                    }
                } catch (RuntimeException unavailable) {
                    // Current not reported by this controller; the other motors still count.
                }
            }
            return readings == 0 ? Double.NaN : sum / readings * count;
        }

        /** Measured motor-shaft speed in rad/s, or NaN when the controller does not report it. */
        private double motorSpeed(int index) {
            MotorHandle handle = actions.motor(devices[index]);
            if (handle == null) {
                return Double.NaN;
            }
            try {
                return handle.integratedVelocityRotationsPerSecond() * TWO_PI;
            } catch (RuntimeException unavailable) {
                return Double.NaN;
            }
        }

        private void publish(double target, double measured, double total, double volts, double load) {
            targetValue = target;
            measuredValue = measured;
            currentCommand = total;
            voltageCommand = volts;
            loadEstimate = load;
        }
    }
}
