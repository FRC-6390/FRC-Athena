package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.BatteryResistanceEstimator;
import ca.frc6390.athena.hardware.power.MotorThermalSpec;
import ca.frc6390.athena.hardware.power.PowerBudget;
import ca.frc6390.athena.hardware.power.PowerPlanner;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.hardware.power.ThermalModel;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * Runs the electrical and thermal pass once per scheduler cycle, after controllers have produced their
 * outputs and before motor commands are applied.
 *
 * <p>Each cycle:</p>
 * <ol>
 *   <li>Reads each motor's measured stator/supply current, applied voltage and speed. Demand is
 *       {@code |V - omega/Kv| / R} when those are known, otherwise the measured supply current.</li>
 *   <li>Updates the battery resistance estimate from bus voltage and total supply current.</li>
 *   <li>Plans per-motor caps ({@link PowerPlanner}). The thermal derate fraction scales the declared
 *       limit before planning.</li>
 *   <li>Writes the effective current limits through {@link MotorHandle#applyRuntimeConfiguration}. Writes
 *       are rate limited: at least {@link #MIN_WRITE_INTERVAL_SECONDS} apart, and only when the change is
 *       at least {@link #WRITE_DEADBAND_AMPS}. A user configuration change is written at once.</li>
 *   <li>Advances each motor's lumped thermal model (I^2 R heating, first-order cooling, vendor fusion).</li>
 * </ol>
 */
final class PowerRuntime {
    /** Minimum time between two current-limit configuration writes for one motor. */
    static final double MIN_WRITE_INTERVAL_SECONDS = 0.25;
    /** Minimum change in amps that justifies a configuration write when the base config is unchanged. */
    static final double WRITE_DEADBAND_AMPS = 2.0;
    private static final double MAX_DT_SECONDS = 0.5;
    private static final double REGEN_EPSILON_VOLTS = 1e-6;

    private PowerPolicy policy = PowerPolicy.defaults();
    private PowerPlanner planner = new PowerPlanner(policy);
    private BatteryResistanceEstimator estimator = newEstimator(policy);
    private DoubleSupplier busVolts = () -> Double.NaN;
    private final Map<MotorDevice, MotorState> states = new IdentityHashMap<>();
    private final List<MotorState> active = new ArrayList<>();
    private final List<PowerPlanner.Channel> channels = new ArrayList<>();
    private double lastNowSeconds = Double.NaN;

    private double measuredVolts = Double.NaN;
    private double openCircuitVolts = Double.NaN;
    private double batteryOhms = PowerPolicy.DEFAULT_BATTERY_OHMS;
    private double predictedVolts = Double.NaN;
    private double budgetAmps = Double.POSITIVE_INFINITY;
    private double demandAmps;
    private boolean shedding;

    PowerPolicy policy() { return policy; }

    void policy(PowerPolicy value) {
        Objects.requireNonNull(value, "policy");
        if (value == policy) return;
        policy = value;
        planner = new PowerPlanner(value);
        estimator = newEstimator(value);
    }

    void busVoltage(DoubleSupplier supplier) {
        busVolts = Objects.requireNonNull(supplier, "supplier");
    }

    /** Returns the latest electrical state for telemetry. */
    PowerSnapshot snapshot() {
        List<PowerSnapshot.Motor> motors = new ArrayList<>(active.size());
        for (MotorState state : active) {
            motors.add(new PowerSnapshot.Motor(
                    state.device.defaultName(),
                    state.demandAmps,
                    state.channel.capAmps(),
                    state.derate,
                    state.temperatureCelsius,
                    state.channel.shareWeight(),
                    state.profileSustainedAmps(),
                    state.status));
        }
        return new PowerSnapshot(measuredVolts, openCircuitVolts, batteryOhms, policy.floorVolts(),
                predictedVolts, budgetAmps, demandAmps, shedding, motors);
    }

    void cycle(Collection<MotorDevice> motors, ActionContext context, RuntimeOverrides overrides, double nowSeconds) {
        double dt = Double.isNaN(lastNowSeconds) ? 0.0 : Math.max(0.0, Math.min(MAX_DT_SECONDS, nowSeconds - lastNowSeconds));
        lastNowSeconds = nowSeconds;
        double bus = readBus();
        measuredVolts = bus;

        active.clear();
        channels.clear();
        double totalSupply = 0.0;
        for (MotorDevice motor : motors) {
            if (motor.isDisabled()) continue;
            MotorState state = states.computeIfAbsent(motor, MotorState::new);
            MotorHandle handle;
            try {
                handle = context.motor(motor);
            } catch (RuntimeException exception) {
                state.status = "Unavailable";
                continue;
            }
            state.handle = handle;
            state.readInputs(handle);
            if (Double.isFinite(state.supplyAmps)) totalSupply += state.supplyAmps;
            MotorRuntimeConfig config = overrides.config(motor);
            state.config = config;
            state.derate = state.curve.derateFraction(motor.temperatureCelsius());
            state.temperatureCelsius = motor.temperatureCelsius();
            double declaredCap = config.supplyCurrentLimitAmps() > 0
                    ? config.supplyCurrentLimitAmps() * state.derate
                    : Double.POSITIVE_INFINITY;
            state.declaredCapAmps = declaredCap;
            state.channel.inputs(motor.power(), state.demandAmps, declaredCap);
            active.add(state);
            channels.add(state.channel);
        }

        if (policy.estimateBatteryResistance()) {
            estimator.sample(bus, totalSupply);
        }
        batteryOhms = policy.estimateBatteryResistance() ? estimator.ohms() : policy.defaultBatteryOhms();
        openCircuitVolts = policy.estimateBatteryResistance()
                ? estimator.openCircuitVolts()
                : Double.isFinite(bus) ? bus + batteryOhms * totalSupply : Double.NaN;
        PowerPlanner.Bus planBus = new PowerPlanner.Bus(bus, openCircuitVolts, batteryOhms);
        planner.plan(planBus, nowSeconds, dt, channels);
        shedding = planner.shedding();
        predictedVolts = planBus.predictedVolts(totalDemand());
        budgetAmps = Double.isFinite(openCircuitVolts)
                ? PowerBudget.budgetAmps(openCircuitVolts, batteryOhms, policy.floorVolts())
                : Double.POSITIVE_INFINITY;
        demandAmps = totalDemand();

        for (MotorState state : active) {
            state.thermalStep(dt);
            writeLimits(state, nowSeconds);
        }
    }

    private double totalDemand() {
        double total = 0.0;
        for (MotorState state : active) total += state.demandAmps;
        return total;
    }

    private double readBus() {
        try {
            double volts = busVolts.getAsDouble();
            return Double.isFinite(volts) && volts > 0.0 ? volts : Double.NaN;
        } catch (RuntimeException exception) {
            return Double.NaN;
        }
    }

    private void writeLimits(MotorState state, double nowSeconds) {
        MotorHandle handle = state.handle;
        MotorRuntimeConfig config = state.config;
        double cap = state.channel.capAmps();
        double supplyEffective = config.supplyCurrentLimitAmps() > 0
                ? config.supplyCurrentLimitAmps() * state.derate
                : 0.0;
        if (Double.isFinite(cap)) {
            // Zero means "no limit" to the controllers, so an active cap is never written below 1 A.
            supplyEffective = Math.max(1.0, supplyEffective > 0.0 ? Math.min(supplyEffective, cap) : cap);
        }
        int supply = toAmps(supplyEffective);
        int stator = config.statorCurrentLimitAmps() > 0 ? toAmps(config.statorCurrentLimitAmps() * state.derate) : 0;
        MotorRuntimeConfig desired = new MotorRuntimeConfig(config.neutralMode(), config.inverted(), supply, stator);
        state.status = status(state, cap);
        if (!handle.supportsRuntimeConfiguration()) {
            state.status = cap < Double.POSITIVE_INFINITY || state.derate < 1.0 ? "Unsupported" : "Declared";
            return;
        }
        boolean baseChanged = !config.equals(state.lastBase);
        if (state.lastBase == null) {
            // First cycle: the declared configuration is already applied by the runtime overrides.
            state.lastBase = config;
            state.lastWritten = config;
        } else if (baseChanged) {
            state.lastBase = config;
            state.lastWritten = null;
        }
        if (desired.equals(state.lastWritten)) return;
        if (!baseChanged && state.lastWritten != null) {
            boolean due = nowSeconds - state.lastWriteSeconds >= MIN_WRITE_INTERVAL_SECONDS;
            boolean significant = Math.abs(desired.supplyCurrentLimitAmps() - state.lastWritten.supplyCurrentLimitAmps()) >= WRITE_DEADBAND_AMPS
                    || Math.abs(desired.statorCurrentLimitAmps() - state.lastWritten.statorCurrentLimitAmps()) >= WRITE_DEADBAND_AMPS
                    || (desired.supplyCurrentLimitAmps() == 0) != (state.lastWritten.supplyCurrentLimitAmps() == 0);
            if (!due || !significant) return;
        }
        try {
            handle.applyRuntimeConfiguration(desired);
            state.lastWritten = desired;
            state.lastWriteSeconds = nowSeconds;
        } catch (RuntimeException exception) {
            state.status = "Error: " + exception.getMessage();
        }
    }

    private String status(MotorState state, double cap) {
        // Power-limited only when the planner lowered the cap below the configured (already derated) limit.
        if (cap < state.declaredCapAmps) return shedding ? "Power shed" : "Power capped";
        if (state.derate < 1.0) return "Thermal derate";
        return "Declared";
    }

    private static int toAmps(double amps) {
        if (!(amps > 0.0) || !Double.isFinite(amps)) return 0;
        return (int) Math.max(1L, Math.min(Integer.MAX_VALUE, Math.round(amps)));
    }

    private static BatteryResistanceEstimator newEstimator(PowerPolicy policy) {
        return new BatteryResistanceEstimator(20.0, 0.02, policy.defaultBatteryOhms());
    }

    private static final class MotorState {
        final MotorDevice device;
        final PowerPlanner.Channel channel = new PowerPlanner.Channel();
        final MotorThermalSpec spec;
        final ca.frc6390.athena.hardware.power.Thermal curve;
        MotorHandle handle;
        MotorRuntimeConfig config;
        MotorRuntimeConfig lastBase;
        MotorRuntimeConfig lastWritten;
        double lastWriteSeconds = Double.NEGATIVE_INFINITY;
        double demandAmps;
        double supplyAmps = Double.NaN;
        double statorAmps = Double.NaN;
        double derate = 1.0;
        double declaredCapAmps = Double.POSITIVE_INFINITY;
        double temperatureCelsius = Double.NaN;
        String status = "Declared";

        double profileSustainedAmps() {
            var profile = device.power();
            return profile == null ? 0.0 : profile.sustainedAmps();
        }

        MotorState(MotorDevice device) {
            this.device = device;
            this.spec = MotorThermalSpec.forKind(device.kind());
            this.curve = device.thermalCurve();
        }

        void readInputs(MotorHandle handle) {
            supplyAmps = read(handle::supplyCurrentAmps);
            statorAmps = read(handle::statorCurrentAmps);
            double volts = read(handle::appliedVoltage);
            double rotationsPerSecond = read(handle::integratedVelocityRotationsPerSecond);
            if (Double.isFinite(volts) && Math.abs(volts) > REGEN_EPSILON_VOLTS && Double.isFinite(rotationsPerSecond)) {
                double omega = rotationsPerSecond * 2.0 * Math.PI;
                double backEmf = omega / spec.kvRadPerSecondPerVolt();
                demandAmps = Math.abs(volts - backEmf) / spec.resistanceOhms();
            } else if (Double.isFinite(volts)) {
                demandAmps = 0.0;
            } else {
                demandAmps = Double.isFinite(supplyAmps) ? Math.max(0.0, supplyAmps) : 0.0;
            }
        }

        void thermalStep(double dt) {
            ThermalModel model = device.thermalModel();
            if (model == null) return;
            if (handle != null) {
                OptionalDouble vendor = handle.temperatureCelsius();
                model.vendorReading(vendor == null || vendor.isEmpty() ? Double.NaN : vendor.getAsDouble());
            }
            model.step(Double.isFinite(statorAmps) ? statorAmps : 0.0, dt);
            temperatureCelsius = model.celsius();
        }

        private static double read(java.util.function.DoubleSupplier reader) {
            try {
                return reader.getAsDouble();
            } catch (RuntimeException exception) {
                return Double.NaN;
            }
        }
    }
}
