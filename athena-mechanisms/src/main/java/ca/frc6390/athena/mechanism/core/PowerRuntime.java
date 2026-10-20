package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.BatteryResistanceEstimator;
import ca.frc6390.athena.hardware.power.PowerBudget;
import ca.frc6390.athena.hardware.power.PowerPlanner;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.hardware.power.ThermalModel;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
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
 *   <li>Publishes each motor's cap through {@link #capAmps(MotorDevice)}. The output path reads it and
 *       folds it into the effort voltage clamp ({@code EffortRuntime}), so the cap applies to every motor
 *       command. Nothing is written to the controllers, so the cycle never blocks on CAN configuration.</li>
 *   <li>Advances each motor's lumped thermal model (I^2 R heating, first-order cooling, vendor fusion).</li>
 * </ol>
 */
final class PowerRuntime {
    private static final double MAX_DT_SECONDS = 0.5;
    /** Smallest active cap. A cap of zero would be read as "no limit", so an active cap stays at or above 1 A. */
    static final double MIN_ACTIVE_CAP_AMPS = 1.0;
    private static final double REGEN_EPSILON_VOLTS = 1e-6;
    /** Store key of the rolling battery internal-resistance estimate. */
    static final String BATTERY_OHMS_KEY = "power/battery/internalResistanceOhms";

    private PowerPolicy policy = PowerPolicy.defaults();
    private PowerPlanner planner = new PowerPlanner(policy);
    private BatteryResistanceEstimator estimator = newEstimator(policy);
    private PersistentStore batteryStore;
    private DoubleSupplier busVolts = () -> Double.NaN;
    private final Map<MotorDevice, MotorState> states = new IdentityHashMap<>();
    private final List<MotorState> active = new ArrayList<>();
    private final List<PowerPlanner.Channel> channels = new ArrayList<>();
    private double lastNowSeconds = Double.NaN;

    private double measuredVolts = Double.NaN;
    private double openCircuitVolts = Double.NaN;
    private double batteryOhms = PowerPolicy.DEFAULT_BATTERY_RESISTANCE.in(edu.wpi.first.units.Units.Ohms);
    private double predictedVolts = Double.NaN;
    private double budgetAmps = Double.POSITIVE_INFINITY;
    private double demandAmps;
    private boolean shedding;
    /**
     * Snapshot of the latest cycle, built on first request and reused until the next {@link #cycle} or policy change.
     * Telemetry, ARCP and the power publisher each read it every robot loop, so it is built at most once per cycle.
     */
    private PowerSnapshot snapshot;

    PowerPolicy policy() { return policy; }

    void policy(PowerPolicy value) {
        Objects.requireNonNull(value, "policy");
        if (value == policy) return;
        policy = value;
        snapshot = null;
        planner = new PowerPlanner(value);
        estimator = newEstimator(value);
        restoreBatteryPrior();
    }

    /**
     * Keeps a rolling battery internal-resistance estimate in the store. The stored value is restored at boot as the
     * prior, so the first budget after a restart uses the last measured resistance instead of the default. The value
     * is written whenever the online estimate has observed excitation; the store flushes it in batches.
     *
     * @param store store, or null to disable
     */
    void batteryStore(PersistentStore store) {
        batteryStore = store;
        restoreBatteryPrior();
    }

    private void restoreBatteryPrior() {
        if (batteryStore == null) return;
        batteryStore.getDouble(BATTERY_OHMS_KEY).ifPresent(estimator::restorePrior);
    }

    void busVoltage(DoubleSupplier supplier) {
        busVolts = Objects.requireNonNull(supplier, "supplier");
    }

    /**
     * Returns the latest electrical state for telemetry. The snapshot is immutable and memoized: repeated calls
     * between two {@link #cycle} calls return the same instance.
     */
    PowerSnapshot snapshot() {
        PowerSnapshot cached = snapshot;
        if (cached != null) return cached;
        PowerSnapshot.Motor[] motors = new PowerSnapshot.Motor[active.size()];
        for (int index = 0; index < motors.length; index++) {
            MotorState state = active.get(index);
            motors[index] = new PowerSnapshot.Motor(
                    state.name,
                    state.demandAmps,
                    state.channel.capAmps(),
                    state.derate,
                    state.temperatureCelsius,
                    state.channel.shareWeight(),
                    state.profileSustainedAmps(),
                    state.status);
        }
        cached = new PowerSnapshot(measuredVolts, openCircuitVolts, batteryOhms, policy.raw().floorVolts(),
                predictedVolts, budgetAmps, demandAmps, shedding, List.of(motors));
        snapshot = cached;
        return cached;
    }

    /**
     * Returns the thermal derate of a motor from the latest cycle as a whole percent of its limit, or -1 when
     * the motor is not derated (or was not part of the cycle).
     */
    int thermalDeratePercent(MotorDevice motor) {
        MotorState state = activeState(motor);
        if (state == null || !(state.derate < 1.0)) return -1;
        return (int) Math.round(Math.max(0.0, state.derate) * 100.0);
    }

    /**
     * Returns the budget or profile cap on a motor from the latest cycle, in whole amps, or -1 when the motor
     * is not power limited.
     */
    int powerCapAmps(MotorDevice motor) {
        MotorState state = activeState(motor);
        if (state == null || !(state.status.equals("Power capped") || state.status.equals("Power shed"))) return -1;
        return toAmps(state.channel.capAmps());
    }

    /**
     * Returns the effective current cap of a motor from the latest cycle, in amps: the planner cap, already
     * combined with the declared limit and thermal derate. Positive infinity when the motor is uncapped or was
     * not part of the cycle. The output path clamps the motor's voltage to this value.
     */
    double capAmps(MotorDevice motor) {
        MotorState state = activeState(motor);
        if (state == null || !Double.isFinite(state.channel.capAmps())) return Double.POSITIVE_INFINITY;
        // Only a cap below the declared limit reaches the clamp. At or above it the controller's own limit
        // already applies, so the output is left unchanged (model error would otherwise under-drive a motor).
        if (state.channel.capAmps() >= state.configuredLimitAmps) return Double.POSITIVE_INFINITY;
        return Math.max(0.0, state.channel.capAmps());
    }

    private MotorState activeState(MotorDevice motor) {
        for (int index = 0; index < active.size(); index++) {
            MotorState state = active.get(index);
            if (state.device == motor) return state;
        }
        return null;
    }

    void cycle(Collection<MotorDevice> motors, ActionContext context, RuntimeOverrides overrides, double nowSeconds) {
        double dt = Double.isNaN(lastNowSeconds) ? 0.0 : Math.max(0.0, Math.min(MAX_DT_SECONDS, nowSeconds - lastNowSeconds));
        lastNowSeconds = nowSeconds;
        snapshot = null;
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
            state.derate = state.curve.raw().derateFractionCelsius(motor.raw().fusedTemperatureCelsius());
            state.temperatureCelsius = motor.raw().fusedTemperatureCelsius();
            // The declared limits (applied once at configuration) bound the cap too, scaled by the derate.
            // The tighter of supply and stator wins, because the cap is applied to the motor's voltage.
            double declaredLimit = Math.min(
                    config.supplyCurrentLimitAmps() > 0 ? config.supplyCurrentLimitAmps() : Double.POSITIVE_INFINITY,
                    config.statorCurrentLimitAmps() > 0 ? config.statorCurrentLimitAmps() : Double.POSITIVE_INFINITY);
            // A derated cap is never below 1 A: zero means "no limit" to the planner.
            double declaredCap = Double.isFinite(declaredLimit)
                    ? Math.max(MIN_ACTIVE_CAP_AMPS, declaredLimit * state.derate)
                    : Double.POSITIVE_INFINITY;
            state.declaredCapAmps = declaredCap;
            state.configuredLimitAmps = declaredLimit;
            state.channel.inputs(motor.power(), state.demandAmps, declaredCap);
            active.add(state);
            channels.add(state.channel);
        }

        if (policy.estimateBatteryResistance()) {
            estimator.sample(bus, totalSupply);
            if (batteryStore != null && estimator.excited()) {
                batteryStore.put(BATTERY_OHMS_KEY, estimator.ohms());
            }
        }
        batteryOhms = policy.estimateBatteryResistance() ? estimator.ohms() : policy.raw().defaultBatteryOhms();
        openCircuitVolts = policy.estimateBatteryResistance()
                ? estimator.openCircuitVolts()
                : Double.isFinite(bus) ? bus + batteryOhms * totalSupply : Double.NaN;
        PowerPlanner.Bus planBus = new PowerPlanner.Bus(bus, openCircuitVolts, batteryOhms);
        planner.plan(planBus, nowSeconds, dt, channels);
        shedding = planner.shedding();
        predictedVolts = planBus.predictedVolts(totalDemand());
        budgetAmps = Double.isFinite(openCircuitVolts)
                ? PowerBudget.budgetAmps(openCircuitVolts, batteryOhms, policy.raw().floorVolts())
                : Double.POSITIVE_INFINITY;
        demandAmps = totalDemand();

        for (MotorState state : active) {
            state.thermalStep(dt);
            state.status = status(state, state.channel.capAmps());
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
        return new BatteryResistanceEstimator(20.0, 0.02, policy.raw().defaultBatteryOhms());
    }

    private static final class MotorState {
        final MotorDevice device;
        /** {@link MotorDevice#defaultName()}, computed once: the device is immutable and the name is a string build. */
        final String name;
        final PowerPlanner.Channel channel = new PowerPlanner.Channel();
        /** Nominal constants, or empty for a motor kind without a MotorSpec row (demand then uses supply current). */
        final Optional<MotorConstants> constants;
        /** Winding resistance of {@link #constants} in ohms, converted once; NaN without constants. */
        final double resistanceOhms;
        /** Speed constant of {@link #constants} in rad/s per volt, converted once; NaN without constants. */
        final double speedConstantRadPerSecPerVolt;
        final ca.frc6390.athena.hardware.power.Thermal curve;
        MotorHandle handle;
        double demandAmps;
        double supplyAmps = Double.NaN;
        double statorAmps = Double.NaN;
        double derate = 1.0;
        double declaredCapAmps = Double.POSITIVE_INFINITY;
        /** Declared supply/stator limit before the derate; infinite when none is declared. */
        double configuredLimitAmps = Double.POSITIVE_INFINITY;
        double temperatureCelsius = Double.NaN;
        String status = "Declared";

        double profileSustainedAmps() {
            var profile = device.power();
            return profile == null ? 0.0 : profile.raw().sustainedAmps();
        }

        MotorState(MotorDevice device) {
            this.device = device;
            this.name = device.defaultName();
            this.constants = MotorConstants.forKind(device.kind());
            this.resistanceOhms = constants.isPresent()
                    ? constants.get().resistance().in(edu.wpi.first.units.Units.Ohms)
                    : Double.NaN;
            this.speedConstantRadPerSecPerVolt = constants.isPresent()
                    ? constants.get().raw().speedConstantRadPerSecPerVolt()
                    : Double.NaN;
            this.curve = device.thermalCurve();
        }

        void readInputs(MotorHandle handle) {
            supplyAmps = read(handle::supplyCurrentAmps);
            statorAmps = read(handle::statorCurrentAmps);
            double volts = read(handle::appliedVoltage);
            double rotationsPerSecond = read(handle::integratedVelocityRotationsPerSecond);
            if (constants.isPresent() && Double.isFinite(volts) && Math.abs(volts) > REGEN_EPSILON_VOLTS
                    && Double.isFinite(rotationsPerSecond)) {
                double omega = rotationsPerSecond * 2.0 * Math.PI;
                double backEmf = omega / speedConstantRadPerSecPerVolt;
                demandAmps = Math.abs(volts - backEmf) / resistanceOhms;
            } else if (constants.isPresent() && Double.isFinite(volts)) {
                demandAmps = 0.0;
            } else {
                // Unknown constants: the measured supply current is the demand estimate.
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
