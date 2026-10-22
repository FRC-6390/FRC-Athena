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
import java.util.function.DoubleUnaryOperator;

/**
 * Samples measured current for battery and thermal estimates, then allocates the prepared outputs
 * before any motor command is committed. Exact shared-profile identity joins independently commanded
 * roots; native/software followers retain their physical root-voltage constraint. Allocation uses
 * positive battery supply amps, while profiles and thermal limits use winding amps. No per-cycle
 * controller configuration or blocking hardware operation is issued by this pass.
 */
final class PowerRuntime {
    private static final double MAX_DT_SECONDS = 0.5;
    private static final double ELECTRICAL_EPSILON = 1e-9;
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
    private RuntimeOverrides cycleOverrides;
    private double cycleDt;
    private long cycleId;
    private boolean commandsPlanned;
    private final Map<Object, CommandPlan> commandPlans = new IdentityHashMap<>();
    private final List<CommandPlan> commands = new ArrayList<>();
    private final Map<ca.frc6390.athena.hardware.power.PowerProfile, CommandPlan> profileCommands = new IdentityHashMap<>();
    private final Map<MotorHandle, CommandPlan> physicalCommands = new IdentityHashMap<>();
    private final Map<MotorHandle, Member> physicalLoads = new IdentityHashMap<>();
    private final java.util.Set<MotorHandle> measuredTargets = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final java.util.Set<MotorHandle> committedTargets = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final java.util.Set<ThermalModel> steppedThermals = java.util.Collections.newSetFromMap(new IdentityHashMap<>());

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
                    state.outputSuppressed ? 0.0 : state.channel.capAmps(),
                    state.derate,
                    state.temperatureCelsius,
                    state.channel.shareWeight(),
                    state.profileSustainedAmps(),
                    state.status,
                    state.supplyDemandAmps);
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
     * Returns the effective winding-current cap. For motors without electrical constants the output path
     * instead uses a supply-current feedback estimate, so their winding cap is converted back to supply amps.
     * Positive infinity means uncapped or not part of the cycle. Zero remains an active limit.
     */
    double capAmps(MotorDevice motor) {
        MotorState state = activeState(motor);
        if (state != null && state.outputSuppressed) return 0.0;
        if (state == null || !Double.isFinite(state.channel.capAmps())) return Double.POSITIVE_INFINITY;
        // Only the controller's winding limit is interchangeable with this winding-current clamp.
        // A supply limit is not an equal winding limit at partial PWM duty.
        if (state.channel.capAmps() >= state.configuredStatorLimitAmps && !state.profileHasCurrentCap()
                && !state.channel.supplyBudgetActive() && state.derate >= 1.0 && !state.softwareSupplyLimit) {
            return Double.POSITIVE_INFINITY;
        }
        double cap = Math.max(0.0, state.channel.capAmps());
        if (state.constants.isEmpty() && state.demandAmps > ELECTRICAL_EPSILON) {
            return cap * state.supplyDemandAmps / state.demandAmps;
        }
        return cap;
    }

    private MotorState activeState(MotorDevice motor) {
        MotorState state = states.get(motor);
        return state != null && state.sampledCycle == cycleId ? state : null;
    }

    void beginCycle(Collection<MotorDevice> motors, ActionContext context, RuntimeOverrides overrides, double nowSeconds) {
        double dt = Double.isNaN(lastNowSeconds) ? 0.0 : Math.max(0.0, Math.min(MAX_DT_SECONDS, nowSeconds - lastNowSeconds));
        lastNowSeconds = nowSeconds;
        cycleDt = dt;
        cycleId++;
        commandsPlanned = false;
        cycleOverrides = overrides;
        commands.clear();
        profileCommands.clear();
        physicalCommands.clear();
        physicalLoads.clear();
        measuredTargets.clear();
        committedTargets.clear();
        steppedThermals.clear();
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
            state.sampledCycle = cycleId;
            state.readInputs(handle, bus);
            if (measuredTargets.add(handle.commandTarget()) && Double.isFinite(state.supplyAmps)) {
                totalSupply += state.supplyAmps;
            }
            MotorRuntimeConfig config = overrides.config(motor);
            state.declaredSupplyLimit = config.supplyCurrentLimitAmps();
            state.planStatus = "";
            state.outputSuppressed = false;
            state.temperatureCelsius = motor.raw().fusedTemperatureCelsius();
            state.derate = state.curve.raw().derateFractionCelsius(state.temperatureCelsius);
            double statorLimit = config.statorCurrentLimitAmps() > 0
                    ? config.statorCurrentLimitAmps() : Double.POSITIVE_INFINITY;
            double declaredCap = statorLimit;
            double supplyAsWinding = config.supplyCurrentLimitAmps() > 0
                    ? state.windingForSupply(config.supplyCurrentLimitAmps()) : Double.POSITIVE_INFINITY;
            state.softwareSupplyLimit = !handle.enforcesSupplyCurrentLimit() && config.supplyCurrentLimitAmps() > 0;
            if (state.softwareSupplyLimit) declaredCap = Math.min(declaredCap, supplyAsWinding);
            if (state.derate < 1.0) {
                // Thermal limits act on winding current, never directly on battery amps. If the only
                // configured limit is supply-side, convert its allowed electrical power at this speed first.
                double reference = Math.min(statorLimit, supplyAsWinding);
                if (!Double.isFinite(reference) && state.constants.isPresent()) {
                    reference = state.constants.get().stallCurrent().in(Units.Amps);
                }
                if (Double.isFinite(reference)) declaredCap = reference * state.derate;
            }
            state.declaredCapAmps = declaredCap;
            // Candidate power is constrained exactly in supply amps below; do not reuse the
            // direction-dependent measured supply-to-winding estimate as a candidate torque cap.
            state.candidateCapAmps = statorLimit;
            if (state.derate < 1.0) {
                // Preserve the thermal winding reference even for a supply-only declaration.
                state.candidateCapAmps = state.derate <= 0.0 ? 0.0 : declaredCap;
            }
            state.configuredStatorLimitAmps = statorLimit;
            state.channel.inputs(motor.power(), state.demandAmps, declaredCap, state.supplyDemandAmps,
                    state.supplyToWinding, state.derate);
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
        for (MotorState state : active) {
            ThermalModel model = state.device.thermalModel();
            if (model != null && steppedThermals.add(model)) state.thermalStep(model, dt);
        }
    }

    /** Compatibility path for direct measured-state tools/tests. Scheduler uses begin/prepare/plan/commit. */
    void cycle(Collection<MotorDevice> motors, ActionContext context, RuntimeOverrides overrides, double nowSeconds) {
        beginCycle(motors, context, overrides, nowSeconds);
        planner.plan(planBus(), nowSeconds, cycleDt, channels);
        updatePlanSnapshot();
    }

    private PowerPlanner.Bus planBus() {
        return new PowerPlanner.Bus(measuredVolts, openCircuitVolts, batteryOhms);
    }

    private void updatePlanSnapshot() {
        shedding = planner.shedding();
        demandAmps = totalDemand();
        predictedVolts = planBus().predictedVolts(demandAmps);
        budgetAmps = Double.isFinite(openCircuitVolts)
                ? PowerBudget.budgetAmps(openCircuitVolts, batteryOhms, policy.raw().floorVolts())
                : Double.POSITIVE_INFINITY;
        snapshot = null;
        for (MotorState state : active) state.status = status(state, state.channel.capAmps());
    }

    /** Native closed loops cannot forecast/enforce this cycle's shared battery allocation. */
    boolean requiresHostClamp(MotorDevice motor) {
        MotorState state = states.get(motor);
        return motor.power() != null || Double.isFinite(measuredVolts)
                || state != null && (state.softwareSupplyLimit || state.derate < 1.0
                || Double.isFinite(state.declaredCapAmps));
    }

    /** Globally commands a physical destination once, even when failover declarations alias it. */
    boolean claimCommandTarget(MotorHandle handle) { return committedTargets.add(handle.commandTarget()); }

    void registerCommand(Object key, List<MotorDevice> motors, MotorHandle[] handles,
            double candidateVolts, double[] localAmps) {
        Objects.requireNonNull(key, "command key");
        CommandPlan command = commandPlans.computeIfAbsent(key, ignored -> new CommandPlan());
        if (command.cycle == cycleId) throw new IllegalStateException("Output prepared twice in one power cycle");
        command.reset(cycleId, candidateVolts);
        commands.add(command);
        for (int index = 0; index < motors.size(); index++) {
            MotorDevice motor = motors.get(index);
            MotorHandle handle = handles[index];
            MotorState state = states.get(motor);
            if (state == null) {
                state = new MotorState(motor);
                states.put(motor, state);
            }
            if (state.sampledCycle != cycleId && !motor.isDisabled()) {
                state.sampledCycle = cycleId;
                state.handle = handle;
                state.readInputs(handle, measuredVolts);
                state.declaredCapAmps = configuredCap(motor, state);
                state.candidateCapAmps = state.declaredCapAmps;
                state.outputSuppressed = false;
                active.add(state);
            }
            Member member = command.member(index);
            member.state = state;
            member.handle = handle;
            member.command = command;
            member.localCap = index < localAmps.length && !Double.isNaN(localAmps[index])
                    ? Math.max(0.0, localAmps[index]) : Double.POSITIVE_INFINITY;
            member.scale = handle.voltageCommandScale();
            member.eligible = !motor.isDisabled() && handle.configurationStatus().ready()
                    && Double.isFinite(candidateVolts) && Double.isFinite(member.scale) && member.scale != 0.0;
            member.accounted = false;
            member.known = state.constants.isPresent() && Double.isFinite(state.backEmfVolts);
            member.voltageAdded = member.supplyAdded = false;
            if (motor.isDisabled()) continue;
            // Even neutral/native outputs participate in collision detection. Aliased declarations
            // must never make the winning physical command depend on mechanism traversal order.
            MotorHandle physical = handle.commandTarget();
            CommandPlan previous = physicalCommands.putIfAbsent(physical, command);
            if (previous != null && previous != command) {
                previous.suppressed = command.suppressed = true;
                previous.failure = command.failure = "Conflicting physical command targets";
            }
            if (!Double.isFinite(candidateVolts)) continue;
            if (!member.eligible) {
                command.suppressed = true;
                command.failure = "Physical command unavailable";
                continue;
            }
            if (physicalLoads.putIfAbsent(physical, member) == null) member.accounted = true;
            if (!command.anchorSet && member.known) {
                command.anchor = state.backEmfVolts / member.scale;
                command.anchorSet = true;
            }
        }
        command.memberCount = motors.size();
        command.slope = candidateVolts - command.anchor;
    }

    private double configuredCap(MotorDevice motor, MotorState state) {
        MotorRuntimeConfig config = cycleOverrides.config(motor);
        state.configuredStatorLimitAmps = config.statorCurrentLimitAmps() > 0
                ? config.statorCurrentLimitAmps() : Double.POSITIVE_INFINITY;
        state.declaredSupplyLimit = config.supplyCurrentLimitAmps();
        state.softwareSupplyLimit = !state.handle.enforcesSupplyCurrentLimit() && state.declaredSupplyLimit > 0.0;
        state.derate = motor.thermalCurve().raw().derateFractionCelsius(motor.raw().fusedTemperatureCelsius());
        double reference = Math.min(state.configuredStatorLimitAmps,
                state.declaredSupplyLimit > 0.0 ? state.windingForSupply(state.declaredSupplyLimit)
                        : Double.POSITIVE_INFINITY);
        if (!Double.isFinite(reference) && state.constants.isPresent())
            reference = state.constants.get().stallCurrent().in(Units.Amps);
        return state.derate <= 0.0 ? 0.0 : state.derate < 1.0 ? reference * state.derate
                : state.configuredStatorLimitAmps;
    }

    /** Allocates requested outputs after every controller (including release holds) was evaluated once. */
    void planCommands() {
        channels.clear();
        for (MotorState state : active) {
            state.demandAmps = 0.0;
            state.supplyDemandAmps = 0.0;
            state.planStatus = "";
            state.channel.couplingGroup(null);
            state.channel.accountedLoad(true);
            state.plannedMember = null;
            state.channel.inputs(state.device.power(), 0.0, state.candidateCapAmps, 0.0,
                    state.supplyToWinding, state.derate);
        }
        for (CommandPlan command : commands) {
            for (int index = 0; index < command.memberCount; index++) {
                Member member = command.members.get(index);
                if (!member.eligible) continue;
                var profile = member.state.device.power();
                if (profile != null) {
                    CommandPlan previous = profileCommands.putIfAbsent(profile, command);
                    if (previous != null) union(command, previous);
                }
            }
        }
        for (CommandPlan command : commands) {
            CommandPlan group = command.root();
            if (command.suppressed) group.groupSuppressed = true;
        }
        for (CommandPlan command : commands) {
            CommandPlan group = command.root();
            for (int index = 0; index < command.memberCount; index++) {
                Member member = command.members.get(index);
                MotorState state = member.state;
                if (!member.eligible || group.groupSuppressed) {
                    state.planStatus = command.failure.isEmpty() ? "Neutral / unavailable" : command.failure;
                    continue;
                }
                state.plannedMember = member;
                double cap = Math.min(state.candidateCapAmps, member.localCap);
                member.prepareModel();
                if (!member.known && state.softwareSupplyLimit && state.declaredSupplyLimit > 0.0
                        && state.constants.isPresent()) {
                    group.groupSuppressed = true;
                    state.planStatus = "Supply limit unavailable";
                }
                if (!member.known && (cap == 0.0 || !(Math.abs(member.currentSlope) > 0.0)
                        && Math.abs(command.candidate) > ELECTRICAL_EPSILON
                        && (requiresHostClamp(state.device) || Double.isFinite(member.localCap)))) {
                    // No current model can turn a zero current allocation into an honest nonzero command.
                    group.groupSuppressed = true;
                    state.planStatus = "Current model unavailable";
                }
                double current = Math.abs(member.currentAtZero + member.currentSlope);
                double supply = member.positiveSupply(1.0);
                state.demandAmps = current;
                state.supplyDemandAmps = member.accounted ? supply : 0.0;
                state.channel.inputs(state.device.power(), current, cap, state.supplyDemandAmps,
                        state.supplyToWinding, state.derate, member.model);
                state.channel.couplingGroup(group);
                state.channel.accountedLoad(member.accounted);
                channels.add(state.channel);
                // The bus limit is a physical feasibility constraint, rather than a post-allocation
                // clip that would change one wheel's torque ratio independently of its profile group.
                if (measuredVolts > 0.0 && Double.isFinite(measuredVolts)) {
                    if (member.needsVoltageConstraint(measuredVolts)) {
                        member.prepareVoltageConstraint();
                        member.voltageChannel.inputs(null, Math.abs(member.scale * command.candidate), measuredVolts,
                                0.0, value -> value, 1.0, member.voltageModel);
                        member.voltageChannel.couplingGroup(group);
                        member.voltageChannel.accountedLoad(false);
                        member.voltageChannel.startupParticipant(false);
                        channels.add(member.voltageChannel);
                        member.voltageAdded = true;
                    }
                } else if (state.softwareSupplyLimit && state.declaredSupplyLimit > 0.0) {
                    group.groupSuppressed = true;
                    state.planStatus = "Supply limit unavailable";
                }
                if (member.known && state.declaredSupplyLimit > 0.0 && measuredVolts > 0.0
                        && member.needsSupplyConstraint(supply)) {
                    member.prepareSupplyConstraint();
                    member.supplyChannel.inputs(null, 0.0, member.supplyRadius, 0.0, value -> value, 1.0,
                            member.supplyModel);
                    member.supplyChannel.couplingGroup(group);
                    member.supplyChannel.accountedLoad(false);
                    member.supplyChannel.startupParticipant(false);
                    channels.add(member.supplyChannel);
                    member.supplyAdded = true;
                }
            }
        }
        for (MotorState state : active) {
            if (state.plannedMember == null) {
                state.channel.couplingGroup(state.channel);
                channels.add(state.channel);
            }
        }
        // A failed group contributes no projected load. Other groups can use the remaining budget.
        channels.removeIf(channel -> channel.couplingGroup() instanceof CommandPlan group
                && group.root().groupSuppressed);
        planner.plan(planBus(), lastNowSeconds, cycleDt, channels);
        for (CommandPlan command : commands) {
            CommandPlan group = command.root();
            for (int index = 0; index < command.memberCount; index++) {
                Member member = command.members.get(index);
                if (!member.eligible || group.groupSuppressed) continue;
                if (!member.state.channel.couplingFeasible()
                        || member.voltageAdded && !member.voltageChannel.couplingFeasible()
                        || member.supplyAdded && !member.supplyChannel.couplingFeasible()) group.groupSuppressed = true;
                group.factor = Math.min(group.factor, member.state.channel.couplingScale());
            }
        }
        for (CommandPlan command : commands) {
            CommandPlan group = command.root();
            command.suppressed |= group.groupSuppressed;
            command.planned = Double.isFinite(command.candidate) && group.factor < 1.0
                    ? command.anchor + group.factor * command.slope : command.candidate;
            for (int index = 0; index < command.memberCount; index++) {
                Member member = command.members.get(index);
                if (command.suppressed) {
                    member.state.outputSuppressed = true;
                    member.state.supplyDemandAmps = 0.0;
                    if (member.state.planStatus.isEmpty()) member.state.planStatus = command.failure.isEmpty()
                            ? "Infeasible power group" : command.failure;
                } else if (member.eligible && !member.known) member.state.planStatus = "Coupling estimated";
            }
        }
        commandsPlanned = true;
        updatePlanSnapshot();
    }

    /** A final guard applies before any write and cancels its entire shared command group. */
    void suppressCommandGroup(Object key, String reason) {
        CommandPlan command = commandPlans.get(key);
        if (command == null || command.cycle != cycleId) return;
        CommandPlan group = command.root();
        group.groupSuppressed = true;
        for (CommandPlan peer : commands) {
            if (peer.root() != group) continue;
            peer.suppressed = true;
            peer.failure = reason;
            for (int index = 0; index < peer.memberCount; index++) {
                MotorState state = peer.members.get(index).state;
                state.outputSuppressed = true;
                state.supplyDemandAmps = 0.0;
                state.planStatus = reason;
                state.status = reason;
            }
        }
        demandAmps = totalDemand();
        predictedVolts = planBus().predictedVolts(demandAmps);
        snapshot = null;
    }

    /** Removes a failed preparation before allocation; collision reservations stay conservative. */
    void discardCommand(Object key) {
        CommandPlan command = commandPlans.get(key);
        if (command != null && command.cycle == cycleId) {
            if (commandsPlanned) suppressCommandGroup(key, "Prepared output failed");
            commands.remove(command);
            command.memberCount = 0;
            command.suppressed = true;
        }
    }

    double plannedVoltage(Object key, double original) {
        CommandPlan command = commandPlans.get(key);
        return command == null || command.cycle != cycleId ? original : command.planned;
    }

    boolean commandSuppressed(Object key) {
        CommandPlan command = commandPlans.get(key);
        return command != null && command.cycle == cycleId && command.suppressed;
    }

    private static void union(CommandPlan first, CommandPlan second) {
        CommandPlan a = first.root(), b = second.root();
        if (a != b) b.parent = a;
    }

    private static final class CommandPlan {
        final List<Member> members = new ArrayList<>();
        int memberCount;
        long cycle;
        double candidate, anchor, slope, planned, factor;
        boolean anchorSet, suppressed, groupSuppressed;
        String failure;
        CommandPlan parent;
        void reset(long cycle, double volts) {
            this.cycle = cycle;
            memberCount = 0;
            candidate = planned = volts;
            anchor = 0.0;
            factor = 1.0;
            parent = this;
            anchorSet = suppressed = groupSuppressed = false;
            failure = "";
        }
        CommandPlan root() {
            CommandPlan result = this;
            while (result.parent != result) result = result.parent;
            CommandPlan current = this;
            while (current.parent != current) { CommandPlan next = current.parent; current.parent = result; current = next; }
            return result;
        }
        Member member(int index) {
            while (members.size() <= index) members.add(new Member());
            return members.get(index);
        }
    }

    private static final class Member implements PowerPlanner.ForwardDemand {
        MotorState state;
        MotorHandle handle;
        CommandPlan command;
        double localCap, scale, currentAtZero, currentSlope, supplyRadius;
        boolean eligible, accounted, known, voltageAdded, supplyAdded;
        final PowerPlanner.Channel voltageChannel = new PowerPlanner.Channel();
        final PowerPlanner.Channel supplyChannel = new PowerPlanner.Channel();
        PowerPlanner.ForwardDemand model;
        final PowerPlanner.ElectricalDemandModel electricalModel = new PowerPlanner.ElectricalDemandModel();
        final PowerPlanner.ElectricalDemandModel voltageModel = new PowerPlanner.ElectricalDemandModel();
        final PowerPlanner.ElectricalDemandModel supplyModel = new PowerPlanner.ElectricalDemandModel();

        void prepareModel() {
            double zeroVolts = scale * command.anchor;
            double slopeVolts = scale * command.slope;
            if (known) {
                currentAtZero = (zeroVolts - state.backEmfVolts) / state.resistanceOhms;
                currentSlope = slopeVolts / state.resistanceOhms;
            } else {
                currentAtZero = 0.0;
                double estimate = state.measuredWindingAmps;
                if (!(estimate > 0.0) && Double.isFinite(localCap)) estimate = Math.max(0.0, localCap);
                currentSlope = Math.copySign(estimate, scale * command.candidate);
            }
            model = known && state.conversionBusVolts > 0.0
                    ? electricalModel.inputs(currentAtZero, currentSlope, zeroVolts, slopeVolts,
                            state.conversionBusVolts) : this;
        }
        boolean needsVoltageConstraint(double busVolts) {
            // An affine voltage stays between its endpoints for every common factor in [0,1].
            // Keep this cycle's channel when either endpoint is out of bounds (including overspeed).
            return !(Math.abs(scale * command.anchor) <= busVolts
                    && Math.abs(scale * command.candidate) <= busVolts);
        }
        void prepareVoltageConstraint() {
            voltageModel.inputs(scale * command.anchor, scale * command.slope, 0.0, 0.0, 1.0);
        }
        boolean needsSupplyConstraint(double requestedSupply) {
            // For a known motor I=(V-E)/R and V=v0+v1*factor, the quadratic coefficient of
            // V*I/bus is v1^2/(R*bus)>=0. Its maximum with zero is also convex. If both endpoints
            // fit, every intermediate factor fits, including the clipped regenerative portion.
            // Followers can draw at factor zero, so checking only the request would be unsafe.
            return !(positiveSupply(0.0) <= state.declaredSupplyLimit
                    && requestedSupply <= state.declaredSupplyLimit);
        }
        @Override public double currentAtZeroAmps() { return currentAtZero; }
        @Override public double currentSlopeAmps() { return currentSlope; }
        @Override public double positiveSupplyAt(double factor) { return positiveSupply(factor); }
        double positiveSupply(double factor) {
            double volts = scale * (command.anchor + factor * command.slope);
            double current = currentAtZero + factor * currentSlope;
            double bus = state.conversionBusVolts;
            return bus > 0.0 ? Math.max(0.0, volts * current / bus)
                    : Math.max(0.0, state.supplyAmps) * Math.max(0.0, factor);
        }
        void prepareSupplyConstraint() {
            double a = scale * command.anchor, b = scale * command.slope;
            double aa = b * currentSlope;
            double bb = a * currentSlope + b * currentAtZero;
            double cc = a * currentAtZero - state.declaredSupplyLimit * state.conversionBusVolts;
            double low = 0.0, high = 1.0;
            if (aa > 1e-12) {
                double discriminant = bb * bb - 4.0 * aa * cc;
                if (discriminant < 0.0) { low = 1.0; high = -1.0; }
                else {
                    double root = Math.sqrt(discriminant);
                    low = Math.max(low, (-bb - root) / (2.0 * aa));
                    high = Math.min(high, (-bb + root) / (2.0 * aa));
                }
            } else if (Math.abs(bb) > 1e-12) {
                if (bb > 0.0) high = Math.min(high, -cc / bb);
                else low = Math.max(low, -cc / bb);
            } else if (cc > 0.0) { low = 1.0; high = -1.0; }
            if (low > high) {
                supplyRadius = 0.0;
                supplyModel.inputs(1.0, 0.0, 0.0, 0.0, 1.0);
            } else {
                supplyRadius = 0.5 * (high - low);
                supplyModel.inputs(-0.5 * (high + low), 1.0, 0.0, 0.0, 1.0);
            }
        }
    }

    private double totalDemand() {
        double total = 0.0;
        for (MotorState state : active) total += state.supplyDemandAmps;
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
        if (!state.planStatus.isEmpty()) return state.planStatus;
        if (state.channel.supplyLimited()) return "Power shed";
        if (state.derate < 1.0) return "Thermal derate";
        if (cap < state.declaredCapAmps) return "Power capped";
        if (state.softwareSupplyLimit && state.constants.isPresent()
                && (!(state.conversionBusVolts > 0.0) || !Double.isFinite(state.backEmfVolts))) {
            return "Supply limit unavailable";
        }
        if (state.softwareSupplyLimit && state.constants.isEmpty()) return "Supply limit estimated";
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
        Optional<MotorConstants> constants;
        ca.frc6390.athena.api.hardware.MotorKind electricalKind;
        /** Winding resistance of {@link #constants} in ohms, converted once; NaN without constants. */
        double resistanceOhms;
        /** Speed constant of {@link #constants} in rad/s per volt, converted once; NaN without constants. */
        double speedConstantRadPerSecPerVolt;
        final ca.frc6390.athena.hardware.power.Thermal curve;
        final DoubleUnaryOperator supplyToWinding = this::windingForSupply;
        MotorHandle handle;
        long sampledCycle = Long.MIN_VALUE;
        /** Winding-current magnitude for torque profiles and thermal limits. */
        double demandAmps;
        double measuredWindingAmps;
        double declaredSupplyLimit;
        double candidateCapAmps = Double.POSITIVE_INFINITY;
        Member plannedMember;
        String planStatus = "";
        boolean outputSuppressed;
        /** Positive battery load; regeneration is not borrowed as headroom for another motor. */
        double supplyDemandAmps;
        double supplyAmps = Double.NaN;
        double statorAmps = Double.NaN;
        double conversionBusVolts;
        double backEmfVolts = Double.NaN;
        double derate = 1.0;
        double declaredCapAmps = Double.POSITIVE_INFINITY;
        /** Controller-enforced winding limit before thermal derating; supply limits stay in their own domain. */
        double configuredStatorLimitAmps = Double.POSITIVE_INFINITY;
        boolean softwareSupplyLimit;
        double temperatureCelsius = Double.NaN;
        String status = "Declared";

        double profileSustainedAmps() {
            var profile = device.power();
            return profile == null ? 0.0 : profile.raw().sustainedAmps();
        }

        boolean profileHasCurrentCap() {
            var profile = device.power();
            return profile != null && (profile.raw().sustainedAmps() > 0.0 || profile.sag() != null
                    || profile.raw().rampAmpsPerSecond() > 0.0);
        }

        MotorState(MotorDevice device) {
            this.device = device;
            this.name = device.defaultName();
            updateConstants(device.kind());
            this.curve = device.thermalCurve();
        }

        void updateConstants(ca.frc6390.athena.api.hardware.MotorKind kind) {
            if (electricalKind == kind) return;
            electricalKind = kind;
            this.constants = MotorConstants.forKind(kind);
            this.resistanceOhms = constants.isPresent()
                    ? constants.get().resistance().in(edu.wpi.first.units.Units.Ohms)
                    : Double.NaN;
            this.speedConstantRadPerSecPerVolt = constants.isPresent()
                    ? constants.get().raw().speedConstantRadPerSecPerVolt()
                    : Double.NaN;
        }

        void readInputs(MotorHandle handle, double bus) {
            updateConstants(handle.commandTarget().device().kind());
            supplyAmps = readSupplyCurrent(handle);
            statorAmps = readStatorCurrent(handle);
            double volts = readVoltage(handle);
            double rotationsPerSecond = readVelocity(handle);
            // A measured terminal voltage is a lower bound on its supplying bus. Do not assume
            // the policy floor still exists when bus telemetry is missing during a brownout.
            conversionBusVolts = Double.isFinite(bus) && bus > 0.0 ? bus
                    : Double.isFinite(volts) ? Math.abs(volts) : 0.0;
            backEmfVolts = constants.isPresent() && Double.isFinite(rotationsPerSecond)
                    ? rotationsPerSecond * 2.0 * Math.PI / speedConstantRadPerSecPerVolt : Double.NaN;
            double modelCurrent = Double.isFinite(volts) && Double.isFinite(backEmfVolts)
                    ? (volts - backEmfVolts) / resistanceOhms : Double.NaN;
            double modelPower = volts * modelCurrent;
            if (Double.isFinite(statorAmps)) {
                demandAmps = Math.abs(statorAmps);
            } else if (Double.isFinite(modelCurrent)) {
                demandAmps = Math.abs(modelCurrent);
            } else {
                // Missing winding telemetry: use electrical power conversion when duty is known,
                // otherwise keep the supply estimate without dividing by zero or inventing a large torque.
                demandAmps = Double.isFinite(supplyAmps) ? Math.abs(supplyAmps) : 0.0;
                if (Double.isFinite(volts) && Math.abs(volts) > ELECTRICAL_EPSILON
                        && Double.isFinite(bus)) demandAmps *= bus / Math.abs(volts);
            }
            if (!Double.isFinite(supplyAmps)) {
                if (Double.isFinite(modelPower)) {
                    supplyAmps = Double.isFinite(bus) && bus > 0.0 ? modelPower / bus
                            : modelPower == 0.0 ? 0.0 : Math.copySign(Math.abs(modelCurrent), modelPower);
                } else if (Double.isFinite(volts)) {
                    supplyAmps = volts == 0.0 ? 0.0
                            : Double.isFinite(bus) && bus > 0.0 ? Math.abs(volts) * demandAmps / bus : demandAmps;
                } else {
                    supplyAmps = demandAmps;
                }
            } else if (Double.isFinite(modelPower) && modelPower < -ELECTRICAL_EPSILON) {
                // Some vendors report supply-current magnitude. Physical negative electrical power
                // identifies regeneration even with such a reading; it is never positive demand.
                supplyAmps = -Math.abs(supplyAmps);
            }
            supplyDemandAmps = Double.isFinite(supplyAmps) ? Math.max(0.0, supplyAmps) : 0.0;
            measuredWindingAmps = demandAmps;
        }

        double windingForSupply(double amps) {
            if (constants.isPresent()) {
                // Without speed, the voltage clamp cannot cancel back-EMF. A zero cap produces
                // zero voltage under that missing-speed fallback instead of guessing a power allowance.
                if (!Double.isFinite(backEmfVolts)) return 0.0;
                return PowerBudget.windingAmpsForSupply(amps, conversionBusVolts, backEmfVolts, resistanceOhms);
            }
            // No motor model: retain the existing feedback estimate; the output path scales voltage
            // from measured supply current instead of claiming a hard model-based winding bound.
            return supplyDemandAmps > ELECTRICAL_EPSILON
                    ? demandAmps * Math.min(1.0, Math.max(0.0, amps) / supplyDemandAmps) : Math.max(0.0, amps);
        }

        void thermalStep(ThermalModel model, double dt) {
            if (handle != null) {
                OptionalDouble vendor = handle.temperatureCelsius();
                model.vendorReading(vendor == null || vendor.isEmpty() ? Double.NaN : vendor.getAsDouble());
            }
            model.step(measuredWindingAmps, dt);
            temperatureCelsius = model.celsius();
        }

        // Direct reads retain the independent failure boundaries without creating four capturing
        // method-reference objects per motor per cycle. Resolve the live handle on every read.
        private static double readSupplyCurrent(MotorHandle handle) {
            try { return handle.supplyCurrentAmps(); }
            catch (RuntimeException exception) { return Double.NaN; }
        }
        private static double readStatorCurrent(MotorHandle handle) {
            try { return handle.statorCurrentAmps(); }
            catch (RuntimeException exception) { return Double.NaN; }
        }
        private static double readVoltage(MotorHandle handle) {
            try { return handle.appliedVoltage(); }
            catch (RuntimeException exception) { return Double.NaN; }
        }
        private static double readVelocity(MotorHandle handle) {
            try { return handle.integratedVelocityRotationsPerSecond(); }
            catch (RuntimeException exception) { return Double.NaN; }
        }
    }
}
