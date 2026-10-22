package ca.frc6390.athena.hardware.power;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

/**
 * Per-cycle electrical planner. Pure computation: it takes demands and bus state and returns a
 * winding-current cap and shared command scale per motor. Battery allocations use supply amps, independently of the winding
 * amps used by profiles, thermal limits and start staggering. Applying caps is the caller's job.
 *
 * <p>For each cycle, in order:</p>
 * <ol>
 *   <li><b>Profile.</b> Sustained cap; burst allowed while the burst time budget lasts; sag response
 *       applied while the measured bus is below its threshold.</li>
 *   <li><b>Stagger.</b> A group whose demand crosses its large-current threshold is admitted only if
 *       {@code staggerSeconds} have passed since the last admitted start. Otherwise it is held at the
 *       large-current threshold. Shared profiles and explicit startup relationships admit atomically.</li>
 *   <li><b>Ramp.</b> Increases of the cap are limited to the profile ramp rate; decreases are immediate.</li>
 *   <li><b>Budget and coupling.</b> Below the bus floor (with 0.25 V hysteresis), water-fill supply amps
 *       across profile groups using the sum of member shares. Solve one command scale satisfying
 *       every physical member's winding limit and the group's battery allocation.</li>
 * </ol>
 */
public final class PowerPlanner {
    /** Demand above which a motor counts as starting when no sustained cap is declared. */
    public static final double LARGE_CURRENT_AMPS = 20.0;
    /** Hysteresis on the floor before a shedding budget is released. */
    public static final double FLOOR_HYSTERESIS_VOLTS = 0.25;
    /** Fraction of real time that refills the burst budget while the motor draws within sustained. */
    public static final double BURST_REFILL_RATIO = 0.5;

    private final double floorVolts;
    private final double staggerSeconds;
    private boolean shedding;
    private double lastAdmittedStartSeconds = Double.NEGATIVE_INFINITY;
    private double[] demands = new double[0];
    private double[] weights = new double[0];
    private double[] allocations = new double[0];
    private final Map<Object, Group> groupByIdentity = new IdentityHashMap<>();
    private final List<Group> groups = new ArrayList<>();
    private final List<Group> groupPool = new ArrayList<>();
    private final Map<Object, StartGroup> startByKey = new HashMap<>();
    private final List<StartGroup> starts = new ArrayList<>();
    private final List<StartGroup> startPool = new ArrayList<>();
    private long cycle;

    public PowerPlanner(PowerPolicy policy) {
        this.floorVolts = policy.floorVolts();
        this.staggerSeconds = policy.staggerSeconds();
    }

    /** Electrical state of the battery and bus for one cycle. */
    public record Bus(double measuredVolts, double openCircuitVolts, double ohms) {
        /** Returns the predicted bus voltage at a total current, or the measured voltage when no model exists. */
        public double predictedVolts(double totalAmps) {
            if (!Double.isFinite(openCircuitVolts)) return measuredVolts;
            return PowerBudget.predictedVolts(openCircuitVolts, ohms, totalAmps);
        }
    }

    /**
     * Electrical demand of one physical motor along a command-root voltage interpolation. Winding
     * current is {@code currentAtZeroAmps + currentSlopeAmps * scale}. The positive supply callback
     * accepts a scale in [0, 1] and returns the non-negative battery demand at that scale. Its cost
     * must be convex, but need not be monotone: a native follower can draw current at scale zero.
     */
    public record DemandModel(double currentAtZeroAmps, double currentSlopeAmps,
            DoubleUnaryOperator positiveSupplyAtScale) {
        public DemandModel {
            if (!Double.isFinite(currentAtZeroAmps) || !Double.isFinite(currentSlopeAmps)) {
                throw new IllegalArgumentException("Demand-model currents must be finite.");
            }
            Objects.requireNonNull(positiveSupplyAtScale, "positiveSupplyAtScale");
        }

        public double currentAt(double scale) {
            return currentAtZeroAmps + currentSlopeAmps * scale;
        }
    }

    /** Per-motor planner channel. Keep one per motor across cycles: it holds the ramp and burst state. */
    public static final class Channel {
        private PowerProfile profile;
        private double demandAmps;
        private double supplyDemandAmps;
        private DoubleUnaryOperator supplyToWinding;
        private DemandModel demandModel;
        private Object couplingGroup;
        private boolean accountedLoad = true;
        private boolean startupParticipant = true;
        private double thermalFraction = 1.0;
        private boolean supplyLimited;
        private boolean supplyAllocationReduced;
        private boolean supplyBudgetActive;
        private double declaredCapAmps = Double.POSITIVE_INFINITY;
        private double capAmps = Double.POSITIVE_INFINITY;
        private double lastCapAmps = Double.POSITIVE_INFINITY;
        private double burstBucketSeconds = -1.0;
        private boolean admitted;
        private double couplingScale = 1.0;
        private boolean couplingFeasible = true;
        private long lastPlannedCycle = Long.MIN_VALUE;

        /**
         * Sets this cycle's inputs.
         *
         * @param profile profile or {@code null}
         * @param demandAmps requested current, non-negative
         * @param declaredCapAmps cap already applied by configuration (thermal and limits); zero or infinity means none
         */
        public void inputs(PowerProfile profile, double demandAmps, double declaredCapAmps) {
            assignProfile(profile);
            this.demandAmps = Double.isFinite(demandAmps) ? Math.max(0.0, demandAmps) : 0.0;
            this.supplyDemandAmps = this.demandAmps;
            this.supplyToWinding = null;
            this.demandModel = null;
            this.thermalFraction = 1.0;
            this.declaredCapAmps = declaredCapAmps > 0.0 ? declaredCapAmps : Double.POSITIVE_INFINITY;
        }

        /**
         * Separate battery and winding domains. While shedding, positive loads keep their allocated
         * supply cap, including inside the hysteresis band; idle and regenerative channels receive no budget cap.
         * The older overload assumes unity conversion. Unlike it, a declared winding cap of zero is an active limit.
         *
         * @param profile winding-current profile or null
         * @param windingDemandAmps measured or estimated winding-current magnitude
         * @param declaredWindingCapAmps winding limit after thermal derating, infinity when absent
         * @param supplyDemandAmps non-negative battery load, excluding regeneration
         * @param supplyToWinding conversion at the current speed and bus voltage
         * @param thermalFraction fraction applied to the profile's winding-current limits
         */
        public void inputs(PowerProfile profile, double windingDemandAmps, double declaredWindingCapAmps,
                double supplyDemandAmps, DoubleUnaryOperator supplyToWinding, double thermalFraction) {
            assignProfile(profile);
            this.demandAmps = Double.isFinite(windingDemandAmps) ? Math.max(0.0, windingDemandAmps) : 0.0;
            this.supplyDemandAmps = Double.isFinite(supplyDemandAmps) ? Math.max(0.0, supplyDemandAmps) : 0.0;
            this.declaredCapAmps = Double.isNaN(declaredWindingCapAmps)
                    ? Double.POSITIVE_INFINITY : Math.max(0.0, declaredWindingCapAmps);
            this.supplyToWinding = Objects.requireNonNull(supplyToWinding, "supplyToWinding");
            this.demandModel = null;
            if (!Double.isFinite(thermalFraction) || thermalFraction < 0.0 || thermalFraction > 1.0) {
                throw new IllegalArgumentException("Thermal fraction must be finite and between zero and one.");
            }
            this.thermalFraction = thermalFraction;
        }

        /**
         * Plans from an upcoming output rather than the previously measured output. The forward
         * model captures the requested current's sign and electrical state, including physical
         * follower mappings. Its callback must be bounded and side-effect free.
         * This permits one battery allocation to be solved across a shared profile without changing
         * the requested torque ratios.
         */
        public void inputs(PowerProfile profile, double windingDemandAmps, double declaredWindingCapAmps,
                double supplyDemandAmps, DoubleUnaryOperator supplyToWinding, double thermalFraction,
                DemandModel demandModel) {
            inputs(profile, windingDemandAmps, declaredWindingCapAmps, supplyDemandAmps,
                    supplyToWinding, thermalFraction);
            this.demandModel = Objects.requireNonNull(demandModel, "demandModel");
        }

        /**
         * Supplies a transient grouping identity after command families have been joined with shared
         * profile identities. Set it every cycle; {@code null} uses the profile or independent channel.
         */
        public void couplingGroup(Object identity) { couplingGroup = identity; }

        /** Returns the caller-supplied grouping identity, or null when profile identity is used. */
        public Object couplingGroup() { return couplingGroup; }

        /**
         * Internal physical-load accounting: aliases still impose their caps, but their shared
         * command target contributes battery demand and share weight through one owner only.
         */
        public void accountedLoad(boolean accounted) { accountedLoad = accounted; }

        /** Internal: disables start-current interpretation for synthetic voltage/interval constraints. */
        public void startupParticipant(boolean participant) { startupParticipant = participant; }

        private void assignProfile(PowerProfile next) {
            if (profile != next) {
                burstBucketSeconds = -1.0;
                lastCapAmps = Double.POSITIVE_INFINITY;
                admitted = false;
            }
            profile = next;
        }

        /** Returns the planned cap for this cycle, or positive infinity when uncapped. */
        public double capAmps() { return capAmps; }

        /** Returns the demand used in the last plan. */
        public double demandAmps() { return demandAmps; }

        /** Returns the battery-side load used for voltage prediction and allocation, excluding regeneration. */
        public double supplyDemandAmps() { return supplyDemandAmps; }

        /** Whether a reduced battery allocation is the active winding-current bound. */
        public boolean supplyLimited() { return supplyLimited; }

        /** Whether the battery allocation reduced positive supply demand, even when another cap was tighter. */
        public boolean supplyAllocationReduced() { return supplyAllocationReduced; }

        /** Whether a finite supply allocation is held on this channel, including inside the hysteresis band. */
        public boolean supplyBudgetActive() { return supplyBudgetActive; }

        /** Returns the share used in the last plan. */
        public double shareWeight() { return accountedLoad ? profile == null ? 1.0 : profile.share() : 0.0; }

        /** Returns the burst time remaining in seconds, or -1 when none has been initialized. */
        public double burstBucketSeconds() { return burstBucketSeconds; }

        /** Common requested-winding-current factor for this profile group in the latest plan. */
        public double couplingScale() { return couplingScale; }

        /** Whether one common scale could satisfy every physical member's winding and supply caps. */
        public boolean couplingFeasible() { return couplingFeasible; }
    }

    /**
     * Plans caps for all channels for one cycle.
     *
     * @param bus bus state
     * @param nowSeconds monotonic time
     * @param dtSeconds cycle period, non-negative
     * @param channels channels, in a stable order
     */
    public void plan(Bus bus, double nowSeconds, double dtSeconds, List<Channel> channels) {
        cycle++;
        prepareGroups(channels);
        admitStarts(nowSeconds);
        for (Channel channel : channels) {
            planChannel(channel, bus, dtSeconds);
        }
        int count = groups.size();
        ensureCapacity(count);
        double total = 0.0;
        for (int i = 0; i < count; i++) {
            Group group = groups.get(i);
            constrainGroup(group);
            demands[i] = !group.feasible ? 0.0
                    : group.forwardModel ? supplyAt(group, group.localFactor) : group.supplyDemand;
            if (!Double.isFinite(demands[i])) {
                group.feasible = false;
                demands[i] = 0.0;
            }
            group.plannedSupplyDemand = demands[i];
            weights[i] = group.weight > 0.0 ? group.weight : 1.0;
            total += demands[i];
        }
        double predicted = bus.predictedVolts(total);
        shedding = updateShedding(predicted);
        if (shedding) {
            double budget = Double.isFinite(bus.openCircuitVolts())
                    ? PowerBudget.budgetAmps(bus.openCircuitVolts(), bus.ohms(), floorVolts)
                    : Double.POSITIVE_INFINITY;
            PowerBudget.waterFill(demands, weights, count, budget, allocations);
        }
        for (int i = 0; i < count; i++) {
            applyGroup(groups.get(i), shedding ? allocations[i] : Double.POSITIVE_INFINITY);
        }
    }

    private void prepareGroups(List<Channel> channels) {
        groups.clear();
        groupByIdentity.clear();
        starts.clear();
        startByKey.clear();
        for (Group group : groupPool) group.reset();
        for (StartGroup start : startPool) start.reset();
        int startsUsed = 0;
        for (Channel channel : channels) {
            if (channel.lastPlannedCycle != cycle - 1) channel.admitted = false;
            channel.lastPlannedCycle = cycle;
            Object identity = channel.couplingGroup != null ? channel.couplingGroup
                    : channel.profile == null ? channel : channel.profile;
            Group group = groupByIdentity.get(identity);
            if (group == null) {
                int index = groups.size();
                if (index == groupPool.size()) groupPool.add(new Group());
                group = groupPool.get(index);
                groupByIdentity.put(identity, group);
                groups.add(group);
            }
            group.members.add(channel);
            if (channel.accountedLoad) group.supplyDemand += channel.supplyDemandAmps;
            group.weight += channel.shareWeight();
            group.forwardModel &= channel.demandModel != null;
        }
        for (Group group : groups) {
            StartGroup start = null;
            for (Channel channel : group.members) {
                String key = channel.profile == null ? null : channel.profile.startupGroup();
                StartGroup existing = key == null ? null : startByKey.get(key);
                if (existing != null && existing != start) {
                    if (start == null) start = existing;
                    else {
                        StartGroup merged = existing;
                        start.members.addAll(merged.members);
                        starts.remove(merged);
                        StartGroup replacement = start;
                        startByKey.replaceAll((unused, value) -> value == merged ? replacement : value);
                    }
                }
            }
            if (start == null) {
                if (startsUsed == startPool.size()) startPool.add(new StartGroup());
                start = startPool.get(startsUsed++);
                starts.add(start);
            }
            start.members.addAll(group.members);
            for (Channel channel : group.members) {
                String key = channel.profile == null ? null : channel.profile.startupGroup();
                if (key != null) startByKey.put(key, start);
            }
        }
        for (StartGroup start : starts) {
            for (Channel channel : start.members) {
                if (channel.startupParticipant) {
                    start.requested |= channel.demandAmps >= largeThreshold(channel);
                    start.admitted |= channel.admitted;
                }
            }
        }
    }

    private void admitStarts(double nowSeconds) {
        for (StartGroup start : starts) {
            if (!start.requested) {
                for (Channel channel : start.members) channel.admitted = false;
                continue;
            }
            if (!start.admitted && (staggerSeconds <= 0.0
                    || nowSeconds - lastAdmittedStartSeconds >= staggerSeconds)) {
                start.admitted = true;
                lastAdmittedStartSeconds = nowSeconds;
            }
            for (Channel channel : start.members) channel.admitted = start.admitted;
        }
    }

    private static double largeThreshold(Channel channel) {
        PowerProfile profile = channel.profile;
        return profile != null && profile.sustainedAmps() > 0.0
                ? Math.max(LARGE_CURRENT_AMPS * 0.25, 0.5 * profile.sustainedAmps())
                : LARGE_CURRENT_AMPS;
    }

    private void applyGroup(Group group, double allocation) {
        double factor = group.localFactor;
        boolean budgetActive = Double.isFinite(allocation) && group.plannedSupplyDemand > 0.0;
        boolean budgetReduced = false;
        if (group.feasible && budgetActive && group.forwardModel) {
            if (supplyAt(group, factor) > allocation) {
                double lower = minimumSupplyScale(group);
                if (supplyAt(group, lower) > allocation + 1e-9) {
                    group.feasible = false;
                }
                double upper = factor;
                for (int iteration = 0; group.feasible && iteration < 64; iteration++) {
                    double middle = lower + (upper - lower) * 0.5;
                    if (supplyAt(group, middle) <= allocation) lower = middle;
                    else upper = middle;
                }
                factor = lower;
                budgetReduced = true;
            }
        } else if (group.feasible && budgetActive) {
            budgetReduced = allocation < group.supplyDemand;
            for (Channel channel : group.members) {
                double cap = legacyBudgetCap(channel, allocation, group.supplyDemand);
                if (cap < channel.capAmps) {
                    channel.capAmps = Math.max(0.0, cap);
                }
            }
            factor = currentFactor(group);
        }
        boolean coupled = group.members.size() > 1;
        for (Channel channel : group.members) {
            boolean positiveLoad = channel.accountedLoad && (group.forwardModel
                    ? channel.demandModel.positiveSupplyAtScale().applyAsDouble(factor) > 0.0
                    : channel.supplyDemandAmps > 0.0);
            if (group.forwardModel && group.feasible && factor < 1.0) {
                // The application layer must use couplingScale rather than treating this magnitude
                // as an independent voltage clamp when a follower has non-zero current at scale 0.
                double current = Math.abs(channel.demandModel.currentAt(factor));
                if (current > 0.0) channel.capAmps = Math.min(channel.capAmps, current);
            } else if (!group.forwardModel && channel.demandAmps > 0.0 && coupled && factor < 1.0) {
                channel.capAmps = Math.min(channel.capAmps, channel.demandAmps * factor);
            }
            channel.couplingScale = group.feasible ? factor : 0.0;
            channel.couplingFeasible = group.feasible;
            channel.supplyAllocationReduced = budgetReduced && positiveLoad;
            channel.supplyBudgetActive = budgetActive && positiveLoad;
            channel.supplyLimited = channel.supplyBudgetActive && budgetReduced;
            channel.lastCapAmps = channel.capAmps;
        }
    }

    private static double legacyBudgetCap(Channel channel, double allocation, double demand) {
        if (!channel.accountedLoad || !(channel.supplyDemandAmps > 0.0)) {
            return Double.POSITIVE_INFINITY;
        }
        double share = allocation * (channel.supplyDemandAmps / demand);
        double cap = channel.supplyToWinding == null ? share : channel.supplyToWinding.applyAsDouble(share);
        return Double.isNaN(cap) ? 0.0 : cap;
    }

    private static double currentFactor(Group group) {
        double factor = 1.0;
        for (Channel channel : group.members) {
            if (channel.demandAmps > 0.0) {
                factor = Math.min(factor, channel.capAmps / channel.demandAmps);
            }
        }
        return Math.max(0.0, factor);
    }

    private static void constrainGroup(Group group) {
        if (!group.forwardModel) {
            group.localFactor = currentFactor(group);
            return;
        }
        for (Channel channel : group.members) {
            if (!Double.isFinite(channel.capAmps)) continue;
            DemandModel model = channel.demandModel;
            double initial = model.currentAtZeroAmps();
            double slope = model.currentSlopeAmps();
            double cap = channel.capAmps;
            if (slope == 0.0) {
                if (Math.abs(initial) > cap) group.feasible = false;
            } else {
                double first = (-cap - initial) / slope;
                double second = (cap - initial) / slope;
                group.lowerFactor = Math.max(group.lowerFactor, Math.min(first, second));
                group.localFactor = Math.min(group.localFactor, Math.max(first, second));
            }
        }
        if (group.lowerFactor > group.localFactor) group.feasible = false;
    }

    private static double minimumSupplyScale(Group group) {
        double lower = group.lowerFactor;
        double upper = group.localFactor;
        for (int iteration = 0; iteration < 80; iteration++) {
            double first = lower + (upper - lower) / 3.0;
            double second = upper - (upper - lower) / 3.0;
            if (supplyAt(group, first) <= supplyAt(group, second)) upper = second;
            else lower = first;
        }
        double middle = lower + (upper - lower) * 0.5;
        double best = middle;
        if (supplyAt(group, group.lowerFactor) < supplyAt(group, best)) best = group.lowerFactor;
        if (supplyAt(group, group.localFactor) < supplyAt(group, best)) best = group.localFactor;
        return best;
    }

    private static double supplyAt(Group group, double factor) {
        double total = 0.0;
        for (Channel channel : group.members) {
            if (!channel.accountedLoad) continue;
            double supply = channel.demandModel.positiveSupplyAtScale().applyAsDouble(factor);
            if (!Double.isFinite(supply)) return Double.POSITIVE_INFINITY;
            total += Math.max(0.0, supply);
        }
        return total;
    }

    private static final class Group {
        private final List<Channel> members = new ArrayList<>();
        private double supplyDemand;
        private double plannedSupplyDemand;
        private double weight;
        private double localFactor = 1.0;
        private double lowerFactor;
        private boolean feasible = true;
        private boolean forwardModel = true;

        private void reset() {
            members.clear();
            supplyDemand = plannedSupplyDemand = weight = lowerFactor = 0.0;
            localFactor = 1.0;
            feasible = forwardModel = true;
        }
    }

    private static final class StartGroup {
        private final List<Channel> members = new ArrayList<>();
        private boolean requested;
        private boolean admitted;

        private void reset() {
            members.clear();
            requested = admitted = false;
        }
    }

    /** Returns whether the last plan shed current to keep the bus at the floor. */
    public boolean shedding() { return shedding; }

    private boolean updateShedding(double predicted) {
        if (!Double.isFinite(predicted)) return false;
        if (shedding) return predicted < floorVolts + FLOOR_HYSTERESIS_VOLTS;
        return predicted < floorVolts;
    }

    private void planChannel(Channel channel, Bus bus, double dtSeconds) {
        PowerProfile profile = channel.profile;
        double requested = channel.demandAmps;
        double cap = channel.declaredCapAmps;
        if (profile != null) {
            double sustained = profile.sustainedAmps();
            if (sustained > 0.0) {
                cap = Math.min(cap, sustainedOrBurst(channel, profile, requested, dtSeconds, sustained)
                        * channel.thermalFraction);
            }
            Sag sag = profile.sag();
            if (sag != null && Double.isFinite(bus.measuredVolts()) && bus.measuredVolts() < sag.threshold().in(edu.wpi.first.units.Units.Volts)) {
                double base = Math.min(cap, requested);
                cap = base * (1.0 - sag.shedFraction());
            }
        }
        double largeThreshold = largeThreshold(channel);
        if (channel.startupParticipant && requested >= largeThreshold && !channel.admitted) {
            cap = Math.min(cap, largeThreshold);
        }
        if (profile != null && profile.rampAmpsPerSecond() > 0.0
                && Double.isFinite(channel.lastCapAmps) && cap > channel.lastCapAmps) {
            cap = Math.min(cap, channel.lastCapAmps + profile.rampAmpsPerSecond() * Math.max(0.0, dtSeconds));
        }
        channel.capAmps = Math.max(0.0, cap);
    }

    private static double sustainedOrBurst(Channel channel, PowerProfile profile, double demand, double dt, double sustained) {
        double burstAmps = profile.burstAmps();
        double burstSeconds = profile.burstSeconds();
        if (channel.burstBucketSeconds < 0.0) {
            channel.burstBucketSeconds = burstSeconds;
        }
        if (burstAmps <= sustained || burstSeconds <= 0.0) {
            channel.burstBucketSeconds = burstSeconds;
            return sustained;
        }
        if (demand > sustained && channel.burstBucketSeconds > 0.0) {
            channel.burstBucketSeconds = Math.max(0.0, channel.burstBucketSeconds - dt);
            return burstAmps;
        }
        if (demand <= sustained) {
            channel.burstBucketSeconds = Math.min(burstSeconds,
                    channel.burstBucketSeconds + BURST_REFILL_RATIO * dt);
        }
        return sustained;
    }

    private void ensureCapacity(int count) {
        if (demands.length < count) {
            demands = new double[count];
            weights = new double[count];
            allocations = new double[count];
        }
    }
}
