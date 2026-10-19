package ca.frc6390.athena.hardware.power;

import java.util.List;

/**
 * Per-cycle electrical planner. Pure computation: it takes demands and bus state and returns a
 * current cap per motor. Applying the caps to hardware is the caller's job, see the mechanism runtime.
 *
 * <p>For each cycle, in order:</p>
 * <ol>
 *   <li><b>Budget.</b> Predicted bus voltage {@code Voc - R * sum(demand)}. Below the floor (with 0.25 V
 *       hysteresis) the total budget {@link PowerBudget#budgetAmps} is water-filled across motors by
 *       share ({@link PowerBudget#waterFill}).</li>
 *   <li><b>Profile.</b> Sustained cap; burst allowed while the burst time budget lasts; sag response
 *       applied while the measured bus is below its threshold.</li>
 *   <li><b>Stagger.</b> A motor whose demand crosses its large-current threshold is admitted only if
 *       {@code staggerSeconds} have passed since the last admitted start. Otherwise it is held at the
 *       large-current threshold.</li>
 *   <li><b>Ramp.</b> Increases of the cap are limited to the profile ramp rate; decreases are immediate.</li>
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

    /** Per-motor planner channel. Keep one per motor across cycles: it holds the ramp and burst state. */
    public static final class Channel {
        private PowerProfile profile;
        private double demandAmps;
        private double declaredCapAmps = Double.POSITIVE_INFINITY;
        private double capAmps = Double.POSITIVE_INFINITY;
        private double lastCapAmps = Double.POSITIVE_INFINITY;
        private double burstBucketSeconds = -1.0;
        private boolean admitted;

        /**
         * Sets this cycle's inputs.
         *
         * @param profile profile or {@code null}
         * @param demandAmps requested current, non-negative
         * @param declaredCapAmps cap already applied by configuration (thermal and limits); zero or infinity means none
         */
        public void inputs(PowerProfile profile, double demandAmps, double declaredCapAmps) {
            this.profile = profile;
            this.demandAmps = Double.isFinite(demandAmps) ? Math.max(0.0, demandAmps) : 0.0;
            this.declaredCapAmps = declaredCapAmps > 0.0 ? declaredCapAmps : Double.POSITIVE_INFINITY;
        }

        /** Returns the planned cap for this cycle, or positive infinity when uncapped. */
        public double capAmps() { return capAmps; }

        /** Returns the demand used in the last plan. */
        public double demandAmps() { return demandAmps; }

        /** Returns the share used in the last plan. */
        public double shareWeight() { return profile == null ? 1.0 : profile.share(); }

        /** Returns the burst time remaining in seconds, or -1 when none has been initialized. */
        public double burstBucketSeconds() { return burstBucketSeconds; }
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
        int count = channels.size();
        ensureCapacity(count);
        double total = 0.0;
        for (int i = 0; i < count; i++) {
            Channel channel = channels.get(i);
            demands[i] = channel.demandAmps;
            weights[i] = channel.shareWeight();
            total += channel.demandAmps;
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
            Channel channel = channels.get(i);
            double budgetCap = shedding ? allocations[i] : Double.POSITIVE_INFINITY;
            planChannel(channel, bus, nowSeconds, dtSeconds, budgetCap);
        }
    }

    /** Returns whether the last plan shed current to keep the bus at the floor. */
    public boolean shedding() { return shedding; }

    private boolean updateShedding(double predicted) {
        if (!Double.isFinite(predicted)) return false;
        if (shedding) return predicted < floorVolts + FLOOR_HYSTERESIS_VOLTS;
        return predicted < floorVolts;
    }

    private void planChannel(Channel channel, Bus bus, double nowSeconds, double dtSeconds, double budgetCap) {
        PowerProfile profile = channel.profile;
        double requested = channel.demandAmps;
        double cap = Math.min(channel.declaredCapAmps, budgetCap);
        if (profile != null) {
            double sustained = profile.sustainedAmps();
            if (sustained > 0.0) {
                cap = Math.min(cap, sustainedOrBurst(channel, profile, requested, dtSeconds, sustained));
            }
            Sag sag = profile.sag();
            if (sag != null && Double.isFinite(bus.measuredVolts()) && bus.measuredVolts() < sag.threshold().in(edu.wpi.first.units.Units.Volts)) {
                double base = Math.min(cap, requested);
                cap = base * (1.0 - sag.shedFraction());
            }
        }
        double largeThreshold = profile != null && profile.sustainedAmps() > 0.0
                ? Math.max(LARGE_CURRENT_AMPS * 0.25, 0.5 * profile.sustainedAmps())
                : LARGE_CURRENT_AMPS;
        if (requested >= largeThreshold) {
            if (!channel.admitted) {
                if (staggerSeconds <= 0.0 || nowSeconds - lastAdmittedStartSeconds >= staggerSeconds) {
                    channel.admitted = true;
                    lastAdmittedStartSeconds = nowSeconds;
                } else {
                    cap = Math.min(cap, largeThreshold);
                }
            }
        } else {
            channel.admitted = false;
        }
        if (profile != null && profile.rampAmpsPerSecond() > 0.0
                && Double.isFinite(channel.lastCapAmps) && cap > channel.lastCapAmps) {
            cap = Math.min(cap, channel.lastCapAmps + profile.rampAmpsPerSecond() * Math.max(0.0, dtSeconds));
        }
        channel.capAmps = Math.max(0.0, cap);
        channel.lastCapAmps = channel.capAmps;
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
