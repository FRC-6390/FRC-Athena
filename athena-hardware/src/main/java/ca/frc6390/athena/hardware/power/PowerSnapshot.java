package ca.frc6390.athena.hardware.power;

import java.util.List;

/**
 * Read-only view of the electrical state from the latest runtime cycle, for telemetry.
 *
 * @param busVolts measured bus voltage, NaN when unavailable
 * @param openCircuitVolts estimated open-circuit voltage, NaN before the first sample
 * @param batteryOhms estimated (or default) internal resistance
 * @param floorVolts configured floor
 * @param predictedVolts predicted bus voltage at the latest non-regenerative supply load
 * @param budgetAmps total current budget at the floor
 * @param demandAmps total battery-side supply load, excluding regeneration
 * @param shedding whether the budget is currently shedding current
 * @param motors per-motor entries
 * @param supplyWatts electrical power drawn from the battery by all motors (negative while they regenerate)
 * @param consumedJoules energy drawn from the battery by all motors since boot
 * @param regeneratedJoules energy returned to the battery by all motors since boot
 * @param windingLossJoules energy lost as heat in all motor windings since boot (I squared R)
 */
public record PowerSnapshot(
        double busVolts,
        double openCircuitVolts,
        double batteryOhms,
        double floorVolts,
        double predictedVolts,
        double budgetAmps,
        double demandAmps,
        boolean shedding,
        List<Motor> motors,
        double supplyWatts,
        double consumedJoules,
        double regeneratedJoules,
        double windingLossJoules) {

    public PowerSnapshot {
        motors = List.copyOf(motors);
    }

    /** Snapshot without energy totals; they read zero. */
    public PowerSnapshot(double busVolts, double openCircuitVolts, double batteryOhms, double floorVolts,
            double predictedVolts, double budgetAmps, double demandAmps, boolean shedding, List<Motor> motors) {
        this(busVolts, openCircuitVolts, batteryOhms, floorVolts, predictedVolts, budgetAmps, demandAmps, shedding,
                motors, 0.0, 0.0, 0.0, 0.0);
    }

    /** Empty snapshot before the first cycle. */
    public static PowerSnapshot empty(PowerPolicy policy) {
        return new PowerSnapshot(Double.NaN, Double.NaN, policy.defaultBatteryOhms(), policy.floorVolts(),
                Double.NaN, Double.POSITIVE_INFINITY, 0.0, false, List.of());
    }

    /**
     * Per-motor state.
     *
     * @param name motor name
     * @param demandAmps winding-current magnitude (estimated from supply telemetry for unknown motors)
     * @param capAmps winding-current cap, {@link Double#POSITIVE_INFINITY} when none; zero is an active cap
     * @param derateFraction thermal derate fraction in effect
     * @param temperatureCelsius fused winding temperature, NaN when unknown
     * @param shareWeight relative share weight
     * @param sustainedAmps declared sustained cap of the profile, zero when none
     * @param status applied status text
     * @param supplyDemandAmps battery-side supply load, excluding regeneration
     * @param supplyWatts electrical power drawn from the battery (negative while regenerating), NaN when unknown
     * @param consumedJoules energy drawn from the battery since boot
     * @param regeneratedJoules energy returned to the battery since boot
     * @param windingLossJoules energy lost as heat in the winding since boot (I squared R), zero without a model
     */
    public record Motor(
            String name,
            double demandAmps,
            double capAmps,
            double derateFraction,
            double temperatureCelsius,
            double shareWeight,
            double sustainedAmps,
            String status,
            double supplyDemandAmps,
            double supplyWatts,
            double consumedJoules,
            double regeneratedJoules,
            double windingLossJoules) {
        /** Entry without energy accounting; energy reads zero and power NaN. */
        public Motor(String name, double demandAmps, double capAmps, double derateFraction,
                double temperatureCelsius, double shareWeight, double sustainedAmps, String status,
                double supplyDemandAmps) {
            this(name, demandAmps, capAmps, derateFraction, temperatureCelsius, shareWeight, sustainedAmps, status,
                    supplyDemandAmps, Double.NaN, 0.0, 0.0, 0.0);
        }

        /** Compatibility constructor for telemetry sources that assume unity winding/supply conversion. */
        public Motor(String name, double demandAmps, double capAmps, double derateFraction,
                double temperatureCelsius, double shareWeight, double sustainedAmps, String status) {
            this(name, demandAmps, capAmps, derateFraction, temperatureCelsius, shareWeight,
                    sustainedAmps, status, demandAmps);
        }
    }
}
