package ca.frc6390.athena.hardware.power;

import java.util.List;

/**
 * Read-only view of the electrical state from the latest runtime cycle, for telemetry.
 *
 * @param busVolts measured bus voltage, NaN when unavailable
 * @param openCircuitVolts estimated open-circuit voltage, NaN before the first sample
 * @param batteryOhms estimated (or default) internal resistance
 * @param floorVolts configured floor
 * @param predictedVolts predicted bus voltage at the requested total current
 * @param budgetAmps total current budget at the floor
 * @param demandAmps total requested current
 * @param shedding whether the budget is currently shedding current
 * @param motors per-motor entries
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
        List<Motor> motors) {

    public PowerSnapshot {
        motors = List.copyOf(motors);
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
     * @param demandAmps requested current
     * @param capAmps current cap applied by the planner, {@link Double#POSITIVE_INFINITY} when none
     * @param derateFraction thermal derate fraction in effect
     * @param temperatureCelsius fused winding temperature, NaN when unknown
     * @param shareWeight relative share weight
     * @param sustainedAmps declared sustained cap of the profile, zero when none
     * @param status applied status text
     */
    public record Motor(
            String name,
            double demandAmps,
            double capAmps,
            double derateFraction,
            double temperatureCelsius,
            double shareWeight,
            double sustainedAmps,
            String status) {
    }
}
