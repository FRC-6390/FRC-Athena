package ca.frc6390.athena.hardware.power;

/**
 * Pure electrical budget math shared by {@link PowerPlanner} and tests.
 *
 * <p>Bus model: {@code V_bus = Voc - R * I_total}. The largest total current that keeps the bus at
 * the floor is {@code I_max = (Voc - floor) / R}.</p>
 *
 * <p>Allocation is weighted max-min fairness ("water filling"). Each motor {@code i} has a demand
 * {@code d_i} and a share weight {@code w_i}. For a level {@code L} the allocation is
 * {@code a_i = min(d_i, L * w_i)}, and {@code L} is chosen so that {@code sum a_i = budget}. Motors
 * whose demand per unit weight is lowest are fully served first, so the lowest-share motors are
 * the first to be shed when the budget is short.</p>
 */
public final class PowerBudget {
    private PowerBudget() {
    }

    /**
     * Returns the total battery current that keeps the bus at the floor.
     *
     * @param openCircuitVolts open-circuit voltage
     * @param ohms internal resistance, positive
     * @param floorVolts floor voltage
     * @return budget in amps, zero when the open-circuit voltage is already at or below the floor
     */
    public static double budgetAmps(double openCircuitVolts, double ohms, double floorVolts) {
        if (!(ohms > 0.0) || !Double.isFinite(openCircuitVolts)) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, (openCircuitVolts - floorVolts) / ohms);
    }

    /**
     * Predicts the bus voltage for a total battery current.
     *
     * @param openCircuitVolts open-circuit voltage
     * @param ohms internal resistance
     * @param totalAmps total battery current
     * @return predicted bus voltage
     */
    public static double predictedVolts(double openCircuitVolts, double ohms, double totalAmps) {
        return openCircuitVolts - ohms * totalAmps;
    }

    /**
     * Water-fills {@code budget} across demands with weights.
     *
     * @param demand non-negative demands, length at least {@code count}
     * @param weight positive weights, length at least {@code count}
     * @param count number of entries to use
     * @param budget total allocation available
     * @param allocation output, length at least {@code count}
     * @return the allocation array for chaining
     */
    public static double[] waterFill(double[] demand, double[] weight, int count, double budget, double[] allocation) {
        if (count == 0) {
            return allocation;
        }
        Integer[] order = new Integer[count];
        for (int i = 0; i < count; i++) {
            order[i] = i;
        }
        java.util.Arrays.sort(order, (a, b) -> Double.compare(demand[a] / weight[a], demand[b] / weight[b]));
        double remainingBudget = Math.max(0.0, budget);
        double remainingWeight = 0.0;
        for (int i = 0; i < count; i++) {
            remainingWeight += weight[i];
        }
        for (int k = 0; k < count; k++) {
            int i = order[k];
            double level = remainingBudget / remainingWeight;
            if (demand[i] / weight[i] <= level) {
                allocation[i] = demand[i];
                remainingBudget = Math.max(0.0, remainingBudget - demand[i]);
                remainingWeight -= weight[i];
            } else {
                for (int j = k; j < count; j++) {
                    int rest = order[j];
                    allocation[rest] = level * weight[rest];
                }
                return allocation;
            }
        }
        return allocation;
    }
}
