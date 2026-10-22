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
     * Converts a battery-side allocation to a conservative winding-current limit at the measured speed.
     * The worst motoring direction consumes {@code R I^2 + |backEmf| I} watts; the other direction
     * consumes less or regenerates. Solving that quadratic avoids dividing by PWM duty near zero.
     * This bounds electrical power at the supplied bus voltage, not mechanical power or controller losses.
     *
     * @param supplyAmps non-negative battery current allocation; infinity leaves the motor uncapped
     * @param busVolts positive bus voltage
     * @param backEmfVolts signed motor back-EMF
     * @param resistanceOhms positive winding resistance
     * @return winding amps; zero for a zero allocation or unusable electrical inputs
     */
    public static double windingAmpsForSupply(
            double supplyAmps, double busVolts, double backEmfVolts, double resistanceOhms) {
        if (supplyAmps == Double.POSITIVE_INFINITY) return Double.POSITIVE_INFINITY;
        if (!(supplyAmps > 0.0) || !Double.isFinite(busVolts) || !(busVolts > 0.0)
                || !Double.isFinite(backEmfVolts) || !Double.isFinite(resistanceOhms)
                || !(resistanceOhms > 0.0)) return 0.0;
        double power = busVolts * supplyAmps;
        if (power == 0.0) return 0.0;
        if (Double.isInfinite(power)) return Double.POSITIVE_INFINITY;
        double emf = Math.abs(backEmfVolts);
        double root = Math.hypot(emf, 2.0 * Math.sqrt(resistanceOhms) * Math.sqrt(power));
        // This form avoids cancellation when the back-EMF is large relative to the allowed winding drop.
        return power / (0.5 * root + 0.5 * emf);
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
        return waterFill(demand, weight, count, budget, allocation, new int[count]);
    }

    /** Water-fill overload with reusable primitive sorting workspace; allocates nothing per call. */
    public static double[] waterFill(double[] demand, double[] weight, int count, double budget,
            double[] allocation, int[] order) {
        if (count == 0) {
            return allocation;
        }
        if (order.length < count) throw new IllegalArgumentException("Water-fill workspace is too small.");
        for (int i = 0; i < count; i++) {
            // Typical plans contain only a few motor/profile groups. Insertion sort avoids
            // boxed indices, a comparator closure and TimSort scratch storage on the robot thread.
            double ratio = demand[i] / weight[i];
            int position = i;
            while (position > 0
                    && Double.compare(demand[order[position - 1]] / weight[order[position - 1]], ratio) > 0) {
                order[position] = order[position - 1];
                position--;
            }
            order[position] = i;
        }
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

    /** Larger real root of {@code a*x*x + b*x + c = 0}, or NaN when no finite root exists. */
    static double largerQuadraticRoot(double a, double b, double c) {
        double magnitude = Math.max(Math.abs(a), Math.max(Math.abs(b), Math.abs(c)));
        if (!(magnitude > 0.0) || !Double.isFinite(magnitude)) return Double.NaN;
        a /= magnitude;
        b /= magnitude;
        c /= magnitude;
        if (a == 0.0) return b == 0.0 ? Double.NaN : -c / b;
        double discriminant = Math.fma(-4.0 * a, c, b * b);
        if (!(discriminant >= 0.0)) return Double.NaN;
        double q = -0.5 * (b + Math.copySign(Math.sqrt(discriminant), b));
        if (q == 0.0) return -b / (2.0 * a);
        // This form avoids subtracting nearly equal values for the smaller root.
        return Math.max(q / a, c / q);
    }
}
