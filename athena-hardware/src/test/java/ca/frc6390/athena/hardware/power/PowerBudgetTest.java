package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;

class PowerBudgetTest {
    private static final double EPS = 1e-9;

    @Test
    void budgetIsHeadroomOverResistance() {
        // (12.5 - 7.0) / 0.025 = 220 A
        assertEquals(220.0, PowerBudget.budgetAmps(12.5, 0.025, 7.0), EPS);
        assertEquals(0.0, PowerBudget.budgetAmps(6.5, 0.025, 7.0), EPS);
        assertTrue(Double.isInfinite(PowerBudget.budgetAmps(12.5, 0.0, 7.0)));
    }

    @Test
    void predictedVoltageUsesOhmsLaw() {
        assertEquals(12.5 - 0.02 * 100.0, PowerBudget.predictedVolts(12.5, 0.02, 100.0), EPS);
    }

    @Test
    void supplyConversionBoundsPowerForBothMotorDirectionsWithoutDividingByDuty() {
        for (double emf : new double[] {0.0, 0.001, 4.0, -4.0, 15.0, -15.0}) {
            for (double allocation : new double[] {0.0, 0.000001, 2.0, 50.0}) {
                double current = PowerBudget.windingAmpsForSupply(allocation, 12.0, emf, 0.03);
                assertTrue(Double.isFinite(current) && current >= 0.0);
                double maximumPower = 0.03 * current * current + Math.abs(emf) * current;
                assertEquals(12.0 * allocation, maximumPower, 1e-8);
                // The opposite torque direction is either less motoring power or regeneration.
                assertTrue(0.03 * current * current - Math.abs(emf) * current <= maximumPower);
            }
        }
        assertEquals(Math.sqrt(12.0 * 30.0 / 0.03),
                PowerBudget.windingAmpsForSupply(30.0, 12.0, 0.0, 0.03), EPS);
        assertEquals(100.0, PowerBudget.windingAmpsForSupply(100.0, 12.0, 9.0, 0.03), EPS,
                "at full duty, winding and supply amps agree");
    }

    @Test
    void unusableConversionInputsNeverProduceNaNOrTurnZeroIntoNoLimit() {
        assertEquals(0.0, PowerBudget.windingAmpsForSupply(0.0, 12.0, 4.0, 0.03));
        assertEquals(0.0, PowerBudget.windingAmpsForSupply(10.0, Double.NaN, 4.0, 0.03));
        assertEquals(0.0, PowerBudget.windingAmpsForSupply(10.0, 12.0, Double.NaN, 0.03));
        assertEquals(0.0, PowerBudget.windingAmpsForSupply(10.0, 12.0, 4.0, 0.0));
        assertEquals(Double.POSITIVE_INFINITY,
                PowerBudget.windingAmpsForSupply(Double.POSITIVE_INFINITY, 12.0, 4.0, 0.03));
    }

    @Test
    void waterFillReturnsDemandWhenBudgetIsAmple() {
        double[] alloc = new double[3];
        PowerBudget.waterFill(new double[] {10, 20, 30}, new double[] {1, 1, 1}, 3, 100, alloc);
        assertArrayEquals(new double[] {10, 20, 30}, alloc, EPS);
    }

    @Test
    void waterFillShedsLowestShareFirst() {
        // Equal demands, weights 1 and 3: the heavier share keeps 3x the current.
        double[] alloc = new double[2];
        PowerBudget.waterFill(new double[] {100, 100}, new double[] {1, 3}, 2, 40, alloc);
        assertEquals(10.0, alloc[0], EPS);
        assertEquals(30.0, alloc[1], EPS);
        assertEquals(40.0, alloc[0] + alloc[1], EPS);
    }

    @Test
    void waterFillServesSmallDemandsFullyThenSplitsRest() {
        // Motor 0 needs only 10 A and is fully served; the other two split the remaining 50 A equally.
        double[] alloc = new double[3];
        PowerBudget.waterFill(new double[] {10, 100, 100}, new double[] {1, 1, 1}, 3, 60, alloc);
        assertEquals(10.0, alloc[0], EPS);
        assertEquals(25.0, alloc[1], EPS);
        assertEquals(25.0, alloc[2], EPS);
    }

    @Test
    void waterFillWithZeroBudgetAllocatesNothing() {
        double[] alloc = new double[2];
        PowerBudget.waterFill(new double[] {10, 10}, new double[] {1, 1}, 2, 0, alloc);
        assertEquals(0.0, alloc[0] + alloc[1], EPS);
    }

    @Test
    void reusableWaterFillMatchesWeightedFairnessAcrossChangingCounts() {
        Random random = new Random(6390L);
        int[] order = new int[40];
        double[] demand = new double[40], weight = new double[40], allocation = new double[40];
        for (int trial = 0; trial < 200; trial++) {
            int count = 1 + random.nextInt(order.length);
            double totalDemand = 0.0;
            for (int index = 0; index < count; index++) {
                totalDemand += demand[index] = 200.0 * random.nextDouble();
                weight[index] = 0.1 + 5.0 * random.nextDouble();
            }
            double budget = 1.2 * totalDemand * random.nextDouble();
            PowerBudget.waterFill(demand, weight, count, budget, allocation, order);
            double totalAllocation = 0.0, fairLevel = -1.0;
            for (int index = 0; index < count; index++) {
                assertTrue(allocation[index] >= 0.0 && allocation[index] <= demand[index] + EPS);
                totalAllocation += allocation[index];
                if (allocation[index] < demand[index] - EPS) {
                    double level = allocation[index] / weight[index];
                    if (fairLevel >= 0.0) assertEquals(fairLevel, level, EPS);
                    fairLevel = level;
                }
            }
            assertEquals(Math.min(totalDemand, budget), totalAllocation, 1e-8);
            for (int index = 0; index < count; index++) {
                if (fairLevel >= 0.0 && allocation[index] == demand[index])
                    assertTrue(demand[index] / weight[index] <= fairLevel + EPS);
            }
        }
    }

    @Test
    void quadraticRootsRemainStableAcrossMagnitudeAndCancellation() {
        assertEquals((Math.sqrt(5.0) - 1.0) / 2.0,
                PowerBudget.largerQuadraticRoot(1e308, 1e308, -1e308), EPS);
        assertEquals(1e-10, PowerBudget.largerQuadraticRoot(1e-300, 1.0, -1e-10), 1e-24);
        assertEquals(2.0, PowerBudget.largerQuadraticRoot(0.0, 1e-250, -2e-250), EPS);
        assertEquals(0.5, PowerBudget.largerQuadraticRoot(1.0, -1.0, 0.25), EPS);
        assertTrue(Double.isNaN(PowerBudget.largerQuadraticRoot(1.0, 0.0, 1.0)));
    }
}
