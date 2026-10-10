package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
