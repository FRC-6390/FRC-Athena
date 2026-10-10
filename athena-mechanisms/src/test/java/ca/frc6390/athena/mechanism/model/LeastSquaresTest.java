package ca.frc6390.athena.mechanism.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;

class LeastSquaresTest {
    @Test
    void recoversCoefficientsFromNoisySyntheticData() {
        Random random = new Random(7);
        int n = 400;
        double[][] x = new double[n][3];
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            x[i][0] = random.nextGaussian() * 2.0;
            x[i][1] = random.nextGaussian() * 0.5;
            x[i][2] = 1.0;
            y[i] = 2.0 * x[i][0] - 3.0 * x[i][1] + 0.5 + 0.05 * random.nextGaussian();
        }

        LeastSquares.Fit fit = LeastSquares.fit(x, y);

        assertEquals(2.0, fit.coefficients()[0], 0.01);
        assertEquals(-3.0, fit.coefficients()[1], 0.02);
        assertEquals(0.5, fit.coefficients()[2], 0.01);
        for (double se : fit.standardErrors()) {
            assertTrue(se > 0.0 && se < 0.05);
        }
        assertTrue(fit.rSquared() > 0.99);
        assertEquals(n, fit.samples());
    }

    @Test
    void standardErrorsShrinkWithMoreData() {
        double small = standardErrorOfSlope(40, 11);
        double large = standardErrorOfSlope(2000, 11);
        assertTrue(large < small / 3.0, small + " vs " + large);
    }

    @Test
    void rejectsConstantColumnsAndCollinearColumns() {
        double[][] constant = {{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};
        assertThrows(IllegalArgumentException.class, () -> LeastSquares.fit(constant, new double[] {1, 2, 3, 4}));

        double[][] collinear = {{1.0, 2.0}, {2.0, 4.0}, {3.0, 6.0}, {4.0, 8.0}, {5.0, 10.0}};
        assertThrows(IllegalArgumentException.class,
                () -> LeastSquares.fit(collinear, new double[] {1, 2, 3, 4, 5}));
        assertEquals(Double.POSITIVE_INFINITY, LeastSquares.conditionNumber(collinear));
    }

    @Test
    void rejectsTooFewSamples() {
        double[][] rows = {{1.0, 2.0}, {2.0, 1.0}};
        assertThrows(IllegalArgumentException.class, () -> LeastSquares.fit(rows, new double[] {1, 2}));
    }

    @Test
    void conditionNumberIsInvariantToColumnScale() {
        Random random = new Random(3);
        double[][] base = new double[50][2];
        double[][] scaled = new double[50][2];
        for (int i = 0; i < base.length; i++) {
            base[i][0] = random.nextGaussian();
            base[i][1] = random.nextGaussian();
            scaled[i][0] = base[i][0] * 1.0e6;
            scaled[i][1] = base[i][1] * 1.0e-6;
        }
        assertEquals(LeastSquares.conditionNumber(base), LeastSquares.conditionNumber(scaled), 1.0e-6);
    }

    private static double standardErrorOfSlope(int n, long seed) {
        Random random = new Random(seed);
        double[][] x = new double[n][2];
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            x[i][0] = random.nextGaussian();
            x[i][1] = 1.0;
            y[i] = 1.5 * x[i][0] + 0.2 * random.nextGaussian();
        }
        return LeastSquares.fit(x, y).standardErrors()[0];
    }
}
