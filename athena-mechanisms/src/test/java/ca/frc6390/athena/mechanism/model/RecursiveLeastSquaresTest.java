package ca.frc6390.athena.mechanism.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;

class RecursiveLeastSquaresTest {
    @Test
    void convergesToTrueParametersAndShrinksStandardError() {
        RecursiveLeastSquares rls = new RecursiveLeastSquares(2, 1.0, 1.0e4);
        Random random = new Random(21);
        double firstError = Double.NaN;
        for (int i = 0; i < 1000; i++) {
            double[] regressor = {random.nextGaussian(), random.nextGaussian()};
            double target = 0.8 * regressor[0] - 1.7 * regressor[1] + 0.01 * random.nextGaussian();
            rls.update(regressor, target);
            if (i == 20) {
                firstError = rls.standardError(0);
            }
        }
        assertEquals(0.8, rls.coefficients()[0], 0.01);
        assertEquals(-1.7, rls.coefficients()[1], 0.01);
        assertTrue(rls.standardError(0) < firstError);
        assertEquals(1000, rls.updates());
    }

    @Test
    void forgettingLetsTheEstimateFollowAChange() {
        RecursiveLeastSquares rls = new RecursiveLeastSquares(1, 0.95, 1.0e4);
        Random random = new Random(5);
        for (int i = 0; i < 300; i++) {
            double x = random.nextGaussian();
            rls.update(new double[] {x}, 1.0 * x);
        }
        for (int i = 0; i < 300; i++) {
            double x = random.nextGaussian();
            rls.update(new double[] {x}, 3.0 * x);
        }
        assertEquals(3.0, rls.coefficients()[0], 0.05);
    }

    @Test
    void seedAndResetBehave() {
        RecursiveLeastSquares rls = new RecursiveLeastSquares(2, 1.0, 10.0);
        rls.seed(new double[] {0.5, 0.25});
        assertEquals(0.5, rls.coefficients()[0], 1.0e-12);
        rls.reset();
        assertEquals(0.0, rls.coefficients()[0], 1.0e-12);
        assertEquals(0L, rls.updates());
        assertThrows(IllegalArgumentException.class, () -> rls.seed(new double[] {1.0}));
        assertThrows(IllegalArgumentException.class, () -> new RecursiveLeastSquares(1, 0.0, 1.0));
    }
}
