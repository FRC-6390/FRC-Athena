package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class PoseBoundingBox2dTest {

    @Test
    void containsInsideAndOnBoundary() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 4.0, 6.0);

        assertTrue(box.contains(2.5, 3.5));
        assertTrue(box.contains(1.0, 6.0));
    }

    @Test
    void acceptsCornersInAnyOrder() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(4.0, 6.0, 1.0, 2.0);

        assertTrue(box.contains(2.0, 3.0));
        assertFalse(box.contains(0.99, 3.0));
    }

    @Test
    void rejectsNonFiniteCoordinates() {
        assertThrows(IllegalArgumentException.class,
                () -> new PoseBoundingBox2d(Double.NaN, 0.0, 1.0, 2.0));
        assertThrows(IllegalArgumentException.class,
                () -> new PoseBoundingBox2d(0.0, 0.0, Double.POSITIVE_INFINITY, 2.0));
    }
}
