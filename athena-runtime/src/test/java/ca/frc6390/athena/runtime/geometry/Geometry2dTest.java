package ca.frc6390.athena.runtime.geometry;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import org.junit.jupiter.api.Test;

class Geometry2dTest {
    @Test
    void shapesShareContainmentAndExpansion() {
        Geometry2d rectangle = Rectangle2d.of(1.0, 2.0, 3.0, 4.0);
        Geometry2d circle = Circle2d.of(2.0, 3.0, 1.0);
        Geometry2d polygon = Polygon2d.of(
                new Point2d(1.0, 2.0),
                new Point2d(3.0, 2.0),
                new Point2d(2.0, 4.0));

        assertTrue(rectangle.contains(new Point2d(1.5, 2.5)));
        assertTrue(circle.contains(new Point2d(2.0, 3.5)));
        assertTrue(polygon.contains(new Point2d(2.0, 2.5)));
        assertFalse(rectangle.contains(new Point2d(0.8, 3.0)));
        assertTrue(rectangle.expanded(0.25).contains(new Point2d(0.8, 3.0)));
    }

    @Test
    void geometryDirectlyContainsPoseSnapshots() {
        Geometry2d geometry = Rectangle2d.of(0.0, 0.0, 1.0, 1.0);

        assertTrue(geometry.contains(new PoseSnapshot(0.5, 0.5, 0.0)));
        assertFalse(geometry.contains(new PoseSnapshot(1.1, 0.5, 0.0)));
    }

}
