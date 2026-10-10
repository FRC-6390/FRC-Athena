package ca.frc6390.athena.runtime.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

class AllianceTransformTest {
    private static final double EPSILON = 1.0e-9;
    private static final double LENGTH = 16.0;
    private static final double WIDTH = 8.0;

    @Test
    void blueAndUnknownAlliancesReturnTheInputUnchanged() {
        PoseSnapshot blue = new PoseSnapshot(1.0, 2.0, 0.5);
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH);

        assertSame(blue, rotated.pose(blue));
        assertSame(blue, rotated.withAlliance(() -> false).pose(blue));
        assertSame(blue, rotated.withAlliance(null).pose(blue));
    }

    @Test
    void identityNeverChangesGeometryOnRed() {
        AllianceTransform identity = AllianceTransform.identity().withAlliance(() -> true);
        PoseSnapshot pose = new PoseSnapshot(3.0, 4.0, 1.0);

        assertSame(pose, identity.pose(pose));
        assertEquals(3.0, identity.translation(new Point2d(3.0, 4.0)).x(), EPSILON);
        assertEquals(1.0, identity.rotation(1.0), EPSILON);
        assertEquals(Rectangle2d.of(1.0, 1.0, 2.0, 2.0),
                identity.region(Rectangle2d.of(1.0, 1.0, 2.0, 2.0)));
    }

    @Test
    void rotate180MapsPositionsThroughTheFieldCenter() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true);

        PoseSnapshot pose = rotated.pose(new PoseSnapshot(1.0, 2.0, 0.0));

        assertEquals(15.0, pose.xMeters(), EPSILON);
        assertEquals(6.0, pose.yMeters(), EPSILON);
        assertEquals(Math.PI, pose.headingRadians(), EPSILON);
    }

    @Test
    void rotate180TurnsEveryHeadingByHalfACircle() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true);

        double[] headings = {-3.0, -1.2, 0.0, 0.7, 1.5, 2.9};
        for (double heading : headings) {
            // A half-turn leaves cos(difference) at -1 regardless of wrapping into (-pi, pi].
            assertEquals(-1.0, Math.cos(rotated.rotation(heading) - heading), EPSILON);
        }
        assertEquals(-Math.PI / 2.0, rotated.rotation(Math.PI / 2.0), EPSILON);
    }

    @Test
    void mirrorXFlipsXAndReflectsHeadingAcrossTheVerticalAxis() {
        AllianceTransform mirrored = AllianceTransform.mirrorX(LENGTH).withAlliance(() -> true);

        PoseSnapshot facingForward = mirrored.pose(new PoseSnapshot(1.0, 2.0, 0.0));
        assertEquals(15.0, facingForward.xMeters(), EPSILON);
        assertEquals(2.0, facingForward.yMeters(), EPSILON);
        assertEquals(Math.PI, facingForward.headingRadians(), EPSILON);

        assertEquals(Math.PI / 2.0, mirrored.rotation(Math.PI / 2.0), EPSILON);
        assertEquals(Math.PI - 0.3, mirrored.rotation(0.3), EPSILON);
    }

    @Test
    void mirrorYFlipsYAndNegatesHeading() {
        AllianceTransform mirrored = AllianceTransform.mirrorY(WIDTH).withAlliance(() -> true);

        PoseSnapshot pose = mirrored.pose(new PoseSnapshot(1.0, 2.0, Math.PI / 4.0));

        assertEquals(1.0, pose.xMeters(), EPSILON);
        assertEquals(6.0, pose.yMeters(), EPSILON);
        assertEquals(-Math.PI / 4.0, pose.headingRadians(), EPSILON);
    }

    @Test
    void regionIsTheBoundingBoxOfTheMappedCorners() {
        Rectangle2d blue = Rectangle2d.of(1.0, 2.0, 3.0, 4.0);

        assertEquals(Rectangle2d.of(13.0, 4.0, 15.0, 6.0),
                AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true).region(blue));
        assertEquals(Rectangle2d.of(13.0, 2.0, 15.0, 4.0),
                AllianceTransform.mirrorX(LENGTH).withAlliance(() -> true).region(blue));
        assertEquals(Rectangle2d.of(1.0, 4.0, 3.0, 6.0),
                AllianceTransform.mirrorY(WIDTH).withAlliance(() -> true).region(blue));
    }

    @Test
    void translationAndRotationResolveForTheCurrentAlliance() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH);
        Point2d blue = new Point2d(2.0, 3.0);

        assertSame(blue, rotated.withAlliance(() -> false).translation(blue));
        Point2d red = rotated.withAlliance(() -> true).translation(blue);
        assertEquals(14.0, red.x(), EPSILON);
        assertEquals(5.0, red.y(), EPSILON);
    }

    @Test
    void poseSupplierFollowsTheAllianceAtCallTime() {
        AtomicBoolean red = new AtomicBoolean(false);
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(red::get);
        PoseSnapshot blue = new PoseSnapshot(1.0, 2.0, 0.0);
        Supplier<PoseSnapshot> supplier = rotated.poseSupplier(blue);

        assertSame(blue, supplier.get());
        red.set(true);
        assertEquals(15.0, supplier.get().xMeters(), EPSILON);
        red.set(false);
        assertSame(blue, supplier.get());
    }

    @Test
    void customRedMappingAppliesOnlyOnRed() {
        AllianceTransform custom = AllianceTransform.of(
                pose -> new PoseSnapshot(pose.xMeters() + 1.0, pose.yMeters(), pose.headingRadians()))
                .withAlliance(() -> true);

        assertEquals(2.0, custom.pose(new PoseSnapshot(1.0, 0.0, 0.0)).xMeters(), EPSILON);
        assertEquals(1.0, AllianceTransform.of(pose -> pose).pose(new PoseSnapshot(1.0, 0.0, 0.0)).xMeters(), EPSILON);
    }

    @Test
    void rejectsNonFiniteFieldDimensions() {
        assertThrows(IllegalArgumentException.class, () -> AllianceTransform.rotate180(Double.NaN, WIDTH));
        assertThrows(IllegalArgumentException.class, () -> AllianceTransform.mirrorX(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> AllianceTransform.mirrorY(Double.NaN));
    }
}
