package ca.frc6390.athena.wpilib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class AllianceTransformTest {
    private static final double EPSILON = 1.0e-9;
    private static final double LENGTH = 16.0;
    private static final double WIDTH = 8.0;

    @Test
    void rotate180RotatesPoseAndHeadingOnRed() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true);

        Pose2d red = rotated.pose(new Pose2d(1.0, 2.0, new Rotation2d(0.0)));

        assertEquals(15.0, red.getX(), EPSILON);
        assertEquals(6.0, red.getY(), EPSILON);
        assertEquals(Math.PI, Math.abs(red.getRotation().getRadians()), EPSILON);
    }

    @Test
    void rotate180ChangesHeadingByHalfACircle() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true);
        Rotation2d blue = new Rotation2d(0.6);

        Rotation2d red = rotated.rotation(blue);

        assertEquals(Math.PI, Math.abs(red.minus(blue).getRadians()), EPSILON);
    }

    @Test
    void mirrorXFlipsXAndReflectsHeading() {
        AllianceTransform mirrored = AllianceTransform.mirrorX(LENGTH).withAlliance(() -> true);

        Pose2d facingUp = mirrored.pose(new Pose2d(2.0, 3.0, new Rotation2d(Math.PI / 2.0)));
        assertEquals(14.0, facingUp.getX(), EPSILON);
        assertEquals(3.0, facingUp.getY(), EPSILON);
        assertEquals(Math.PI / 2.0, facingUp.getRotation().getRadians(), EPSILON);

        Pose2d facingForward = mirrored.pose(new Pose2d(2.0, 3.0, new Rotation2d(0.0)));
        assertEquals(Math.PI, Math.abs(facingForward.getRotation().getRadians()), EPSILON);
    }

    @Test
    void mirrorYFlipsYAndNegatesHeading() {
        AllianceTransform mirrored = AllianceTransform.mirrorY(WIDTH).withAlliance(() -> true);

        Pose2d pose = mirrored.pose(new Pose2d(2.0, 3.0, new Rotation2d(Math.PI / 4.0)));

        assertEquals(5.0, pose.getY(), EPSILON);
        assertEquals(-Math.PI / 4.0, pose.getRotation().getRadians(), EPSILON);
    }

    @Test
    void blueAndUnknownAllianceReturnTheInputUnchanged() {
        Pose2d blue = new Pose2d(2.0, 3.0, new Rotation2d(0.4));
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH);

        assertEquals(blue, rotated.withAlliance(() -> false).pose(blue));
        assertEquals(blue, rotated.withAlliance(() -> false).pose(blue));
    }

    @Test
    void translationRotationAndRegionFollowTheResolvedAlliance() {
        AllianceTransform rotated = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(() -> true);

        Translation2d translation = rotated.translation(new Translation2d(2.0, 3.0));
        assertEquals(14.0, translation.getX(), EPSILON);
        assertEquals(5.0, translation.getY(), EPSILON);

        assertEquals(Rectangle2d.of(13.0, 4.0, 15.0, 6.0),
                rotated.region(Rectangle2d.of(1.0, 2.0, 3.0, 4.0)));
    }

    @Test
    void poseSupplierResolvesTheAllianceOnEveryCall() {
        AtomicBoolean red = new AtomicBoolean(false);
        Pose2d blue = new Pose2d(1.0, 2.0, new Rotation2d(0.0));
        var supplier = AllianceTransform.rotate180(LENGTH, WIDTH).withAlliance(red::get).poseSupplier(blue);

        assertEquals(blue, supplier.get());
        red.set(true);
        assertEquals(15.0, supplier.get().getX(), EPSILON);
    }

    @Test
    void customWpilibMappingIsAppliedOnRed() {
        AllianceTransform custom = AllianceTransform.of(
                pose -> new Pose2d(pose.getX() + 1.0, pose.getY(), pose.getRotation()))
                .withAlliance(() -> true);

        assertEquals(2.0, custom.pose(new Pose2d(1.0, 0.0, new Rotation2d())).getX(), EPSILON);
    }
}
