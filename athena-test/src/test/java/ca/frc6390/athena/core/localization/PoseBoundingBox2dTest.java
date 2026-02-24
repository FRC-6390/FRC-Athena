package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

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

    @Test
    void mirrorsVerticalAcrossAxisX() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorVertical(5.0);

        assertEquals(7.0, mirrored.minX(), 1e-9);
        assertEquals(2.0, mirrored.minY(), 1e-9);
        assertEquals(9.0, mirrored.maxX(), 1e-9);
        assertEquals(6.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsHorizontalAcrossAxisY() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorHorizontal(10.0);

        assertEquals(1.0, mirrored.minX(), 1e-9);
        assertEquals(14.0, mirrored.minY(), 1e-9);
        assertEquals(3.0, mirrored.maxX(), 1e-9);
        assertEquals(18.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsAcrossCustomXYAxes() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirror(5.0, 10.0);

        assertEquals(7.0, mirrored.minX(), 1e-9);
        assertEquals(14.0, mirrored.minY(), 1e-9);
        assertEquals(9.0, mirrored.maxX(), 1e-9);
        assertEquals(18.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsPointsAcrossCustomAxes() {
        Translation2d point = new Translation2d(2.0, 3.0);

        Translation2d mirrored = PoseBoundingBox2d.mirrorPoint(point, 5.0, 10.0);

        assertEquals(8.0, mirrored.getX(), 1e-9);
        assertEquals(17.0, mirrored.getY(), 1e-9);
    }

    @Test
    void mirrorsTopBottomAcrossFieldMidline() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorTopBottom(10.0);

        assertEquals(1.0, mirrored.minX(), 1e-9);
        assertEquals(4.0, mirrored.minY(), 1e-9);
        assertEquals(3.0, mirrored.maxX(), 1e-9);
        assertEquals(8.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsLeftRightAcrossFieldMidline() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorLeftRight(10.0);

        assertEquals(7.0, mirrored.minX(), 1e-9);
        assertEquals(2.0, mirrored.minY(), 1e-9);
        assertEquals(9.0, mirrored.maxX(), 1e-9);
        assertEquals(6.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsToRedAllianceAcrossFieldCenter() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 1.0, 3.0, 2.0);

        PoseBoundingBox2d mirrored = box.mirrorToAlliance(DriverStation.Alliance.Red, 10.0, 8.0);

        assertEquals(7.0, mirrored.minX(), 1e-9);
        assertEquals(6.0, mirrored.minY(), 1e-9);
        assertEquals(9.0, mirrored.maxX(), 1e-9);
        assertEquals(7.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsAllianceBetweenExplicitSourceAndTarget() {
        PoseBoundingBox2d redBox = new PoseBoundingBox2d(7.0, 6.0, 9.0, 7.0);

        PoseBoundingBox2d blueBox = redBox.mirrorAlliance(
                DriverStation.Alliance.Red,
                DriverStation.Alliance.Blue,
                10.0,
                8.0);

        assertEquals(1.0, blueBox.minX(), 1e-9);
        assertEquals(1.0, blueBox.minY(), 1e-9);
        assertEquals(3.0, blueBox.maxX(), 1e-9);
        assertEquals(2.0, blueBox.maxY(), 1e-9);
    }

    @Test
    void mirrorAllianceNoOpWhenSourceEqualsTarget() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(2.0, 3.0, 4.0, 5.0);

        PoseBoundingBox2d same = box.mirrorAlliance(
                DriverStation.Alliance.Blue,
                DriverStation.Alliance.Blue,
                10.0,
                8.0);

        assertEquals(box, same);
    }

    @Test
    void mirrorsDiagonalMainAcrossSquareField() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorDiagonalMain(10.0, 10.0);

        assertEquals(2.0, mirrored.minX(), 1e-9);
        assertEquals(1.0, mirrored.minY(), 1e-9);
        assertEquals(6.0, mirrored.maxX(), 1e-9);
        assertEquals(3.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsDiagonalAntiAcrossSquareField() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 2.0, 3.0, 6.0);

        PoseBoundingBox2d mirrored = box.mirrorDiagonalAnti(10.0, 10.0);

        assertEquals(4.0, mirrored.minX(), 1e-9);
        assertEquals(7.0, mirrored.minY(), 1e-9);
        assertEquals(8.0, mirrored.maxX(), 1e-9);
        assertEquals(9.0, mirrored.maxY(), 1e-9);
    }

    @Test
    void mirrorsPointDiagonalMainAcrossSquareField() {
        Translation2d point = new Translation2d(2.0, 7.0);

        Translation2d mirrored = PoseBoundingBox2d.mirrorPointDiagonalMain(point, 10.0, 10.0);

        assertEquals(7.0, mirrored.getX(), 1e-9);
        assertEquals(2.0, mirrored.getY(), 1e-9);
    }

    @Test
    void rotatesAroundOriginByNinetyDegrees() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 1.0, 3.0, 2.0);

        PoseBoundingBox2d rotated = box.rotateAroundOrigin(Rotation2d.fromDegrees(90.0));

        assertEquals(-2.0, rotated.minX(), 1e-9);
        assertEquals(1.0, rotated.minY(), 1e-9);
        assertEquals(-1.0, rotated.maxX(), 1e-9);
        assertEquals(3.0, rotated.maxY(), 1e-9);
    }

    @Test
    void rotatesAroundCenterPivotByNinetyDegrees() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 1.0, 3.0, 2.0);

        PoseBoundingBox2d rotated = box.rotate(Rotation2d.fromDegrees(90.0), PoseBoundingBox2d.RotationPivot.CENTER);

        assertEquals(1.5, rotated.minX(), 1e-9);
        assertEquals(0.5, rotated.minY(), 1e-9);
        assertEquals(2.5, rotated.maxX(), 1e-9);
        assertEquals(2.5, rotated.maxY(), 1e-9);
    }

    @Test
    void rotatesAroundTopPivotByOneEightyDegrees() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 1.0, 3.0, 2.0);

        PoseBoundingBox2d rotated = box.rotate(Rotation2d.fromDegrees(180.0), PoseBoundingBox2d.RotationPivot.TOP);

        assertEquals(1.0, rotated.minX(), 1e-9);
        assertEquals(2.0, rotated.minY(), 1e-9);
        assertEquals(3.0, rotated.maxX(), 1e-9);
        assertEquals(3.0, rotated.maxY(), 1e-9);
    }

    @Test
    void rotatesAroundBottomLeftPivotByNinetyDegrees() {
        PoseBoundingBox2d box = new PoseBoundingBox2d(1.0, 1.0, 3.0, 2.0);

        PoseBoundingBox2d rotated =
                box.rotate(Rotation2d.fromDegrees(90.0), PoseBoundingBox2d.RotationPivot.BOTTOM_LEFT);

        assertEquals(0.0, rotated.minX(), 1e-9);
        assertEquals(1.0, rotated.minY(), 1e-9);
        assertEquals(1.0, rotated.maxX(), 1e-9);
        assertEquals(3.0, rotated.maxY(), 1e-9);
    }

    @Test
    void rotatesPointAroundCustomOriginByNinetyDegrees() {
        Translation2d point = new Translation2d(3.0, 2.0);
        Translation2d origin = new Translation2d(2.0, 2.0);

        Translation2d rotated = PoseBoundingBox2d.rotatePoint(point, Rotation2d.fromDegrees(90.0), origin);

        assertEquals(2.0, rotated.getX(), 1e-9);
        assertEquals(3.0, rotated.getY(), 1e-9);
    }
}
