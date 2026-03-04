package ca.frc6390.athena.commands.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class SwerveDriveCommandSkewRegressionTest {

    @Test
    void fieldRelativeTransformMatchesWpilibWhenNotTurning() {
        double xSpeed = 2.4;
        double ySpeed = -1.1;
        double thetaSpeed = 0.0;
        Rotation2d heading = Rotation2d.fromDegrees(37.0);

        ChassisSpeeds expected = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                heading);
        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                true,
                heading);

        assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void robotRelativeTransformIsPassThrough() {
        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                1.9,
                -0.6,
                3.7,
                false,
                Rotation2d.fromDegrees(120.0));

        assertEquals(1.9, actual.vxMetersPerSecond, 1e-9);
        assertEquals(-0.6, actual.vyMetersPerSecond, 1e-9);
        assertEquals(3.7, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void turningAppliesFieldRelativeLeadCompensation() {
        double xSpeed = 4.0;
        double ySpeed = 0.0;
        double thetaSpeed = 14.0;
        double leadSeconds = 0.026;

        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                true,
                Rotation2d.kZero,
                leadSeconds);

        assertTrue(actual.vxMetersPerSecond < xSpeed);
        assertTrue(Math.abs(actual.vyMetersPerSecond) > 1e-6);
        assertEquals(thetaSpeed, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void turningWithoutLeadMatchesWpilib() {
        double xSpeed = 4.0;
        double ySpeed = -0.8;
        double thetaSpeed = 5.0;
        Rotation2d heading = Rotation2d.fromDegrees(18.0);

        ChassisSpeeds expected = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                heading);
        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                true,
                heading);

        assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void nullHeadingDefaultsToZeroForFieldRelative() {
        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                1.0,
                2.0,
                0.0,
                true,
                null);

        assertEquals(1.0, actual.vxMetersPerSecond, 1e-9);
        assertEquals(2.0, actual.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, actual.omegaRadiansPerSecond, 1e-9);
    }
}
