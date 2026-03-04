package ca.frc6390.athena.commands.control;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class SwerveDriveCommandSkewRegressionTest {

    @Test
    void fieldRelativeTransformMatchesWpilib() {
        double xSpeed = 2.4;
        double ySpeed = -1.1;
        double thetaSpeed = 5.2;
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
    void highOmegaFieldRelativeInputDoesNotGetPreRotated() {
        double xSpeed = 4.0;
        double ySpeed = 0.0;
        double thetaSpeed = 14.0;

        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                true,
                Rotation2d.kZero);

        assertEquals(xSpeed, actual.vxMetersPerSecond, 1e-9);
        assertEquals(ySpeed, actual.vyMetersPerSecond, 1e-9);
        assertEquals(thetaSpeed, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void nullHeadingDefaultsToZeroForFieldRelative() {
        ChassisSpeeds actual = SwerveDriveCommand.computeChassisSpeeds(
                1.0,
                2.0,
                3.0,
                true,
                null);

        assertEquals(1.0, actual.vxMetersPerSecond, 1e-9);
        assertEquals(2.0, actual.vyMetersPerSecond, 1e-9);
        assertEquals(3.0, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void highOmegaIntegratedSimulationStaysOnRequestedFieldAxis() {
        double xField = 3.5;
        double yField = 0.0;
        double theta = 12.0;
        double dt = 0.02;
        int steps = 150;

        Pose2d pose = new Pose2d();
        for (int i = 0; i < steps; i++) {
            ChassisSpeeds robotRelative = SwerveDriveCommand.computeChassisSpeeds(
                    xField,
                    yField,
                    theta,
                    true,
                    pose.getRotation());
            ChassisSpeeds discretized = ChassisSpeeds.discretize(robotRelative, dt);
            pose = pose.exp(new Twist2d(
                    discretized.vxMetersPerSecond * dt,
                    discretized.vyMetersPerSecond * dt,
                    discretized.omegaRadiansPerSecond * dt));
        }

        assertEquals(0.0, pose.getY(), 0.15);
        assertEquals(xField * dt * steps, pose.getX(), 0.30);
    }
}
