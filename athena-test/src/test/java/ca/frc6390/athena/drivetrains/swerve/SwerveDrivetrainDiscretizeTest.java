package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainDiscretizeTest {

    @Test
    void discretizeMatchesWpilibForPositiveDt() {
        ChassisSpeeds input = new ChassisSpeeds(3.8, -1.4, 4.2);
        double dtSeconds = 0.02;

        ChassisSpeeds expected = ChassisSpeeds.discretize(input, dtSeconds);
        ChassisSpeeds actual = SwerveDrivetrain.discretizeChassisSpeeds(input, dtSeconds);

        assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void discretizeReturnsCopyForInvalidDt() {
        ChassisSpeeds input = new ChassisSpeeds(2.3, -0.7, 1.1);

        ChassisSpeeds fromZeroDt = SwerveDrivetrain.discretizeChassisSpeeds(input, 0.0);
        ChassisSpeeds fromNaNDt = SwerveDrivetrain.discretizeChassisSpeeds(input, Double.NaN);

        assertEquals(input.vxMetersPerSecond, fromZeroDt.vxMetersPerSecond, 1e-9);
        assertEquals(input.vyMetersPerSecond, fromZeroDt.vyMetersPerSecond, 1e-9);
        assertEquals(input.omegaRadiansPerSecond, fromZeroDt.omegaRadiansPerSecond, 1e-9);
        assertNotSame(input, fromZeroDt);

        assertEquals(input.vxMetersPerSecond, fromNaNDt.vxMetersPerSecond, 1e-9);
        assertEquals(input.vyMetersPerSecond, fromNaNDt.vyMetersPerSecond, 1e-9);
        assertEquals(input.omegaRadiansPerSecond, fromNaNDt.omegaRadiansPerSecond, 1e-9);
        assertNotSame(input, fromNaNDt);
    }

    @Test
    void discretizeHandlesNullInput() {
        ChassisSpeeds actual = SwerveDrivetrain.discretizeChassisSpeeds(null, 0.02);

        assertEquals(0.0, actual.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, actual.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, actual.omegaRadiansPerSecond, 1e-9);
    }
}
