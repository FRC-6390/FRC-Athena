package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.drivetrains.swerve.examples.SwerveDrivetrainExamples;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainExamplesTest {

    @Test
    void moduleExampleBuildsExpectedPidAndFeedforward() {
        SwerveModule.SwerveModuleConfig module = SwerveDrivetrainExamples.moduleWithPidAndFeedforward();

        assertNotNull(module.rotationPID());
        assertEquals(0.3, module.rotationPID().getP(), 1e-9);
        assertEquals(0.02, module.rotationPID().getI(), 1e-9);
        assertEquals(0.001, module.rotationPID().getD(), 1e-9);

        assertNotNull(module.driveFeedforward());
        assertTrue(module.driveFeedforwardEnabled());
        assertEquals(0.16, module.driveFeedforward().getKs(), 1e-9);
        assertEquals(2.35, module.driveFeedforward().getKv(), 1e-9);
        assertEquals(0.12, module.driveFeedforward().getKa(), 1e-9);

        assertTrue(module.driveInvertedExplicit());
        assertTrue(module.encoderInvertedExplicit());
    }
}
