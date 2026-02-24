package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.lang.reflect.Method;

import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainApiConsistencyTest {

    @Test
    void driftCorrectionApiExposesOnlyPreferredSpelling() throws Exception {
        Method preferred = SwerveDrivetrain.class.getMethod("withDriftCorrection", PIDController.class);
        assertEquals(SwerveDrivetrain.class, preferred.getReturnType());
        assertThrows(
                NoSuchMethodException.class,
                () -> SwerveDrivetrain.class.getMethod("withDriftCorretion", PIDController.class));
    }
}
