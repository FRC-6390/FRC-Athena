package ca.frc6390.athena.commands.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Collection;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import ca.frc6390.athena.commands.examples.VisionCommandExamples;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import org.junit.jupiter.api.Test;

final class AlignAndDriveToTagCommandContractTest {

    @Test
    void sanitizeCameraTablesTrimsDedupesAndRejectsEmptyInput() throws Exception {
        Set<String> result = AlignAndDriveToTagPolicy.sanitizeCameraTables(
                Arrays.asList(" front ", "rear", "front", "", " ", null));

        assertEquals(Set.of("front", "rear"), result);
        assertThrows(IllegalArgumentException.class, () -> AlignAndDriveToTagPolicy.sanitizeCameraTables(List.of(" ", "")));
    }

    @Test
    void pickBetterObservationPrefersConfidenceThenDistance() throws Exception {
        VisionCamera.TargetObservation lowConfidence =
                VisionCommandExamples.targetObservation(5.0, 1.2, 1.2, 0.0, 0.3);
        VisionCamera.TargetObservation highConfidence =
                VisionCommandExamples.targetObservation(5.0, 2.5, 2.5, 0.0, 0.8);

        VisionCamera.TargetObservation chosenByConfidence =
                AlignAndDriveToTagPolicy.pickBetterObservation(lowConfidence, highConfidence);
        assertEquals(highConfidence, chosenByConfidence);

        VisionCamera.TargetObservation nearer =
                VisionCommandExamples.targetObservation(5.0, 0.8, 0.8, 0.0, 0.8);
        VisionCamera.TargetObservation chosenByDistance =
                AlignAndDriveToTagPolicy.pickBetterObservation(highConfidence, nearer);
        assertEquals(nearer, chosenByDistance);
    }

    @Test
    void toleranceSanitizersReturnAbsoluteFiniteValues() throws Exception {
        assertEquals(0.25, AlignAndDriveToTagPolicy.sanitizeToleranceMeters(-0.25), 1e-9);
        assertEquals(12.0, AlignAndDriveToTagPolicy.sanitizeToleranceDegrees(-12.0), 1e-9);
        assertEquals(0.0, AlignAndDriveToTagPolicy.sanitizeToleranceMeters(Double.NaN), 1e-9);
        assertEquals(0.0, AlignAndDriveToTagPolicy.sanitizeToleranceDegrees(Double.POSITIVE_INFINITY), 1e-9);
    }

    @Test
    void sanitizeRolesReturnsDefensiveCopy() throws Exception {
        EnumSet<CameraRole> empty = AlignAndDriveToTagPolicy.sanitizeRoles(null);
        assertTrue(empty.isEmpty());

        EnumSet<CameraRole> source = EnumSet.of(CameraRole.DRIVER);
        EnumSet<CameraRole> copy = AlignAndDriveToTagPolicy.sanitizeRoles(source);
        source.clear();
        assertFalse(copy.isEmpty());
        assertEquals(EnumSet.of(CameraRole.DRIVER), copy);
    }
}
