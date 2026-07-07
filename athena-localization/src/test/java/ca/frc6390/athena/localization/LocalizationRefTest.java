package ca.frc6390.athena.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.ref.ActionContext;
import ca.frc6390.athena.localization.config.Localizations;
import ca.frc6390.athena.localization.ref.FieldBounds;
import ca.frc6390.athena.localization.ref.LocalizationFilters;
import ca.frc6390.athena.localization.ref.LocalizationRef;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import org.junit.jupiter.api.Test;

class LocalizationRefTest {
    @Test
    void localizationRefsComposeIntermediateResults() {
        LocalizationRef leftVision = Localizations.vision()
                .input(Measurements.measurement(() -> Measurements
                        .pose(new PoseSnapshot(1.0, 2.0, 0.0))
                        .stdDevs(MeasurementStdDevs.of(0.5, 0.5, 1.0))));
        LocalizationRef rightVision = Localizations.vision()
                .input(Measurements.measurement(() -> Measurements
                        .pose(new PoseSnapshot(3.0, 4.0, 0.0))
                        .stdDevs(MeasurementStdDevs.of(1.0, 1.0, 1.0))));
        LocalizationRef combinedVision = Localizations.weightedAverage()
                .input(leftVision, rightVision);

        PoseSnapshot pose = combinedVision.pose();

        assertEquals(1.4, pose.xMeters(), 1.0e-9);
        assertEquals(2.4, pose.yMeters(), 1.0e-9);
    }

    @Test
    void filtersRejectMeasurementsBeforeEstimation() {
        LocalizationRef field = Localizations.vision()
                .input(Measurements.measurements(() -> java.util.List.of(
                        Measurements.pose(new PoseSnapshot(1.0, 1.0, 0.0)).visionMetadata(0.0, 2),
                        Measurements.pose(new PoseSnapshot(10.0, 10.0, 0.0)).visionMetadata(0.0, 1))))
                .filter(LocalizationFilters.tagCountAtLeast(2))
                .filter(FieldBounds.field(5.0, 5.0));

        assertEquals(1.0, field.pose().xMeters(), 1.0e-9);
        assertEquals(1, field.latest().orElseThrow().acceptedMeasurements().size());
        assertEquals(1, field.latest().orElseThrow().rejectedMeasurements().size());
    }

    @Test
    void localizationRefsCanResetDirectlyFromOtherRefs() {
        LocalizationRef odometry = Localizations.odometry()
                .input(Measurements.poseAndSpeeds(
                        () -> new PoseSnapshot(2.0, 3.0, 0.25),
                        () -> new RobotVelocity(1.0, 0.0, 0.1)));
        LocalizationRef field = Localizations.latestValid();

        field.resetTo(odometry).apply(ActionContext.empty());

        assertEquals(2.0, field.pose().xMeters(), 1.0e-9);
        assertEquals(3.0, field.pose().yMeters(), 1.0e-9);
        assertTrue(field.inputs().isEmpty());
    }

    @Test
    void metadataStaysOnLocalizationRef() {
        LocalizationRef field = Localizations.kalman()
                .name("field")
                .publishNetworkTables();

        assertEquals("kalman", field.estimatorName());
        assertEquals("field", field.debugName());
        assertTrue(field.publishNetworkTablesEnabled());
    }
}
