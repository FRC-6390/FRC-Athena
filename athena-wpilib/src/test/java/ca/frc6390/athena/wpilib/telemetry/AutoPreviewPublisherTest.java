package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.auto.AutoPreview;
import ca.frc6390.athena.auto.PathPreview;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.junit.jupiter.api.Test;

class AutoPreviewPublisherTest {
    @Test
    void publishesSelectedPlanAndAdvantageScopePoseArrays() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (AutoPreviewPublisher publisher = new AutoPreviewPublisher(instance);
                var path = instance.getStructArrayTopic(
                        "/Athena/Auto/Path", Pose2d.struct).subscribe(new Pose2d[0]);
                var events = instance.getStructArrayTopic(
                        "/Athena/Auto/Events", Pose2d.struct).subscribe(new Pose2d[0])) {
            publisher.publish(List.of(new AutoPreview(
                    "Score Two",
                    List.of("SEQUENCE", "  PATH choreo:out"),
                    List.of(new PathPreview("choreo:out", List.of(
                            new PathPreview.Pose(1.0, 2.0, 0.0),
                            new PathPreview.Pose(3.0, 4.0, 1.0)), List.of(
                            new PathPreview.Event("shoot", 0.5,
                                    new PathPreview.Pose(2.0, 3.0, 0.5))))))), "Score Two");

            assertEquals("Score Two", instance.getStringTopic(
                    "/Athena/Auto/Selected").subscribe("").get());
            assertEquals("Score Two", instance.getStringTopic(
                    "/Athena/Auto/Running").subscribe("").get());
            assertArrayEquals(new String[] {"SEQUENCE", "  PATH choreo:out"}, instance
                    .getStringArrayTopic("/Athena/Auto/Plan").subscribe(new String[0]).get());
            assertEquals(2, path.get().length);
            assertEquals(3.0, path.get()[1].getX(), 1e-9);
            assertEquals(1, events.get().length);
        } finally {
            instance.close();
        }
    }

    @Test
    void unchangedPreviewIdentityDoesNotRewriteNetworkTables() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        List<AutoPreview> previews = List.of(new AutoPreview("Idle", List.of("IDLE"), List.of()));
        try (AutoPreviewPublisher publisher = new AutoPreviewPublisher(instance)) {
            publisher.publish(previews, "");
            long firstChange = instance.getEntry("/Athena/Auto/Selected").getLastChange();
            publisher.publish(previews, "");
            assertEquals(firstChange, instance.getEntry("/Athena/Auto/Selected").getLastChange());
        } finally {
            instance.close();
        }
    }
}
