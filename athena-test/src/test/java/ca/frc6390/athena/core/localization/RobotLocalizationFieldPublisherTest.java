package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.junit.jupiter.api.Test;

final class RobotLocalizationFieldPublisherTest {

    @Test
    void setPlannedPathPublishesProvidedAllianceFramePosesWithoutExtraFlip() {
        RobotLocalizationFieldPublisher publisher = new RobotLocalizationFieldPublisher(() -> null);
        List<Pose2d> poses = List.of(
                new Pose2d(14.2, 5.1, Rotation2d.fromDegrees(172.0)),
                new Pose2d(13.6, 4.8, Rotation2d.fromDegrees(175.0)));

        publisher.setPlannedPath(poses);

        List<Pose2d> published = publisher.getField().getObject("AutoPlan").getPoses();
        assertEquals(poses.size(), published.size());
        for (int i = 0; i < poses.size(); i++) {
            Pose2d expected = poses.get(i);
            Pose2d actual = published.get(i);
            assertEquals(expected.getX(), actual.getX(), 1e-9);
            assertEquals(expected.getY(), actual.getY(), 1e-9);
            assertEquals(expected.getRotation().getDegrees(), actual.getRotation().getDegrees(), 1e-9);
        }
    }
}
