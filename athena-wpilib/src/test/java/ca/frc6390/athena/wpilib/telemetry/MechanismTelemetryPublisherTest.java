package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import org.junit.jupiter.api.Test;

class MechanismTelemetryPublisherTest {
    @Test
    void dashboardWritesUpdateTunableWithoutRepublishingCustomValuesEveryLoop() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try {
            TelemetryValue gain = TelemetryValue.number(2.0);
            MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance);
            TelemetryValue value = gain;
            publisher.publish(Map.of("robot/shooter/p", value), 0.0);
            assertEquals(2.0, instance.getEntry(
                    "/Athena/Mechanisms/robot/shooter/p").getDouble(-1), 1e-9);

            instance.getEntry("/Athena/Mechanisms/robot/shooter/p").setDouble(3.5);
            publisher.publish(Map.of("robot/shooter/p", value), 0.02);
            assertEquals(3.5, gain.number(), 1e-9);
        } finally {
            instance.close();
        }
    }

    @Test
    void rectangleGeometryPublishesAClosedPoseArrayForFieldVisualization() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (var subscriber = instance.getStructArrayTopic(
                "/Athena/Mechanisms/robot/shooter/blueLeftTrenchZone",
                Pose2d.struct).subscribe(new Pose2d[0])) {
            MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance);
            publisher.publish(Map.of(
                    "robot/shooter/blueLeftTrenchZone",
                    TelemetryValue.geometry(Rectangle2d.of(1.0, 2.0, 3.0, 4.0))), 0.0);

            Pose2d[] outline = subscriber.get();
            assertEquals(5, outline.length);
            assertEquals(1.0, outline[0].getX(), 1e-9);
            assertEquals(2.0, outline[0].getY(), 1e-9);
            assertEquals(outline[0], outline[outline.length - 1]);
        } finally {
            instance.close();
        }
    }
}
