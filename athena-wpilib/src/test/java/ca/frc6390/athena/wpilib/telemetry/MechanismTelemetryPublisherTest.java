package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import edu.wpi.first.networktables.NetworkTableInstance;
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
}
