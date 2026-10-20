package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.junit.jupiter.api.Test;

class PowerTelemetryPublisherTest {
    private static final String VOLTS = PowerTelemetryPublisher.ROOT + "/Bus/Volts";
    private static final String SHEDDING = PowerTelemetryPublisher.ROOT + "/Bus/Shedding";
    private static final String DEMAND = PowerTelemetryPublisher.ROOT + "/Motors/krakenx60_1/Demand";
    private static final String STATUS = PowerTelemetryPublisher.ROOT + "/Motors/krakenx60_1/Status";

    private static PowerSnapshot snapshot(double busVolts, double demandAmps, String status) {
        return new PowerSnapshot(busVolts, 12.6, 0.02, 7.0, 11.9, 280.0, demandAmps, false, List.of(
                new PowerSnapshot.Motor("krakenx60_1", demandAmps, Double.POSITIVE_INFINITY, 1.0, 30.0, 1.0, 0.0,
                        status)));
    }

    @Test
    void unchangedValuesAreNotRewritten() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (PowerTelemetryPublisher publisher = new PowerTelemetryPublisher(instance)) {
            publisher.publish(snapshot(12.4, 20.0, "Declared"));
            assertEquals(12.4, instance.getEntry(VOLTS).getDouble(-1.0), 1e-9);
            assertEquals(0.0, instance.getEntry(SHEDDING).getDouble(-1.0), 1e-9);
            assertEquals(20.0, instance.getEntry(DEMAND).getDouble(-1.0), 1e-9);
            assertEquals("Declared", instance.getEntry(STATUS).getString(""));
            long volts = instance.getEntry(VOLTS).getLastChange();
            long demand = instance.getEntry(DEMAND).getLastChange();
            long status = instance.getEntry(STATUS).getLastChange();

            // A new snapshot object with the same values (one per cycle) writes nothing.
            for (int cycle = 0; cycle < 10; cycle++) {
                publisher.publish(snapshot(12.4, 20.0, "Declared"));
            }
            assertEquals(volts, instance.getEntry(VOLTS).getLastChange());
            assertEquals(demand, instance.getEntry(DEMAND).getLastChange());
            assertEquals(status, instance.getEntry(STATUS).getLastChange());
        } finally {
            instance.close();
        }
    }

    @Test
    void changedValuesPublishInTheSameCall() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (PowerTelemetryPublisher publisher = new PowerTelemetryPublisher(instance)) {
            publisher.publish(snapshot(12.4, 20.0, "Declared"));
            long volts = instance.getEntry(VOLTS).getLastChange();

            publisher.publish(snapshot(11.8, 45.0, "Power capped"));
            assertEquals(11.8, instance.getEntry(VOLTS).getDouble(-1.0), 1e-9);
            assertEquals(45.0, instance.getEntry(DEMAND).getDouble(-1.0), 1e-9);
            assertEquals("Power capped", instance.getEntry(STATUS).getString(""));
            assertTrue(instance.getEntry(VOLTS).getLastChange() > volts);

            // Not-a-number is a value like any other: it is written once, then not again.
            publisher.publish(snapshot(Double.NaN, 45.0, "Power capped"));
            assertTrue(Double.isNaN(instance.getEntry(VOLTS).getDouble(-1.0)));
            long nan = instance.getEntry(VOLTS).getLastChange();
            publisher.publish(snapshot(Double.NaN, 45.0, "Power capped"));
            assertEquals(nan, instance.getEntry(VOLTS).getLastChange());

            publisher.publish(snapshot(12.0, 45.0, "Declared"));
            assertEquals(12.0, instance.getEntry(VOLTS).getDouble(-1.0), 1e-9);
            assertEquals("Declared", instance.getEntry(STATUS).getString(""));
        } finally {
            instance.close();
        }
    }
}
