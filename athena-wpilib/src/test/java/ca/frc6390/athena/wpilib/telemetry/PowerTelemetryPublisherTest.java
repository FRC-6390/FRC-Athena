package ca.frc6390.athena.wpilib.telemetry;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;
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

    @Test
    void newMotorChannelsAreSpreadAcrossPublishesWhileExistingValuesStayCurrent() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (PowerTelemetryPublisher publisher = new PowerTelemetryPublisher(instance).bindBudget(1)) {
            publisher.publish(motors(12.4, 20.0), 0.0);
            assertEquals(20.0, instance.getEntry(motorDemand("one")).getDouble(-1.0));
            assertFalse(instance.getTopic(motorDemand("two")).exists());
            assertFalse(instance.getTopic(motorDemand("three")).exists());

            publisher.publish(motors(12.0, 30.0), 0.02);
            assertEquals(30.0, instance.getEntry(motorDemand("one")).getDouble(-1.0));
            assertEquals(30.0, instance.getEntry(motorDemand("two")).getDouble(-1.0));
            assertFalse(instance.getTopic(motorDemand("three")).exists());

            publisher.publish(motors(11.5, 40.0), 0.04);
            for (String name : List.of("one", "two", "three")) {
                assertEquals(40.0, instance.getEntry(motorDemand(name)).getDouble(-1.0));
            }
            assertEquals(11.5, instance.getEntry(VOLTS).getDouble(-1.0));
        } finally {
            instance.close();
        }
    }

    @Test
    void slowBusOrMotorBindingDefersOtherEntriesWithoutStarvingThem() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        AtomicLong clock = new AtomicLong();
        try (PowerTelemetryPublisher publisher = new PowerTelemetryPublisher(
                instance, () -> clock.getAndAdd(1_000_000L)).bindTimeBudget(Seconds.of(0.001))) {
            publisher.publish(motors(12.4, 20.0), 0.0);
            assertEquals(12.4, instance.getEntry(VOLTS).getDouble(-1.0));
            assertFalse(instance.getTopic(motorDemand("one")).exists(),
                    "expensive first bus binding must not also create every motor's entries");

            publisher.publish(motors(12.2, 30.0), 0.02);
            assertEquals(30.0, instance.getEntry(motorDemand("one")).getDouble(-1.0));
            assertFalse(instance.getTopic(motorDemand("two")).exists());
            publisher.publish(motors(12.0, 40.0), 0.04);
            publisher.publish(motors(11.8, 50.0), 0.06);
            for (String name : List.of("one", "two", "three")) {
                assertEquals(50.0, instance.getEntry(motorDemand(name)).getDouble(-1.0));
            }
        } finally {
            instance.close();
        }
    }

    @Test
    void diagnosticPeriodAvoidsSamplingUntilDueAndAcceptsClockReset() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        AtomicInteger samples = new AtomicInteger();
        Supplier<PowerSnapshot> source = () -> snapshot(12.0, samples.incrementAndGet(), "Declared");
        try (PowerTelemetryPublisher publisher = new PowerTelemetryPublisher(instance).readOnlyPeriod(Seconds.of(0.1))) {
            publisher.publish(source, 1.0);
            for (int i = 1; i <= 4; i++) publisher.publish(source, 1.0 + 0.02 * i);
            assertEquals(1, samples.get(), "do not build a per-motor snapshot for diagnostics between publishes");
            publisher.publish(source, 1.1);
            assertEquals(2, samples.get());
            assertEquals(2.0, instance.getEntry(DEMAND).getDouble(-1.0));
            publisher.publish(source, 0.0);
            assertEquals(3, samples.get(), "a reset simulation clock must not freeze diagnostics");
        } finally {
            instance.close();
        }
    }

    private static String motorDemand(String name) {
        return PowerTelemetryPublisher.ROOT + "/Motors/" + name + "/Demand";
    }

    private static PowerSnapshot motors(double busVolts, double demand) {
        return new PowerSnapshot(busVolts, 12.6, 0.02, 7.0, 11.9, 280.0, demand * 3.0, false,
                List.of("one", "two", "three").stream().map(name -> new PowerSnapshot.Motor(
                        name, demand, 60.0, 1.0, 30.0, 1.0, 60.0, "Declared")).toList());
    }
}
