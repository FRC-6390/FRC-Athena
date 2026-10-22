package ca.frc6390.athena.wpilib.telemetry;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Meters;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

class MechanismTelemetryPublisherTest {
    @Test
    void publishesNativePointArraysForAdvantageScope() {
        DashboardMechanism mechanism = new DashboardMechanism();
        Translation2d[][] source = {{
                new Translation2d(-1.0, -1.0), new Translation2d(0.0, 1.0), new Translation2d(1.0, -1.0)}};
        mechanism.points = TelemetryValue.points(() -> source[0]);
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        NetworkTableInstance instance = NetworkTableInstance.create();
        String path = "/Athena/Mechanisms/dashboardMechanism/Values/points";
        try (var points = instance.getStructArrayTopic(path, Translation2d.struct)
                        .subscribe(new Translation2d[0]);
                MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.0);
            Translation2d[] published = points.get();
            assertEquals(3, published.length);
            assertEquals(-1.0, published[0].getX(), 1.0e-9);
            assertEquals(1.0, published[2].getX(), 1.0e-9);

            source[0] = new Translation2d[] {new Translation2d(0.6, -0.25)};
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.02);
            published = points.get();
            assertEquals(1, published.length);
            assertEquals(0.6, published[0].getX(), 1.0e-9);
            assertEquals(-0.25, published[0].getY(), 1.0e-9);
        } finally {
            instance.close();
        }
    }

    @Test
    void publishesWritableCustomValuesUnderTheirOwningMechanism() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.0);
            String topic = "/Athena/Mechanisms/dashboardMechanism/Values/outputPercent";
            assertEquals(0.2, instance.getEntry(topic).getDouble(-1), 1e-9);

            instance.getEntry(topic).setDouble(0.6);
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.02);
            assertEquals(0.6, mechanism.outputPercent, 1e-9);

            String restore = "/Athena/Mechanisms/dashboardMechanism/Devices/motor/Config/Restore";
            instance.getEntry(restore).setBoolean(true);
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.04);
            assertEquals(false, instance.getEntry(restore).getBoolean(true));
        } finally {
            instance.close();
        }
    }

    @Test
    void publishesGeometryAndCommandSendableAtMechanismScopedPaths() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        NetworkTableInstance instance = NetworkTableInstance.create();
        String geometryPath = "/Athena/Mechanisms/dashboardMechanism/Values/field/zone";
        try (var geometry = instance.getStructArrayTopic(geometryPath, Pose2d.struct).subscribe(new Pose2d[0]);
                MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.0);

            Pose2d[] outline = geometry.get();
            assertEquals(5, outline.length);
            assertEquals(outline[0], outline[outline.length - 1]);

            String actionPath = "/Athena/Mechanisms/dashboardMechanism/Actions/run";
            assertEquals("Command", instance.getEntry(actionPath + "/.type").getString(""));
            assertEquals("run", instance.getEntry(actionPath + "/.name").getString(""));
            assertEquals(mechanism.run.getClass().getSimpleName(),
                    instance.getEntry(actionPath + "/ActionType").getString(""));
            assertTrue(instance.getEntry(actionPath + "/Running").exists());
            assertTrue(instance.getEntry(actionPath + "/Complete").exists());

            instance.getEntry(actionPath + "/running").setBoolean(true);
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.02);
            assertTrue(runtime.isActionRunning(mechanism.run));
            instance.getEntry(actionPath + "/running").setBoolean(false);
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.04);
            assertEquals(false, runtime.isActionRunning(mechanism.run));

            var action = runtime.mechanismTelemetrySchema()
                    .find("dashboardMechanism/Actions").orElseThrow().actions().get("run");
            var command = new MechanismTelemetryPublisher.ActionCommand(action);
            command.initialize();
            assertTrue(runtime.isActionRunning(mechanism.run));
            command.end(true);
            assertEquals(false, runtime.isActionRunning(mechanism.run));
        } finally {
            instance.close();
        }
    }

    @Test
    void steadyStateSuppressesWritesAndWritableCallbacks() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            String temperaturePath = "/Athena/Mechanisms/dashboardMechanism/Values/temperature";
            String gainPath = "/Athena/Mechanisms/dashboardMechanism/Values/gain";
            publisher.publish(schema, 0.0);
            long initialChange = instance.getEntry(temperaturePath).getLastChange();

            for (int index = 1; index <= 20; index++) publisher.publish(schema, index * 0.02);
            assertEquals(initialChange, instance.getEntry(temperaturePath).getLastChange());
            assertEquals(0, mechanism.gainWrites.get());

            mechanism.temperature = 42.0;
            publisher.publish(schema, 0.5);
            assertTrue(instance.getEntry(temperaturePath).getLastChange() > initialChange);

            instance.getEntry(gainPath).setDouble(3.0);
            publisher.publish(schema, 0.52);
            publisher.publish(schema, 0.54);
            assertEquals(1, mechanism.gainWrites.get());
            assertEquals(3.0, mechanism.gain, 1e-9);

            mechanism.gain = 4.0;
            publisher.publish(schema, 0.7);
            assertEquals(4.0, instance.getEntry(gainPath).getDouble(-1.0), 1e-9);
            assertEquals(1, mechanism.gainWrites.get());
        } finally {
            instance.close();
        }
    }

    @Test
    void idleWritableValuesStillApplyADashboardWriteInTheNextPublish() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        NetworkTableInstance instance = NetworkTableInstance.create();
        String gainPath = "/Athena/Mechanisms/dashboardMechanism/Values/gain";
        String runPath = "/Athena/Mechanisms/dashboardMechanism/Actions/run/running";
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            for (int index = 0; index <= 50; index++) publisher.publish(schema, index * 0.02);
            long idle = instance.getEntry(gainPath).getLastChange();
            assertEquals(0, mechanism.gainWrites.get());

            instance.getEntry(gainPath).setDouble(2.5);
            publisher.publish(schema, 1.02);
            assertEquals(2.5, mechanism.gain, 1e-9, "the write applies in the first publish after it");
            assertEquals(1, mechanism.gainWrites.get());

            for (int index = 0; index < 20; index++) publisher.publish(schema, 1.04 + index * 0.02);
            assertEquals(1, mechanism.gainWrites.get(), "no repeated write while the entry is unchanged");
            assertTrue(instance.getEntry(gainPath).getLastChange() > idle);

            // The robot changes the value: published in the next read-only period, without a callback.
            mechanism.gain = 5.0;
            publisher.publish(schema, 2.0);
            assertEquals(5.0, instance.getEntry(gainPath).getDouble(-1.0), 1e-9);
            assertEquals(1, mechanism.gainWrites.get());

            // Action controls are watched the same way, after any number of idle publishes.
            instance.getEntry(runPath).setBoolean(true);
            publisher.publish(schema, 2.02);
            assertTrue(runtime.isActionRunning(mechanism.run));
            for (int index = 0; index < 10; index++) publisher.publish(schema, 2.04 + index * 0.02);
            instance.getEntry(runPath).setBoolean(false);
            publisher.publish(schema, 2.3);
            assertEquals(false, runtime.isActionRunning(mechanism.run));
        } finally {
            instance.close();
        }
    }

    @Test
    void lockStatusFollowsTheSharedLockCondition() {
        boolean[] inMatch = {false};
        ca.frc6390.athena.mechanism.persist.TuningLocks locks = new ca.frc6390.athena.mechanism.persist.TuningLocks()
                .define(ca.frc6390.athena.mechanism.persist.Locks.InMatch.class, () -> inMatch[0]);
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).tuningLocks(locks).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        NetworkTableInstance instance = NetworkTableInstance.create();
        String supply = "/Athena/Mechanisms/dashboardMechanism/Devices/motor/Config/SupplyCurrentLimit/Lock";
        String stator = "/Athena/Mechanisms/dashboardMechanism/Devices/motor/Config/StatorCurrentLimit/Lock";
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            publisher.publish(schema, 0.0);
            assertEquals("InMatch (unlocked)", instance.getEntry(supply).getString(""));
            assertEquals("InMatch (unlocked)", instance.getEntry(stator).getString(""));

            inMatch[0] = true;
            publisher.publish(schema, 0.02);
            assertEquals("InMatch (locked)", instance.getEntry(supply).getString(""));
            assertEquals("InMatch (locked)", instance.getEntry(stator).getString(""));

            inMatch[0] = false;
            publisher.publish(schema, 0.04);
            assertEquals("InMatch (unlocked)", instance.getEntry(supply).getString(""));
            assertEquals("InMatch (unlocked)", instance.getEntry(stator).getString(""));
        } finally {
            instance.close();
        }
    }

    @Test
    void pressurePeriodThrottlesOnlyReadOnlyValues() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        NetworkTableInstance instance = NetworkTableInstance.create();
        String temperaturePath = "/Athena/Mechanisms/dashboardMechanism/Values/temperature";
        String gainPath = "/Athena/Mechanisms/dashboardMechanism/Values/gain";
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)
                .readOnlyPeriod(Seconds.of(0.5))) {
            publisher.publish(schema, 0.0);
            mechanism.temperature = 42.0;
            instance.getEntry(gainPath).setDouble(3.0);

            publisher.publish(schema, 0.02);
            assertEquals(20.0, instance.getEntry(temperaturePath).getDouble(-1), 1e-9);
            assertEquals(3.0, mechanism.gain, 1e-9);

            publisher.publish(schema, 0.5);
            assertEquals(42.0, instance.getEntry(temperaturePath).getDouble(-1), 1e-9);
        } finally {
            instance.close();
        }
    }

    @Test
    void publishesNumericArraysForTransferCurveGraphs() {
        double[][] samples = {{-1.0, 0.0, 1.0}};
        DashboardMechanism mechanism = new DashboardMechanism();
        mechanism.curveSamples = TelemetryValue.numberArray(() -> samples[0]);
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        NetworkTableInstance instance = NetworkTableInstance.create();
        String path = "/Athena/Mechanisms/dashboardMechanism/Values/curveSamples";

        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)) {
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.0);
            org.junit.jupiter.api.Assertions.assertArrayEquals(
                    new double[] {-1.0, 0.0, 1.0}, instance.getEntry(path).getDoubleArray(new double[0]));

            samples[0] = new double[] {-1.0, -0.25, 0.0, 0.25, 1.0};
            publisher.publish(runtime.mechanismTelemetrySchema(), 0.1);
            org.junit.jupiter.api.Assertions.assertArrayEquals(
                    samples[0], instance.getEntry(path).getDoubleArray(new double[0]));
        } finally {
            instance.close();
        }
    }

    @Test
    void bindBudgetCreatesTheSchemaOverSeveralPublishesWithoutChangingTheResult() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        String values = "/Athena/Mechanisms/dashboardMechanism/Values/";
        List<String> topics = List.of(values + "outputPercent", values + "gain", values + "temperature",
                values + "points");

        NetworkTableInstance everything = NetworkTableInstance.create();
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(everything)) {
            publisher.publish(schema, 0.0);
            assertTrue(topics.stream().allMatch(topic -> everything.getTopic(topic).exists()),
                    "the default publisher binds the whole schema in one publish");
            assertEquals(0.2, everything.getEntry(topics.get(0)).getDouble(-1), 1e-9);
        } finally {
            everything.close();
        }

        NetworkTableInstance spread = NetworkTableInstance.create();
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(spread).bindBudget(1)) {
            publisher.publish(schema, 0.0);
            assertFalse(topics.stream().allMatch(topic -> spread.getTopic(topic).exists()),
                    "a budget of one binding cannot create every entry in the first publish");
            int publishes = 1;
            while (!topics.stream().allMatch(topic -> spread.getTopic(topic).exists()) && publishes < 100) {
                publisher.publish(schema, 0.02 * publishes);
                publishes++;
            }
            assertTrue(publishes > 1 && publishes < 100, "the schema is bound over " + publishes + " publishes");
            assertEquals(0.2, spread.getEntry(topics.get(0)).getDouble(-1), 1e-9);
        } finally {
            spread.close();
        }
    }

    @Test
    void bindingTimeBudgetKeepsMakingProgressWithoutDelayingBoundWritableControls() {
        DashboardMechanism mechanism = new DashboardMechanism();
        try (RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism)) {
            var schema = runtime.mechanismTelemetrySchema();
            NetworkTableInstance instance = NetworkTableInstance.create();
            // Each binding consumes the entire budget even on a fast test computer.
            AtomicLong wallClock = new AtomicLong();
            try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(
                    instance, () -> wallClock.getAndAdd(1_000_000L)).bindTimeBudget(Seconds.of(0.001))) {
                String output = "/Athena/Mechanisms/dashboardMechanism/Values/outputPercent";
                publisher.publish(schema, 0.0);
                assertFalse(instance.getTopic(output).exists(), "the entire schema must not bind in the first cycle");
                int calls = 1;
                while (!instance.getTopic(output).exists() && calls < 200) {
                    publisher.publish(schema, calls++ * 0.02);
                }
                assertTrue(calls > 1 && calls < 200, "a slow binding must not prevent eventual progress");
                instance.getEntry(output).setDouble(0.65);
                publisher.publish(schema, calls++ * 0.02);
                assertEquals(0.65, mechanism.outputPercent, 1e-9,
                        "already-bound writable controls are serviced while other entries are still binding");
                for (; calls < 200; calls++) publisher.publish(schema, calls * 0.02);
                assertTrue(instance.getTopic("/Athena/Mechanisms/dashboardMechanism/Actions/run/Complete").exists());
            } finally {
                instance.close();
            }
        }
    }

    private static final class DashboardMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        public final Action run = motor.percent(0.2);

        @Telemetry(writable = true, min = 0.0, max = 1.0)
        private double outputPercent = 0.2;

        @Telemetry("field/zone")
        private final Rectangle2d zone = Rectangle2d.of(Meters.of(1.0), Meters.of(2.0), Meters.of(3.0), Meters.of(4.0));

        private double temperature = 20.0;
        private double gain = 1.0;
        private final AtomicInteger gainWrites = new AtomicInteger();
        @Telemetry
        private TelemetryValue curveSamples = TelemetryValue.constant(new double[0]);
        @Telemetry
        private TelemetryValue points = TelemetryValue.points(() -> new Translation2d[0]);
        @Telemetry("gain")
        private final TelemetryValue gainValue = TelemetryValue.writableNumber(
                () -> gain,
                value -> {
                    gain = value;
                    gainWrites.incrementAndGet();
                });

        @Telemetry
        private double temperature() {
            return temperature;
        }
    }
}
