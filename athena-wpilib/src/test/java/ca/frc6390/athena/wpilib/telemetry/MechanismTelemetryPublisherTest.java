package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.runtime.geometry.Point2d;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class MechanismTelemetryPublisherTest {
    @Test
    void publishesNativePointArraysForAdvantageScope() {
        DashboardMechanism mechanism = new DashboardMechanism();
        Point2d[][] source = {{
                new Point2d(-1.0, -1.0), new Point2d(0.0, 1.0), new Point2d(1.0, -1.0)}};
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

            source[0] = new Point2d[] {new Point2d(0.6, -0.25)};
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
    void pressurePeriodThrottlesOnlyReadOnlyValues() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        var schema = runtime.mechanismTelemetrySchema();
        NetworkTableInstance instance = NetworkTableInstance.create();
        String temperaturePath = "/Athena/Mechanisms/dashboardMechanism/Values/temperature";
        String gainPath = "/Athena/Mechanisms/dashboardMechanism/Values/gain";
        try (MechanismTelemetryPublisher publisher = new MechanismTelemetryPublisher(instance)
                .readOnlyPeriodSeconds(0.5)) {
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

    private static final class DashboardMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        public final Action run = motor.percent(0.2);

        @Telemetry(writable = true, min = 0.0, max = 1.0)
        private double outputPercent = 0.2;

        @Telemetry("field/zone")
        private final Rectangle2d zone = Rectangle2d.of(1.0, 2.0, 3.0, 4.0);

        private double temperature = 20.0;
        private double gain = 1.0;
        private final AtomicInteger gainWrites = new AtomicInteger();
        @Telemetry
        private TelemetryValue curveSamples = TelemetryValue.constant(new double[0]);
        @Telemetry
        private TelemetryValue points = TelemetryValue.points(() -> new Point2d[0]);
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
