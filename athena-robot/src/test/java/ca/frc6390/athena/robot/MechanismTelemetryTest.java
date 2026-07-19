package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class MechanismTelemetryTest {
    @Test
    void exposesStandardMechanismGroupsAndOnlyPublicNamedActions() {
        DashboardMechanism mechanism = new DashboardMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);
        TelemetrySchema schema = runtime.mechanismTelemetrySchema();

        for (TelemetrySchema.Group group : TelemetrySchema.Group.values()) {
            assertTrue(schema.find("dashboardMechanism/" + group.path()).isPresent());
        }
        var actions = schema.find("dashboardMechanism/Actions").orElseThrow().actions();
        assertTrue(actions.containsKey("run"));
        assertTrue(actions.containsKey("calibration/zero"));
        assertFalse(actions.containsKey("internal"));

        actions.get("run").request();
        assertTrue(actions.get("run").running());
        assertEquals("run", schema.values().get("dashboardMechanism/State/ActiveAction").value());
        assertTrue(schema.values().get("dashboardMechanism/State/ActionRunning").bool());
        runtime.robotPeriodic(0.0, 0.02);
        actions.get("run").cancel();
        assertFalse(actions.get("run").running());
    }

    @Test
    void annotatedFieldsAndMethodsUseTelemetryValueRuntimeWithoutWrapperFields() {
        AnnotatedMechanism mechanism = new AnnotatedMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        assertEquals(0.5, runtime.mechanismTelemetrySchema().values()
                .get("annotatedMechanism/Values/outputPercent").number(), 1e-9);
        assertTrue(runtime.mechanismTelemetrySchema().values()
                .get("annotatedMechanism/Values/status/ready").bool());
        assertEquals("idle", runtime.mechanismTelemetrySchema().values()
                .get("annotatedMechanism/Values/state").value());
        assertEquals(7.0, runtime.mechanismTelemetrySchema().values()
                .get("annotatedMechanism/Values/custom").number(), 1e-9);
        assertEquals(TelemetryValue.Type.GEOMETRY, runtime.mechanismTelemetrySchema().values()
                .get("annotatedMechanism/Values/field/zone").type());

        runtime.mechanismTelemetrySchema().values().get("annotatedMechanism/Values/outputPercent").set(2.0);
        assertEquals(1.0, mechanism.outputPercent, 1e-9);
        runtime.mechanismTelemetrySchema().values().get("annotatedMechanism/Values/label").set("testing");
        runtime.mechanismTelemetrySchema().values().get("annotatedMechanism/Values/mode").set("active");
        assertEquals("testing", mechanism.label);
        assertEquals(TestMode.ACTIVE, mechanism.mode);
    }

    @Test
    void rejectsInvalidWritableAnnotationDuringRegistration() {
        assertThrows(IllegalArgumentException.class, () -> RobotRuntime
                .simulated(SimulationSession.create())
                .register(new InvalidAnnotatedMechanism()));
    }

    @Test
    void discoversExistingDeclarationsAndAppliesRuntimeDisable() {
        TunedMechanism mechanism = new TunedMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/motor/Config/Disabled"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Controls/control/Config/Disabled"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Values/pid/p"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Values/feedforward/velocity"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Values/position"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/motor/State/PositionRotations"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/motor/State/AppliedVoltage"));
        assertEquals("Motor", runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/motor/Info/Type").value());
        assertEquals("Encoder", runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/encoder/Info/Type").value());
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/encoder/State/AbsolutePosition"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/encoder/Setup/SetPosition"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Devices/imu/Setup/SetYaw"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Controls/control/State/FeedbackPosition"));
        assertFalse(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Controls/control/Devices/0/Config/Disabled"));
        assertFalse(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Controls/control/Config/PID/disabled"));
        assertFalse(runtime.mechanismTelemetrySchema().values().containsKey("tunedMechanism/Controls/control/Config/Feedforward/disabled"));

        runtime.mechanismTelemetrySchema().values().get("tunedMechanism/Values/pid/p").set(3.0);
        runtime.mechanismTelemetrySchema().values().get("tunedMechanism/Values/feedforward/velocity").set(1.5);
        assertEquals(3.0, mechanism.pid.p(), 1e-9);
        assertEquals(1.5, mechanism.feedforward.velocityGain(), 1e-9);

        runtime.request(mechanism.run);
        runtime.robotPeriodic(0.0, 0.02);
        assertEquals(6.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        simulation.motor(mechanism.motor).state(2.5, 4.0);
        assertEquals(2.5, runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/motor/State/PositionRotations").number(), 1e-9);
        assertEquals(4.0, runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Controls/control/State/FeedbackVelocity").number(), 1e-9);
        assertEquals(0.5, runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/motor/State/CommandValue").number(), 1e-9);

        runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/encoder/Setup/RequestedPosition").set(3.0);
        runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/encoder/Setup/SetPosition").set(true);
        assertEquals(3.0, mechanism.encoder.position(), 1e-9);
        runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/imu/Setup/RequestedYawDegrees").set(42.0);
        runtime.mechanismTelemetrySchema().values()
                .get("tunedMechanism/Devices/imu/Setup/SetYaw").set(true);
        assertEquals(42.0, mechanism.imu.yawDegrees(), 1e-9);

        runtime.mechanismTelemetrySchema().values().get("tunedMechanism/Controls/control/Config/Disabled").set(true);
        runtime.robotPeriodic(0.02, 0.02);
        assertEquals(0.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);

        runtime.mechanismTelemetrySchema().values().get("tunedMechanism/Controls/control/Config/Disabled").set(false);
        runtime.mechanismTelemetrySchema().values().get("tunedMechanism/Devices/motor/Config/Disabled").set(true);
        runtime.robotPeriodic(0.04, 0.02);
        assertEquals(0.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
    }

    @Test
    void tracesIdleMotorsDeclaredOnlyThroughAControlBinding() {
        ControlOnlyMechanism mechanism = new ControlOnlyMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .register(mechanism)
                .mechanismTraceLevel(ca.frc6390.athena.mechanism.core.MechanismTraceLevel.CAPTURE)
                .mechanismTracePeriodSeconds(0.0);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, runtime.mechanismTraces().get(0).motors().size());
        assertEquals(mechanism.control.output().defaultName(),
                runtime.mechanismTraces().get(0).motors().get(0).name());
        assertTrue(runtime.mechanismTelemetrySchema().values()
                .containsKey("controlOnlyMechanism/Controls/control/Devices/0/State/PositionRotations"));
    }

    @Test
    void keepsEmbeddedLoopControlsButOmitsTheirDirectMotorAlias() {
        EmbeddedLoopMechanism mechanism = new EmbeddedLoopMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("embeddedLoopMechanism/Devices/motor/Config/Disabled"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey("embeddedLoopMechanism/Controls/control/Config/Disabled"));
        assertTrue(runtime.mechanismTelemetrySchema().values()
                .containsKey("embeddedLoopMechanism/Controls/control/Config/PID/disabled"));
        assertFalse(runtime.mechanismTelemetrySchema().values()
                .containsKey("embeddedLoopMechanism/Controls/control/Devices/0/Config/Disabled"));
    }

    @Test
    void interpolatedActionsPublishCurveVisualizationWithoutAnotherDeclaration() {
        InterpolationTelemetryMechanism mechanism = new InterpolationTelemetryMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey(
                "interpolationTelemetryMechanism/Values/shot/Visualization/Points"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey(
                "interpolationTelemetryMechanism/Values/shot/Visualization/Curve"));
        assertTrue(runtime.mechanismTelemetrySchema().values().containsKey(
                "interpolationTelemetryMechanism/Values/shot/Visualization/Current"));
    }

    private static final class TunedMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        final EncoderDevice encoder = motor.encoder();
        final ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 2);
        final PidGains pid = PidGains.of(1.0, 0.0, 0.0);
        final FeedforwardGains feedforward = FeedforwardGains.of(0.0, 1.0, 0.0, 0.0);
        final ControlBinding control = Controls.position(motor)
                .feedback(encoder).pid(pid).feedforward(feedforward);
        final TelemetryValue position = TelemetryValue.number(encoder::position);
        final Action run = control.percent(0.5);
    }

    private static final class ControlOnlyMechanism implements Mechanism {
        final ControlBinding control = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 9));
    }

    private static final class EmbeddedLoopMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 10);
        final ControlBinding control = Controls.velocity(motor).pid(1.0, 0.0, 0.0);
    }

    private static final class InterpolationTelemetryMechanism implements Mechanism {
        private final ControlBinding velocity = Controls.velocity(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 11));
        public final Action shot = velocity
                .interpolate(ca.frc6390.athena.mechanism.interpolation.InterpolationKinds.LINEAR, () -> 2.0)
                .at(1.0, 20.0)
                .at(3.0, 40.0);
    }

    private static final class DashboardMechanism implements Mechanism {
        public final Action run = ca.frc6390.athena.mechanism.core.Actions.waitSeconds(1.0);
        @Telemetry("calibration/zero")
        private final Action zero = ca.frc6390.athena.mechanism.core.Actions.neutral();
        private final Action internal = ca.frc6390.athena.mechanism.core.Actions.neutral();
    }

    private static final class AnnotatedMechanism implements Mechanism {
        @Telemetry("field/zone")
        private static final ca.frc6390.athena.runtime.geometry.Rectangle2d ZONE =
                ca.frc6390.athena.runtime.geometry.Rectangle2d.of(1.0, 2.0, 3.0, 4.0);
        @Telemetry(writable = true, min = 0.0, max = 1.0)
        private double outputPercent = 0.5;
        @Telemetry(writable = true)
        private String label = "idle";
        @Telemetry(writable = true)
        private TestMode mode = TestMode.IDLE;
        private final TelemetryValue custom = TelemetryValue.number(7.0);

        @Telemetry("status/ready")
        private boolean ready() {
            return true;
        }

        @Telemetry
        private String state() {
            return "idle";
        }
    }

    private enum TestMode { IDLE, ACTIVE }

    private static final class InvalidAnnotatedMechanism implements Mechanism {
        @Telemetry(writable = true)
        private final double immutable = 1.0;
    }
}
