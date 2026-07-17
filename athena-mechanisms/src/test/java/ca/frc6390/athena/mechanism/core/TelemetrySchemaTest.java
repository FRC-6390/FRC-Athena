package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import java.util.List;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;
import org.junit.jupiter.api.Test;

class TelemetrySchemaTest {
    @Test
    void constantValuesAreExplicitReadOnlyEndpoints() {
        TelemetryValue number = TelemetryValue.constant(7.0);
        TelemetryValue bool = TelemetryValue.constant(true);
        TelemetryValue text = TelemetryValue.constant("motor");

        assertTrue(number.constant());
        assertTrue(bool.constant());
        assertTrue(text.constant());
        assertFalse(number.writable());
        assertEquals(7.0, number.number());
        assertTrue(bool.bool());
        assertEquals("motor", text.value());
        assertThrows(UnsupportedOperationException.class, () -> number.set(8.0));
        assertFalse(TelemetryValue.number(() -> 7.0).constant());
    }
    @Test
    void schemaAndFlattenedValuesAreCachedUntilRegistrationChanges() {
        MechanismScheduler scheduler = MechanismScheduler.create().register(new Root());
        TelemetrySchema first = scheduler.telemetrySchema();

        for (int index = 0; index < 10_000; index++) {
            assertSame(first, scheduler.telemetrySchema());
            assertSame(first.values(), scheduler.telemetrySchema().values());
        }

        scheduler.register(new SysIdMechanism());
        TelemetrySchema expanded = scheduler.telemetrySchema();
        assertNotSame(first, expanded);
        assertTrue(expanded.find("sysIdMechanism/Actions").isPresent());
    }

    @Test
    void traceLevelsAvoidDetailMaterializationWhenNotCaptured() {
        CommissioningMechanism mechanism = new CommissioningMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism).bindInMemoryRuntime();
        Action action = mechanism.control.percent(0.5);
        scheduler.request(action).traceLevel(MechanismTraceLevel.SUMMARY).periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, false, false), EventContext.empty());
        MechanismTraceSnapshot summary = scheduler.traceSnapshots().get(0);
        assertTrue(summary.candidates().isEmpty());
        assertTrue(summary.controls().isEmpty());
        assertTrue(summary.motors().isEmpty());

        scheduler.traceLevel(MechanismTraceLevel.OFF).periodic(
                new MechanismContext(0.02, 0.0, 0.02, true, false, false), EventContext.empty());
        assertSame(scheduler.traceSnapshots(), scheduler.traceSnapshots());
        assertTrue(scheduler.traceSnapshots().isEmpty());

        scheduler.traceLevel(MechanismTraceLevel.CAPTURE).periodic(
                new MechanismContext(0.04, 0.0, 0.02, true, false, false), EventContext.empty());
        assertTrue(!scheduler.traceSnapshots().get(0).motors().isEmpty());
    }
    @Test
    void collectionMechanismsRetainFieldAndIndexOwnership() {
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(new Root())
                .bindInMemoryRuntime();

        TelemetrySchema schema = scheduler.telemetrySchema();
        assertTrue(schema.find("root/modules/0/Devices/drive/State").isPresent());
        assertTrue(schema.find("/root//modules/0/Devices/drive/State/").isPresent());
        assertTrue(schema.find("root/modules/1/Devices/drive/Config").isPresent());
        var values = schema.values();
        String info = "root/modules/0/Devices/drive/Info/";
        assertTrue(values.get(info + "Type").constant());
        assertTrue(values.get(info + "Kind").constant());
        assertTrue(values.get(info + "Id").constant());
        assertTrue(values.get(info + "Bus").constant());
        assertTrue(values.get(info + "Follower").constant());
        assertFalse(values.get("root/modules/0/Devices/drive/Config/NeutralMode").constant());
    }

    @Test
    void motorRuntimeConfigurationAppliesAndRestoresDeclaredValues() {
        Root root = new Root();
        MechanismScheduler scheduler = MechanismScheduler.create().register(root).bindInMemoryRuntime();
        var values = scheduler.telemetrySchema().values();
        String config = "root/modules/0/Devices/drive/Config/";

        values.get(config + "NeutralMode").set("BRAKE");
        values.get(config + "Inverted").set(true);
        values.get(config + "SupplyCurrentLimit").set(35.0);
        assertEquals("BRAKE", values.get(config + "NeutralMode").value());
        assertTrue(values.get(config + "Inverted").bool());
        assertEquals("Applied", values.get(config + "Status").value());

        values.get(config + "Restore").set(true);
        assertEquals("COAST", values.get(config + "NeutralMode").value());
        assertEquals(0.0, values.get(config + "SupplyCurrentLimit").number());
    }

    @Test
    void publicSysIdRoutineExpandsIntoFourDashboardActions() {
        SysIdMechanism mechanism = new SysIdMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        var actions = scheduler.telemetrySchema().find("sysIdMechanism/Actions").orElseThrow().actions();

        assertTrue(actions.containsKey("sysId/QuasistaticForward"));
        assertTrue(actions.containsKey("sysId/QuasistaticReverse"));
        assertTrue(actions.containsKey("sysId/DynamicForward"));
        assertTrue(actions.containsKey("sysId/DynamicReverse"));
        actions.get("sysId/DynamicForward").request();
        assertTrue(actions.get("sysId/DynamicForward").running());
        actions.get("sysId/DynamicForward").cancel();
    }

    @Test
    void controlCommissioningPercentAndConstraintsAreWritableAndEnforced() {
        CommissioningMechanism mechanism = new CommissioningMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        TelemetrySchema schema = scheduler.telemetrySchema();
        var values = schema.values();
        String config = "commissioningMechanism/Controls/control/Config/Constraints/";
        String test = "commissioningMechanism/Controls/control/Test/";

        assertEquals(-1.0, values.get(config + "MinimumPosition").number());
        assertEquals(2.0, values.get(config + "MaximumPosition").number());
        assertEquals(5.0, values.get(config + "MaximumVelocity").number());
        assertEquals(3.0, values.get(config + "MaximumAcceleration").number());
        values.get(config + "MaximumVelocity").set(4.0);
        values.get(config + "MaximumAcceleration").set(2.0);
        values.get(config + "MinimumPosition").set(-2.0);
        values.get(config + "MaximumPosition").set(3.0);
        values.get(test + "Percent").set(0.8);

        var run = schema.find("commissioningMechanism/Controls/control/Test")
                .orElseThrow().actions().get("RunPercent");
        run.request();
        scheduler.periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, false, false),
                EventContext.empty());
        assertEquals(0.8, scheduler.traceSnapshots().get(0).motors().get(0).commandValue(), 1e-9);
        run.cancel();

        Action beyondMaximum = mechanism.control.position(10.0);
        scheduler.request(beyondMaximum).periodic(
                new MechanismContext(0.02, 0.0, 0.02, true, false, false),
                EventContext.empty());
        assertEquals(2.0, scheduler.traceSnapshots().get(0).controls().get(0).goal(), 1e-9);
        scheduler.cancel(beyondMaximum);

        values.get("commissioningMechanism/Controls/control/Config/Restore").set(true);
        assertEquals(5.0, values.get(config + "MaximumVelocity").number());
        assertEquals(0.0, values.get(test + "Percent").number());
    }

    @Test
    void velocityMotionLimitsClampAndSlewTargets() {
        VelocityCommissioningMechanism mechanism = new VelocityCommissioningMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        var values = scheduler.telemetrySchema().values();
        String config = "velocityCommissioningMechanism/Controls/control/Config/Constraints/";
        assertEquals(10.0, values.get(config + "MaximumVelocity").number());
        assertEquals(2.0, values.get(config + "MaximumAcceleration").number());

        Action target = mechanism.control.velocity(20.0);
        scheduler.request(target).periodic(
                new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());
        var first = scheduler.traceSnapshots().get(0).controls().get(0);
        assertEquals(10.0, first.transformedValue(), 1e-9);
        assertEquals(0.2, first.referenceVelocity(), 1e-9);

        scheduler.periodic(
                new MechanismContext(0.1, 0.0, 0.1, true, false, false), EventContext.empty());
        assertEquals(0.4,
                scheduler.traceSnapshots().get(0).controls().get(0).referenceVelocity(), 1e-9);
    }

    private static final class Root implements Mechanism {
        private final List<Module> modules = List.of(new Module(1), new Module(2));
    }

    private static final class Module implements MechanismTemplate {
        private final MotorDevice drive;
        private Module(int id) { drive = MotorDevice.of(MotorKinds.KRAKEN_X60, id); }
    }

    private static final class SysIdMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
        private final ControlBinding control = Controls.position(motor).feedback(motor.encoder());
        public final ControlSysId sysId = control.sysId();
    }

    private static final class CommissioningMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 4);
        private final ControlBinding control = Controls.position(motor)
                .feedback(motor.encoder())
                .constraints(
                        Constraints.range(Range.of(-1.0, 2.0)),
                        Constraints.motion(5.0, 3.0),
                        Constraints.clamp(9.0));
    }

    private static final class VelocityCommissioningMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 5);
        private final ControlBinding control = Controls.velocity(motor)
                .feedback(motor.encoder())
                .constraints(
                        Constraints.motion(10.0, 2.0),
                        Constraints.clamp(9.6));
    }
}
