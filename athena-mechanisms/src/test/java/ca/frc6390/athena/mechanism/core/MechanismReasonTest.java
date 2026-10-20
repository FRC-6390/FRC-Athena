package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Map;
import org.junit.jupiter.api.Test;

class MechanismReasonTest {
    @Test
    void reasonSummarisesIdleRunningAndDisabledStates() {
        ReasonMechanism mechanism = new ReasonMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        TelemetryValue reason = reasonValue(scheduler.telemetrySchema());
        assertEquals("Idle", reason.value());

        scheduler.request(mechanism.move);
        scheduler.robotPeriodic(0.0, 0.02);
        assertEquals("Running: move", reason.value());

        scheduler.disabledPeriodic(0.04, 0.02);
        assertEquals("Disabled", reason.value());
    }

    @Test
    void reasonIsRebuiltOnlyWhenItsSubjectChanges() {
        ReasonMechanism mechanism = new ReasonMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        TelemetryValue reason = reasonValue(scheduler.telemetrySchema());
        scheduler.request(mechanism.move);
        scheduler.robotPeriodic(0.0, 0.02);
        Object first = reason.value();
        scheduler.robotPeriodic(0.02, 0.02);
        assertEquals(first, reason.value());
    }

    private static TelemetryValue reasonValue(TelemetrySchema schema) {
        return schema.values().entrySet().stream()
                .filter(entry -> entry.getKey().endsWith("/State/Reason"))
                .map(Map.Entry::getValue)
                .findFirst()
                .orElseThrow();
    }

    static final class ReasonMechanism implements Mechanism {
        private final ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 9), Units.Rotations);
        public final Action move = control.position(Units.Rotations.of(5.0));
    }
}
