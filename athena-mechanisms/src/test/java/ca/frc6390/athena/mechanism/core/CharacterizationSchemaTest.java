package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.model.Characterization;
import org.junit.jupiter.api.Test;

class CharacterizationSchemaTest {
    @Test
    void characterizationFieldExpandsIntoDashboardActions() {
        CharacterizedMechanism mechanism = new CharacterizedMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        var actions = scheduler.telemetrySchema().find("characterizedMechanism/Actions").orElseThrow().actions();

        assertTrue(actions.containsKey("characterization/QuasistaticForward"));
        assertTrue(actions.containsKey("characterization/QuasistaticReverse"));
        assertTrue(actions.containsKey("characterization/DynamicForward"));
        assertTrue(actions.containsKey("characterization/DynamicReverse"));
        assertTrue(actions.containsKey("characterization/FrictionSweep"));
        assertTrue(actions.containsKey("characterization/Backlash"));
        assertTrue(actions.containsKey("characterization/Latency"));
        assertTrue(actions.containsKey("characterization/FitModel"));
        assertTrue(actions.containsKey("characterization/AdoptRunningEstimate"));
        assertTrue(actions.containsKey("characterization/Rebaseline"));
        actions.get("characterization/DynamicForward").request();
        assertTrue(actions.get("characterization/DynamicForward").running());
        actions.get("characterization/DynamicForward").cancel();
        assertFalse(actions.get("characterization/DynamicForward").running());
    }

    private static final class CharacterizedMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 7);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 8);
        private final ControlBinding<?> control = Controls.position(motor).feedback(encoder);
        public final Characterization characterization = control.characterization();
    }
}
