package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class MechanismControlLoopDisableTest {

    @Test
    void pidDisableStopsNamedPidLoopOutputImmediately() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.pid("hold", p -> p.kp(1.0).ki(0.0).kd(0.0)).periodic("hold"));

        Mechanism mechanism = cfg.build();
        mechanism.control().setpoint(2.0);
        mechanism.update();
        assertTrue(Math.abs(mechanism.output()) > 1e-9);

        mechanism.control().pidEnabled(false);
        mechanism.update();
        assertEquals(0.0, mechanism.output(), 1e-9);
    }

    @Test
    void feedforwardDisableStopsNamedFeedforwardLoopOutputImmediately() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.ff("vel", ff -> ff
                .simple(0.1, 0.5, 0.0)
                .source(MechanismConfig.InputSource.setpoint))
                .periodic("vel"));

        Mechanism mechanism = cfg.build();
        mechanism.control().setpoint(4.0);
        mechanism.update();
        assertTrue(Math.abs(mechanism.output()) > 1e-9);

        mechanism.control().feedforwardEnabled(false);
        mechanism.update();
        assertEquals(0.0, mechanism.output(), 1e-9);
    }

    @Test
    void disableAllControlLoopsTurnsOffCustomAndNamedLoops() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.controlLoop("custom", 20.0, ctx -> 1.25));

        Mechanism mechanism = cfg.build();
        mechanism.update();
        assertEquals(1.25, mechanism.output(), 1e-9);

        mechanism.disableAllControlLoops();
        mechanism.update();

        assertEquals(0.0, mechanism.output(), 1e-9);
        assertFalse(mechanism.loops().enabled("custom"));
    }

    @Test
    void disableAllHooksStopsPeriodicHookExecution() {
        AtomicInteger runs = new AtomicInteger();
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.hooks(h -> h.onRobotPeriodic(mech -> runs.incrementAndGet()));

        Mechanism mechanism = cfg.build();
        mechanism.periodic();
        assertEquals(1, runs.get());

        mechanism.disableAllHooks();
        mechanism.periodic();
        assertEquals(1, runs.get());
    }

    @Test
    void feedforwardBuilderSupportsTypedProfiles() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.ff("ff", ff -> ff.arm(0.2, 0.4, 0.6, 0.8).tolerance(0.05)));

        MechanismConfig.FeedforwardProfile profile = cfg.controlLoopFeedforwardProfiles().get("ff");
        assertEquals(MechanismConfig.FeedforwardType.ARM, profile.type());
        assertEquals(0.2, profile.kS(), 1e-9);
        assertEquals(0.4, profile.kG(), 1e-9);
        assertEquals(0.6, profile.kV(), 1e-9);
        assertEquals(0.8, profile.kA(), 1e-9);
        assertEquals(0.05, profile.tolerance(), 1e-9);
    }
}
