package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.runtime.MechanismRuntimeLowerer;

final class MechanismPidControlTest {

    @Test
    void controlSectionRegistersProfiledPidConstraints() {
        var runtime = MechanismRuntimeLowerer.lower(
                Mechanisms.create("pid-profiled")
                        .behavior(behavior -> behavior.control(control -> control
                                .pid("profiled", pid -> pid
                                        .kp(0.2)
                                        .ki(0.0)
                                        .kd(0.0)
                                        .profiled(35.0, 120.0))))
                        .definition());

        MechanismRuntimeConfig.PidProfile resolved = runtime.controlLoopPidProfiles().get("profiled");
        assertEquals(35.0, resolved.maxVelocity(), 1e-9);
        assertEquals(120.0, resolved.maxAcceleration(), 1e-9);
        assertEquals(MechanismInputSource.Position, resolved.inputSource());
        assertEquals(1, runtime.controlLoops().size());
        assertEquals("profiled", runtime.controlLoops().get(0).name());
    }

    @Test
    void pidBuilderSupportsInputSourceSelection() {
        var runtime = MechanismRuntimeLowerer.lower(
                Mechanisms.create("pid-source")
                        .behavior(behavior -> behavior.control(control -> control
                                .pid("velPid", pid -> pid
                                        .kp(0.2)
                                        .inputSource(MechanismInputSource.Velocity))))
                        .definition());

        MechanismRuntimeConfig.PidProfile resolved = runtime.controlLoopPidProfiles().get("velPid");
        assertEquals(MechanismInputSource.Velocity, resolved.inputSource());
    }

    @Test
    void pidConstraintsRequireBothVelocityAndAcceleration() {
        assertThrows(
                IllegalArgumentException.class,
                () -> Mechanisms.create("bad")
                        .behavior(behavior -> behavior.control(control -> control
                                .pid("bad", pid -> pid.constraints(25.0, Double.NaN))))
                        .definition());
    }
}
