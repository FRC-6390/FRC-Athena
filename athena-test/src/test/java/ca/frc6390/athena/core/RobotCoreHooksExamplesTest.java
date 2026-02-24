package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.RobotCoreHooksExamples;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.OutputType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

final class RobotCoreHooksExamplesTest {

    @Test
    void hooksExposeLifecycleAliasesAndRegistrationOrder() {
        List<String> timeline = new ArrayList<>();
        RobotCoreHooks<SwerveDrivetrain> hooks = RobotCoreHooksExamples.createHooks(timeline);

        hooks.initBindings().forEach(binding -> binding.apply(null));
        hooks.teleopInitBindings().forEach(binding -> binding.apply(null));
        hooks.autonomousInitBindings().forEach(binding -> binding.apply(null));

        assertEquals(List.of(
                "robot.init",
                "tele.init.alias",
                "tele.init",
                "auto.init.alias",
                "auto.init"), timeline);
    }

    @Test
    void periodicLoopAndControlProfilesAreConfigured() {
        RobotCoreHooks<SwerveDrivetrain> hooks = RobotCoreHooksExamples.createHooks(new ArrayList<>());

        assertEquals(1, hooks.periodicLoopBindings().size());
        assertEquals(50.0, hooks.periodicLoopBindings().get(0).periodMs(), 1e-9);
        assertEquals(1, hooks.controlLoopBindings().size());
        assertEquals("headingHold", hooks.controlLoopBindings().get(0).name());
        assertEquals(OutputType.PERCENT, hooks.pidProfiles().get("headingPid").outputType());
        assertEquals(OutputType.VOLTAGE, hooks.feedforwardProfiles().get("driveVoltage").outputType());
    }

    @Test
    void hooksInputsArePublishedToTypedMaps() {
        RobotCoreHooks<SwerveDrivetrain> hooks = RobotCoreHooksExamples.createHooks(new ArrayList<>());

        assertTrue(hooks.inputs().containsKey("slowMode"));
        assertEquals(0.65, hooks.doubleInputs().get("speedScale").getAsDouble(), 1e-9);
        assertEquals(2, hooks.intInputs().get("selectedLane").getAsInt());
        assertEquals("pilot", hooks.stringInputs().get("driver").get());
    }

    @Test
    void duplicateControlLoopNamesAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> RobotCoreHooks.<SwerveDrivetrain>empty().hooks(h -> {
                    h.controlLoop("loop", 20.0, ctx -> 0.0);
                    h.controlLoop("loop", 20.0, ctx -> 0.0);
                }));
    }

    @Test
    void invalidProfileOutputTypesAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> RobotCoreHooks.<SwerveDrivetrain>empty().hooks(h ->
                        h.pidProfile("bad", OutputType.POSITION, 1.0, 0.0, 0.0)));

        assertThrows(IllegalArgumentException.class,
                () -> RobotCoreHooks.<SwerveDrivetrain>empty().hooks(h ->
                        h.feedforwardProfile("bad", OutputType.PERCENT, 0.1, 0.2, 0.0)));
    }
}
