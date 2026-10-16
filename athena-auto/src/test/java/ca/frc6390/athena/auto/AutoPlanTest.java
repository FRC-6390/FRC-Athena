package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Paths;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class AutoPlanTest {
    @Test
    void extractsOrderedPathsMarkersAndBothConditionalBranches() {
        Action first = Paths.choreo("first").marker("collect", Actions.waitSeconds(0.2));
        Action action = Actions.sequence()
                .run(first)
                .then(Actions.when(() -> true)
                        .run(Paths.choreo("score"))
                        .otherwise(Paths.choreo("exit")));

        AutoPlan plan = AutoPlan.inspect(action);

        assertEquals(
                java.util.List.of("first", "score", "exit"),
                plan.paths().stream().map(path -> path.name()).toList());
        assertTrue(plan.steps().stream().anyMatch(step -> step.contains("MARKER collect")));
        assertTrue(plan.steps().stream().anyMatch(step -> step.contains("TRUE PATH choreo:score")));
        assertTrue(plan.steps().stream().anyMatch(step -> step.contains("FALSE PATH choreo:exit")));
    }

    @Test
    void identifiesResetPoseOnlyPathActions() {
        var reset = Paths.choreo("starting-position").resetPoseOnly();

        assertTrue(reset.resetsOdometry());
        assertTrue(reset.onlyResetsPose());
        assertTrue(AutoPlan.inspect(reset).steps().get(0).contains("[reset pose only]"));
    }

    @Test
    void listsSettledVelocityLimitedEffortDeadlineRaceAndMarkers() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        ControlBinding<?> arm = Controls.position(motor);
        Action armMove = arm.position(1.0)
                .effort(Units.Amps.of(10))
                .untilWithin(0.1)
                .settled(0.2)
                .maxVelocity(0.5);
        Action action = Actions.sequence()
                .run(Actions.deadline(Paths.choreo("approach").marker("intake", Actions.waitSeconds(0.2)), armMove))
                .then(Actions.race(
                        Actions.waitSeconds(2.0),
                        Paths.choreo("score").marker("shoot", Actions.timeout(motor.percent(0.5), 1.0))));

        AutoPlan plan = AutoPlan.inspect(action);
        String steps = String.join("\n", plan.steps());

        assertTrue(steps.contains("DEADLINE"), steps);
        assertTrue(steps.contains("PRIMARY PATH choreo:approach"), steps);
        assertTrue(steps.contains("MARKER intake"), steps);
        assertTrue(steps.contains("POSITION 1.0 effort 10.0A"), steps);
        assertTrue(steps.contains("WITH WITHIN 0.1 settled 0.2s maxVelocity 0.5"), steps);
        assertTrue(steps.contains("RACE"), steps);
        assertTrue(steps.contains("PATH choreo:score"), steps);
        assertTrue(steps.contains("MARKER shoot"), steps);
        assertTrue(steps.contains("TIMEOUT 1.0s"), steps);
        assertEquals(
                java.util.List.of("approach", "score"),
                plan.paths().stream().map(path -> path.name()).toList());
    }
}
