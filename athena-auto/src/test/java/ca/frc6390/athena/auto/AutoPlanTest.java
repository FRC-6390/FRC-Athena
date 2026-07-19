package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Paths;
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
}
