package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicBoolean;

import ca.frc6390.athena.mechanisms.examples.ExampleSuperstructure;
import ca.frc6390.athena.mechanisms.examples.ExampleSuperstructure.SuperStructureState;
import org.junit.jupiter.api.Test;

final class SuperstructureExamplesTest {

    @Test
    void graphGuardsRequireReadinessBeforeScoringTransition() {
        AtomicBoolean armReady = new AtomicBoolean(false);
        AtomicBoolean elevatorReady = new AtomicBoolean(false);
        AtomicBoolean wristReady = new AtomicBoolean(false);

        StatefulMechanism<SuperStructureState> superstructure =
                ExampleSuperstructure.createConfig(armReady::get, elevatorReady::get, wristReady::get)
                        .stateMachineDelay(0.0)
                        .build();

        superstructure.stateMachine().force(SuperStructureState.SCORE_HIGH);
        superstructure.periodic();
        assertEquals(SuperStructureState.STOWED, superstructure.stateMachine().goal());

        armReady.set(true);
        elevatorReady.set(true);
        superstructure.periodic();
        assertEquals(SuperStructureState.CLEARANCE, superstructure.stateMachine().goal());

        wristReady.set(true);
        superstructure.periodic();
        assertEquals(SuperStructureState.SCORE_HIGH, superstructure.stateMachine().goal());
    }
}
