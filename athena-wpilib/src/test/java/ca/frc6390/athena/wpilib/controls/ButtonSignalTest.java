package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.HookBinding.Phase;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import java.util.List;
import org.junit.jupiter.api.Test;

class ButtonSignalTest {
    @Test
    void mapsTheFourButtonOperationsToHookPhases() {
        boolean[] active = {false};
        ButtonSignal button = new ButtonSignal("test", () -> active[0])
                .onActive(() -> {})
                .whileActive(() -> {})
                .onDeactive(() -> {})
                .whileDeactive(() -> {});

        assertEquals(
                List.of(Phase.ON_START, Phase.WHILE_ACTIVE, Phase.ON_END, Phase.WHILE_INACTIVE),
                button.binding().actions().stream().map(action -> action.phase()).toList());
        assertFalse(button.getAsBoolean());
        active[0] = true;
        assertTrue(button.getAsBoolean());
    }

    @Test
    void acceptsRuntimeActionBindings() {
        ActionBinding resetPose = context -> {};
        ButtonSignal button = new ButtonSignal("test", () -> true).onActive(resetPose);

        assertEquals(resetPose, button.binding().actions().get(0).action());
    }
}
