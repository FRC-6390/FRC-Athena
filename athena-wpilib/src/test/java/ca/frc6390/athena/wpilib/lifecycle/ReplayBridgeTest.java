package ca.frc6390.athena.wpilib.lifecycle;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.Retention;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ReplayBridgeTest {
    @Test
    void discoversTheRobotsReplayPolicyFieldWhateverItsVisibility() {
        ReplayPolicy policy = ReplayPolicy.of().stream(true).bufferMegabytes(8)
                .retention(Retention.maxTotalMegabytes(100));
        CustomRobot robot = new CustomRobot(policy);

        ReplayPolicy discovered = ReplayBridge.discoverPolicy(robot);

        assertSame(policy, discovered);
        assertEquals(8L * 1024 * 1024, discovered.bufferBytes());
    }

    @Test
    void usesDefaultsWhenTheRobotDeclaresNoPolicy() {
        ReplayPolicy discovered = ReplayBridge.discoverPolicy(new Object());

        assertEquals(ReplayPolicy.DEFAULT_BUFFER_BYTES, discovered.bufferBytes());
        assertEquals(false, discovered.streamEnabled());
    }

    @Test
    void nullFieldFallsBackToDefaults() {
        ReplayPolicy discovered = ReplayBridge.discoverPolicy(new NullPolicyRobot());

        assertEquals(ReplayPolicy.DEFAULT_BUFFER_BYTES, discovered.bufferBytes());
    }

    @Test
    void defaultDirectoriesDependOnSimulation() {
        assertEquals(Path.of("/home/lvuser/athena/replays"), ReplayBridge.defaultDirectory(false));
        assertEquals(Path.of(System.getProperty("java.io.tmpdir"), "athena", "replays"),
                ReplayBridge.defaultDirectory(true));
    }

    private static class BaseRobot {
        @SuppressWarnings("unused")
        private final String name = "base";
    }

    private static final class CustomRobot extends BaseRobot {
        private final ReplayPolicy replay;

        CustomRobot(ReplayPolicy replay) {
            this.replay = replay;
        }
    }

    private static final class NullPolicyRobot {
        @SuppressWarnings("unused")
        private final ReplayPolicy replay = null;
    }
}
