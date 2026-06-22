package ca.frc6390.athena.vendor.choreo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class ChoreoAutoFactoryAdapterTest {
    @Test
    void createsTrajectoryCommandsThroughFactoryClient() {
        var client = new RecordingFactoryClient();
        var adapter = new ChoreoAutoFactoryAdapter(client);

        Command command = adapter.trajectoryCommand(" ScorePreload ");

        assertEquals("ScorePreload", client.trajectoryName);
        assertSame(client.trajectoryCommand, command);
    }

    @Test
    void createsSplitTrajectoryCommandsThroughFactoryClient() {
        var client = new RecordingFactoryClient();
        var adapter = new ChoreoAutoFactoryAdapter(client);

        Command command = adapter.trajectoryCommand("ScorePreload", 2);

        assertEquals("ScorePreload", client.trajectoryName);
        assertEquals(2, client.splitIndex);
        assertSame(client.splitTrajectoryCommand, command);
    }

    @Test
    void createsResetRoutineAndWarmupCommandsThroughFactoryClient() {
        var client = new RecordingFactoryClient();
        var adapter = new ChoreoAutoFactoryAdapter(client);

        assertSame(client.resetCommand, adapter.resetOdometryCommand(" ScorePreload "));
        assertEquals("ScorePreload", client.trajectoryName);

        assertSame(client.splitResetCommand, adapter.resetOdometryCommand("ScorePreload", 1));
        assertEquals(1, client.splitIndex);

        assertSame(client.routineCommand, adapter.routineCommand(" TwoPiece "));
        assertEquals("TwoPiece", client.routineName);

        assertSame(client.warmupCommand, adapter.warmupCommand());
    }

    private static final class RecordingFactoryClient implements ChoreoAutoFactoryAdapter.FactoryClient {
        private final Command trajectoryCommand = new RecordingCommand("trajectory");
        private final Command splitTrajectoryCommand = new RecordingCommand("splitTrajectory");
        private final Command resetCommand = new RecordingCommand("reset");
        private final Command splitResetCommand = new RecordingCommand("splitReset");
        private final Command routineCommand = new RecordingCommand("routine");
        private final Command warmupCommand = new RecordingCommand("warmup");
        private String trajectoryName;
        private String routineName;
        private int splitIndex;

        @Override
        public Command trajectoryCommand(String trajectoryName) {
            this.trajectoryName = trajectoryName;
            return trajectoryCommand;
        }

        @Override
        public Command trajectoryCommand(String trajectoryName, int splitIndex) {
            this.trajectoryName = trajectoryName;
            this.splitIndex = splitIndex;
            return splitTrajectoryCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName) {
            this.trajectoryName = trajectoryName;
            return resetCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName, int splitIndex) {
            this.trajectoryName = trajectoryName;
            this.splitIndex = splitIndex;
            return splitResetCommand;
        }

        @Override
        public Command routineCommand(String routineName) {
            this.routineName = routineName;
            return routineCommand;
        }

        @Override
        public Command warmupCommand() {
            return warmupCommand;
        }
    }

    private static final class RecordingCommand extends Command {
        private RecordingCommand(String name) {
            setName(name);
        }
    }
}
