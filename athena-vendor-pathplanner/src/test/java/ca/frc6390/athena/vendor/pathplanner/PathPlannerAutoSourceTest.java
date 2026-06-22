package ca.frc6390.athena.vendor.pathplanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.auto.AutoRegistry;

class PathPlannerAutoSourceTest {
    @Test
    void loadsCommandSpecFromPathName() {
        var command = new PathPlannerAutoSource().load("LeaveCommunity");

        assertEquals("pathplanner:LeaveCommunity", command.name());
    }

    @Test
    void registersSourceInRegistry() {
        AutoRegistry registry = new AutoRegistry();

        PathPlannerAutos.register(registry);

        assertTrue(registry.find(PathPlannerAutoSource.KEY).isPresent());
        assertEquals("pathplanner:ScorePreload", registry.require(PathPlannerAutoSource.KEY).load("ScorePreload").name());
    }

    @Test
    void loadsRealWpilibCommandThroughPathPlannerClient() {
        var client = new RecordingPathPlannerClient();
        var source = new PathPlannerAutoSource(client);

        Command command = source.loadCommand(" ScorePreload ");

        assertEquals("ScorePreload", client.requestedAutoName);
        assertEquals(client.command, command);
        assertEquals(List.of("ScorePreload", "LeaveCommunity"), source.autoNames());
    }

    private static final class RecordingPathPlannerClient implements PathPlannerAutoSource.PathPlannerClient {
        private final Command command = new InstantCommand();
        private String requestedAutoName;

        @Override
        public Command buildAuto(String autoName) {
            requestedAutoName = autoName;
            return command;
        }

        @Override
        public List<String> autoNames() {
            return List.of("ScorePreload", "LeaveCommunity");
        }
    }
}
