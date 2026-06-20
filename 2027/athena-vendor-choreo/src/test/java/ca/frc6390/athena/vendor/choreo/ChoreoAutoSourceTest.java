package ca.frc6390.athena.vendor.choreo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.auto.AutoRegistry;

class ChoreoAutoSourceTest {
    @Test
    void loadsCommandSpecFromTrajectoryName() {
        var command = new ChoreoAutoSource().load("LeaveCommunity");

        assertEquals("choreo:LeaveCommunity", command.name());
    }

    @Test
    void registersSourceInRegistry() {
        AutoRegistry registry = new AutoRegistry();

        ChoreoAutos.register(registry);

        assertTrue(registry.find(ChoreoAutoSource.KEY).isPresent());
        assertEquals("choreo:ScorePreload", registry.require(ChoreoAutoSource.KEY).load("ScorePreload").name());
    }

    @Test
    void loadsRealChoreoTrajectoryThroughClient() {
        var trajectory = new Trajectory<SwerveSample>("ScorePreload", List.of(), List.of(), List.of());
        var client = new RecordingChoreoClient(trajectory);
        var source = new ChoreoAutoSource(client);

        var loaded = source.loadTrajectory(" ScorePreload ");

        assertEquals("ScorePreload", client.requestedTrajectoryName);
        assertTrue(loaded.isPresent());
        assertSame(trajectory, loaded.orElseThrow());
        assertEquals(List.of("ScorePreload", "LeaveCommunity"), source.trajectoryNames());
    }

    private static final class RecordingChoreoClient implements ChoreoAutoSource.ChoreoClient {
        private final Trajectory<? extends TrajectorySample<?>> trajectory;
        private String requestedTrajectoryName;

        private RecordingChoreoClient(Trajectory<? extends TrajectorySample<?>> trajectory) {
            this.trajectory = trajectory;
        }

        @Override
        public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String trajectoryName) {
            requestedTrajectoryName = trajectoryName;
            return Optional.of(trajectory);
        }

        @Override
        public List<String> trajectoryNames() {
            return List.of("ScorePreload", "LeaveCommunity");
        }
    }
}
