package ca.frc6390.athena.vendor.choreo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class ChoreoPathProviderTest {
    @Test
    void loaderNormalizesCachesAndDiscoversMetadata() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        ChoreoPathProvider provider = new ChoreoPathProvider(client, null, null, null, () -> false);
        assertEquals("Score", Paths.choreo(" Score ").name());
        assertTrue(provider.trajectory(" Score ").isPresent());
        assertTrue(provider.trajectory("Score").isPresent());
        assertEquals(1, client.loads.size());
        assertEquals(List.of("Intake", "Shoot"), provider.markerNames("Score"));
        var preview = provider.preview(Paths.choreo("Score")
                .marker("Intake", Actions.neutral()).marker("Shoot", Actions.neutral())).orElseThrow();
        assertEquals(2, preview.poses().size());
        assertEquals(List.of("Intake", "Shoot"), preview.events().stream().map(event -> event.name()).toList());
        assertThrows(IllegalStateException.class, provider::runtime);
    }

    @Test
    void nativeRuntimeProducesDriveResetAndMarkerActionsWithoutCommands() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        AtomicInteger resetCalls = new AtomicInteger();
        AtomicInteger followCalls = new AtomicInteger();
        Action reset = Actions.waitSeconds(0.0);
        Action drive = Actions.waitSeconds(1.0);
        Action intake = Actions.waitSeconds(2.0);
        Action shoot = Actions.waitSeconds(3.0);
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client,
                Pose2d::new,
                pose -> { resetCalls.incrementAndGet(); return reset; },
                sample -> { followCalls.incrementAndGet(); return drive; },
                () -> false);
        PathAction path = Paths.choreo("Score").resetOdometry()
                .marker("Intake", intake).marker("Shoot", shoot);
        PathRuntime runtime = provider.runtime();

        runtime.initialize(path, context(0.0));
        runtime.execute(path, context(0.6));
        assertSame(drive, runtime.output(path, context(0.6)));
        assertSame(reset, runtime.activeMarkers(path, context(0.6)).get("@resetOdometry"));
        assertSame(intake, runtime.activeMarkers(path, context(0.6)).get("Intake"));
        assertFalse(runtime.activeMarkers(path, context(0.6)).containsKey("Shoot"));
        assertFalse(runtime.isFinished(path, context(0.6)));

        runtime.execute(path, context(1.1));
        assertSame(shoot, runtime.activeMarkers(path, context(1.1)).get("Shoot"));
        assertFalse(runtime.isFinished(path, context(1.1)));
        assertTrue(runtime.isFinished(path, context(1.12)));
        runtime.end(path, context(1.1), false);
        assertEquals(1, resetCalls.get());
        assertEquals(1, followCalls.get());
    }

    @Test
    void resetPoseOnlyDoesNotEvaluateFollowerOutputOrPathMarkers() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        AtomicInteger resetCalls = new AtomicInteger();
        AtomicInteger followCalls = new AtomicInteger();
        Action reset = Actions.waitSeconds(0.0);
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client,
                Pose2d::new,
                pose -> { resetCalls.incrementAndGet(); return reset; },
                sample -> { followCalls.incrementAndGet(); return Actions.neutral(); },
                () -> false);
        PathAction path = Paths.choreo("Score").resetPoseOnly();
        PathRuntime runtime = provider.runtime();

        runtime.initialize(path, context(0.0));
        runtime.execute(path, context(0.5));

        assertSame(reset, runtime.activeMarkers(path, context(0.5)).get("@resetOdometry"));
        assertEquals(0, followCalls.get());
        assertEquals(1, resetCalls.get());
        assertEquals(null, runtime.output(path, context(0.5)));
        assertTrue(runtime.ownership(path).isEmpty());
        assertFalse(runtime.isFinished(path, context(0.5)));
        assertTrue(runtime.isFinished(path, context(0.52)));
    }

    @Test
    void odometryResetIsAppliedBeforeTheFirstFollowSample() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        Pose2d[] pose = {new Pose2d(5.0, 5.0, new edu.wpi.first.math.geometry.Rotation2d())};
        List<Pose2d> followPoses = new ArrayList<>();
        ca.frc6390.athena.hardware.device.MotorDevice motor = ca.frc6390.athena.hardware.device.MotorDevice.of(
                ca.frc6390.athena.api.hardware.MotorKinds.KRAKEN_X60, 1);
        Action drive = motor.percent(0.2);
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client,
                () -> pose[0],
                initial -> Actions.doOnce(() -> pose[0] = initial),
                sample -> { followPoses.add(pose[0]); return drive; },
                () -> false);
        DriveMechanism mechanism = new DriveMechanism(motor);
        PathAction path = Paths.choreo("Score").resetOdometry();
        ca.frc6390.athena.mechanism.core.MechanismScheduler scheduler =
                ca.frc6390.athena.mechanism.core.MechanismScheduler.create()
                        .register(mechanism)
                        .bindInMemoryRuntime()
                        .path(path, provider.runtime())
                        .declarationOwner(path, mechanism);

        scheduler.request(path);
        scheduler.autoPeriodic(0.0, 0.02);

        assertEquals(1, followPoses.size());
        assertEquals(0.0, followPoses.get(0).getX(), 1.0e-9, "follower used the pre-reset pose");
    }

    @Test
    void completedMarkersCanRetriggerAndPendingMarkersAreReported() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client, Pose2d::new, pose -> Actions.neutral(), sample -> Actions.neutral(), () -> false);
        PathAction path = Paths.choreo("Score").marker("Intake", Actions.neutral()).marker("Shoot", Actions.neutral());
        PathRuntime runtime = provider.runtime();

        runtime.initialize(path, context(0.0));
        assertEquals(List.of("Intake", "Shoot"), List.copyOf(runtime.pendingMarkers(path)));
        runtime.execute(path, context(0.6));
        assertEquals(List.of("Shoot"), List.copyOf(runtime.pendingMarkers(path)));
        assertTrue(runtime.activeMarkers(path, context(0.6)).containsKey("Intake"));

        runtime.markerCompleted(path, "Intake");
        assertFalse(runtime.activeMarkers(path, context(0.6)).containsKey("Intake"));
        runtime.execute(path, context(0.7));
        assertFalse(runtime.activeMarkers(path, context(0.7)).containsKey("Intake"), "a marker fires once per crossing");
    }

    @Test
    void samePathActionCannotRunTwiceConcurrently() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client, Pose2d::new, pose -> Actions.neutral(), sample -> Actions.neutral(), () -> false);
        PathAction path = Paths.choreo("Score");
        PathRuntime runtime = provider.runtime();

        runtime.initialize(path, context(0.0));
        IllegalStateException error = assertThrows(
                IllegalStateException.class, () -> runtime.initialize(path, context(0.0)));
        assertTrue(error.getMessage().contains("already running"));

        runtime.end(path, context(0.1), true);
        runtime.initialize(path, context(0.2));
    }

    private static final class DriveMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        @SuppressWarnings("unused")
        private final ca.frc6390.athena.hardware.device.MotorDevice motor;

        private DriveMechanism(ca.frc6390.athena.hardware.device.MotorDevice motor) {
            this.motor = motor;
        }
    }

    private static MechanismContext context(double time) {
        return new MechanismContext(time, time, 0.02, true, true, true);
    }

    private static Trajectory<SwerveSample> trajectory() {
        return new Trajectory<>(
                "Score",
                List.of(sample(0.0), sample(1.0)),
                List.of(0),
                List.of(new EventMarker(0.5, "Intake"), new EventMarker(1.0, "Shoot")));
    }

    private static SwerveSample sample(double time) {
        return new SwerveSample(time, time, 0, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]);
    }

    private static final class FakeClient implements ChoreoPathProvider.ChoreoClient {
        private final Map<String, Trajectory<? extends TrajectorySample<?>>> trajectories;
        private final List<String> loads = new ArrayList<>();
        private FakeClient(Map<String, Trajectory<? extends TrajectorySample<?>>> trajectories) {
            this.trajectories = trajectories;
        }
        @Override public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String name) {
            loads.add(name); return Optional.ofNullable(trajectories.get(name));
        }
        @Override public List<String> trajectoryNames() { return List.copyOf(trajectories.keySet()); }
    }
}
