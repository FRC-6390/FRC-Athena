package ca.frc6390.athena.vendor.choreo;

import static edu.wpi.first.units.Units.Seconds;

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
        PathAction previewPath = Paths.choreo("Score")
                .marker("Intake", Actions.neutral()).marker("Shoot", Actions.neutral());
        prepare(provider, previewPath);
        var preview = provider.preview(previewPath).orElseThrow();
        assertEquals(2, preview.poses().size());
        assertEquals(List.of("Intake", "Shoot"), preview.events().stream().map(event -> event.name()).toList());
        assertThrows(IllegalStateException.class, provider::runtime);
    }

    @Test
    void nativeRuntimeProducesDriveResetAndMarkerActionsWithoutCommands() {
        FakeClient client = new FakeClient(Map.of("Score", trajectory()));
        AtomicInteger resetCalls = new AtomicInteger();
        AtomicInteger followCalls = new AtomicInteger();
        Action reset = Actions.waitSeconds(Seconds.of(0.0));
        Action drive = Actions.waitSeconds(Seconds.of(1.0));
        Action intake = Actions.waitSeconds(Seconds.of(2.0));
        Action shoot = Actions.waitSeconds(Seconds.of(3.0));
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client,
                Pose2d::new,
                pose -> { resetCalls.incrementAndGet(); return reset; },
                sample -> { followCalls.incrementAndGet(); return drive; },
                () -> false);
        PathAction path = Paths.choreo("Score").resetOdometry()
                .marker("Intake", intake).marker("Shoot", shoot);
        prepare(provider, path);
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
        Action reset = Actions.waitSeconds(Seconds.of(0.0));
        ChoreoPathProvider provider = new ChoreoPathProvider(
                client,
                Pose2d::new,
                pose -> { resetCalls.incrementAndGet(); return reset; },
                sample -> { followCalls.incrementAndGet(); return Actions.neutral(); },
                () -> false);
        PathAction path = Paths.choreo("Score").resetPoseOnly();
        prepare(provider, path);
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
        prepare(provider, path);
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
        prepare(provider, path);
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
        prepare(provider, path);
        PathRuntime runtime = provider.runtime();

        runtime.initialize(path, context(0.0));
        IllegalStateException error = assertThrows(
                IllegalStateException.class, () -> runtime.initialize(path, context(0.0)));
        assertTrue(error.getMessage().contains("already running"));

        runtime.end(path, context(0.1), true);
        runtime.initialize(path, context(0.2));
    }

    @Test
    void heldLoaderDoesNotBlockCallbacksOrConsumePathTime() throws Exception {
        java.util.concurrent.CountDownLatch entered = new java.util.concurrent.CountDownLatch(1);
        java.util.concurrent.CountDownLatch release = new java.util.concurrent.CountDownLatch(1);
        ChoreoPathProvider.ChoreoClient client = new ChoreoPathProvider.ChoreoClient() {
            @Override public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String name) {
                entered.countDown();
                try { release.await(); }
                catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return Optional.empty(); }
                return Optional.of(trajectory());
            }
            @Override public List<String> trajectoryNames() { return List.of("Score"); }
        };
        List<Double> followedTimes = new ArrayList<>();
        try (ChoreoPathProvider provider = new ChoreoPathProvider(client, Pose2d::new,
                ignored -> Actions.neutral(), sample -> {
                    followedTimes.add(sample.getTimestamp()); return Actions.neutral();
                }, () -> false)) {
            PathAction path = Paths.choreo("Score");
            PathRuntime runtime = provider.runtime();
            runtime.initialize(path, context(0));
            assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            org.junit.jupiter.api.Assertions.assertTimeout(java.time.Duration.ofMillis(100), () -> {
                for (int i = 0; i < 100; i++) {
                    runtime.execute(path, context(5));
                    assertEquals(null, runtime.output(path, context(5)));
                    assertFalse(runtime.isFinished(path, context(5)));
                    assertTrue(provider.preview(path).isEmpty());
                }
            });
            long pendingRevision = provider.previewRevision();
            release.countDown();
            assertTrue(provider.awaitPrepared(java.time.Duration.ofSeconds(2)));
            assertTrue(provider.previewRevision() != pendingRevision);
            runtime.execute(path, context(5));
            runtime.output(path, context(5));
            assertEquals(List.of(0.0), followedTimes, "loading delay must not skip trajectory samples");
            assertTrue(provider.preview(path).isPresent());
            assertFalse(runtime.isFinished(path, context(5)));
        } finally { release.countDown(); }
    }

    @Test
    void failedPreparedPathKeepsItsOriginalFailureOnRepeatedInitialization() throws Exception {
        try (ChoreoPathProvider provider = new ChoreoPathProvider(new FakeClient(Map.of()), Pose2d::new,
                ignored -> Actions.neutral(), ignored -> Actions.neutral(), () -> false)) {
            PathAction path = Paths.choreo("Missing");
            provider.prepare(path);
            assertThrows(java.util.concurrent.CompletionException.class,
                    () -> provider.awaitPrepared(java.time.Duration.ofSeconds(2)));
            var first = assertThrows(java.util.concurrent.CompletionException.class,
                    () -> provider.runtime().initialize(path, context(0)));
            var retry = assertThrows(java.util.concurrent.CompletionException.class,
                    () -> provider.runtime().initialize(path, context(0.02)));
            assertTrue(first.getCause().getMessage().contains("Missing Choreo trajectory"));
            assertSame(first.getCause(), retry.getCause(), "no phantom active entry hides the missing-path error");
        }
    }

    @Test
    void throwingInitializationCallbackLeavesNoActivePathEntry() {
        AtomicInteger callbacks = new AtomicInteger();
        try (ChoreoPathProvider provider = new ChoreoPathProvider(new FakeClient(Map.of("Score", trajectory())),
                Pose2d::new, ignored -> {
                    callbacks.incrementAndGet();
                    throw new IllegalStateException("reset initialization failed");
                }, ignored -> Actions.neutral(), () -> false)) {
            PathAction path = Paths.choreo("Score").resetOdometry();
            prepare(provider, path);
            for (int retry = 0; retry < 2; retry++) {
                var failure = assertThrows(IllegalStateException.class,
                        () -> provider.runtime().initialize(path, context(0)));
                assertEquals("reset initialization failed", failure.getMessage());
            }
            assertEquals(2, callbacks.get());
        }
    }

    @Test
    void mirroredPreviewMatchesRuntimeAndAllianceChangeDoesNotFlipActivePath() {
        var mirror = new java.util.concurrent.atomic.AtomicBoolean(true);
        List<Pose2d> followed = new ArrayList<>();
        try (ChoreoPathProvider provider = new ChoreoPathProvider(
                new FakeClient(Map.of("Score", trajectory())), Pose2d::new,
                pose -> Actions.neutral(), sample -> { followed.add(sample.getPose()); return Actions.neutral(); },
                mirror::get)) {
            PathAction path = Paths.choreo("Score");
            prepare(provider, path);
            double flippedX = trajectory().sampleAt(0, true).orElseThrow().getPose().getX();
            assertEquals(flippedX, provider.preview(path).orElseThrow().poses().get(0).x().in(
                    edu.wpi.first.units.Units.Meters), 1e-9);
            PathRuntime runtime = provider.runtime();
            runtime.initialize(path, context(0));
            mirror.set(false);
            runtime.execute(path, context(0.5));
            runtime.output(path, context(0.5));
            assertEquals(trajectory().sampleAt(0.5, true).orElseThrow().getPose(), followed.get(0));
            provider.preview(path); // queue the new preview variant
            prepare(provider, path);
            assertEquals(0, provider.preview(path).orElseThrow().poses().get(0).x().in(
                    edu.wpi.first.units.Units.Meters), 1e-9);
        }
    }

    private static void prepare(ChoreoPathProvider provider, PathAction path) {
        provider.prepare(path);
        try { assertTrue(provider.awaitPrepared(java.time.Duration.ofSeconds(2))); }
        catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); throw new AssertionError(interrupted); }
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
