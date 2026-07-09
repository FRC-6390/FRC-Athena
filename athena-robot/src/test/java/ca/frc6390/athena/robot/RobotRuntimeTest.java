package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.localization.pipeline.LocalizationPipeline;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.CameraAdapters;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class RobotRuntimeTest {
    @Test
    void discoversHardwareVendorBackendsFromServiceDescriptorsOnRootClasspath() {
        BackendRegistry.setGlobal(null);
        BackendRegistry registry = BackendRegistry.discover();

        assertTrue(registry.motorBackendFor(MotorKinds.TALON_FX).isPresent());
        assertTrue(registry.motorBackendFor(MotorKinds.SPARK_MAX_BRUSHLESS).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.CANCODER).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.REV_THROUGH_BORE).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.PIGEON_2).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.NAVX).isPresent());
    }

    @Test
    void discoversVisionVendorAdaptersFromServiceDescriptorsOnRootClasspath() {
        List<String> adapterNames = CameraAdapters.discover().stream()
                .map(adapter -> adapter.getClass().getName())
                .toList();

        assertTrue(adapterNames.contains("ca.frc6390.athena.vendor.limelight.LimelightCameraAdapter"));
        assertTrue(adapterNames.contains("ca.frc6390.athena.vendor.photonvision.PhotonVisionCameraAdapter"));
    }

    @Test
    void inlineRuntimeWorkersRunWhenDueFromMainPeriodic() {
        AtomicInteger runs = new AtomicInteger();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .workers(RuntimeWorkers.inline(RuntimeWorker.every("counter", 0.05, runs::incrementAndGet)));

        runtime.robotPeriodic(0.0, 0.02);
        runtime.robotPeriodic(0.02, 0.02);
        runtime.robotPeriodic(0.05, 0.02);

        assertEquals(2, runs.get());
    }

    @Test
    void inlineRuntimeWorkerFailuresAreRecordedAndDoNotStopPeriodic() {
        AtomicInteger runs = new AtomicInteger();
        RuntimeWorkers workers = RuntimeWorkers.inline(RuntimeWorker.every("unstable", 0.01, () -> {
            runs.incrementAndGet();
            throw new IllegalStateException("boom");
        }));
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).workers(workers);

        runtime.robotPeriodic(0.0, 0.02);
        runtime.robotPeriodic(0.02, 0.02);

        assertEquals(2, runs.get());
        assertEquals(2, workers.failures().size());
        assertEquals("unstable", workers.failures().get(0).worker().name());
    }

    @Test
    void runtimeWorkerFailureHandlerRunsOutsideWorkerLock() {
        AtomicInteger failuresSeenByHandler = new AtomicInteger();
        RuntimeWorkers[] holder = new RuntimeWorkers[1];
        RuntimeWorkers workers = RuntimeWorkers.inline(RuntimeWorker.every("unstable", 0.01, () -> {
            throw new IllegalStateException("boom");
        })).onFailure(failure -> {
            assertEquals(false, Thread.holdsLock(holder[0]));
            failuresSeenByHandler.set(holder[0].failures().size());
        });
        holder[0] = workers;
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).workers(workers);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, failuresSeenByHandler.get());
    }

    @Test
    void asyncRuntimeWorkersRunOnlyAfterExplicitStart() throws Exception {
        CountDownLatch latch = new CountDownLatch(1);
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .workers(RuntimeWorkers.async(
                        executor,
                        RuntimeWorker.every("async", 0.001, latch::countDown)));
        try {
            assertEquals(1, latch.getCount());

            runtime.startWorkers();

            assertTrue(latch.await(1, TimeUnit.SECONDS));
        } finally {
            runtime.stopWorkers();
            executor.shutdownNow();
        }
    }

    @Test
    void asyncRuntimeWorkerFailuresAreRecordedAndFutureRunsContinue() throws Exception {
        CountDownLatch secondRun = new CountDownLatch(2);
        AtomicInteger runs = new AtomicInteger();
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        RuntimeWorkers workers = RuntimeWorkers.async(
                executor,
                RuntimeWorker.every("async-unstable", 0.001, () -> {
                    secondRun.countDown();
                    if (runs.incrementAndGet() == 1) {
                        throw new IllegalStateException("first");
                    }
                }));
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).workers(workers);
        try {
            runtime.startWorkers();

            assertTrue(secondRun.await(1, TimeUnit.SECONDS));
            assertEquals(1, workers.failures().size());
            assertTrue(runs.get() >= 2);
        } finally {
            runtime.stopWorkers();
            executor.shutdownNow();
        }
    }


    @Test
    void ownsCommandAndAutoLifecycleFromOneRoot() {
        AtomicInteger commandExecute = new AtomicInteger();
        AtomicInteger commandEnd = new AtomicInteger();
        AtomicInteger autoExecute = new AtomicInteger();
        AtomicInteger autoEnd = new AtomicInteger();
        CommandAction command = CommandAction.create("drive")
                .onExecute(commandExecute::incrementAndGet)
                .onEnd(commandEnd::incrementAndGet)
                .build();
        CommandAction auto = CommandAction.create("auto")
                .onExecute(autoExecute::incrementAndGet)
                .onEnd(autoEnd::incrementAndGet)
                .build();
        AutoRuntime autoRuntime = Autos.runtime(Autos.routine("auto", auto));
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .schedule(command)
                .auto(autoRuntime, null);

        runtime.robotPeriodic(0.0, 0.02);
        runtime.autoPeriodic(0.02, 0.02);
        runtime.disabledPeriodic(0.04, 0.02);

        assertEquals(2, commandExecute.get());
        assertEquals(1, commandEnd.get());
        assertEquals(1, autoExecute.get());
        assertEquals(1, autoEnd.get());
    }

    @Test
    void genericAutonomousPeriodicRunsAutoOnce() {
        AtomicInteger autoExecute = new AtomicInteger();
        CommandAction auto = CommandAction.create("auto")
                .onExecute(autoExecute::incrementAndGet)
                .build();
        AutoRuntime autoRuntime = Autos.runtime(Autos.routine("auto", auto));
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).auto(autoRuntime, null);

        runtime.periodic(
                new ca.frc6390.athena.mechanism.core.MechanismContext(0.0, 0.0, 0.02, true, true, false),
                new ca.frc6390.athena.mechanism.core.EventContext(
                        0.0,
                        0.02,
                        ca.frc6390.athena.mechanism.core.LifecycleMode.AUTONOMOUS,
                        ca.frc6390.athena.mechanism.core.LifecyclePhase.PERIODIC,
                        true,
                        false));

        assertEquals(1, autoExecute.get());
    }

    @Test
    void exposesSimulationBackedHardwareGraph() {
        SimulationSession session = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(session);

        assertNotNull(runtime.hardwareGraph());
        assertEquals(session, runtime.simulationSession());
    }

    @Test
    void exposesLatestHardwareRefreshFailuresFromRootRuntime() {
        FailingRefreshMotorBackend backend = new FailingRefreshMotorBackend();
        MotorMechanism mechanism = new MotorMechanism();
        RobotRuntime runtime = RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend)))
                .register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        backend.handle.failRefresh = true;
        runtime.robotPeriodic(0.02, 0.02);

        assertEquals(1, runtime.hardwareRefreshFailures().size());
        assertEquals(HardwareIdentity.motor(mechanism.motor), runtime.hardwareRefreshFailures().get(0).identity());
    }

    @Test
    void registrationActivatesFollowerThatIsNotTargetedByAnAction() {
        FollowerMotorBackend backend = new FollowerMotorBackend();
        FollowerMechanism mechanism = new FollowerMechanism();

        RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend)))
                .register(mechanism);

        assertEquals(2, backend.created);
        assertSame(backend.leader, backend.follower.followLeader);
    }

    @Test
    void genericSimulationPeriodicStepsSimulationSessionOnce() {
        MotorMechanism mechanism = new MotorMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.periodic(
                new ca.frc6390.athena.mechanism.core.MechanismContext(0.0, 0.0, 0.5, true, false, true),
                new ca.frc6390.athena.mechanism.core.EventContext(
                        0.0,
                        0.5,
                        ca.frc6390.athena.mechanism.core.LifecycleMode.SIMULATION,
                        ca.frc6390.athena.mechanism.core.LifecyclePhase.PERIODIC,
                        true,
                        true));

        MotorHandle handle = simulation.motor(mechanism.motor);
        assertEquals(0.5, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(1.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
    }

    @Test
    void convenienceSimulationPeriodicStepsSimulationSessionOnce() {
        MotorMechanism mechanism = new MotorMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 0.5);

        MotorHandle handle = simulation.motor(mechanism.motor);
        assertEquals(0.5, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(1.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
    }

    @Test
    void simulationPeriodicDoesNotEvaluateMechanismActionsAgain() {
        AtomicInteger evaluations = new AtomicInteger();
        CountingOutputMechanism mechanism = new CountingOutputMechanism(evaluations);
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 0.02);

        assertEquals(1, evaluations.get());
    }

    @Test
    void registerMountsMechanismSimulationModelsIntoSession() {
        SimulatedMotorMechanism mechanism = new SimulatedMotorMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 0.2);

        assertEquals(1, simulation.registeredModels().size());
        assertTrue(simulation.encoder(mechanism.encoder).positionRotations() > 0.0);
        assertTrue(simulation.encoder(mechanism.encoder).velocityRotationsPerSecond() > 0.0);
    }

    @Test
    void refreshesVisionGraphsDuringRobotPeriodic() {
        AtomicInteger reads = new AtomicInteger();
        Measurement measurement = Measurements.custom("target", null);
        CameraDevice camera = Cameras.photonVision("front").bindTargets(() -> {
            reads.incrementAndGet();
            return List.of(measurement);
        });
        VisionGraph vision = VisionGraph.of(camera);
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).vision(vision);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(List.of(measurement), vision.targetMeasurements());
    }

    @Test
    void mountsVendorKindCameraDeclarationsThroughRootRuntime() {
        AtomicInteger limelightReads = new AtomicInteger();
        AtomicInteger photonReads = new AtomicInteger();
        Measurement limelightTarget = Measurements.custom("limelight-target", null);
        Measurement photonTarget = Measurements.custom("photon-target", null);
        CameraDevice limelight = Cameras.limelight("limelight-front").bindTargets(() -> {
            limelightReads.incrementAndGet();
            return List.of(limelightTarget);
        });
        CameraDevice photon = Cameras.photonVision("photon-front").bindTargets(() -> {
            photonReads.incrementAndGet();
            return List.of(photonTarget);
        });
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).cameras(limelight, photon);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, limelightReads.get());
        assertEquals(1, photonReads.get());
    }

    @Test
    void refreshesLocalizationSnapshotsDuringRobotPeriodic() {
        AtomicInteger reads = new AtomicInteger();
        LocalizationPipeline localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5)));
                });
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).localization(localization);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(2.0, localization.pose().xMeters(), 1.0e-9);
        assertEquals(2, reads.get());
    }

    @Test
    void localizationRefreshPolicyAppliesRootAgeAndDisabledRules() {
        AtomicInteger reads = new AtomicInteger();
        LocalizationPipeline localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(new TestPoseMeasurement(new PoseSnapshot(4.0, 5.0, 0.25), 1.0));
                });
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .localization(localization)
                .localizationMaxAge(1.0);

        runtime.robotPeriodic(5.0, 0.02);
        runtime.disabledPeriodic(5.02, 0.02);
        runtime.localizationRefreshWhileDisabled(true).disabledPeriodic(5.04, 0.02);

        assertTrue(runtime.localizationMeasurements().isEmpty());
        assertEquals(2, reads.get());
    }

    @Test
    void simulatedRootRuntimeUsesSessionScopedDigitalInputSignals() {
        DigitalInputDevice input = DigitalInputDevice.rio(7);
        AtomicInteger firstStarts = new AtomicInteger();
        AtomicInteger secondStarts = new AtomicInteger();
        SimulationSession firstSession = SimulationSession.create();
        SimulationSession secondSession = SimulationSession.create();
        RobotRuntime firstRuntime = RobotRuntime.simulated(firstSession)
                .register(new DigitalHookMechanism(input, firstStarts));
        RobotRuntime secondRuntime = RobotRuntime.simulated(secondSession)
                .register(new DigitalHookMechanism(input, secondStarts));

        firstSession.digitalInput(input).raw(true);
        secondSession.digitalInput(input).raw(false);
        firstRuntime.robotPeriodic(0.0, 0.02);
        secondRuntime.robotPeriodic(0.0, 0.02);

        assertEquals(1, firstStarts.get());
        assertEquals(0, secondStarts.get());

        firstSession.digitalInput(input).raw(false);
        secondSession.digitalInput(input).raw(true);
        firstRuntime.robotPeriodic(0.02, 0.02);
        secondRuntime.robotPeriodic(0.02, 0.02);

        assertEquals(1, firstStarts.get());
        assertEquals(1, secondStarts.get());
    }

    private record TestPoseMeasurement(PoseSnapshot pose, double timestampSeconds) implements PoseMeasurementSample {
        @Override
        public RobotVelocity speeds() {
            return RobotVelocity.zero();
        }

        @Override
        public double latencySeconds() {
            return 0.0;
        }

        @Override
        public double ambiguity() {
            return 0.0;
        }

        @Override
        public int targetCount() {
            return 1;
        }

        @Override
        public MeasurementStdDevs stdDevs() {
            return null;
        }

        @Override
        public Object source() {
            return null;
        }
    }

    private static final class MotorMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 42);
        @SuppressWarnings("unused")
        private final Action initial = motor.percent(1.0);
    }

    private static final class FollowerMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 45);
        @SuppressWarnings("unused")
        private final MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 46).follow(leader);
        @SuppressWarnings("unused")
        private final Action drive = leader.percent(0.5);
    }

    private static final class FollowerMotorBackend implements MotorBackend {
        private int created;
        private FollowerMotorHandle leader;
        private FollowerMotorHandle follower;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.KRAKEN_X60;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            created++;
            FollowerMotorHandle handle = new FollowerMotorHandle(device);
            if (device.follower() == null) {
                leader = handle;
            } else {
                follower = handle;
            }
            return handle;
        }
    }

    private static final class FollowerMotorHandle implements MotorHandle {
        private final MotorDevice device;
        private MotorHandle followLeader;

        private FollowerMotorHandle(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void follow(MotorHandle leader, boolean inverted) {
            followLeader = leader;
        }
    }

    private static final class CountingOutputMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 44);
        @SuppressWarnings("unused")
        private final Action initial;

        private CountingOutputMechanism(AtomicInteger evaluations) {
            initial = motor.percent(() -> evaluations.incrementAndGet());
        }
    }

    private static final class FailingRefreshMotorBackend implements MotorBackend {
        private FailingRefreshMotorHandle handle;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.KRAKEN_X60;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            handle = new FailingRefreshMotorHandle(device);
            return handle;
        }
    }

    private static final class FailingRefreshMotorHandle implements MotorHandle {
        private final MotorDevice device;
        private boolean failRefresh;

        private FailingRefreshMotorHandle(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void refreshInputs() {
            if (failRefresh) {
                throw new IllegalStateException("refresh failed");
            }
        }

        @Override
        public void setPercentOutput(double percent) {
        }
    }

    private static final class SimulatedMotorMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 43);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 43);
        @SuppressWarnings("unused")
        private final SimModel simulation = SimModels.motor(motor).encoder(encoder);
        @SuppressWarnings("unused")
        private final Action initial = motor.percent(1.0);
    }

    private static final class DigitalHookMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        @SuppressWarnings("unused")
        private final DigitalInputDevice input;
        @SuppressWarnings("unused")
        private final Action initial = Actions.neutral();
        @SuppressWarnings("unused")
        private final HookBinding hook;

        private DigitalHookMechanism(DigitalInputDevice input, AtomicInteger starts) {
            this.input = input;
            hook = Events.when(input).rising().onStart(starts::incrementAndGet);
        }
    }
}
