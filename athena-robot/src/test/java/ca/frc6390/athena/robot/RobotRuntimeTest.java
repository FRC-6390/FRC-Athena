package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
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
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class RobotRuntimeTest {
    @Test
    void bindsDeclaredImuReadingsThroughTheRobotRuntime() {
        SimulationSession simulation = SimulationSession.create();
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 30);
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(new ImuMechanism(imu));

        simulation.imu(imu).yawDegrees(72.5);
        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(72.5, imu.yawDegrees(), 1.0e-9);
        assertEquals(72.5, imu.angleDegrees(), 1.0e-9);
    }

    @Test
    void discoversHardwareVendorBackendsFromServiceDescriptorsOnRootClasspath() {
        BackendRegistry.setGlobal(null);
        BackendRegistry registry = BackendRegistry.discover();

        assertTrue(registry.motorBackendFor(MotorKinds.FALCON_500).isPresent());
        assertTrue(registry.motorBackendFor(MotorKinds.NEO).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.CANCODER).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.REV_THROUGH_BORE).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.PIGEON_2).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.NAVX).isPresent());
    }

    @Test
    void registrationEagerlyActivatesOrdinaryDeclaredMotors() {
        FollowerMotorBackend backend = new FollowerMotorBackend();

        RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend)))
                .register(new MotorMechanism());

        assertEquals(1, backend.created);
        assertNotNull(backend.leader);
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
                    try {
                        if (runs.incrementAndGet() == 1) {
                            throw new IllegalStateException("first");
                        }
                    } finally {
                        secondRun.countDown();
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
    void closeCancelsWorkersCommandsAndHardwareAndRejectsFurtherWork() throws Exception {
        FollowerMotorBackend backend = new FollowerMotorBackend();
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        CountDownLatch workerStarted = new CountDownLatch(1);
        AtomicInteger workerRuns = new AtomicInteger();
        AtomicInteger commandEnds = new AtomicInteger();
        RobotRuntime runtime = RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend)))
                .register(new MotorMechanism())
                .workers(RuntimeWorkers.async(
                        executor,
                        RuntimeWorker.every("lifecycle", 0.01, () -> {
                            workerRuns.incrementAndGet();
                            workerStarted.countDown();
                        })))
                .startWorkers()
                .schedule(CommandAction.create("active")
                        .onEnd(commandEnds::incrementAndGet)
                        .build());
        try {
            assertTrue(workerStarted.await(1, TimeUnit.SECONDS));

            runtime.close();
            runtime.close();
            int runsAfterClose = workerRuns.get();
            Thread.sleep(50L);

            assertEquals(runsAfterClose, workerRuns.get());
            assertEquals(1, commandEnds.get());
            assertEquals(1, backend.leader.closeCalls);
            assertFalse(executor.isShutdown());
            assertThrows(IllegalStateException.class, () -> runtime.request(Actions.neutral()));
            assertThrows(IllegalStateException.class, () -> runtime.robotPeriodic(0.0, 0.02));
            assertThrows(IllegalStateException.class, () -> runtime.register(new MotorMechanism()));
        } finally {
            runtime.close();
            executor.shutdownNow();
        }
    }


    @Test
    void selectedAutoIsAnOrdinaryMechanismActionAndCancelsWhenDisabled() {
        SimulationSession simulation = SimulationSession.create();
        AutoMechanism mechanism = new AutoMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.autoPeriodic(0.02, 0.02);
        assertEquals(12.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        assertEquals("Forward", mechanism.autos.runningName().orElseThrow());
        runtime.disabledPeriodic(0.04, 0.02);
        assertEquals(0.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        assertTrue(mechanism.autos.runningAction().isEmpty());
    }

    @Test
    void shooterActionCanBeRequestedImmediatelyAfterAutonomousExit() {
        SimulationSession simulation = SimulationSession.create();
        PracticeTransitionMechanism mechanism = new PracticeTransitionMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.periodic(
                new MechanismContext(0.02, 0.0, 0.02, true, true, true),
                new EventContext(0.02, 0.02, LifecycleMode.AUTONOMOUS,
                        LifecyclePhase.PERIODIC, true, true));
        assertEquals(6.0, simulation.motor(mechanism.shooter.motor).appliedVoltage(), 1e-9);

        runtime.periodic(
                new MechanismContext(0.04, 0.0, 0.02, true, true, true),
                new EventContext(0.04, 0.02, LifecycleMode.AUTONOMOUS,
                        LifecyclePhase.EXIT, true, true));
        runtime.request(mechanism.shooter.shoot);
        runtime.periodic(
                new MechanismContext(0.06, 0.0, 0.02, true, false, true),
                new EventContext(0.06, 0.02, LifecycleMode.TELEOP,
                        LifecyclePhase.INIT, true, true));

        assertEquals(12.0, simulation.motor(mechanism.shooter.motor).appliedVoltage(), 1e-9);
    }

    @Test
    void chooserChangesNeverReplaceTheRunningAuto() {
        SimulationSession simulation = SimulationSession.create();
        AutoMechanism mechanism = new AutoMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.autoPeriodic(0.02, 0.02);
        mechanism.autos.select("Reverse");
        runtime.autoPeriodic(0.04, 0.02);

        assertEquals(12.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        assertEquals("Forward", mechanism.autos.runningName().orElseThrow());
        assertEquals("Reverse", mechanism.autos.selectedName());

        runtime.disabledPeriodic(0.06, 0.02);
        runtime.autoPeriodic(0.08, 0.02);
        assertEquals(-12.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        assertEquals("Reverse", mechanism.autos.runningName().orElseThrow());
    }

    @Test
    void registrationDiscoversProviderAndNestedPathsFromTheChooser() {
        SimulationSession simulation = SimulationSession.create();
        PathAutoMechanism mechanism = new PathAutoMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.autoPeriodic(0.02, 0.02);

        assertEquals(6.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        assertTrue(runtime.selectedAutoPreviews().get(0).steps().stream()
                .anyMatch(step -> step.contains("PATH test:leave")));
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
        assertEquals(1, runtime.inspect().faults().size());
        assertEquals(mechanism.motor.defaultName(), runtime.inspect().faults().get(0).name());
    }

    @Test
    void exposesImmutableCompletedCycleForRuntimeInspection() {
        SimulationSession simulation = SimulationSession.create();
        MotorMechanism mechanism = new MotorMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation)
                .register(mechanism)
                .mechanismTraceLevel(ca.frc6390.athena.mechanism.core.MechanismTraceLevel.CAPTURE)
                .mechanismTracePeriodSeconds(0.0);
        runtime.request(mechanism.initial);

        runtime.robotPeriodic(1.0, 0.02);
        RuntimeCycleSnapshot first = runtime.cycleSnapshot();

        assertEquals(1, first.sequence());
        assertEquals(1.0, first.timestampSeconds(), 1.0e-9);
        assertTrue(first.hardware().captured());
        assertEquals("percent", runtime.inspect().motor(mechanism.motor.defaultName()).orElseThrow().commandMode());
        assertEquals(1.0, runtime.inspect().motor(mechanism.motor.defaultName()).orElseThrow().commandValue(), 1.0e-9);

        runtime.robotPeriodic(1.02, 0.02);

        assertEquals(2, runtime.cycleSnapshot().sequence());
        assertEquals(1, first.sequence());
        assertEquals(1.0, first.timestampSeconds(), 1.0e-9);
    }

    @Test
    void recoveredHardwareReenablesItsMechanismAfterHealthySamples() throws InterruptedException {
        FailingRefreshMotorBackend backend = new FailingRefreshMotorBackend();
        MotorMechanism mechanism = new MotorMechanism();
        RobotRuntime runtime = RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend)))
                .register(mechanism);
        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        assertEquals(1.0, backend.handle.percent, 1e-9);

        backend.handle.failRefresh = true;
        runtime.robotPeriodic(0.02, 0.02);
        assertEquals(0.0, backend.handle.percent, 1e-9);

        backend.handle.failRefresh = false;
        Thread.sleep(110L);
        runtime.robotPeriodic(0.04, 0.02);
        runtime.robotPeriodic(0.06, 0.02);
        runtime.robotPeriodic(0.08, 0.02);

        assertEquals(1.0, backend.handle.percent, 1e-9);
        assertTrue(runtime.hardwareRefreshFailures().isEmpty());
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
    void simulatedMatchLifecycleOnlyAdvancesPhysicsDuringSimulationPeriodic() {
        MotorMechanism mechanism = new MotorMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        runtime.request(mechanism.initial);

        runtime.periodic(
                new ca.frc6390.athena.mechanism.core.MechanismContext(0.0, 0.0, 0.5, true, true, true),
                new ca.frc6390.athena.mechanism.core.EventContext(
                        0.0,
                        0.5,
                        ca.frc6390.athena.mechanism.core.LifecycleMode.AUTONOMOUS,
                        ca.frc6390.athena.mechanism.core.LifecyclePhase.PERIODIC,
                        true,
                        true));

        MotorHandle handle = simulation.motor(mechanism.motor);
        assertEquals(0.0, handle.integratedPositionRotations(), 1.0e-9);
        runtime.simulationPeriodic(0.5, 0.5);
        assertEquals(0.5, handle.integratedPositionRotations(), 1.0e-9);
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
    void controlBindingsReceiveAutomaticModelsWhenNothingExplicitClaimsThem() {
        AutomaticControlMechanism mechanism = new AutomaticControlMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 0.2);

        assertEquals(1, simulation.registeredModels().size());
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
        Localization localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5)));
                });
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).localization(localization);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(2.0, localization.pose().xMeters(), 1.0e-9);
        assertEquals(1, reads.get());
    }

    @Test
    void discoversLocalizationAndCameraGraphsFromRegisteredMechanisms() {
        AtomicInteger reads = new AtomicInteger();
        CameraDevice camera = Cameras.photonVision("automatic-front").bindPose(() -> {
            reads.incrementAndGet();
            return List.of(Measurements.pose(new PoseSnapshot(6.0, 2.0, 0.25)));
        });
        ca.frc6390.athena.runtime.measurement.PoseSignal configuredCameraPose = camera.pose()
                .multiTagStdDevs(0.2, 0.2, 0.1)
                .distanceStdDevScaling(2.0, 2.0);
        Localization filtered = Localizations.filter().input(configuredCameraPose);
        Localization output = Localizations.latestValid().input(filtered);
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .register(new LocalizationMechanism(output));

        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(6.0, output.pose().xMeters(), 1.0e-9);
        assertEquals(1, runtime.localizationMeasurements().size());
    }

    @Test
    void discoveredCameraLocalizationRefreshesWhileDisabledByDefault() {
        AtomicInteger reads = new AtomicInteger();
        CameraDevice camera = Cameras.photonVision("disabled-front").bindPose(() -> {
            reads.incrementAndGet();
            return List.of(Measurements.pose(new PoseSnapshot(7.0, 3.0, 0.5)));
        });
        Localization output = Localizations.latestValid().input(camera.pose());
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .register(new LocalizationMechanism(output));

        runtime.disabledPeriodic(1.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(7.0, output.pose().xMeters(), 1.0e-9);
        assertEquals(3.0, output.pose().yMeters(), 1.0e-9);
        assertEquals(0.5, output.pose().headingRadians(), 1.0e-9);
    }

    @Test
    void refreshesDiscoveredSwerveKalmanLocalizationFromIntegratedDriveEncoders() {
        SimulationSession simulation = SimulationSession.create();
        SwerveLocalizationMechanism mechanism = new SwerveLocalizationMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.robotPeriodic(0.0, 0.02);
        mechanism.driveMotors().forEach(motor -> simulation.motor(motor).state(2.0, 2.0));
        runtime.robotPeriodic(1.0, 1.0);

        assertTrue(mechanism.odometry.pose().xMeters() > 0.0);
        assertTrue(mechanism.estimatedPose.pose().xMeters() > 0.0);
        assertEquals(mechanism.odometry.pose().xMeters(), mechanism.estimatedPose.pose().xMeters(), 1.0e-9);
    }

    @Test
    void localizationRefreshPolicyAppliesRootAgeAndDisabledOptOut() {
        AtomicInteger reads = new AtomicInteger();
        Localization localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(new TestPoseMeasurement(new PoseSnapshot(4.0, 5.0, 0.25), 1.0));
                });
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create())
                .localization(localization)
                .localizationMaxAge(1.0);

        runtime.robotPeriodic(5.0, 0.02);
        runtime.disabledPeriodic(5.02, 0.02);
        runtime.localizationRefreshWhileDisabled(false).disabledPeriodic(5.04, 0.02);

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

    @Test
    void hostResolverBindsDigitalInputsForRealRuntimeRegistration() {
        DigitalInputDevice input = DigitalInputDevice.rio(27);
        AtomicBoolean raw = new AtomicBoolean();
        AtomicInteger starts = new AtomicInteger();
        RobotRuntime runtime = RobotRuntime.create()
                .digitalInputs(ignored -> raw::get)
                .register(new DigitalHookMechanism(input, starts));

        runtime.robotPeriodic(0.0, 0.02);
        raw.set(true);
        runtime.robotPeriodic(0.02, 0.02);

        assertTrue(input.active());
        assertEquals(1, starts.get());
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

    private static final class AutoMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 47);
        private final Action forward = motor.percent(1.0);
        private final Action reverse = motor.percent(-1.0);
        private final AutoChooser autos = Autos.chooser("Auto Chooser")
                .defaultAuto("Forward", forward)
                .auto("Reverse", reverse);
    }

    private static final class PracticeTransitionMechanism
            implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice driveMotor = MotorDevice.of(MotorKinds.KRAKEN_X60, 57);
        private final Action autoDrive = driveMotor.percent(0.25);
        private final PracticeShooter shooter = new PracticeShooter();
        private final PathProvider paths = new PathProvider() {
            @Override public String source() { return "practice"; }
            @Override public PathRuntime runtime() {
                return new PathRuntime() {
                    @Override public Action output(PathAction path, MechanismContext context) { return autoDrive; }
                    @Override public java.util.Map<String, Action> activeMarkers(
                            PathAction path,
                            MechanismContext context) {
                        return path.markers();
                    }
                    @Override public boolean isFinished(PathAction path, MechanismContext context) { return false; }
                };
            }
        };
        private final AutoChooser autos = Autos.chooser("Practice Match")
                .defaultAuto("Drive and shoot", Paths.of("practice", "match")
                        .marker("Shoot", shooter.autoShoot));
    }

    private static final class PracticeShooter
            implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 58);
        private final Action autoShoot = motor.percent(0.5);
        private final Action shoot = motor.percent(1.0);
    }

    private static final class PathAutoMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 48);
        private final Action drive = motor.percent(0.5);
        private final PathProvider paths = new PathProvider() {
            @Override public String source() { return "test"; }
            @Override public PathRuntime runtime() {
                return new PathRuntime() {
                    @Override public Action output(PathAction path, MechanismContext context) { return drive; }
                    @Override public boolean isFinished(PathAction path, MechanismContext context) { return false; }
                };
            }
        };
        private final AutoChooser autos = Autos.chooser("Auto Chooser")
                .defaultAuto("Leave", Actions.sequence().run(Paths.of("test", "leave")));
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
        public boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader) {
            return follower.canbus().equalsIgnoreCase(leader.canbus());
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

    private static final class FollowerMotorHandle implements MotorHandle, AutoCloseable {
        private final MotorDevice device;
        private MotorHandle followLeader;
        private int closeCalls;

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

        @Override
        public void close() {
            closeCalls++;
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
        private double percent;

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
            this.percent = percent;
        }
    }

    private static final class SimulatedMotorMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 43);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 43);
        @SuppressWarnings("unused")
        private final SimModel simulation = SimModel.motor(motor).encoder(encoder);
        @SuppressWarnings("unused")
        private final Action initial = motor.percent(1.0);
    }

    private static final class AutomaticControlMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 44);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 44);
        @SuppressWarnings("unused")
        private final ControlBinding velocity = Controls.velocity(motor).feedback(encoder);
        private final Action initial = velocity.velocity(5.0);
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

    private record ImuMechanism(ImuDevice imu) implements ca.frc6390.athena.mechanism.core.Mechanism {
    }

    private record LocalizationMechanism(Localization output)
            implements ca.frc6390.athena.mechanism.core.Mechanism {
    }

    private static final class SwerveLocalizationMechanism
            implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final ImuDevice imuDevice = ImuDevice.of(ImuKinds.PIGEON_2, 60);
        private final ImuSource imu = imuDevice.relative();
        private final SwerveModule frontLeft = module(61, 71, 81);
        private final SwerveModule frontRight = module(62, 72, 82);
        private final SwerveModule backLeft = module(63, 73, 83);
        private final SwerveModule backRight = module(64, 74, 84);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.5, 0.5, 5.0, frontLeft, frontRight, backLeft, backRight);
        private final SwerveOdometry odometry = kinematics.odometry(imu);
        private final Localization estimatedPose = Localizations.kalman().input(odometry);

        private List<MotorDevice> driveMotors() {
            return List.of(
                    frontLeft.drive.get(),
                    frontRight.drive.get(),
                    backLeft.drive.get(),
                    backRight.drive.get());
        }

        private static SwerveModule module(int driveId, int steerId, int angleId) {
            TestSwerveModule module = new TestSwerveModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, angleId)
                    .units(EncoderUnit.ROTATIONS));
            return module;
        }
    }

    private static final class TestSwerveModule extends SwerveModule {
        private TestSwerveModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }
}
