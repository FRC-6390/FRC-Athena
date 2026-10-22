package ca.frc6390.athena.mechanism.model;

import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import ca.frc6390.athena.mechanism.model.DriftMonitor.DriftReport;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

class CharacterizationTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final EncoderDevice ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 2)
            .measures(Units.Degrees);

    // Plant used by the closed-loop tests: kA * a = V - friction - kV * v, with Coulomb friction.
    private static final double PLANT_KS = 0.4;
    private static final double PLANT_STATIC = 0.5;
    private static final double PLANT_KV = 2.0;
    private static final double PLANT_KA = 0.2;
    private static final double DT = 0.02;

    @Test
    void closedLoopTestsRunThroughTheSchedulerAndFitTheTrueParameters() {
        Simulated sim = new Simulated();
        Characterization characterization = sim.mechanism.characterization;

        sim.run(sim.mechanism.quasistaticForward, 10.0);
        sim.run(sim.mechanism.quasistaticReverse, 10.0);
        sim.run(sim.mechanism.dynamicForward, 3.0);
        sim.run(sim.mechanism.dynamicReverse, 3.0);

        assertTrue(sim.maxVoltage <= Characterization.DEFAULT_MAX_VOLTAGE + 1.0e-9,
                "voltage must never exceed the configured maximum, was " + sim.maxVoltage);
        assertTrue(characterization.samples().size() > 500);

        MechanismModel fitted = characterization.fitRuns();
        assertEquals(PLANT_KV, fitted.kV(), 0.1);
        assertEquals(PLANT_KA, fitted.kA(), 0.05);
        assertEquals(PLANT_KS, fitted.kineticFrictionVolts(), 0.05);
        assertEquals(PLANT_STATIC, fitted.staticFrictionVolts(), 0.06);
        assertEquals(0.0, fitted.kG(), 0.05);
        assertTrue(fitted.rSquared() > 0.95, "r2 " + fitted.rSquared());
        assertTrue(fitted.standardErrors().get("kV") < 0.1);
        assertEquals(1, fitted.motorCount());
        assertEquals(List.of(MotorKinds.KRAKEN_X60.key()), fitted.motorKinds());
    }

    @Test
    void fitActionMakesTheModelActiveAndBaseline() throws Exception {
        Simulated sim = new Simulated();
        sim.run(sim.mechanism.quasistaticForward, 10.0);
        sim.run(sim.mechanism.quasistaticReverse, 10.0);
        sim.run(sim.mechanism.dynamicForward, 3.0);

        sim.scheduler.request(sim.mechanism.fitModel);
        long deadline = System.nanoTime() + java.util.concurrent.TimeUnit.SECONDS.toNanos(5);
        do {
            sim.clock[0] += DT;
            sim.scheduler.teleopPeriodic(sim.clock[0], DT);
            Thread.sleep(1); // Explicit asynchronous completion wait in the test, never in the action.
            assertTrue(System.nanoTime() < deadline, characterizationStatus(sim));
        } while (sim.scheduler.isRunning(sim.mechanism.fitModel));

        assertEquals(PLANT_KV, sim.mechanism.characterization.model().kV(), 0.1);
        assertEquals(sim.mechanism.characterization.model(), sim.mechanism.characterization.baseline());
        assertFalse(sim.mechanism.characterization.drift().alert());
    }

    private static String characterizationStatus(Simulated sim) {
        return sim.mechanism.characterization.fitStatus();
    }

    @Test
    void heldFitExecutorDoesNotBlockRobotAndStaleOrInterruptedFitsCannotPublish() {
        Simulated sim = new Simulated();
        sim.run(sim.mechanism.quasistaticForward, 10);
        sim.run(sim.mechanism.quasistaticReverse, 10);
        sim.run(sim.mechanism.dynamicForward, 3);
        Characterization characterization = sim.mechanism.characterization;
        List<Runnable> queued = new ArrayList<>();
        characterization.fitExecutor(queued::add);
        var fit = (ca.frc6390.athena.mechanism.core.LifecycleAction) characterization.fitModel();
        assertTrue(fit.requirements().contains(characterization.control()));
        MechanismModel original = characterization.model();
        long before = System.nanoTime();
        fit.onStart().run();
        for (int cycle = 0; cycle < 100; cycle++) fit.onExecute().run();
        assertTrue(System.nanoTime() - before < java.util.concurrent.TimeUnit.MILLISECONDS.toNanos(100));
        assertEquals(original, characterization.model());
        assertFalse(fit.isFinished().getAsBoolean());
        MechanismModel replacement = linearModel(.9, 8, .7);
        characterization.defaults(replacement);
        queued.remove(0).run();
        fit.onExecute().run();
        assertTrue(fit.isFinished().getAsBoolean());
        assertEquals(replacement, characterization.model());
        assertTrue(characterization.fitStatus().contains("discarded"));
        fit.onEnd().accept(false);

        fit.onStart().run();
        fit.onEnd().accept(true);
        queued.remove(0).run();
        fit.onExecute().run();
        assertEquals(replacement, characterization.model());
        assertEquals("cancelled", characterization.fitStatus());
    }

    @Test
    void rejectedFitAdmissionIsVisibleAndLeavesExistingModelActive() {
        Characterization characterization = Characterization.of(rawControl()).fitExecutor(task -> {
            throw new java.util.concurrent.RejectedExecutionException("test busy");
        });
        var fit = (ca.frc6390.athena.mechanism.core.LifecycleAction) characterization.fitModel();
        var original = characterization.model();
        fit.onStart().run(); fit.onExecute().run();
        assertTrue(fit.isFinished().getAsBoolean());
        assertEquals(original, characterization.model());
        assertTrue(characterization.fitStatus().contains("queue full"));
        fit.onEnd().accept(false);
    }

    @Test
    void blockedExportProviderCannotHoldRobotRunLogAndSamplesEndsRemainOrdered() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var release = new java.util.concurrent.CountDownLatch(1);
        var ended = new java.util.concurrent.CountDownLatch(1);
        List<Double> recorded = java.util.Collections.synchronizedList(new ArrayList<>());
        java.util.concurrent.atomic.AtomicReference<String> thread = new java.util.concurrent.atomic.AtomicReference<>();
        var export = new AsyncSysIdLog(() -> new ca.frc6390.athena.mechanism.sysid.SysIdLog() {
            @Override public void record(ca.frc6390.athena.mechanism.sysid.SysIdSample sample) {
                thread.set(Thread.currentThread().getName());
                if (sample.position() == 0) {
                    entered.countDown();
                    try { release.await(5, java.util.concurrent.TimeUnit.SECONDS); }
                    catch (InterruptedException interrupted) { throw new IllegalStateException(interrupted); }
                }
                recorded.add(sample.position());
            }
            @Override public void end() { ended.countDown(); }
        });
        export.record(exportSample(0));
        assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
        try {
            long before = System.nanoTime();
            for (int i = 1; i <= 400; i++) export.record(exportSample(i));
            export.end();
            assertTrue(System.nanoTime() - before < java.util.concurrent.TimeUnit.MILLISECONDS.toNanos(100));
            assertTrue(export.dropped() > 0);
            assertTrue(export.pending() <= AsyncSysIdLog.SAMPLE_CAPACITY + 1);
        } finally { release.countDown(); }
        assertTrue(ended.await(5, java.util.concurrent.TimeUnit.SECONDS));
        assertEquals("athena-sysid-export", thread.get());
        assertEquals(0.0, recorded.get(0));
        assertEquals(400.0, recorded.get(recorded.size() - 1));
        for (int i = 1; i < recorded.size(); i++) assertTrue(recorded.get(i) > recorded.get(i - 1));
        assertEquals("", export.failure());
    }

    @Test
    void blockedRealExportDoesNotHoldCharacterizationRunLogDuringPeriodic() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var release = new java.util.concurrent.CountDownLatch(1);
        Simulated sim = new Simulated();
        Characterization characterization = sim.mechanism.characterization.exportTo(
                new ca.frc6390.athena.mechanism.sysid.SysIdLog() {
                    @Override public void end() { }
                    @Override public void record(ca.frc6390.athena.mechanism.sysid.SysIdSample sample) {
                        entered.countDown();
                        try { release.await(5, java.util.concurrent.TimeUnit.SECONDS); }
                        catch (InterruptedException interrupted) { throw new IllegalStateException(interrupted); }
                    }
                });
        sim.scheduler.request(sim.mechanism.dynamicForward);
        sim.clock[0] = .02;
        sim.scheduler.teleopPeriodic(.02, DT);
        assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
        try {
            long before = System.nanoTime();
            for (int cycle = 2; cycle < 102; cycle++) {
                sim.clock[0] = cycle * DT;
                sim.scheduler.teleopPeriodic(sim.clock[0], DT);
            }
            assertTrue(System.nanoTime() - before < java.util.concurrent.TimeUnit.MILLISECONDS.toNanos(100));
            assertTrue(characterization.samples().size() > 0, "run data advances while external export is blocked");
        } finally { release.countDown(); sim.scheduler.cancelAll(); }
        assertTrue(characterization.awaitExport(java.time.Duration.ofSeconds(5)));
    }

    @Test
    void exportProviderFailuresAreVisibleAndDoNotEscapeOnRobotAdmission() {
        List<Runnable> queued = new ArrayList<>();
        AsyncSysIdLog export = new AsyncSysIdLog(() -> { throw new UnsatisfiedLinkError("JNI unavailable"); }, queued::add);
        export.record(exportSample(1));
        assertEquals("", export.failure());
        queued.remove(0).run();
        assertTrue(export.failure().contains("JNI unavailable"));
    }

    private static ca.frc6390.athena.mechanism.sysid.SysIdSample exportSample(int index) {
        return new ca.frc6390.athena.mechanism.sysid.SysIdSample(SysIdState.DYNAMIC_FORWARD, true, 1, index, 1, 1);
    }

    @Test
    void fitWithoutDataFailsWithAClearMessage() {
        Characterization characterization = Characterization.of(rawControl());
        IllegalStateException failure = assertThrows(IllegalStateException.class, characterization::fitRuns);
        assertTrue(failure.getMessage().contains("Run the characterization tests"));
    }

    @Test
    void exposesNamedActionsForIntrospection() {
        Characterization characterization = Characterization.of(rawControl());
        assertEquals(11, characterization.actions().size());
        assertTrue(characterization.actions().containsKey("QuasistaticForward"));
        assertTrue(characterization.actions().containsKey("Routine"));
        assertTrue(characterization.actions().containsKey("AdoptRunningEstimate"));
        assertTrue(characterization.actions().containsValue(characterization.latency()));
    }

    @Test
    void continuousLearningIgnoresStationaryDataAndTracksMovingData() {
        Characterization characterization = Characterization.of(rawControl())
                .learnContinuously(true)
                .defaults(linearModel(0.4, 2.0, 0.2));
        Random random = new Random(12);

        for (int i = 0; i < 300; i++) {
            characterization.observeNormalOperation(0.9 + 0.01 * random.nextGaussian(), 0.0, 0.0, 0.0, 1.0);
        }
        assertEquals(0, characterization.runningEstimate().sampleCount());

        feedMotion(characterization, 2.0, 500, random);

        MechanismModel running = characterization.runningEstimate();
        assertTrue(running.sampleCount() > 200);
        assertEquals(2.0, running.kV(), 0.05);
        assertEquals(0.2, running.kA(), 0.03);
        assertEquals(0.4, running.kineticFrictionVolts(), 0.05);
    }

    @Test
    void driftAlertsWhenTheRunningEstimateLeavesTheBand() {
        Characterization characterization = Characterization.of(rawControl())
                .learnContinuously(true)
                .driftTolerance(0.25)
                .defaults(linearModel(0.4, 2.0, 0.2));
        Random random = new Random(3);
        assertFalse(characterization.drift().alert());

        feedMotion(characterization, 3.0, 600, random);

        DriftReport report = characterization.drift();
        assertTrue(report.alert());
        assertTrue(report.message().contains("kV"), report.message());

    }

    @Test
    void adoptingTheRunningEstimateReplacesTheActiveModel() {
        Characterization characterization = Characterization.of(rawControl())
                .learnContinuously(true)
                .defaults(linearModel(0.4, 2.0, 0.2));
        feedMotion(characterization, 2.5, 600, new Random(8));

        MechanismModel before = characterization.model();
        runAction(characterization.adoptRunningEstimate());

        assertFalse(before.equals(characterization.model()));
        assertEquals(2.5, characterization.model().kV(), 0.1);
    }

    @Test
    void payloadIsLearnedOnlyWhileTheGateIsTrueAndIsConfident() {
        boolean[] loaded = {true};
        Characterization characterization = Characterization.of(linearControl())
                .payloadWhen(() -> loaded[0])
                .defaults(linearModel(0.0, 2.0, 0.2));
        Random random = new Random(6);
        for (int i = 0; i < 150; i++) {
            double hint = i % 2 == 0 ? 1.0 : -1.0;
            double volts = 0.5 * hint + 0.3 + 0.01 * random.nextGaussian();
            characterization.observeNormalOperation(volts, 0.0, 0.0, 0.0, hint);
        }
        loaded[0] = false;
        for (int i = 0; i < 150; i++) {
            characterization.observeNormalOperation(9.0, 0.0, 0.0, 0.0, 1.0);
        }

        Characterization.Payload payload = characterization.payload();
        assertTrue(payload.confident());
        assertEquals(0.3, payload.deltaKG(), 0.02);
        assertEquals(150, payload.samples());
    }

    @Test
    void payloadStaysUnconfidentWithTooFewHolds() {
        Characterization characterization = Characterization.of(rawControl())
                .payloadWhen(() -> true)
                .defaults(linearModel(0.0, 2.0, 0.2));
        for (int i = 0; i < 20; i++) {
            characterization.observeNormalOperation(0.3, 0.0, 0.0, 0.0, 1.0);
        }
        assertFalse(characterization.payload().confident());
        assertEquals(0.0, characterization.payload().deltaKG(), 1.0e-12);
    }

    @Test
    void recordsHistoryOnlyForLongEnablesAndAfterMatches() {
        Characterization characterization = Characterization.of(rawControl()).defaults(linearModel(0.4, 2.0, 0.2));

        assertFalse(characterization.recordDisable(29.0));
        assertTrue(characterization.recordDisable(31.0));
        characterization.recordMatchEnd();

        List<ModelHistory.Snapshot> snapshots = characterization.history().snapshots();
        assertEquals(2, snapshots.size());
        assertEquals("disable", snapshots.get(0).reason());
        assertEquals("match", snapshots.get(1).reason());
    }

    @Test
    void controlBindingFeedforwardUsesTheModelUntilFfReplacesIt() {
        ControlBinding<?> control = Controls.velocity(MOTOR, Units.Degrees).feedback(ENCODER);
        ControlBinding<?> modelled = control.model(linearModel(0.4, 2.0, 0.2));

        assertTrue(modelled.loops().stream().anyMatch(loop -> loop instanceof ModelFeedforward));
        ControlBinding<?> replaced = modelled.ff(1.0, 1.0, 0.0);
        assertFalse(replaced.loops().stream().anyMatch(loop -> loop instanceof ModelFeedforward));
        assertTrue(replaced.loops().stream().anyMatch(loop -> loop instanceof ca.frc6390.athena.mechanism.control.FeedforwardGains));
    }

    @Test
    void modelFeedforwardDrivesTheMotorThroughTheScheduler() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> control = Controls.velocity(MOTOR, Units.Degrees).feedback(ENCODER)
                .model(linearModel(0.4, 2.0, 0.0));
        VelocityMechanism mechanism = new VelocityMechanism(control);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(control.velocity(Units.DegreesPerSecond.of(0.5)));
        scheduler.teleopPeriodic(0.0, DT);
        scheduler.teleopPeriodic(DT, DT);

        // Typed degree bindings fit and evaluate models in rotations: 0.5 deg/s is 0.5 / 360 rotations/s.
        assertEquals(0.4 + 2.0 * (0.5 / 360.0), hardware.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void checksRequireTheirDataAndDebounce() {
        ControlBinding<?> control = Controls.position(MOTOR, Units.Degrees).feedback(ENCODER);
        assertThrows(IllegalStateException.class, control::load);
        assertThrows(IllegalStateException.class, control::blocked);
        assertThrows(IllegalStateException.class, control::backdriven);
        assertTrue(control.feedbackStalled() != null);
    }

    @Test
    void slipCheckIsOnlyCreatedWhenConfiguredAndReportsDisagreement() {
        ControlBinding<?> control = Controls.position(MOTOR, Units.Rotations).feedback(() -> 10.0, () -> 0.0);
        EncoderDevice second = EncoderDevice.of(EncoderKinds.CANCODER, 6);
        SlipCheck check = control.slipCheck(second, new GearRatio(1.0), Units.Rotations.of(0.5));

        assertEquals(0.5, check.tolerance(), 1.0e-12);
        SlipCheck slipping = new SlipCheck(() -> 10.0, () -> 9.4, 0.5);
        SlipCheck holding = new SlipCheck(() -> 10.0, () -> 9.8, 0.5);
        assertTrue(slipping.slipping());
        assertFalse(holding.slipping());
    }

    // ----- helpers -----

    private static MechanismModel linearModel(double kS, double kV, double kA) {
        return new MechanismModel(MotionKind.UNSPECIFIED, kS, kS, kV, kA, 0.0, 0.0, FrictionTable.EMPTY,
                Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0, Map.of(), 0.0, 0);
    }

    @Test
    void velocityBindingsDefaultToNoGravityAndFitWithoutAGravityTerm() {
        Characterization characterization = Characterization.of(Controls.velocity(MOTOR, Units.Degrees).feedback(ENCODER));
        assertEquals(Gravity.NONE, characterization.gravity());

        Random random = new Random(7);
        List<CharacterizationSample> samples = new ArrayList<>();
        for (int i = 0; i < 400; i++) {
            double velocity = (random.nextDouble() * 2.0 - 1.0) * 6.0;
            double acceleration = (random.nextDouble() * 2.0 - 1.0) * 3.0;
            double voltage = 0.4 * Math.signum(velocity) + 2.0 * velocity + 0.2 * acceleration;
            samples.add(new CharacterizationSample(SysIdState.DYNAMIC_FORWARD, i * DT, voltage,
                    i * DT * velocity, velocity, acceleration, Double.NaN));
        }
        Identification.FeedforwardFit fit = Identification.fitFeedforward(samples, Gravity.NONE, 0.02);
        assertEquals(0.0, fit.kG(), 1.0e-9);
        assertEquals(2.0, fit.kV(), 1.0e-6);
        assertEquals(0.2, fit.kA(), 1.0e-6);
        assertEquals(0.4, fit.kineticStaticVolts(), 1.0e-6);
    }

    @Test
    void gravityCanBeChosenPerDeclaration() {
        Characterization characterization = Characterization.of(rawControl()).gravity(Gravity.CONSTANT);
        assertEquals(Gravity.CONSTANT, characterization.gravity());
        assertEquals(Gravity.CONSTANT, Characterization.of(rawControl()).gravity(Gravity.CONSTANT).gravity());
        assertEquals(Gravity.COSINE, Gravity.of(MotionKind.ROTARY));
    }

    @Test
    void typedRangeRequiresTheMotionKindOfTheBinding() {
        Characterization rotary = Characterization.of(rawControl());
        assertThrows(IllegalArgumentException.class,
                () -> rotary.range(Meters.of(0.0), Meters.of(1.0)));
        assertThrows(IllegalArgumentException.class,
                () -> rotary.range(Degrees.of(10.0), Degrees.of(-10.0)));
        rotary.range(Degrees.of(-90.0), Degrees.of(90.0));
    }

    private static ControlBinding<?> rawControl() {
        return Controls.position(MOTOR, Units.Degrees).feedback(ENCODER);
    }

    /** A linear (meters) binding, so the fit uses a linear gravity model as the payload test expects. */
    private static ControlBinding<?> linearControl() {
        return Controls.position(MOTOR, Units.Meters)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 9).measures(Units.Meters));
    }

    /** Runs a single Action (for example one that only mutates the characterization) to completion. */
    private static void runAction(Action action) {
        ActionMechanism mechanism = new ActionMechanism(action);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);
        scheduler.request(action);
        scheduler.teleopPeriodic(0.0, DT);
        scheduler.teleopPeriodic(DT, DT);
    }

    private static final class ActionMechanism implements Mechanism {
        final Action action;

        private ActionMechanism(Action action) {
            this.action = action;
        }
    }

    /** Feeds normal-operation samples of a moving mechanism whose voltage follows the given kV. */
    private static void feedMotion(Characterization characterization, double kV, int samples, Random random) {
        for (int i = 0; i < samples; i++) {
            double t = i * DT;
            double velocity = 1.0 * Math.sin(0.9 * t) + 0.3 * Math.sin(2.3 * t);
            double accel = 0.9 * Math.cos(0.9 * t) + 0.3 * 2.3 * Math.cos(2.3 * t);
            double volts = 0.4 * Math.signum(velocity) + kV * velocity + 0.2 * accel
                    + 0.01 * random.nextGaussian();
            characterization.observeNormalOperation(volts, t * 0.1, velocity, accel, velocity);
        }
    }

    /** Mechanism with characterization Actions and a closed-loop plant behind the motor. */
    private static final class CharacterizationMechanism implements Mechanism {
        final Characterization characterization;
        final Action quasistaticForward;
        final Action quasistaticReverse;
        final Action dynamicForward;
        final Action dynamicReverse;
        final Action fitModel;

        private CharacterizationMechanism(double[] clock) {
            characterization = rawControl().characterization()
                    .rampRate(Volts.per(Seconds).of(0.5))
                    .maxVoltage(Volts.of(4.0))
                    .clock(() -> clock[0]);
            quasistaticForward = characterization.quasistaticForward();
            quasistaticReverse = characterization.quasistaticReverse();
            dynamicForward = characterization.dynamicForward();
            dynamicReverse = characterization.dynamicReverse();
            fitModel = characterization.fitModel();
        }
    }

    private static final class Simulated {
        final double[] clock = {0.0};
        final RecordingContext hardware = new RecordingContext();
        final CharacterizationMechanism mechanism = new CharacterizationMechanism(clock);
        final MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);
        double position;
        double velocity;
        double maxVoltage;

        /** Requests an action and runs cycles until it is no longer running or the time budget is spent. */
        void run(Action action, double seconds) {
            scheduler.request(action);
            int cycles = (int) Math.ceil(seconds / DT);
            for (int i = 0; i < cycles; i++) {
                clock[0] += DT;
                scheduler.teleopPeriodic(clock[0], DT);
                step();
                if (!scheduler.isRunning(action)) {
                    break;
                }
            }
            scheduler.cancel(action);
            scheduler.teleopPeriodic(clock[0] + DT, DT);
        }

        private void step() {
            double volts = hardware.motor(MOTOR).voltage;
            maxVoltage = Math.max(maxVoltage, Math.abs(volts));
            double friction;
            if (velocity != 0.0) {
                friction = Math.signum(velocity) * PLANT_KS;
            } else if (Math.abs(volts) > PLANT_STATIC) {
                friction = Math.signum(volts) * PLANT_STATIC;
            } else {
                friction = 0.0;
            }
            double accel = velocity == 0.0 && friction == 0.0 ? 0.0 : (volts - friction - PLANT_KV * velocity) / PLANT_KA;
            if (velocity == 0.0 && friction == 0.0) {
                accel = 0.0;
            }
            velocity += accel * DT;
            if (velocity * (velocity - accel * DT) < 0.0) {
                velocity = 0.0;
            }
            position += velocity * DT;
            hardware.encoder.positionRotations = position;
            hardware.encoder.velocityRotationsPerSecond = velocity;
        }
    }

    private static final class VelocityMechanism implements Mechanism {
        private final ControlBinding<?> control;

        private VelocityMechanism(ControlBinding<?> control) {
            this.control = control;
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new IdentityHashMap<>();
        private final RecordingEncoder encoder = new RecordingEncoder();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }

        @Override
        public EncoderHandle encoder(EncoderDevice encoder) {
            return this.encoder;
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double voltage;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setVoltage(double volts) {
            voltage = volts;
        }

        @Override
        public double appliedVoltage() {
            return voltage;
        }

        @Override
        public double statorCurrentAmps() {
            return 4.0;
        }

        @Override
        public void stop() {
            voltage = 0.0;
        }
    }

    private static final class RecordingEncoder implements EncoderHandle {
        private double positionRotations;
        private double velocityRotationsPerSecond;

        @Override
        public EncoderDevice device() {
            return ENCODER;
        }

        @Override
        public double positionRotations() {
            return positionRotations;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocityRotationsPerSecond;
        }
    }
}
