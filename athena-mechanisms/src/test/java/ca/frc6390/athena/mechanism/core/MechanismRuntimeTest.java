package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.ControlRoute;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class MechanismRuntimeTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice CHILD_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
    private static final MotorDevice SECOND_CHILD_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
    private static final EncoderDevice ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 1);

    @Test
    void traceCapturesConstrainedProfiledControlAndSelectedMotorOutput() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 1.0;
        actions.encoder(ENCODER).velocity = 0.25;
        ControlBinding position = Controls.position(MOTOR)
                .feedback(ENCODER)
                .constraint(Constraints.range(Range.of(0.0, 5.0)))
                .profile(MotionProfiles.trapezoid(2.0, 4.0))
                .pid(1.0, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(position.position(10.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.traceLevel(MechanismTraceLevel.CAPTURE);
        runtime.tracePeriodSeconds(0.0);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(2.0), EventContext.empty());

        MechanismTraceSnapshot trace = runtime.traceSnapshot();
        assertEquals("testMechanism", trace.mechanism());
        assertEquals("initial", trace.requestedAction());
        assertEquals(1, trace.candidates().size());
        assertTrue(trace.candidates().get(0).selected());
        assertEquals(1, trace.controls().size());
        MechanismTraceSnapshot.Control control = trace.controls().get(0);
        assertEquals(10.0, control.requestedValue(), 1.0e-9);
        assertEquals(5.0, control.goal(), 1.0e-9);
        assertTrue(control.constrained());
        assertFalse(control.blocked());
        assertEquals(1.0, control.measuredPosition(), 1.0e-9);
        assertEquals(0.25, control.measuredVelocity(), 1.0e-9);
        assertEquals(control.appliedValue(), control.proportionalVolts(), 1.0e-9);
        assertEquals(0.0, control.integralVolts(), 1.0e-9);
        assertEquals(1, trace.motors().size());
        assertEquals(MOTOR.defaultName(), trace.motors().get(0).name());
        assertEquals("voltage", trace.motors().get(0).commandMode());
        assertEquals(control.appliedValue(), trace.motors().get(0).commandValue(), 1.0e-9);
    }

    @Test
    void traceExplainsHookLeaseAndArbitrationWinner() {
        boolean[] trigger = {true};
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TriggerReleaseMechanism mechanism = new TriggerReleaseMechanism(trigger);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism)
                .traceLevel(MechanismTraceLevel.CAPTURE).tracePeriodSeconds(0.0);

        scheduler.teleopPeriodic(1.0, 0.02);

        MechanismTraceSnapshot trace = scheduler.traceSnapshots().get(0);
        assertEquals(1, trace.activeLeaseCount());
        assertTrue(trace.candidates().stream().anyMatch(candidate ->
                candidate.selected() && candidate.source().equals("lease")));
        assertEquals(1, trace.hooks().size());
        assertTrue(trace.hooks().get(0).active());
        assertTrue(trace.hooks().get(0).triggeredThisCycle());
    }

    @Test
    void traceSchedulerStateFollowsNewestRequestedLease() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DeclaredMotorMechanism mechanism = new DeclaredMotorMechanism(MOTOR);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism)
                .traceLevel(MechanismTraceLevel.CAPTURE).tracePeriodSeconds(0.0);
        Action routine = Actions.sequence().forTime(0.02, MOTOR.percent(0.5));
        scheduler.request(routine);

        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.03, 0.02);

        MechanismTraceSnapshot trace = scheduler.traceSnapshots().get(0);
        assertEquals("Sequence", trace.requestedActionType());
        assertTrue(trace.schedulerComplete());
        assertEquals(1, trace.activeLeaseCount());
    }

    @Test
    void traceSnapshotsExposePackedViewsForNestedMechanisms() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DeclaredMotorMechanism child = new DeclaredMotorMechanism(MOTOR);
        DeclaredParentMechanism parent = new DeclaredParentMechanism(child);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(parent)
                .traceLevel(MechanismTraceLevel.CAPTURE).tracePeriodSeconds(0.0);
        scheduler.request(child.initial);

        scheduler.teleopPeriodic(0.0, 0.02);

        List<MechanismTraceSnapshot> traces = scheduler.traceSnapshots();
        assertEquals(2, traces.size());
        MechanismTraceSnapshot childTrace = traces.stream()
                .filter(trace -> trace.mechanism().endsWith("/child"))
                .findFirst()
                .orElseThrow();
        assertEquals(1, childTrace.candidates().size());
        assertEquals(1, childTrace.motors().size());
        assertEquals(MOTOR.defaultName(), childTrace.motors().get(0).name());
    }

    @Test
    void traceCapturesSoftwarePidAndFeedforwardComponents() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding position = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(2.0, 0.0, 0.0)
                .ff(0.5, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(position.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.traceLevel(MechanismTraceLevel.CAPTURE);
        runtime.tracePeriodSeconds(0.0);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        MechanismTraceSnapshot.Control trace = runtime.traceSnapshot().controls().get(0);
        assertEquals(4.0, trace.proportionalVolts(), 1.0e-9);
        assertEquals(0.5, trace.staticFeedforwardVolts(), 1.0e-9);
        assertEquals(4.5, trace.appliedValue(), 1.0e-9);
    }

    @Test
    void readingTraceDoesNotReadMotorOrEncoderAgain() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding position = Controls.position(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(position.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);
        runtime.periodic(contextAt(0.0), EventContext.empty());
        int motorReads = actions.motor(MOTOR).reads;
        int encoderReads = actions.encoder(ENCODER).reads;

        runtime.traceSnapshot();
        runtime.traceSnapshot();

        assertEquals(motorReads, actions.motor(MOTOR).reads);
        assertEquals(encoderReads, actions.encoder(ENCODER).reads);
    }

    @Test
    void disabledMotorDoesNotApplyOutput() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DisabledMotorMechanism mechanism = new DisabledMotorMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism);

        scheduler.request(mechanism.move);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertTrue(Double.isNaN(actions.motor(MOTOR).percent));
        assertEquals(MOTOR, MOTOR.disabled(false));
    }

    @Test
    void disabledControlDoesNotApplyOutput() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DisabledControlMechanism mechanism = new DisabledControlMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism);

        scheduler.request(mechanism.move);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertTrue(Double.isNaN(actions.motor(MOTOR).percent));
    }

    @Test
    void disabledHookDoesNotRun() {
        AtomicInteger calls = new AtomicInteger();
        DisabledHookMechanism mechanism = new DisabledHookMechanism(calls);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingActionContext(MOTOR))
                .register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0, calls.get());
    }


    @Test
    void activeHookRequestsRunningActionAndReleaseRequestsTargetedNeutral() {
        boolean[] trigger = {false};
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TriggerReleaseMechanism mechanism = new TriggerReleaseMechanism(trigger);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism);
        ActionRequests.bind(scheduler::request);
        try {
            trigger[0] = true;
            scheduler.teleopPeriodic(0.0, 0.02);
            assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);

            trigger[0] = false;
            scheduler.teleopPeriodic(0.02, 0.02);
            assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
            assertSame(mechanism.coast, scheduler.action(mechanism));
        } finally {
            ActionRequests.clear();
        }
    }

    @Test
    void nestedTeleopHookRunsLazyCallbackAndRequestsDynamicAction() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        AtomicInteger callbacks = new AtomicInteger();
        LazyHookDrive drive = new LazyHookDrive();
        LazyHookRoot root = new LazyHookRoot(drive, callbacks);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(root);
        ActionRequests.bind(scheduler::request);
        try {
            scheduler.teleopPeriodic(0.0, 0.02);
            assertEquals(0.65, actions.motor(MOTOR).percent, 1.0e-9);
            scheduler.robotPeriodic(0.02, 0.02);

            assertEquals(1, callbacks.get());
            assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
        } finally {
            ActionRequests.clear();
        }
    }

    @Test
    void teleopHookResolvesComputedChildrenInTheSameCycle() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        double[] output = {0.25};
        ComputedHookRoot root = new ComputedHookRoot(output);
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(root);
        ActionRequests.bind(scheduler::request);
        try {
            scheduler.teleopPeriodic(0.0, 0.02);
            assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);

            output[0] = 0.75;
            scheduler.teleopPeriodic(0.02, 0.02);
            assertEquals(0.75, actions.motor(MOTOR).percent, 1.0e-9);
        } finally {
            ActionRequests.clear();
        }
    }

    @Test
    void sequenceAdvancesByTimeAndRoutesOutputsToMotorHandle() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                Actions.sequence()
                        .forTime(0.5, MOTOR.percent(0.25))
                        .then(MOTOR.voltage(6.0)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.25), EventContext.empty());
        assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.5), EventContext.empty());
        assertEquals(6.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void delayedForTimeUsesStepEntryInsteadOfNestedChildEntry() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        Action changingChild = Actions.sequence()
                .forTime(0.1, MOTOR.percent(0.4))
                .then(MOTOR.percent(0.5));
        TestMechanism mechanism = new TestMechanism(Actions.sequence()
                .waitSeconds(10.0)
                .forTime(5.0, changingChild)
                .then(MOTOR.voltage(7.0)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(10.0), EventContext.empty());
        assertEquals(0.4, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(14.9), EventContext.empty());
        assertEquals(0.5, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(15.0), EventContext.empty());
        assertEquals(7.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void nestedSequencesAndParallelsPreserveTheirOwnLifecycle() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        Action firstBranch = Actions.sequence()
                .forTime(0.2, MOTOR.percent(0.2));
        Action secondBranch = Actions.sequence()
                .forTime(0.1, CHILD_MOTOR.percent(0.3))
                .forTime(0.1, CHILD_MOTOR.percent(0.4));
        TestMechanism mechanism = new TestMechanism(Actions.sequence()
                .run(Actions.parallel(firstBranch, secondBranch))
                .then(Actions.parallel(
                        MOTOR.percent(0.8),
                        CHILD_MOTOR.percent(0.9))));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.2, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.3, actions.motor(CHILD_MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.1), EventContext.empty());
        assertEquals(0.4, actions.motor(CHILD_MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.2), EventContext.empty());
        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.9, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void cycleForTimeUsesCycleStepEntryAroundNestedActions() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        Action changingChild = Actions.sequence()
                .forTime(0.1, MOTOR.percent(0.2))
                .then(MOTOR.percent(0.3));
        TestMechanism mechanism = new TestMechanism(Actions.cycle()
                .forTime(0.5, changingChild)
                .forTime(0.5, MOTOR.percent(0.8)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.2, actions.motor(MOTOR).percent, 1.0e-9);
        runtime.periodic(contextAt(0.1), EventContext.empty());
        assertEquals(0.3, actions.motor(MOTOR).percent, 1.0e-9);
        runtime.periodic(contextAt(0.5), EventContext.empty());
        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void sequenceStartCarriesActionThroughFollowingStagesAndReleasesWithSequence() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        Action routine = Actions.sequence()
                .forTime(0.5, MOTOR.percent(0.2))
                .start(CHILD_MOTOR.percent(0.6))
                .waitSeconds(0.2)
                .then(MOTOR.percent(0.8));
        TestMechanism mechanism = new TestMechanism(routine);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(routine);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.2, actions.motor(MOTOR).percent, 1.0e-9);
        assertTrue(Double.isNaN(actions.motor(CHILD_MOTOR).percent));

        runtime.periodic(contextAt(0.5), EventContext.empty());
        assertEquals(0.6, actions.motor(CHILD_MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.71), EventContext.empty());
        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.6, actions.motor(CHILD_MOTOR).percent, 1.0e-9);

        runtime.set(Actions.neutral());
        runtime.periodic(contextAt(0.72), EventContext.empty());
        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.0, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void sequenceRejectsUnreachableStageAfterContinuousRun() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        Action invalid = Actions.sequence()
                .run(MOTOR.percent(0.2))
                .then(CHILD_MOTOR.percent(0.8));
        TestMechanism mechanism = new TestMechanism(invalid);
        IllegalArgumentException error = assertThrows(
                IllegalArgumentException.class,
                () -> MechanismScheduler.create(actions).register(mechanism));

        assertTrue(error.getMessage().contains("cannot advance past run"));
        assertTrue(error.getMessage().contains("start(...)"));
    }

    @Test
    void actionChainRejectsThenAfterContinuousAction() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        Action invalid = MOTOR.percent(0.2).then(CHILD_MOTOR.percent(0.8));
        TestMechanism mechanism = new TestMechanism(invalid);

        IllegalArgumentException error = assertThrows(
                IllegalArgumentException.class,
                () -> MechanismScheduler.create(actions).register(mechanism));

        assertTrue(error.getMessage().contains("cannot advance past then"));
        assertTrue(error.getMessage().contains("forTime(...)"));
    }

    @Test
    void timeoutAndConditionMoveToNextState() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                Actions.timeout(MOTOR.percent(0.4), 0.3)
                        .then(Actions.until(ctx -> ctx.nowSeconds() >= 1.0, MOTOR.percent(0.6))
                                .then(MOTOR.voltage(7.0))));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.4, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.3), EventContext.empty());
        assertEquals(0.6, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(1.0), EventContext.empty());
        assertEquals(7.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void fluentTimeoutCanWrapAnEntireThenChain() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                Actions.waitSeconds(0.2)
                        .then(MOTOR.percent(0.4))
                        .timeout(0.5));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.2), EventContext.empty());
        assertEquals(0.4, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.5), EventContext.empty());
        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void sequenceAdvancesWhenPositionControlEntersTolerance() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding position = Controls.position(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(Actions.sequence()
                .run(position.position(10.0).untilWithin(0.25))
                .then(MOTOR.percent(0.7)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        actions.encoder(ENCODER).position = 9.5;
        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(10.0, actions.motor(MOTOR).positionTarget, 1.0e-9);

        actions.encoder(ENCODER).position = 9.8;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(0.7, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void thenAdvancesWhenVelocityControlEntersTolerance() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding velocity = Controls.velocity(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(
                velocity.velocity(20.0).untilWithin(0.5).then(MOTOR.voltage(4.0)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        actions.encoder(ENCODER).velocity = 19.0;
        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(20.0, actions.motor(MOTOR).velocityTarget, 1.0e-9);

        actions.encoder(ENCODER).velocity = 19.6;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(4.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void cycleRunsSelfCompletingControlStepsRepeatedly() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding position = Controls.position(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(Actions.cycle()
                .run(position.position(1.0).untilWithin(0.05))
                .run(position.position(-1.0).untilWithin(0.05)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        actions.encoder(ENCODER).position = 0.0;
        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(1.0, actions.motor(MOTOR).positionTarget, 1.0e-9);

        actions.encoder(ENCODER).position = 0.98;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(-1.0, actions.motor(MOTOR).positionTarget, 1.0e-9);

        actions.encoder(ENCODER).position = -0.98;
        runtime.periodic(contextAt(0.04), EventContext.empty());
        assertEquals(1.0, actions.motor(MOTOR).positionTarget, 1.0e-9);
    }

    @Test
    void risingHookPulsesStartAndEndsOnFollowingNonPulseTick() {
        boolean[] signal = new boolean[1];
        HookedMechanism mechanism = new HookedMechanism(signal);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new RecordingActionContext(MOTOR));

        runtime.periodic(contextAt(0.0), EventContext.empty());
        signal[0] = true;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());
        signal[0] = false;
        runtime.periodic(contextAt(0.06), EventContext.empty());
        signal[0] = true;
        runtime.periodic(contextAt(0.08), EventContext.empty());

        assertEquals(2, mechanism.starts);
        assertEquals(2, mechanism.whileActive);
        assertEquals(1, mechanism.ends);
    }

    @Test
    void digitalInputHookConsumesLatchedEdgeSampledBetweenRobotTicks() {
        boolean[] raw = new boolean[1];
        DigitalInputDevice input = DigitalInputDevice.rio(9).bind(() -> raw[0]);
        DigitalHookedMechanism mechanism = new DigitalHookedMechanism(input);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new RecordingActionContext(MOTOR));

        input.sample();
        runtime.periodic(contextAt(0.0), EventContext.empty());

        raw[0] = true;
        input.sample();
        raw[0] = false;
        input.sample();

        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, mechanism.starts);
        assertEquals(1, mechanism.ends);
    }

    @Test
    void digitalInputHookResetsIntegratedEncoderImmediatelyWhileDisabled() {
        boolean[] raw = new boolean[1];
        DigitalInputDevice home = DigitalInputDevice.rio(8).bind(() -> raw[0]);
        EncoderZeroingMechanism mechanism = new EncoderZeroingMechanism(home);
        RecordingActionContext actions = new RecordingActionContext(mechanism.motor);
        actions.encoder(mechanism.encoder).position = 4.5;
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism);

        scheduler.periodic(disabledContextAt(0.0), disabledEventAt(0.0));
        raw[0] = true;
        scheduler.periodic(disabledContextAt(0.02), disabledEventAt(0.02));

        assertEquals(0.0, actions.encoder(mechanism.encoder).position, 1.0e-9);
        assertEquals(1, actions.encoder(mechanism.encoder).setPositionCalls);
    }

    @Test
    void hookRuntimeSamplesGenericEventSourceOncePerTick() {
        boolean[] raw = new boolean[] { true };
        AtomicInteger reads = new AtomicInteger();
        CountingHookedMechanism mechanism = new CountingHookedMechanism(raw, reads);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new RecordingActionContext(MOTOR));

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(1, reads.get());
        assertEquals(1, mechanism.starts);
    }

    @Test
    void sharedRisingEventTriggersAllHooksOnSameTick() {
        boolean[] raw = new boolean[1];
        SharedEventHookedMechanism mechanism = new SharedEventHookedMechanism(raw);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new RecordingActionContext(MOTOR));

        runtime.periodic(contextAt(0.0), EventContext.empty());
        raw[0] = true;
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(1, mechanism.firstStarts);
        assertEquals(1, mechanism.secondStarts);
    }

    @Test
    void hookRuntimeSamplesCustomEventSourceOnceForSharedHooks() {
        CountingEvent event = new CountingEvent(true);
        SharedCustomEventHookedMechanism mechanism = new SharedCustomEventHookedMechanism(event);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new RecordingActionContext(MOTOR));

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(1, event.reads.get());
        assertEquals(1, mechanism.firstStarts);
        assertEquals(1, mechanism.secondStarts);
    }

    @Test
    void robotRuntimeSamplesDeclaredDigitalInputsBeforeHooksRun() {
        boolean[] raw = new boolean[1];
        DigitalInputDevice input = DigitalInputDevice.rio(10).bind(() -> raw[0]);
        DigitalHookedMechanism mechanism = new DigitalHookedMechanism(input);
        MechanismScheduler runtime = MechanismScheduler.create(new RecordingActionContext(MOTOR))
                .register(mechanism);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        raw[0] = true;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, mechanism.starts);
        assertEquals(1, mechanism.ends);
    }

    @Test
    void robotRuntimeCanSampleSignalsBetweenHookTicks() {
        boolean[] raw = new boolean[1];
        DigitalInputDevice input = DigitalInputDevice.rio(11).bind(() -> raw[0]);
        DigitalHookedMechanism mechanism = new DigitalHookedMechanism(input);
        MechanismScheduler runtime = MechanismScheduler.create(new RecordingActionContext(MOTOR))
                .register(mechanism);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        raw[0] = true;
        runtime.sampleSignals();
        raw[0] = false;
        runtime.sampleSignals();

        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, mechanism.starts);
        assertEquals(1, mechanism.ends);
    }

    @Test
    void pathRuntimeInitializesOnceExecutesUntilFinishedAndEndsOnce() {
        PathAction path = Paths.pathPlanner("leave-zone");
        RecordingPathRuntime pathRuntime = new RecordingPathRuntime(3);
        TestMechanism mechanism = new TestMechanism(path.then(MOTOR.percent(0.9)));
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions).path(path, pathRuntime);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, pathRuntime.initializes);
        assertEquals(3, pathRuntime.executes);
        assertEquals(1, pathRuntime.ends);
        assertEquals(0.9, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void pathOutputAndMarkerActionsRunInsideTheSameSchedulerTree() {
        PathAction path = Paths.choreo("markers");
        Action marker = CHILD_MOTOR.percent(0.6);
        PathRuntime pathRuntime = new PathRuntime() {
            @Override public boolean isFinished(PathAction path, MechanismContext context) { return false; }
            @Override public Action output(PathAction path, MechanismContext context) { return MOTOR.percent(0.4); }
            @Override public Map<String, Action> activeMarkers(PathAction path, MechanismContext context) {
                return Map.of("marker", marker);
            }
        };
        TestMechanism mechanism = new TestMechanism(path);
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions).path(path, pathRuntime);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.4, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.6, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void loweredSchedulerResetsCompletedSequenceSubtrees() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(Actions.sequence()
                .doOnce(() -> actions.events++)
                .forTime(0.02, MOTOR.percent(0.2))
                .then(Actions.sequence()
                        .doOnce(() -> actions.events++)
                        .then(MOTOR.percent(0.8))));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());
        runtime.periodic(contextAt(0.06), EventContext.empty());

        assertEquals(2, actions.events);
        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void repeatedThenCallsChainInsteadOfReplacingEarlierActions() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(Actions.sequence()
                .doOnce(() -> actions.events++)
                .then(Actions.waitSeconds(0.1))
                .then(Actions.doOnce(() -> actions.events++)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(1, actions.events);
        runtime.periodic(contextAt(0.11), EventContext.empty());
        assertEquals(2, actions.events);
    }

    @Test
    void parallelRoutesChildMechanismOutputToChildMotorHandle() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR);
        TestMechanism child = new TestMechanism(CHILD_MOTOR.percent(0.7));
        TestMechanism parent = new TestMechanism(Actions.parallel(child.initial));
        MechanismRuntime runtime = MechanismRuntime.of(parent, actions);
        runtime.set(parent.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void disabledContextStopsDrivenMotorsWithoutEvaluatingTheRetainedAction() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        AtomicInteger evaluations = new AtomicInteger();
        TestMechanism mechanism = new TestMechanism(MOTOR.percent(() -> {
            evaluations.incrementAndGet();
            return 0.7;
        }));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(
                new MechanismContext(0.02, 0.0, 0.02, false, false, false),
                new EventContext(0.02, 0.02, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));

        assertEquals(1, evaluations.get());
        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeRequestsRootOwnedAction() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(MOTOR.percent(0.1));
        Action requested = MOTOR.percent(0.8);
        mechanism.secondary = requested;
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(mechanism);

        ActionRequests.bind(runtime::request);
        try {
            requested.request();
        } finally {
            ActionRequests.clear();
        }
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeRequestsChildOwnedActionThroughRegisteredRoot() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR);
        TestMechanism child = new TestMechanism(CHILD_MOTOR.percent(0.1));
        Action requested = CHILD_MOTOR.percent(0.9);
        child.secondary = requested;
        ParentMechanism parent = new ParentMechanism(child);
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(parent);

        runtime.request(requested);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.9, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void encoderSetPositionActionInfersOwnershipAndRunsOnce() {
        RecordingActionContext actions = new RecordingActionContext();
        EncoderPositionMechanism mechanism = new EncoderPositionMechanism();
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(mechanism);

        runtime.request((Action) mechanism.setPosition);
        runtime.robotPeriodic(1.0, 0.02);
        runtime.robotPeriodic(1.02, 0.02);

        assertEquals(2.5, actions.encoder(ENCODER).position, 1.0e-9);
        assertEquals(1, actions.encoder(ENCODER).setPositionCalls);
    }

    @Test
    void parallelInfersMultipleChildOwnersUnderOneRegisteredRoot() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR, SECOND_CHILD_MOTOR);
        DeclaredMotorMechanism first = new DeclaredMotorMechanism(CHILD_MOTOR);
        DeclaredMotorMechanism second = new DeclaredMotorMechanism(SECOND_CHILD_MOTOR);
        MultiParentMechanism parent = new MultiParentMechanism(first, second);
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(parent);
        Action requested = Actions.parallel(
                CHILD_MOTOR.percent(0.7),
                SECOND_CHILD_MOTOR.percent(0.8));

        runtime.request(requested);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
        assertEquals(0.8, actions.motor(SECOND_CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void nestedSequencesAndParallelsRouteAcrossChildOwnersUnderOneRoot() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR, SECOND_CHILD_MOTOR);
        DeclaredMotorMechanism first = new DeclaredMotorMechanism(CHILD_MOTOR);
        DeclaredMotorMechanism second = new DeclaredMotorMechanism(SECOND_CHILD_MOTOR);
        MultiParentMechanism parent = new MultiParentMechanism(first, second);
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(parent);
        Action requested = Actions.sequence()
                .forTime(0.1, Actions.parallel(
                        CHILD_MOTOR.percent(0.2),
                        Actions.sequence().forTime(0.1, SECOND_CHILD_MOTOR.percent(0.3))))
                .then(Actions.parallel(
                        CHILD_MOTOR.percent(0.7),
                        SECOND_CHILD_MOTOR.percent(0.8)));

        runtime.request(requested);
        runtime.robotPeriodic(0.0, 0.02);
        assertEquals(0.2, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
        assertEquals(0.3, actions.motor(SECOND_CHILD_MOTOR).percent, 1.0e-9);

        runtime.robotPeriodic(0.1, 0.02);
        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
        assertEquals(0.8, actions.motor(SECOND_CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void parallelRoutesActionsOwnedBySeparateRegisteredRoots() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR, SECOND_CHILD_MOTOR);
        MechanismScheduler runtime = MechanismScheduler.create(actions)
                .register(new DeclaredMotorMechanism(CHILD_MOTOR))
                .register(new DeclaredMotorMechanism(SECOND_CHILD_MOTOR));
        Action requested = Actions.parallel(
                CHILD_MOTOR.percent(0.7),
                SECOND_CHILD_MOTOR.percent(0.8));

        runtime.request(requested);
        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
        assertEquals(0.8, actions.motor(SECOND_CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void registeringAnAlreadyDiscoveredChildDoesNotCreateAmbiguousOwnership() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR);
        TestMechanism child = new TestMechanism(CHILD_MOTOR.percent(0.1));
        Action requested = CHILD_MOTOR.percent(0.9);
        child.secondary = requested;
        ParentMechanism parent = new ParentMechanism(child);
        MechanismScheduler runtime = MechanismScheduler.create(actions)
                .register(parent)
                .register(child);

        runtime.request(requested);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(1, runtime.mechanisms().size());
        assertEquals(0.9, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeRequestsExternalActionByTargetDeclarationOwnership() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DeviceMechanism mechanism = new DeviceMechanism();
        ExternalActions external = new ExternalActions(mechanism);
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(mechanism);

        ActionRequests.bind(runtime::request);
        try {
            external.drive.request();
        } finally {
            ActionRequests.clear();
        }
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.65, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeInfersOwnershipThroughToleranceWrapper() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        PositionControlMechanism mechanism = new PositionControlMechanism();
        MechanismScheduler runtime = MechanismScheduler.create(actions).register(mechanism);
        Action requested = mechanism.position.position(2.0).untilWithin(0.1);

        runtime.request(requested);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(2.0, actions.motor(MOTOR).positionTarget, 1.0e-9);
    }

    @Test
    void robotRuntimeCanUseHardwareGraphAsRuntimeActionContext() {
        RecordingMotorBackend backend = new RecordingMotorBackend();
        HardwareGraph hardware = HardwareGraph.using(BackendRegistry.of(backend));
        TestMechanism mechanism = new TestMechanism(MOTOR.percent(0.55));
        MechanismScheduler runtime = MechanismScheduler.create(hardware)
                .register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.55, backend.handle.percent, 1.0e-9);
        assertEquals(1, backend.created);
    }

    @Test
    void controlPidLoopTransformsPositionRequestIntoMotorVoltage() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.5;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.3, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void positionConstraintCorrectsTargetBeforePidRuns() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.5;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .constraint(Constraints.range(Range.of(0.0, 1.0)));
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.1, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void upperConstraintBlocksOnlyMotionTowardActiveBoundary() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.5;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .constraint(Constraints.upper(() -> true));
        TestMechanism mechanism = new TestMechanism(control.percent(0.2));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.set(control.percent(-0.2));
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(-0.2, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void motionProfileLimitsReferenceVelocityAndAcceleration() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .profile(MotionProfiles.trapezoid(1.0, 2.0))
                .loop(binding -> context -> ControlOutput.percent(context.reference().position()));
        TestMechanism mechanism = new TestMechanism(control.position(10.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());

        assertEquals(0.01, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void profileContinuesDeceleratingNearRangeBoundaryWithoutPulsingNeutral() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.9;
        actions.encoder(ENCODER).velocity = 1.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .constraint(Constraints.range(Range.of(0.0, 1.0)))
                .profile(MotionProfiles.trapezoid(2.0, 2.0));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertTrue(Double.isFinite(actions.motor(MOTOR).voltage));
        assertTrue(Math.abs(actions.motor(MOTOR).voltage) < 0.1);
    }

    @Test
    void profiledRangeTargetRemainsContinuouslyControlledAndInsideTheRange() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.9;
        actions.encoder(ENCODER).velocity = 1.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(1.0, 0.0, 0.0)
                .constraints(
                        Constraints.range(Range.of(0.0, 1.0)),
                        Constraints.motion(2.0, 2.0));
        MechanismRuntime runtime = MechanismRuntime.of(
                new TestMechanism(control.position(1.0)), actions);
        runtime.traceLevel(MechanismTraceLevel.CAPTURE);
        runtime.tracePeriodSeconds(0.0);
        runtime.set(control.position(1.0));

        for (int cycle = 0; cycle < 40; cycle++) {
            runtime.periodic(contextAt(cycle * 0.02), EventContext.empty());
            MechanismTraceSnapshot.Control trace = runtime.traceSnapshot().controls().get(0);
            assertEquals("voltage", trace.appliedMode());
            assertFalse(trace.blocked());
            assertTrue(trace.referencePosition() >= 0.0 && trace.referencePosition() <= 1.0);
            assertTrue(Double.isFinite(trace.appliedValue()));
        }
    }

    @Test
    void profileReversesSmoothlyAwayFromRangeBoundary() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.9;
        actions.encoder(ENCODER).velocity = 1.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(1.0, 0.0, 0.0)
                .constraint(Constraints.range(Range.of(0.0, 1.0)))
                .profile(MotionProfiles.trapezoid(2.0, 2.0));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.set(control.position(1.0));
        runtime.periodic(contextAt(0.0), EventContext.empty());

        actions.encoder(ENCODER).velocity = 0.0;
        runtime.set(control.position(0.5));
        for (int cycle = 1; cycle <= 80; cycle++) {
            runtime.periodic(contextAt(cycle * 0.02), EventContext.empty());
        }

        assertTrue(actions.motor(MOTOR).voltage < 0.0,
                () -> "expected reverse voltage, got " + actions.motor(MOTOR).voltage);
    }

    @Test
    void constraintCanReadRelatedAthenaFeedback() {
        EncoderDevice related = EncoderDevice.of(EncoderKinds.CANCODER, 2);
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.5;
        actions.encoder(related).position = 2.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .constraint(Constraints.require(() -> related.position() >= 5.0));
        TestMechanism mechanism = new TestMechanism(control.percent(0.2));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void invalidTargetTransformFailsClosedBeforePid() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 5.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .loop(ControlLoops.targetTransform(binding -> context -> ControlOutput.voltage(6.0)))
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(10.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void velocityPidUsesVelocityFeedback() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 100.0;
        actions.encoder(ENCODER).velocity = 2.0;
        ControlBinding control = Controls.velocity(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.2, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void velocityRangeDoesNotTreatAccumulatedEncoderPositionAsTheControlledValue() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 150.0;
        actions.encoder(ENCODER).velocity = 0.0;
        ControlBinding control = Controls.velocity(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .constraints(
                        Constraints.range(Range.of(0.0, 100.0)),
                        Constraints.motion(100.0, 200.0),
                        Constraints.clamp(12.0));
        TestMechanism mechanism = new TestMechanism(control.velocity(33.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.8, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void velocityRangeConstrainsTheRequestedVelocity() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 150.0;
        actions.encoder(ENCODER).velocity = 20.0;
        ControlBinding control = Controls.velocity(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .constraint(Constraints.range(Range.of(0.0, 100.0)));
        TestMechanism mechanism = new TestMechanism(control.velocity(150.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(12.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void rangedVelocityControlKeepsRespondingAcrossCollectShootCollectRequests() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 150.0;
        actions.encoder(ENCODER).velocity = 0.0;
        VelocityCompositeMechanism mechanism = new VelocityCompositeMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(actions).register(mechanism);

        scheduler.request(mechanism.collect);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.8, actions.motor(MOTOR).voltage, 1.0e-9);

        scheduler.request(mechanism.shoot);
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(1.6, actions.motor(MOTOR).voltage, 1.0e-9);

        scheduler.request(mechanism.collect);
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(2.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void controlFeedforwardLoopTransformsVelocityRequestIntoMotorVoltage() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.velocity(MOTOR)
                .ff(0.5, 2.0, 1.0);
        TestMechanism mechanism = new TestMechanism(control.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(7.5, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void pidAndFeedforwardAddDirectlyAsVolts() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).velocity = 2.0;
        ControlBinding control = Controls.velocity(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .ff(0.5, 2.0, 1.0);
        TestMechanism mechanism = new TestMechanism(control.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(7.7, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void softwarePidIntegralAccumulatesInsideIZone() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(PidGains.of(0.0, 1.0, 0.0).iZone(2.0));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.02, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(0.04, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.periodic(contextAt(0.04), EventContext.empty());
        assertEquals(0.06, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void softwarePidIntegralWaitsUntilErrorEntersIZone() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(PidGains.of(0.0, 1.0, 0.0).iZone(1.0));
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.0, actions.motor(MOTOR).voltage, 1.0e-9);
        actions.encoder(ENCODER).position = 1.5;
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(0.01, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.periodic(contextAt(0.04), EventContext.empty());
        assertEquals(0.02, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void softwarePidClampsIntegralInsteadOfResettingAfterSaturation() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 100.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        for (int cycle = 0; cycle < 8; cycle++) {
            runtime.periodic(contextAt(cycle * 0.02), EventContext.empty());
        }
        assertEquals(12.0, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.periodic(contextAt(0.16), EventContext.empty());
        assertEquals(12.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void iZoneForcesAthenaPidWhenMotorSupportsDeviceClosedLoop() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        actions.encoder(MOTOR.encoder()).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(PidGains.of(0.0, 1.0, 0.0).iZone(2.0));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(0.04, actions.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
    }

    @Test
    void pidDerivativeUsesRuntimeDtAndCurrentFeedback() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 0.0, 0.1);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());
        assertEquals(0.0, actions.motor(MOTOR).voltage, 1.0e-9);
        actions.encoder(ENCODER).position = 0.5;
        runtime.periodic(new MechanismContext(0.1, 0.0, 0.1, true, false, false), EventContext.empty());
        assertEquals(-0.5, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void pidTargetStepDoesNotCreateDerivativeKick() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 0.0, 1.0);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.set(control.position(5.0));
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(0.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void profileRestartsFromMeasuredStateAfterAnOpenLoopActionOwnsTheMotor() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(1.0, 0.0, 0.0)
                .profile(MotionProfiles.trapezoid(10.0, 10.0));
        TestMechanism mechanism = new TestMechanism(control.position(10.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.set(control.position(10.0));
        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());

        runtime.set(MOTOR.percent(-0.2));
        actions.encoder(ENCODER).position = 5.0;
        runtime.periodic(new MechanismContext(0.1, 0.0, 0.1, true, false, false), EventContext.empty());

        runtime.set(control.position(10.0));
        runtime.periodic(new MechanismContext(0.2, 0.0, 0.1, true, false, false), EventContext.empty());

        assertEquals(0.05, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void pidSkipsIntegralAndDerivativeForInvalidDt() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(1.0, 1.0, 1.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.0, true, false, false), EventContext.empty());
        assertEquals(2.0, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.periodic(new MechanismContext(0.0, 0.0, Double.NaN, true, false, false), EventContext.empty());
        assertEquals(2.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void neutralRequestResetsPidIntegralBeforeControlResumes() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 1.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(0.04, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.set(control.neutral());
        runtime.periodic(contextAt(0.04), EventContext.empty());
        runtime.set(control.position(1.0));
        runtime.periodic(contextAt(0.06), EventContext.empty());

        assertEquals(0.02, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void positionFeedforwardUsesErrorDirectionWhenReferenceIsStationary() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .ff(1.0, 0.0, 0.5);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(1.5, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.set(control.position(-2.0));
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(-0.5, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void profiledFeedforwardIncludesAccelerationGain() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        actions.encoder(ENCODER).velocity = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .profile(MotionProfiles.trapezoid(10.0, 5.0))
                .ff(0.0, 0.0, 2.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());

        assertEquals(10.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void pidAntiWindupAccountsForComposedFeedforward() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        double[] feedforwardVolts = {11.5};
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 1.0, 0.0)
                .loop(binding -> context -> ControlOutput.voltage(feedforwardVolts[0]));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        for (int cycle = 0; cycle < 100; cycle++) {
            runtime.periodic(contextAt(cycle * 0.02), EventContext.empty());
        }
        assertEquals(12.0, actions.motor(MOTOR).voltage, 1.0e-9);

        feedforwardVolts[0] = 0.0;
        actions.encoder(ENCODER).position = 1.0;
        runtime.periodic(contextAt(2.0), EventContext.empty());
        assertEquals(0.5, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void nonFinitePidFeedbackNeutralizesRegardlessOfLoopOrder() {
        RecordingActionContext firstActions = new RecordingActionContext(MOTOR);
        firstActions.encoder(ENCODER).position = Double.NaN;
        ControlBinding pidFirst = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .ff(0.0, 0.0, 1.0);
        MechanismRuntime first = MechanismRuntime.of(new TestMechanism(pidFirst.position(2.0)), firstActions);
        first.set(pidFirst.position(2.0));

        RecordingActionContext secondActions = new RecordingActionContext(MOTOR);
        secondActions.encoder(ENCODER).position = Double.NaN;
        ControlBinding feedforwardFirst = Controls.position(MOTOR)
                .feedback(ENCODER)
                .ff(0.0, 0.0, 1.0)
                .pid(0.2, 0.0, 0.0);
        MechanismRuntime second = MechanismRuntime.of(
                new TestMechanism(feedforwardFirst.position(2.0)), secondActions);
        second.set(feedforwardFirst.position(2.0));

        first.periodic(contextAt(0.0), EventContext.empty());
        second.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.0, firstActions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(0.0, secondActions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void nonFiniteCustomLoopOutputIsNeverSentToHardware() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.position(MOTOR)
                .loop(binding -> context -> ControlOutput.voltage(Double.NaN));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.0, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void structurallyEqualBindingsKeepIndependentPidState() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.0;
        ControlBinding firstControl = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 1.0, 0.0);
        ControlBinding secondControl = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.0, 1.0, 0.0);
        assertEquals(firstControl, secondControl);
        MechanismRuntime runtime = MechanismRuntime.of(
                new TestMechanism(firstControl.position(1.0)), actions);
        runtime.set(firstControl.position(1.0));

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        assertEquals(0.04, actions.motor(MOTOR).voltage, 1.0e-9);
        runtime.set(secondControl.position(1.0));
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(0.02, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void controlLoopsReceiveRuntimeDtSeconds() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.position(MOTOR)
                .loop(binding -> context -> ControlOutput.percent(context.dtSeconds()));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());

        assertEquals(0.1, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void mixedFeedbackBindingSuppliesIndependentPositionAndVelocity() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        FeedbackBinding feedback = new FeedbackBinding(() -> 4.5, () -> -1.5);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(feedback)
                .loop(binding -> context -> ControlOutput.voltage(context.position() + context.velocity()));
        TestMechanism mechanism = new TestMechanism(control.position(10.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(3.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void explicitlyMixedPercentAndVoltageLoopsConvertPercentToNominalVoltage() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.velocity(MOTOR)
                .loop(binding -> context -> ControlOutput.percent(0.25))
                .loop(binding -> context -> ControlOutput.voltage(2.0));
        TestMechanism mechanism = new TestMechanism(control.velocity(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(5.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void controlLoopRuntimePreservesStateWhenSameModeTargetChanges() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ResetCountingLoop loop = new ResetCountingLoop();
        ControlBinding control = Controls.position(MOTOR).loop(loop);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.set(control.position(2.0));
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, loop.runtime.resets);
        assertEquals(3, loop.runtime.calculations);
    }

    @Test
    void pidControlUsesDeviceClosedLoopWhenSupported() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(2.0, actions.motor(MOTOR).positionTarget, 1.0e-9);
        assertEquals(0.2, actions.motor(MOTOR).closedLoopRequest.config().p(), 1.0e-9);
        assertEquals(ControlRoute.DEVICE_CLOSED_LOOP, actions.motor(MOTOR).closedLoopRequest.route());
    }

    @Test
    void livePidEditRefreshesDeviceClosedLoopConfiguration() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        PidGains gains = PidGains.of(0.2, 0.0, 0.0);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(gains);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        gains.telemetry().get("p").set(0.45);
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(0.45, actions.motor(MOTOR).closedLoopRequest.config().p(), 1.0e-9);
    }

    @Test
    void mixedCapabilityFollowersPreventDeviceOffload() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR, CHILD_MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        actions.motor(CHILD_MOTOR).capabilities = MotorControlCapabilities.OPEN_LOOP_ONLY;
        actions.encoder(MOTOR.encoder()).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .follower(CHILD_MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.4, actions.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(0.4, actions.motor(CHILD_MOTOR).voltage, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
        assertEquals(null, actions.motor(CHILD_MOTOR).closedLoopRequest);
    }

    @Test
    void absoluteMotorEncoderDoesNotOffloadWithoutFeedbackSelection() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        actions.encoder(MOTOR.absoluteEncoder()).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.absoluteEncoder())
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.4, actions.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
    }

    @Test
    void positionStaticFeedforwardUsesAthenaDefinedErrorSign() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        actions.encoder(MOTOR.encoder()).position = 0.0;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.2, 0.0, 0.0)
                .ff(1.0, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(1.4, actions.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
    }

    @Test
    void deviceClosedLoopUsesDeclaredMotorSlot() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        ControlBinding control = Controls.position(MOTOR)
                .slot(2)
                .feedback(MOTOR.encoder())
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(2, actions.motor(MOTOR).closedLoopRequest.config().slot());
        assertEquals(ControlRoute.DEVICE_CLOSED_LOOP, actions.motor(MOTOR).closedLoopRequest.route());
    }

    @Test
    void arbitraryFeedforwardLoopCanHybridizeWithDeviceClosedLoop() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        ControlBinding control = Controls.velocity(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.1, 0.0, 0.0)
                .loop(ControlLoops.arbitraryFeedforward(binding -> context -> ControlOutput.voltage(1.5)));
        TestMechanism mechanism = new TestMechanism(control.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(3.0, actions.motor(MOTOR).velocityTarget, 1.0e-9);
        assertEquals(1.5, actions.motor(MOTOR).closedLoopRequest.arbitraryFeedforwardVolts(), 1.0e-9);
        assertEquals(ControlRoute.HYBRID_CLOSED_LOOP, actions.motor(MOTOR).closedLoopRequest.route());
    }

    @Test
    void unclassifiedCustomLoopFallsBackToAthenaClosedLoop() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(MOTOR.encoder())
                .loop(binding -> context -> ControlOutput.percent(0.25));
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
    }

    @Test
    void mixedFeedbackBindingDoesNotUseDeviceNativeClosedLoop() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(MOTOR.encoder()).position = 0.5;
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        FeedbackBinding feedback = new FeedbackBinding(MOTOR.encoder(), () -> 0.0);
        ControlBinding control = Controls.position(MOTOR)
                .feedback(feedback)
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.3, actions.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(null, actions.motor(MOTOR).closedLoopRequest);
    }

    @Test
    void deviceClosedLoopClearsArbitraryFeedforwardWhenHybridLoopIsNotActive() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.motor(MOTOR).capabilities = MotorControlCapabilities.voltageClosedLoop(4);
        ControlBinding hybrid = Controls.velocity(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.1, 0.0, 0.0)
                .loop(ControlLoops.arbitraryFeedforward(binding -> context -> ControlOutput.voltage(1.5)));
        ControlBinding deviceOnly = Controls.velocity(MOTOR)
                .feedback(MOTOR.encoder())
                .pid(0.1, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(hybrid.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);
        runtime.set(mechanism.initial);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.set(deviceOnly.velocity(3.0));
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(0.0, actions.motor(MOTOR).closedLoopRequest.arbitraryFeedforwardVolts(), 1.0e-9);
        assertEquals(ControlRoute.DEVICE_CLOSED_LOOP, actions.motor(MOTOR).closedLoopRequest.route());
    }

    @Test
    void robotRuntimeStepsDiscoveredSimulationModels() {
        SimulatedMechanism mechanism = new SimulatedMechanism();
        SimulationSession session = SimulationSession.create()
                .model("mechanism", mechanism.simulation);
        MechanismScheduler runtime = MechanismScheduler.create(session.hardwareGraph())
                .register(mechanism);

        runtime.request(mechanism.initial);
        runtime.simulationPeriodic(0.0, 0.2);
        session.step(0.2);

        EncoderHandle encoder = session.hardwareGraph().encoder(mechanism.encoder);
        assertTrue(encoder.positionRotations() > 0.0);
        assertTrue(encoder.velocityRotationsPerSecond() > 0.0);
    }

    private static MechanismContext contextAt(double nowSeconds) {
        return new MechanismContext(nowSeconds, 0.0, 0.02, true, false, false);
    }

    private static MechanismContext disabledContextAt(double nowSeconds) {
        return new MechanismContext(nowSeconds, 0.0, 0.02, false, false, false);
    }

    private static EventContext disabledEventAt(double nowSeconds) {
        return new EventContext(
                nowSeconds,
                0.02,
                LifecycleMode.DISABLED,
                LifecyclePhase.PERIODIC,
                false,
                false);
    }

    private static final class TestMechanism implements Mechanism {
        private final Action initial;
        private Action secondary;

        private TestMechanism(Action initial) {
            this.initial = initial;
        }
    }

    private static final class VelocityCompositeMechanism implements Mechanism {
        private final ControlBinding speed = Controls.velocity(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0)
                .constraints(
                        Constraints.range(Range.of(0.0, 100.0)),
                        Constraints.motion(100.0, 200.0),
                        Constraints.clamp(12.0));
        private final Action collect = speed.velocity(10.0);
        private final Action shoot = speed.velocity(33.0);
    }

    private static final class ParentMechanism implements Mechanism {
        private final Action initial = Actions.neutral();
        private final TestMechanism child;

        private ParentMechanism(TestMechanism child) {
            this.child = child;
        }
    }

    private static final class MultiParentMechanism implements Mechanism {
        private final DeclaredMotorMechanism first;
        private final DeclaredMotorMechanism second;

        private MultiParentMechanism(DeclaredMotorMechanism first, DeclaredMotorMechanism second) {
            this.first = first;
            this.second = second;
        }
    }

    private static final class DeclaredMotorMechanism implements Mechanism {
        private final MotorDevice motor;
        private final Action initial;

        private DeclaredMotorMechanism(MotorDevice motor) {
            this.motor = motor;
            initial = motor.percent(0.0);
        }
    }

    private static final class DeclaredParentMechanism implements Mechanism {
        private final DeclaredMotorMechanism child;
        private final Action initial = Actions.neutral();

        private DeclaredParentMechanism(DeclaredMotorMechanism child) {
            this.child = child;
        }
    }

    private static final class DeviceMechanism implements Mechanism {
        private final MotorDevice motor = MOTOR;
        private final Action initial = Actions.neutral();
    }

    private static final class PositionControlMechanism implements Mechanism {
        private final MotorDevice motor = MOTOR;
        private final EncoderDevice encoder = ENCODER;
        private final ControlBinding position = Controls.position(motor).feedback(encoder);
        private final Action initial = Actions.neutral();
    }

    private static final class EncoderPositionMechanism implements Mechanism {
        private final EncoderDevice encoder = ENCODER;
        private final DeviceAction setPosition = encoder.setPosition(2.5);
    }

    private static final class ExternalActions {
        private final Action drive;

        private ExternalActions(DeviceMechanism mechanism) {
            drive = mechanism.motor.percent(0.65);
        }
    }

    private static final class SimulatedMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 21);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 21);
        private final SimModel simulation = SimModel.motor(motor).encoder(encoder);
        private final Action initial = motor.percent(1.0);
    }

    private static final class LazyHookDrive implements Mechanism {
        private final MotorDevice motor = MOTOR;

        private Action drive() {
            return motor.percent(0.65);
        }
    }

    private static final class LazyHookControls implements Mechanism {
        private final HookBinding driverControl;

        private LazyHookControls(LazyHookDrive drive, AtomicInteger callbacks) {
            driverControl = Events.teleopPeriodic().whileActive(() -> {
                callbacks.incrementAndGet();
                drive.drive().request();
            });
        }
    }

    private static final class LazyHookRoot implements Mechanism {
        private final LazyHookDrive drive;
        private final LazyHookControls controls;

        private LazyHookRoot(LazyHookDrive drive, AtomicInteger callbacks) {
            this.drive = drive;
            controls = new LazyHookControls(drive, callbacks);
        }
    }

    private static final class ComputedHookRoot implements Mechanism {
        private final MotorDevice motor = MOTOR;
        private final Action drive;
        private final HookBinding driverControl;

        private ComputedHookRoot(double[] output) {
            drive = Actions.compute(() -> Actions.parallel(motor.percent(output[0])), motor);
            driverControl = Events.teleopPeriodic().whileActive(drive);
        }
    }

    private static final class HookedMechanism implements Mechanism {
        private final Action idle = Actions.neutral();
        private final HookBinding hook;
        private int starts;
        private int whileActive;
        private int ends;

        private HookedMechanism(boolean[] signal) {
            hook = Events.when(() -> signal[0]).rising()
                    .onStart(() -> starts++)
                    .whileActive(() -> whileActive++)
                    .onEnd(() -> ends++);
        }
    }

    private static final class DigitalHookedMechanism implements Mechanism {
        private final Action idle = Actions.neutral();
        private final HookBinding hook;
        private int starts;
        private int ends;

        private DigitalHookedMechanism(DigitalInputDevice input) {
            hook = Events.when(input).rising()
                    .onStart(() -> starts++)
                    .onEnd(() -> ends++);
        }
    }

    private static final class CountingHookedMechanism implements Mechanism {
        private final Action idle = Actions.neutral();
        private final HookBinding hook;
        private int starts;

        private CountingHookedMechanism(boolean[] signal, AtomicInteger reads) {
            hook = Events.when(() -> {
                        reads.incrementAndGet();
                        return signal[0];
                    })
                    .rising()
                    .onStart(() -> starts++);
        }
    }

    private static final class SharedEventHookedMechanism implements Mechanism {
        private final Action idle = Actions.neutral();
        private final EventBinding event;
        private final HookBinding first;
        private final HookBinding second;
        private int firstStarts;
        private int secondStarts;

        private SharedEventHookedMechanism(boolean[] signal) {
            event = Events.when(() -> signal[0]).rising();
            first = event.onStart(() -> firstStarts++);
            second = event.onStart(() -> secondStarts++);
        }
    }

    private static final class SharedCustomEventHookedMechanism implements Mechanism {
        private final Action idle = Actions.neutral();
        private final HookBinding first;
        private final HookBinding second;
        private int firstStarts;
        private int secondStarts;

        private SharedCustomEventHookedMechanism(CountingEvent event) {
            first = event.onStart(() -> firstStarts++);
            second = event.onStart(() -> secondStarts++);
        }
    }

    private static final class CountingEvent implements EventBinding {
        private final boolean active;
        private final AtomicInteger reads = new AtomicInteger();

        private CountingEvent(boolean active) {
            this.active = active;
        }

        @Override
        public String name() {
            return "counting";
        }

        @Override
        public boolean sourceActive(EventContext context) {
            reads.incrementAndGet();
            return active;
        }
    }

    private static final class ResetCountingLoop implements ControlLoop {
        private final Runtime runtime = new Runtime();

        @Override
        public ControlLoopRuntime bind(ControlLoopBinding binding) {
            return runtime;
        }

        private static final class Runtime implements ControlLoopRuntime {
            private int resets;
            private int calculations;

            @Override
            public void reset(ControlLoopContext context) {
                resets++;
            }

            @Override
            public ControlOutput calculate(ControlLoopContext context) {
                calculations++;
                return ControlOutput.percent(0.0);
            }
        }
    }

    private static final class RecordingPathRuntime implements PathRuntime {
        private final int finishAfterExecutes;
        private int initializes;
        private int executes;
        private int ends;

        private RecordingPathRuntime(int finishAfterExecutes) {
            this.finishAfterExecutes = finishAfterExecutes;
        }

        @Override
        public void initialize(PathAction path, MechanismContext context) {
            initializes++;
        }

        @Override
        public void execute(PathAction path, MechanismContext context) {
            executes++;
        }

        @Override
        public boolean isFinished(PathAction path, MechanismContext context) {
            return executes >= finishAfterExecutes;
        }

        @Override
        public void end(PathAction path, MechanismContext context, boolean interrupted) {
            ends++;
        }
    }

    private static final class RecordingActionContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotorHandle> motors = new HashMap<>();
        private final Map<EncoderDevice, RecordingEncoderHandle> encoders = new HashMap<>();
        private int events;

        private RecordingActionContext(MotorDevice... motors) {
            for (MotorDevice motor : motors) {
                this.motors.put(motor, new RecordingMotorHandle(motor));
            }
        }

        @Override
        public RecordingMotorHandle motor(MotorDevice ref) {
            return motors.computeIfAbsent(ref, RecordingMotorHandle::new);
        }

        @Override
        public RecordingEncoderHandle encoder(EncoderDevice ref) {
            return encoders.computeIfAbsent(ref, RecordingEncoderHandle::new);
        }
    }

    private static final class RecordingMotorHandle implements MotorHandle {
        private final MotorDevice device;
        private double percent = Double.NaN;
        private double voltage = Double.NaN;
        private double positionTarget = Double.NaN;
        private double velocityTarget = Double.NaN;
        private MotorControlCapabilities capabilities = MotorControlCapabilities.OPEN_LOOP_ONLY;
        private MotorClosedLoopRequest closedLoopRequest;
        private int reads;

        private RecordingMotorHandle(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override
        public void setVoltage(double volts) {
            this.voltage = volts;
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            positionTarget = rotations;
        }

        @Override
        public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
            positionTarget = rotations;
            closedLoopRequest = request;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocityTarget = rotationsPerSecond;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond, MotorClosedLoopRequest request) {
            velocityTarget = rotationsPerSecond;
            closedLoopRequest = request;
        }

        @Override
        public MotorControlCapabilities controlCapabilities() {
            return capabilities;
        }

        @Override
        public double integratedPositionRotations() {
            reads++;
            return 0.0;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            reads++;
            return 0.0;
        }

        @Override
        public double appliedVoltage() {
            reads++;
            return Double.isFinite(voltage) ? voltage : 0.0;
        }

        @Override
        public double supplyCurrentAmps() {
            reads++;
            return 0.0;
        }

        @Override
        public double statorCurrentAmps() {
            reads++;
            return 0.0;
        }
    }

    private static final class RecordingEncoderHandle implements EncoderHandle {
        private final EncoderDevice device;
        private double position;
        private double velocity;
        private int setPositionCalls;
        private int reads;

        private RecordingEncoderHandle(EncoderDevice device) {
            this.device = device;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            reads++;
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            reads++;
            return velocity;
        }

        @Override
        public void setPositionRotations(double rotations) {
            position = rotations;
            setPositionCalls++;
        }
    }

    private static final class EncoderZeroingMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 58);
        private final EncoderDevice encoder = motor.encoder().gearRatio(1.0 / 9.0);
        private final DigitalInputDevice home;
        private final ControlBinding position = Controls.position(motor).feedback(encoder);
        private final Action armOut = position.position(0.35);
        private final HookBinding zeroAtHome;

        private EncoderZeroingMechanism(DigitalInputDevice home) {
            this.home = home;
            zeroAtHome = Events.when(home).rising().onStart(encoder.setPosition(0.0));
        }
    }

    private static final class TriggerReleaseMechanism implements Mechanism {
        private final ControlBinding speed = Controls.velocity(MOTOR);
        private final Action running = speed.percent(0.8);
        private final Action coast = speed.neutral();
        private final HookBinding trigger;

        private TriggerReleaseMechanism(boolean[] active) {
            trigger = Events.when(() -> active[0]).active()
                    .whileActive(running)
                    .onEnd(coast);
        }
    }

    private static final class DisabledMotorMechanism implements Mechanism {
        private final MotorDevice motor = MOTOR.disabled();
        private final Action move = Actions.percent(motor, 0.7);
    }

    private static final class DisabledControlMechanism implements Mechanism {
        private final ControlBinding control = Controls.position(MOTOR).disabled();
        private final Action move = control.position(1.0);
    }

    private static final class DisabledHookMechanism implements Mechanism {
        private final HookBinding hook;

        private DisabledHookMechanism(AtomicInteger calls) {
            hook = Events.teleopPeriodic().whileActive(calls::incrementAndGet).disabled();
        }
    }

    private static final class RecordingMotorBackend implements MotorBackend {
        private final RecordingMotorHandle handle = new RecordingMotorHandle(MOTOR);
        private int created;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.KRAKEN_X60;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            created++;
            return handle;
        }
    }
}
