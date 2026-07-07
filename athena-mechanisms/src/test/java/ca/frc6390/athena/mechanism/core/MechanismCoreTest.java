package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.ref.ActionContext;
import ca.frc6390.athena.hardware.ref.BooleanRef;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.DigitalInputs;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.MappedActionContext;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.NumberRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.hardware.ref.RuntimeBoolean;
import ca.frc6390.athena.hardware.ref.RuntimeEncoder;
import ca.frc6390.athena.hardware.ref.RuntimeMotor;
import ca.frc6390.athena.hardware.ref.RuntimeNumber;
import ca.frc6390.athena.hardware.ref.Sim;
import ca.frc6390.athena.hardware.ref.SimRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.util.List;
import org.junit.jupiter.api.Test;

class MechanismCoreTest {
    @Test
    void boundOutputStatesCarryTheirOutputIntent() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        AxisRef positionControl = Axes.position()
                .motor(motor)
                .encoder(encoder)
                .pid(0.1, 0.0, 0.0);
        MechanismState running = States.percent(motor, 0.75);
        MechanismState voltage = positionControl.voltage(6.0);
        MechanismState aimed = positionControl.position(42.0);

        Output.Percent percent = assertInstanceOf(Output.Percent.class, running);
        Output.Voltage volts = assertInstanceOf(Output.Voltage.class, voltage);
        Output.Position position = assertInstanceOf(Output.Position.class, aimed);
        States.MotorPercent motorPercent = assertInstanceOf(States.MotorPercent.class, running);
        States.AxisVoltage axisVoltage = assertInstanceOf(States.AxisVoltage.class, voltage);
        States.AxisPosition axisPosition = assertInstanceOf(States.AxisPosition.class, aimed);

        assertEquals(0.75, percent.percent(), 1.0e-9);
        assertEquals(6.0, volts.volts(), 1.0e-9);
        assertEquals(42.0, position.position(), 1.0e-9);
        assertSame(motor, motorPercent.motor());
        assertSame(positionControl, axisVoltage.axis());
        assertSame(positionControl, axisPosition.axis());
    }

    @Test
    void childSetStateCarriesCompositeTargetsDirectly() {
        TestMechanism feeder = new TestMechanism(States.neutral());
        TestMechanism kicker = new TestMechanism(States.neutral());
        MotorRef feederMotor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef kickerVelocity = Axes.velocity()
                .motor(MotorRef.of(AthenaMotor.SIM, 2))
                .encoder(EncoderRef.of(AthenaEncoder.SIM, 3));
        MechanismState feeding = States.percent(feederMotor, 1.0);
        MechanismState kicking = kickerVelocity.velocity(1200.0);

        States.ChildSet set = States.set()
                .set(feeder, feeding)
                .set(kicker, kicking);

        assertEquals(2, set.targets().size());
        assertSame(feeder, set.targets().get(0).mechanism());
        assertSame(feeding, set.targets().get(0).state());
        assertSame(kicker, set.targets().get(1).mechanism());
        assertSame(kicking, set.targets().get(1).state());

        MechanismState stopped = States.percent(feederMotor, 0.0);
        States.ChildSet inherited = States.from(set)
                .set(feeder, stopped);

        assertEquals(3, inherited.targets().size());
        assertSame(set.targets().get(0).mechanism(), inherited.targets().get(0).mechanism());
        assertSame(set.targets().get(0).state(), inherited.targets().get(0).state());
        assertSame(stopped, inherited.targets().get(2).state());
    }

    @Test
    void conditionsAndSequencesUseLifecycleContextOnly() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        AxisRef positionControl = Axes.position()
                .motor(motor)
                .encoder(encoder);
        MechanismState stowed = positionControl.position(0.0);
        MechanismState homing = States.percent(motor, 0.25)
                .until(ctx -> ctx.timeInStateSeconds() > 0.5)
                .then(stowed);

        MechanismState.Conditional conditional = assertInstanceOf(MechanismState.Conditional.class, homing);
        assertSame(stowed, conditional.next());
        assertTrue(conditional.condition().test(new MechanismContext(1.0, 0.6, 0.02, true, false, false)));

        States.Sequence sequence = States.sequence()
                .forTime(0.2, States.percent(motor, -0.2))
                .until(ctx -> ctx.timeInStateSeconds() > 0.4, States.percent(motor, 0.1))
                .then(stowed);

        assertEquals(2, sequence.steps().size());
        assertSame(stowed, sequence.next());
        assertTrue(sequence.steps().get(0).complete().test(new MechanismContext(1.0, 0.25, 0.02, true, false, false)));
        assertTrue(sequence.steps().get(1).complete().test(new MechanismContext(1.0, 0.5, 0.02, true, false, false)));
    }

    @Test
    void controlledStatesAndClampsCarryExplicitRefs() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = EncoderRef.of(AthenaEncoder.SIM, 2);
        RangeRef range = RangeRef.degrees(-45.0, 90.0);
        AxisRef control = Axes.position()
                .motor(motor)
                .encoder(encoder)
                .range(range);

        MechanismState aimed = States.position(control, 120.0).clamp(range);
        MechanismState.Clamped clamped = assertInstanceOf(MechanismState.Clamped.class, aimed);
        States.AxisPosition position = assertInstanceOf(States.AxisPosition.class, clamped.state());

        assertSame(control, position.axis());
        assertSame(motor, control.motors().get(0));
        assertSame(encoder, control.encoders().get(0));
        assertSame(range, clamped.range());
    }

    @Test
    void sequenceCanCarryDoOnceActions() {
        boolean[] ran = {false};

        States.Sequence sequence = States.sequence()
                .doOnce(() -> ran[0] = true)
                .then(States.neutral());

        States.DoOnce action = assertInstanceOf(States.DoOnce.class, sequence.steps().get(0).state());
        action.action().run();

        assertTrue(ran[0]);
    }

    @Test
    void stateFlowPrimitivesCarryBranchingWaitingTimeoutsAndActions() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        boolean[] ready = {false};
        boolean[] ran = {false};
        MechanismState active = States.percent(motor, 0.5);

        States.WhenBranch branch = States.when(() -> ready[0]).run(active);
        assertInstanceOf(Output.Neutral.class, branch.choose(MechanismContext.empty()));
        ready[0] = true;
        assertSame(active, branch.choose(MechanismContext.empty()));

        States.Choice choice = branch.otherwise(States.neutral());
        assertSame(active, choice.choose(MechanismContext.empty()));

        States.WaitSeconds wait = assertInstanceOf(States.WaitSeconds.class, States.waitSeconds(1.5));
        assertFalse(wait.complete(new MechanismContext(1.0, 1.0, 0.02, true, false, false)));
        assertTrue(wait.complete(new MechanismContext(2.0, 1.5, 0.02, true, false, false)));

        States.WaitUntil waitUntil = assertInstanceOf(States.WaitUntil.class, States.waitUntil(() -> ready[0]));
        assertTrue(waitUntil.complete(MechanismContext.empty()));

        States.Timeout timeout = assertInstanceOf(States.Timeout.class, States.timeout(active, 2.0));
        assertSame(active, timeout.state());
        assertTrue(timeout.expired(new MechanismContext(3.0, 2.0, 0.02, true, false, false)));

        States.Action action = assertInstanceOf(States.Action.class, States.doOnce(ctx -> ran[0] = true));
        action.action().apply(ActionContext.empty());
        assertTrue(ran[0]);

        States.Sequence sequence = States.sequence()
                .doOnce(ctx -> ran[0] = false)
                .waitSeconds(0.25)
                .waitUntil(() -> ready[0])
                .run(active)
                .timeout(5.0)
                .then(States.neutral());

        assertEquals(4, sequence.steps().size());
        assertEquals(5.0, sequence.timeoutSeconds(), 1.0e-9);
        assertInstanceOf(States.Action.class, sequence.steps().get(0).state());
        assertInstanceOf(States.WaitSeconds.class, sequence.steps().get(1).state());
        assertInstanceOf(States.WaitUntil.class, sequence.steps().get(2).state());
        assertSame(active, sequence.steps().get(3).state());
    }

    @Test
    void pathsAreMechanismStatesAndCanBeSequenced() {
        PathRef path = Paths.choreo("approach").seconds(1.25);
        MechanismState routine = States.sequence()
                .run(path)
                .then(States.neutral());

        States.Sequence sequence = assertInstanceOf(States.Sequence.class, routine);
        assertSame(path, sequence.steps().get(0).state());
        assertEquals("choreo", path.source());
        assertEquals("approach", path.name());
        assertEquals("choreo:approach", path.key());
        assertEquals(1.25, path.expectedDurationSeconds().orElseThrow(), 1.0e-9);
    }

    @Test
    void pathIntrospectorDiscoversNestedAutoPathRefs() {
        AutoDeclarations autos = new AutoDeclarations();

        List<PathRef> paths = PathIntrospector.inspect(autos);

        assertEquals(List.of(autos.test.leave, autos.score.score), paths);
    }

    @Test
    void dynamicBoundStatesReadLatestSupplierValues() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        AxisRef velocityControl = Axes.velocity()
                .motor(motor)
                .encoder(encoder);
        double[] target = {10.0};

        MechanismState velocity = velocityControl.velocity(() -> target[0]);
        Output.Velocity output = assertInstanceOf(Output.Velocity.class, velocity);

        assertEquals(10.0, output.velocity(), 1.0e-9);
        target[0] = 42.0;
        assertEquals(42.0, output.velocity(), 1.0e-9);
    }

    @Test
    void controlStatesCarryOutputFeedbackAndLoops() {
        MotorRef drive = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef follower = MotorRef.of(AthenaMotor.SIM, 2);
        EncoderRef feedback = drive.encoder();
        ControlRef loop = Control.velocity(drive)
                .follower(follower)
                .feedback(feedback)
                .pid(0.2, 0.0, 0.0)
                .ff(0.0, 1.0, 0.0);

        MechanismState state = loop.velocity(12.0);
        States.ControlVelocity velocity = assertInstanceOf(States.ControlVelocity.class, state);

        assertSame(loop, velocity.control());
        assertEquals(12.0, velocity.velocity(), 1.0e-9);
        assertEquals(List.of(drive, follower), loop.motors());
        assertSame(feedback, loop.feedback().get(0));
        assertEquals(2, loop.loops().size());
    }

    @Test
    void loopRefsBindRuntimeLogicDirectly() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        ControlRef control = Control.position(motor);
        PidRef pid = PidRef.of(0.1, 0.0, 0.0);
        FeedforwardRef feedforward = FeedforwardRef.simple(0.2, 0.5, 0.0);
        ControlLoopBinding binding = new ControlLoopBinding(control, ActionContext.empty());

        ControlOutput.Percent pidOutput = assertInstanceOf(
                ControlOutput.Percent.class,
                pid.bind(binding).calculate(loopContext(Outputs.position(10.0), 4.0, 0.0)));
        ControlOutput.Voltage ffOutput = assertInstanceOf(
                ControlOutput.Voltage.class,
                feedforward.bind(binding).calculate(loopContext(Outputs.velocity(6.0), 0.0, 0.0)));

        assertEquals(0.6, pidOutput.value(), 1.0e-9);
        assertEquals(3.2, ffOutput.value(), 1.0e-9);
    }

    @Test
    void cycleStatesCarryRepeatingSteps() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);

        States.Cycle cycle = States.cycle()
                .forTime(3.0, States.percent(motor, 0.5))
                .forTime(0.25, States.percent(motor, -0.2));

        assertEquals(2, cycle.steps().size());
        assertTrue(cycle.steps().get(0).advance().test(new MechanismContext(1.0, 3.1, 0.02, true, false, false)));
    }

    @Test
    void rulesEvaluateAgainstTargetsAndReturnResults() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder());
        RuleRef range = Rules.range(RangeRef.degrees(-10.0, 10.0))
                .appliesTo(OutputTarget.axis(axis).position())
                .clamp("outside arm travel");
        RuleContext context = new SimpleRuleContext(axis, Outputs.position(14.0), 0.0, 0.0);

        RuleResult result = range.evaluate(context);

        assertEquals(RuleResult.Decision.CLAMP, result.decision());
        assertEquals(10.0, result.value().orElseThrow(), 1.0e-9);
        assertEquals("mechanism request was clamped: outside arm travel", result.message());
        assertTrue(OutputTarget.axis(axis).closedLoop().matches(OutputRequest.of(axis, Outputs.position(1.0))));
        assertFalse(OutputTarget.axis(axis).negative().matches(OutputRequest.of(axis, Outputs.percent(0.5))));
    }

    @Test
    void axisRefsCanCarryLocalRules() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        RuleRef hardstop = Rules.when(() -> true)
                .appliesTo(OutputTarget.any().openLoop().positive())
                .block()
                .named("home hardstop");
        AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder())
                .rule(hardstop);

        assertEquals(1, axis.rules().size());
        assertSame(hardstop, axis.rules().get(0));
        assertEquals("home hardstop", axis.rules().get(0).name());
    }

    @Test
    void outputResolverAppliesAxisRulesBeforeOutputsReachHardware() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder())
                .rule(Rules.range(RangeRef.degrees(-5.0, 5.0)).clamp("outside axis travel"));

        ResolvedOutput resolved = OutputResolver.empty()
                .resolve(OutputRequest.of(axis, Outputs.position(9.0)), AxisStateSource.empty());

        Output.Position output = assertInstanceOf(Output.Position.class, resolved.output());
        assertEquals(5.0, output.position(), 1.0e-9);
        assertTrue(resolved.clamped());
        assertFalse(resolved.blocked());
        assertEquals("mechanism request was clamped: outside axis travel", resolved.messages().get(0));
    }

    @Test
    void outputResolverBlocksHigherPriorityThanClamp() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef axis = Axes.percent()
                .motor(motor)
                .rule(Rules.when(() -> true)
                        .appliesTo(OutputTarget.any().openLoop().positive())
                        .block("home switch active"))
                .rule(Rules.when(() -> true)
                        .appliesTo(OutputTarget.any().percent())
                        .clamp(0.25, "slow zone"));

        ResolvedOutput resolved = OutputResolver.empty()
                .resolve(OutputRequest.of(axis, Outputs.percent(1.0)), AxisStateSource.empty());

        assertInstanceOf(Output.Neutral.class, resolved.output());
        assertTrue(resolved.blocked());
        assertFalse(resolved.clamped());
        assertEquals("mechanism request was blocked: home switch active", resolved.messages().get(0));
    }

    @Test
    void blockedAxisCanHoldCurrentPosition() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder())
                .onBlocked(BlockPolicy.HOLD_POSITION)
                .rule(Rules.when(() -> true)
                        .appliesTo(OutputTarget.any().closedLoop())
                        .block("blocked"));

        ResolvedOutput resolved = OutputResolver.empty()
                .resolve(
                        OutputRequest.of(axis, Outputs.position(100.0)),
                        AxisStateSource.of(candidate -> candidate == axis ? axisState(42.0, 3.0) : null));

        Output.Position hold = assertInstanceOf(Output.Position.class, resolved.output());
        assertEquals(42.0, hold.position(), 1.0e-9);
        assertTrue(resolved.blocked());
    }

    @Test
    void outputResolverAppliesCompositeRulesToChildAxisRequests() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder());
        RuleRef parentRule = Rules.when(ctx -> ctx.axis(axis).position() < 10.0)
                .appliesTo(OutputTarget.axis(axis).position().negative())
                .block("child axis would move into parent limit");
        ChildMechanism child = new ChildMechanism(axis);
        ParentMechanism parent = new ParentMechanism(child, parentRule);

        var outputs = OutputResolver.empty()
                .resolve(
                        parent,
                        parent.DOWN,
                        AxisStateSource.of(candidate -> candidate == axis ? axisState(4.0, 0.0) : null));

        assertEquals(1, outputs.size());
        assertTrue(outputs.get(0).blocked());
        assertInstanceOf(Output.Neutral.class, outputs.get(0).output());
        assertEquals("mechanism request was blocked: child axis would move into parent limit",
                outputs.get(0).messages().get(0));
    }

    @Test
    void outputResolverPreservesDirectMotorTargets() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        TestMechanism mechanism = new TestMechanism(States.neutral());

        var outputs = OutputResolver.empty()
                .resolve(mechanism, States.percent(motor, 0.6), AxisStateSource.empty());

        assertEquals(1, outputs.size());
        assertSame(motor, outputs.get(0).request().motor());
        assertInstanceOf(Output.Percent.class, outputs.get(0).output());
    }

    @Test
    void outputResolverResolvesComposedStatesThroughExistingOutputPath() {
        MotorRef left = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef right = MotorRef.of(AthenaMotor.SIM, 2);
        TestMechanism mechanism = new TestMechanism(States.neutral());
        OutputResolver resolver = OutputResolver.empty();
        MechanismContext running = new MechanismContext(1.0, 0.5, 0.02, true, false, false);

        MechanismState branch = States.when(() -> true)
                .run(States.percent(left, 0.4))
                .otherwise(States.percent(right, 0.1));
        var branchOutputs = resolver.resolve(mechanism, branch, running, AxisStateSource.empty());

        assertEquals(1, branchOutputs.size());
        assertSame(left, branchOutputs.get(0).request().motor());
        assertEquals(0.4, assertInstanceOf(Output.Percent.class, branchOutputs.get(0).output()).percent(), 1.0e-9);

        MechanismState parallel = States.parallel(
                States.percent(left, 0.4),
                States.voltage(right, 6.0));
        var parallelOutputs = resolver.resolve(mechanism, parallel, running, AxisStateSource.empty());

        assertEquals(2, parallelOutputs.size());
        assertSame(left, parallelOutputs.get(0).request().motor());
        assertSame(right, parallelOutputs.get(1).request().motor());

        MechanismState deadline = States.deadline(
                States.percent(left, 0.4),
                States.voltage(right, 6.0));
        assertEquals(2, resolver.resolve(mechanism, deadline, running, AxisStateSource.empty()).size());

        MechanismState timeout = States.timeout(States.percent(left, 0.4), 0.25);
        assertEquals(0, resolver.resolve(mechanism, timeout, running, AxisStateSource.empty()).size());

        States.Sequence sequence = States.sequence()
                .forTime(0.25, States.percent(left, 0.4))
                .then(States.voltage(right, 6.0));
        var sequenceOutputs = resolver.resolve(mechanism, sequence, running, AxisStateSource.empty());

        assertEquals(1, sequenceOutputs.size());
        assertSame(left, sequenceOutputs.get(0).request().motor());
    }

    @Test
    void motorRefsCreateDirectOutputStates() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MechanismState percent = motor.percent(0.5);
        MechanismState voltage = motor.voltage(6.0);

        var percentOutputs = OutputResolver.empty().resolve(mechanism, percent, AxisStateSource.empty());
        var voltageOutputs = OutputResolver.empty().resolve(mechanism, voltage, AxisStateSource.empty());

        assertInstanceOf(MotorRef.PercentState.class, percent);
        assertInstanceOf(MotorRef.VoltageState.class, voltage);
        assertSame(motor, percentOutputs.get(0).request().motor());
        assertSame(motor, voltageOutputs.get(0).request().motor());
        assertEquals(0.5, assertInstanceOf(Output.Percent.class, percentOutputs.get(0).output()).percent(), 1.0e-9);
        assertEquals(6.0, assertInstanceOf(Output.Voltage.class, voltageOutputs.get(0).output()).volts(), 1.0e-9);
    }

    @Test
    void outputApplierSendsResolvedOutputsToRuntimeMotors() {
        MotorRef left = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef right = MotorRef.of(AthenaMotor.SIM, 2);
        AxisRef axis = Axes.position()
                .motor(left)
                .motor(right)
                .encoder(left.encoder());
        RecordingMotor leftRuntime = new RecordingMotor();
        RecordingMotor rightRuntime = new RecordingMotor();
        MappedActionContext context = new MappedActionContext()
                .motor(left, leftRuntime)
                .motor(right, rightRuntime);

        OutputApplier.using(context)
                .apply(new ResolvedOutput(OutputRequest.of(axis, Outputs.position(12.0)), Outputs.position(12.0), List.of()));

        assertEquals(12.0, leftRuntime.position, 1.0e-9);
        assertEquals(12.0, rightRuntime.position, 1.0e-9);
    }

    @Test
    void outputResolverAndApplierPreserveControlTargets() {
        MotorRef leader = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef follower = MotorRef.of(AthenaMotor.SIM, 2);
        ControlRef loop = Control.position(leader)
                .follower(follower)
                .feedback(leader.encoder())
                .pid(0.3, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(States.neutral());
        RecordingMotor leaderRuntime = new RecordingMotor();
        RecordingMotor followerRuntime = new RecordingMotor();
        MappedActionContext context = new MappedActionContext()
                .motor(leader, leaderRuntime)
                .motor(follower, followerRuntime);

        var outputs = OutputResolver.empty()
                .resolve(mechanism, loop.position(6.0), AxisStateSource.empty());
        OutputApplier.using(context).applyAll(outputs);

        assertSame(loop, outputs.get(0).request().control());
        assertEquals(6.0, leaderRuntime.position, 1.0e-9);
        assertEquals(6.0, followerRuntime.position, 1.0e-9);
    }

    @Test
    void outputApplierStopsMotorWhenResolvedOutputIsBlocked() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        RecordingMotor runtime = new RecordingMotor();
        MappedActionContext context = new MappedActionContext().motor(motor, runtime);
        ResolvedOutput blocked = new ResolvedOutput(
                OutputRequest.of(motor, Outputs.percent(1.0)),
                Outputs.neutral(),
                List.of(RuleResult.block("blocked")));

        OutputApplier.using(context).apply(blocked);

        assertEquals(0.0, runtime.percent, 1.0e-9);
        assertTrue(runtime.stopped);
    }

    @Test
    void mechanismRuntimeUpdatesResolvesAppliesHooksAndSimulationStep() {
        RuntimeMechanism mechanism = new RuntimeMechanism();
        RecordingMotor motor = new RecordingMotor();
        RecordingEncoder encoder = new RecordingEncoder(4.0);
        boolean[] simStepped = {false};
        MappedActionContext context = new MappedActionContext()
                .motor(mechanism.motor, motor)
                .encoder(mechanism.motor.encoder(), encoder);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, context)
                .simulationStep(() -> simStepped[0] = true);

        var outputs = runtime.periodic(
                new MechanismContext(1.0, 0.0, 0.02, true, false, false),
                EventContext.lifecycle(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC));

        assertEquals(1, outputs.size());
        assertEquals(7.0, motor.position, 1.0e-9);
        assertTrue(mechanism.ticked);
        assertTrue(simStepped[0]);
    }

    @Test
    void mechanismRuntimeAdvancesStateSequencesAndRunsActionsOnce() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        RecordingMotor runtimeMotor = new RecordingMotor();
        RecordingEncoder runtimeEncoder = new RecordingEncoder();
        int[] actions = {0};
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MechanismState routine = States.sequence()
                .doOnce(ctx -> {
                    actions[0]++;
                    ctx.encoder(encoder).set(3.0);
                })
                .forTime(0.1, States.percent(motor, 0.25))
                .then(States.voltage(motor, 5.0));
        MappedActionContext context = new MappedActionContext()
                .motor(motor, runtimeMotor)
                .encoder(encoder, runtimeEncoder);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, context);
        runtime.set(routine);

        runtime.periodic(new MechanismContext(1.0, 0.0, 0.02, true, false, false), EventContext.empty());
        runtime.periodic(new MechanismContext(1.04, 0.0, 0.02, true, false, false), EventContext.empty());
        runtime.periodic(new MechanismContext(1.16, 0.0, 0.02, true, false, false), EventContext.empty());

        assertEquals(1, actions[0]);
        assertEquals(3.0, runtimeEncoder.position(), 1.0e-9);
        assertEquals(0.25, runtimeMotor.percent, 1.0e-9);
        assertEquals(5.0, runtimeMotor.voltage, 1.0e-9);
    }

    @Test
    void mechanismRuntimeHandlesRaceDeadlineAndTimeoutCompletion() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        RecordingMotor runtimeMotor = new RecordingMotor();
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MappedActionContext context = new MappedActionContext().motor(motor, runtimeMotor);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, context);

        runtime.set(States.timeout(States.percent(motor, 0.4), 0.1)
                .then(States.voltage(motor, 6.0)));
        runtime.periodic(new MechanismContext(2.0, 0.0, 0.02, true, false, false), EventContext.empty());
        runtime.periodic(new MechanismContext(2.12, 0.0, 0.02, true, false, false), EventContext.empty());

        assertEquals(0.4, runtimeMotor.percent, 1.0e-9);
        assertEquals(6.0, runtimeMotor.voltage, 1.0e-9);

        runtime.set(States.deadline(
                States.waitSeconds(0.1),
                States.percent(motor, 0.8))
                .then(States.voltage(motor, 2.0)));
        runtime.periodic(new MechanismContext(3.0, 0.0, 0.02, true, false, false), EventContext.empty());
        runtime.periodic(new MechanismContext(3.12, 0.0, 0.02, true, false, false), EventContext.empty());

        assertEquals(0.8, runtimeMotor.percent, 1.0e-9);
        assertEquals(2.0, runtimeMotor.voltage, 1.0e-9);
    }

    @Test
    void mechanismRuntimeExecutesBoundPathStatesUntilComplete() {
        PathRef path = Paths.choreo("leave").seconds(0.1);
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        RecordingMotor runtimeMotor = new RecordingMotor();
        RecordingPathRuntime pathRuntime = new RecordingPathRuntime(0.1);
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MechanismRuntime runtime = MechanismRuntime.of(
                        mechanism,
                        new MappedActionContext().motor(motor, runtimeMotor),
                        OutputResolver.empty(),
                        AxisStateSource.empty())
                .path(path, pathRuntime);
        runtime.set(States.sequence()
                .run(path)
                .then(States.percent(motor, 0.7)));

        runtime.periodic(new MechanismContext(1.0, 0.0, 0.02, true, true, false), EventContext.empty());
        runtime.periodic(new MechanismContext(1.04, 0.0, 0.02, true, true, false), EventContext.empty());
        runtime.periodic(new MechanismContext(1.12, 0.0, 0.02, true, true, false), EventContext.empty());

        assertEquals(1, pathRuntime.initialized);
        assertEquals(3, pathRuntime.executed);
        assertEquals(1, pathRuntime.ended);
        assertEquals(0.7, runtimeMotor.percent, 1.0e-9);
    }

    @Test
    void mechanismRuntimeFailsWhenPathHasNoRuntimeBinding() {
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, ActionContext.empty());
        runtime.set(Paths.choreo("missing"));

        IllegalStateException error = assertThrows(
                IllegalStateException.class,
                () -> runtime.periodic(new MechanismContext(1.0, 0.0, 0.02, true, true, false), EventContext.empty()));

        assertTrue(error.getMessage().contains("choreo:missing"));
    }

    @Test
    void mechanismRegistryBindsDiscoveredPathRefs() {
        AutoDeclarations autos = new AutoDeclarations();
        TestMechanism mechanism = new TestMechanism(States.neutral());
        MechanismRegistry registry = MechanismRegistry.create()
                .register(mechanism)
                .paths(autos, path -> new RecordingPathRuntime(0.0));

        registry.set(mechanism, autos.test.leave);
        registry.autoPeriodic(1.0, 0.02);
    }

    @Test
    void mechanismRegistryRegistersBindsSetsAndRunsMechanisms() {
        RuntimeMechanism mechanism = new RuntimeMechanism();
        RecordingMotor motor = new RecordingMotor();
        RecordingEncoder encoder = new RecordingEncoder(2.0);
        boolean[] simStepped = {false};
        MechanismRegistry registry = MechanismRegistry.create()
                .register(mechanism)
                .motor(mechanism.motor, motor)
                .encoder(mechanism.motor.encoder(), encoder)
                .simulationStep(() -> simStepped[0] = true)
                .set(mechanism, mechanism.axis.position(11.0));

        var outputs = registry.teleopPeriodic(1.0, 0.02);

        assertEquals(1, registry.mechanisms().size());
        assertEquals(1, outputs.size());
        assertEquals(11.0, motor.position, 1.0e-9);
        assertTrue(mechanism.ticked);
        assertTrue(simStepped[0]);
        assertSame(mechanism.axis.position(11.0).getClass(), registry.state(mechanism).getClass());
    }

    @Test
    void mechanismRegistryCanBindInMemoryRuntimeForTestbeds() {
        RuntimeMechanism mechanism = new RuntimeMechanism();
        MechanismRegistry registry = MechanismRegistry.create()
                .register(mechanism)
                .bindInMemoryRuntime();

        var outputs = registry.teleopPeriodic(1.0, 0.02);

        assertEquals(1, outputs.size());
        assertTrue(mechanism.ticked);
    }

    @Test
    void simulationRefsDriveLinkedEncodersDuringSimulationPeriodic() {
        SimulatedArmMechanism mechanism = new SimulatedArmMechanism();
        MechanismRegistry registry = MechanismRegistry.create()
                .register(mechanism)
                .bindInMemoryRuntime()
                .set(mechanism, mechanism.OUT);

        registry.simulationPeriodic(0.0, 0.02);

        RuntimeEncoder encoder = registry.actionContext().encoder(mechanism.position);
        assertTrue(encoder.position() > 0.0);
        assertTrue(encoder.velocity() > 0.0);
    }

    @Test
    void simulationRefsCanDriveLinkedLimitInputs() {
        SimulatedArmMechanism mechanism = new SimulatedArmMechanism();
        MechanismRegistry registry = MechanismRegistry.create()
                .register(mechanism)
                .bindInMemoryRuntime()
                .set(mechanism, mechanism.OUT);

        for (int i = 0; i < 50; i++) {
            registry.simulationPeriodic(i * 0.02, 0.02);
        }

        assertEquals(1.0, registry.actionContext().encoder(mechanism.position).position(), 1.0e-9);
        assertTrue(mechanism.outLimit.active());
    }

    @Test
    void signalEventsCarryLevelAndEdgeSemantics() {
        boolean[] active = {false};
        EventRef level = Events.when(() -> active[0]).active();
        EventRef rising = Events.when(() -> active[0]).rising();
        EventRef falling = Events.when(() -> active[0]).falling();

        assertFalse(level.active(EventContext.empty(), false));
        active[0] = true;
        assertTrue(level.active(EventContext.empty(), false));
        assertTrue(rising.active(EventContext.empty(), false));
        assertFalse(rising.active(EventContext.empty(), true));
        active[0] = false;
        assertTrue(falling.active(EventContext.empty(), true));
        assertFalse(falling.active(EventContext.empty(), false));
    }

    @Test
    void lifecycleEventsCoverRobotModesAndPhases() {
        EventRef teleopInit = Events.teleopInit();
        EventRef autoPeriodic = Events.autoPeriodic();
        EventRef disabledExit = Events.disableExit();
        EventRef simPeriodic = Events.simPeriodic();

        assertTrue(teleopInit.active(EventContext.lifecycle(LifecycleMode.TELEOP, LifecyclePhase.INIT), false));
        assertTrue(autoPeriodic.active(
                EventContext.lifecycle(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC), false));
        assertTrue(disabledExit.active(EventContext.lifecycle(LifecycleMode.DISABLED, LifecyclePhase.EXIT), false));
        assertTrue(simPeriodic.active(
                EventContext.lifecycle(LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC), false));
        assertFalse(teleopInit.active(EventContext.lifecycle(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC), false));
        assertTrue(teleopInit.pulse());
    }

    @Test
    void hooksCarryNamedTriggerBindings() {
        boolean[] started = {false};
        boolean[] active = {false};
        boolean[] ended = {false};

        HookRef hook = Events.when(() -> true).active()
                .onStart(() -> started[0] = true)
                .whileActive(() -> active[0] = true)
                .onEnd(() -> ended[0] = true)
                .onInactive(() -> ended[0] = true)
                .whileInactive(() -> ended[0] = true);

        assertEquals(5, hook.bindings().size());
        assertEquals(HookTrigger.ON_START, hook.bindings().get(0).trigger());
        assertEquals(HookTrigger.WHILE_ACTIVE, hook.bindings().get(1).trigger());
        assertEquals(HookTrigger.ON_END, hook.bindings().get(2).trigger());
        assertEquals(HookTrigger.ON_INACTIVE, hook.bindings().get(3).trigger());
        assertEquals(HookTrigger.WHILE_INACTIVE, hook.bindings().get(4).trigger());
        assertTrue(HookTrigger.ON_START.shouldRun(hook.event(), false, true));
        assertTrue(HookTrigger.WHILE_ACTIVE.shouldRun(hook.event(), true, true));
        assertTrue(HookTrigger.ON_END.shouldRun(hook.event(), true, false));
        assertTrue(HookTrigger.ON_INACTIVE.shouldRun(hook.event(), true, false));
        assertTrue(HookTrigger.WHILE_INACTIVE.shouldRun(hook.event(), false, false));

        hook.bindings().forEach(binding -> binding.action().apply(ActionContext.empty()));
        assertTrue(started[0]);
        assertTrue(active[0]);
        assertTrue(ended[0]);

        boolean[] periodicRan = {false};
        HookRef periodic = Hooks.teleop.whileActive(() -> periodicRan[0] = true);
        assertTrue(HookTrigger.ON_START.shouldRun(periodic.event(), true, true));
        assertFalse(HookTrigger.ON_END.shouldRun(periodic.event(), true, false));
        periodic.bindings().get(0).action().apply(ActionContext.empty());
        assertTrue(periodicRan[0]);
    }

    @Test
    void hookActionsResolveRefsThroughActionContext() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        EncoderRef absolute = EncoderRef.of(AthenaEncoder.SIM, 2).absolute();
        BooleanRef ready = BooleanRef.value(false);
        NumberRef target = NumberRef.of("target", 0.0);
        RecordingActionContext context = new RecordingActionContext();

        List<HookRef> hooks = List.of(
                Hooks.robot.onStart(ctx -> ctx.encoder(encoder).zero()),
                Hooks.robot.onStart(ctx -> ctx.encoder(encoder).set(12.5)),
                Hooks.robot.onStart(ctx -> ctx.encoder(encoder).syncTo(ctx.encoder(absolute))),
                Hooks.robot.onStart(ctx -> ctx.motor(motor).stop()),
                Hooks.robot.onStart(ctx -> ctx.motor(motor).percent(0.25)),
                Hooks.robot.onStart(ctx -> ctx.motor(motor).voltage(6.0)),
                Hooks.robot.onStart(ctx -> ctx.motor(motor).brake()),
                Hooks.robot.onStart(ctx -> ctx.motor(motor).coast()),
                Hooks.robot.onStart(ctx -> ctx.bool(ready).set(true)),
                Hooks.robot.onStart(ctx -> ctx.number(target).set(42.0)));
        hooks.forEach(hook -> hook.bindings().forEach(binding -> binding.action().apply(context)));

        assertSame(encoder, context.encoderRef);
        assertEquals(18.0, context.encoder.position, 1.0e-9);
        assertSame(context.absoluteEncoder, context.encoder.syncedTo);
        assertSame(motor, context.motorRef);
        assertEquals(0.25, context.motor.percent, 1.0e-9);
        assertEquals(6.0, context.motor.voltage, 1.0e-9);
        assertTrue(context.motor.brake);
        assertTrue(context.motor.coast);
        assertSame(ready, context.booleanRef);
        assertTrue(context.bool.value);
        assertSame(target, context.numberRef);
        assertEquals(42.0, context.number.value, 1.0e-9);
    }

    @Test
    void hookRunnerExecutesLevelHooksOnStartWhileActiveAndOnEnd() {
        boolean[] signal = {false};
        int[] starts = {0};
        int[] active = {0};
        int[] ends = {0};
        int[] inactiveStarts = {0};
        int[] inactive = {0};
        HookRef hook = Hooks.when(Events.when(() -> signal[0]).active())
                .onStart(() -> starts[0]++)
                .whileActive(() -> active[0]++)
                .onEnd(() -> ends[0]++)
                .onInactive(() -> inactiveStarts[0]++)
                .whileInactive(() -> inactive[0]++);
        HookRunner runner = new HookRunner();

        runner.run(EventContext.empty(), ActionContext.empty(), hook);
        signal[0] = true;
        runner.run(EventContext.empty(), ActionContext.empty(), hook);
        runner.run(EventContext.empty(), ActionContext.empty(), hook);
        signal[0] = false;
        runner.run(EventContext.empty(), ActionContext.empty(), hook);

        assertEquals(1, starts[0]);
        assertEquals(2, active[0]);
        assertEquals(1, ends[0]);
        assertEquals(1, inactiveStarts[0]);
        assertEquals(2, inactive[0]);
    }

    @Test
    void hookRunnerExecutesPulseHooksWithoutSyntheticEnd() {
        int[] starts = {0};
        int[] active = {0};
        int[] ends = {0};
        List<HookRef> hooks = List.of(
                Hooks.teleop.whileActive(() -> starts[0]++),
                Hooks.teleop.whileActive(() -> active[0]++),
                Hooks.teleop.onEnd(() -> ends[0]++));
        HookRunner runner = new HookRunner();
        EventContext teleopPeriodic = EventContext.lifecycle(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC);

        runner.run(teleopPeriodic, ActionContext.empty(), hooks);
        runner.run(teleopPeriodic, ActionContext.empty(), hooks);

        assertEquals(2, starts[0]);
        assertEquals(2, active[0]);
        assertEquals(0, ends[0]);
    }

    @Test
    void mappedActionContextResolvesEquivalentRefs() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = motor.encoder();
        RecordingEncoder runtimeEncoder = new RecordingEncoder();
        MappedActionContext context = new MappedActionContext()
                .encoder(encoder, runtimeEncoder);

        EncoderRef equivalent = motor.encoder();
        Hooks.robot.onStart(ctx -> ctx.encoder(equivalent).set(7.0))
                .bindings()
                .get(0)
                .action()
                .apply(context);

        assertEquals(7.0, runtimeEncoder.position, 1.0e-9);
    }

    @Test
    void introspectorDiscoversRefsStatesAndChildrenByFieldName() {
        ExampleMechanism mechanism = new ExampleMechanism();

        MechanismDefinition definition = MechanismIntrospector.inspect(mechanism);

        assertEquals("exampleMechanism", definition.name());
        assertTrue(definition.children().containsKey("child"));
        assertTrue(definition.states().containsKey("HOME"));
        assertEquals("HOME", definition.initialStateName());
        assertSame(mechanism.HOME, definition.initialState());
        assertTrue(definition.refs().containsKey("motor"));
        assertTrue(definition.refs().containsKey("limit"));
        assertTrue(definition.refs().containsKey("control"));
        assertTrue(definition.refs().containsKey("simulation"));
        assertTrue(definition.refs().containsKey("path"));
        assertTrue(definition.hooks().containsKey("periodic"));
        assertSame(mechanism.periodic, definition.hooks().get("periodic"));
        assertTrue(definition.rules().containsKey("hardstop"));
        assertSame(mechanism.hardstop, definition.rules().get("hardstop"));
    }

    @Test
    void hookIntrospectorDiscoversHooksFromNonMechanismRoots() {
        RobotRoot root = new RobotRoot();

        var hooks = HookIntrospector.inspect("robot", root);

        assertTrue(hooks.containsKey("robot.robotStart"));
        assertTrue(hooks.containsKey("robot.mechanism.periodic"));
        assertSame(root.robotStart, hooks.get("robot.robotStart"));
        assertSame(root.mechanism.periodic, hooks.get("robot.mechanism.periodic"));
    }

    @Test
    void initialStateCanBeAnnotatedOrFallsBackToFirstState() {
        AnnotatedInitialState annotated = new AnnotatedInitialState();
        FallbackInitialState fallback = new FallbackInitialState();

        assertSame(annotated.SECOND, annotated.initialState());
        assertSame(fallback.FIRST, fallback.initialState());
    }

    private record TestMechanism(MechanismState initialState) implements Mechanism {
    }

    private record ChildMechanism(AxisRef axis) implements Mechanism {
        private MechanismState down() {
            return axis.position(-20.0);
        }
    }

    private static final class ParentMechanism implements Mechanism {
        private final ChildMechanism child;
        private final RuleRef collision;
        private final MechanismState DOWN;

        private ParentMechanism(ChildMechanism child, RuleRef collision) {
            this.child = child;
            this.collision = collision;
            DOWN = States.set().set(child, child.down());
        }
    }

    private static final class RuntimeMechanism implements Mechanism {
        private final MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        private final AxisRef axis = Axes.position()
                .motor(motor)
                .encoder(motor.encoder());
        private boolean ticked;
        private final HookRef periodic = Hooks.teleop.whileActive(() -> ticked = true);
        public final MechanismState HOME = axis.position(7.0);
    }

    private static final class SimulatedArmMechanism implements Mechanism {
        private final MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        private final EncoderRef position = motor.encoder();
        private final DigitalInputRef outLimit = DigitalInputs.rio(0);
        private final RangeRef travel = RangeRef.of(0.0, 1.0);
        private final SimRef simulation = Sim.arm(motor)
                .encoder(position)
                .range(travel)
                .limit(outLimit, 1.0, 1.0e-3);
        private final AxisRef axis = Axes.percent()
                .motor(motor)
                .encoder(position)
                .range(travel);
        public final MechanismState OUT = axis.percent(1.0);
    }

    private static final class ExampleMechanism implements Mechanism {
        private final MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        private final DigitalInputRef limit = DigitalInputs.rio(0);
        private final AxisRef control = Axes.percent()
                .motor(motor)
                .sensor(limit)
                .loop(new LatchedLoopRef(limit));
        private final PathRef path = Paths.choreo("example");
        private final SimRef simulation = Sim.motor(motor).momentOfInertia(0.01);
        private boolean ticked;
        private final HookRef periodic = Hooks.teleop.whileActive(() -> ticked = true);
        private final RuleRef hardstop = Rules.when(limit::active)
                .appliesTo(OutputTarget.axis(control).positive())
                .block();
        private final TestMechanism child = new TestMechanism(States.neutral());
        @InitialState
        public final MechanismState HOME = States.percent(motor, 0.2).until(limit::active).then(States.neutral());
    }

    private record LatchedLoopRef(DigitalInputRef limit) implements ControlLoopRef {
        @Override
        public ControlLoopRuntime bind(ControlLoopBinding binding) {
            return context -> ControlOutput.neutral();
        }

        @Override
        public List<Object> refs() {
            return List.of(limit);
        }
    }

    private record SimpleRuleContext(AxisRef axis, Output output, double position, double velocity)
            implements RuleContext {
        @Override
        public AxisState axis(AxisRef axis) {
            return axisState(position, velocity);
        }

        @Override
        public OutputRequest request() {
            return OutputRequest.of(axis, output);
        }
    }

    private static AxisState axisState(double position, double velocity) {
        return new AxisState() {
            @Override
            public double position() {
                return position;
            }

            @Override
            public double velocity() {
                return velocity;
            }
        };
    }

    private static ControlLoopContext loopContext(Output request, double position, double velocity) {
        return new ControlLoopContext() {
            @Override
            public Output request() {
                return request;
            }

            @Override
            public double position() {
                return position;
            }

            @Override
            public double velocity() {
                return velocity;
            }
        };
    }

    private static final class AnnotatedInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        @InitialState
        public final MechanismState SECOND = States.percent(MotorRef.of(AthenaMotor.SIM, 1), 0.5);
    }

    private static final class FallbackInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        public final MechanismState SECOND = States.percent(MotorRef.of(AthenaMotor.SIM, 1), 0.5);
    }

    private static final class RobotRoot {
        private boolean started;
        private final HookRef robotStart = Hooks.robot.onStart(() -> started = true);
        private final ExampleMechanism mechanism = new ExampleMechanism();
    }

    private static final class AutoDeclarations {
        private final TestAutoDeclaration test = new TestAutoDeclaration();
        private final ScoreAutoDeclaration score = new ScoreAutoDeclaration();
    }

    private static final class TestAutoDeclaration {
        private final PathRef leave = Paths.choreo("leave");
        private final MechanismState routine = States.sequence().run(leave);
    }

    private static final class ScoreAutoDeclaration {
        private final PathRef score = Paths.pathplanner("score");
    }

    private static final class RecordingActionContext implements ActionContext {
        private final RecordingEncoder encoder = new RecordingEncoder();
        private final RecordingEncoder absoluteEncoder = new RecordingEncoder(18.0);
        private final RecordingMotor motor = new RecordingMotor();
        private final RecordingBoolean bool = new RecordingBoolean();
        private final RecordingNumber number = new RecordingNumber();
        private EncoderRef encoderRef;
        private MotorRef motorRef;
        private BooleanRef booleanRef;
        private NumberRef numberRef;

        @Override
        public RuntimeEncoder encoder(EncoderRef ref) {
            if (ref.source() instanceof EncoderRef.EncoderSource.Standalone) {
                return absoluteEncoder;
            }
            encoderRef = ref;
            return encoder;
        }

        @Override
        public RuntimeMotor motor(MotorRef ref) {
            motorRef = ref;
            return motor;
        }

        @Override
        public RuntimeBoolean bool(BooleanRef ref) {
            booleanRef = ref;
            return bool;
        }

        @Override
        public RuntimeNumber number(NumberRef ref) {
            numberRef = ref;
            return number;
        }
    }

    private static final class RecordingEncoder implements RuntimeEncoder {
        private double position;
        private RuntimeEncoder syncedTo;

        private RecordingEncoder() {
        }

        private RecordingEncoder(double position) {
            this.position = position;
        }

        @Override
        public double position() {
            return position;
        }

        @Override
        public double absolutePosition() {
            return position;
        }

        @Override
        public double velocity() {
            return 0.0;
        }

        @Override
        public void set(double position) {
            this.position = position;
        }

        @Override
        public void syncTo(RuntimeEncoder source) {
            syncedTo = source;
            RuntimeEncoder.super.syncTo(source);
        }
    }

    private static final class RecordingMotor implements RuntimeMotor {
        private double percent;
        private double voltage;
        private double position;
        private double velocity;
        private boolean stopped;
        private boolean brake;
        private boolean coast;

        @Override
        public void stop() {
            stopped = true;
            RuntimeMotor.super.stop();
        }

        @Override
        public void percent(double output) {
            percent = output;
        }

        @Override
        public void voltage(double volts) {
            voltage = volts;
        }

        @Override
        public void position(double position) {
            this.position = position;
        }

        @Override
        public void velocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public void brake() {
            brake = true;
        }

        @Override
        public void coast() {
            coast = true;
        }
    }

    private static final class RecordingBoolean implements RuntimeBoolean {
        private boolean value;

        @Override
        public boolean get() {
            return value;
        }

        @Override
        public void set(boolean value) {
            this.value = value;
        }
    }

    private static final class RecordingNumber implements RuntimeNumber {
        private double value;

        @Override
        public double get() {
            return value;
        }

        @Override
        public void set(double value) {
            this.value = value;
        }
    }

    private static final class RecordingPathRuntime implements PathRuntime {
        private final double finishSeconds;
        private int initialized;
        private int executed;
        private int ended;

        private RecordingPathRuntime(double finishSeconds) {
            this.finishSeconds = finishSeconds;
        }

        @Override
        public void initialize(PathRef path, MechanismContext context) {
            initialized++;
        }

        @Override
        public void execute(PathRef path, MechanismContext context) {
            executed++;
        }

        @Override
        public boolean isFinished(PathRef path, MechanismContext context) {
            return context.timeInStateSeconds() >= finishSeconds;
        }

        @Override
        public void end(PathRef path, MechanismContext context, boolean interrupted) {
            ended++;
        }
    }
}
