package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class MechanismRuntimeTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice CHILD_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
    private static final EncoderDevice ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 1);

    @Test
    void sequenceAdvancesByTimeAndRoutesOutputsToMotorHandle() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                Actions.sequence()
                        .forTime(0.5, MOTOR.percent(0.25))
                        .then(MOTOR.voltage(6.0)));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.25), EventContext.empty());
        assertEquals(0.25, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.5), EventContext.empty());
        assertEquals(6.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void timeoutAndConditionMoveToNextState() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                Actions.timeout(MOTOR.percent(0.4), 0.3)
                        .then(Actions.until(ctx -> ctx.nowSeconds() >= 1.0, MOTOR.percent(0.6))
                                .then(MOTOR.voltage(7.0))));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertEquals(0.4, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(0.3), EventContext.empty());
        assertEquals(0.6, actions.motor(MOTOR).percent, 1.0e-9);

        runtime.periodic(contextAt(1.0), EventContext.empty());
        assertEquals(7.0, actions.motor(MOTOR).voltage, 1.0e-9);
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
        RobotRuntime runtime = RobotRuntime.create(new RecordingActionContext(MOTOR))
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
        RobotRuntime runtime = RobotRuntime.create(new RecordingActionContext(MOTOR))
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

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, pathRuntime.initializes);
        assertEquals(3, pathRuntime.executes);
        assertEquals(1, pathRuntime.ends);
        assertEquals(0.9, actions.motor(MOTOR).percent, 1.0e-9);
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

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());
        runtime.periodic(contextAt(0.06), EventContext.empty());

        assertEquals(2, actions.events);
        assertEquals(0.8, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void childSetRoutesChildMechanismOutputToChildMotorHandle() {
        RecordingActionContext actions = new RecordingActionContext(CHILD_MOTOR);
        TestMechanism child = new TestMechanism(CHILD_MOTOR.percent(0.7));
        TestMechanism parent = new TestMechanism(Actions.set().set(child, child.initialState()));
        MechanismRuntime runtime = MechanismRuntime.of(parent, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeRequestsRootOwnedAction() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(MOTOR.percent(0.1));
        Action requested = MOTOR.percent(0.8);
        mechanism.secondary = requested;
        RobotRuntime runtime = RobotRuntime.create(actions).register(mechanism);

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
        RobotRuntime runtime = RobotRuntime.create(actions).register(parent);

        runtime.request(requested);
        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.9, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeRequestsExternalActionByTargetDeclarationOwnership() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        DeviceMechanism mechanism = new DeviceMechanism();
        ExternalActions external = new ExternalActions(mechanism);
        RobotRuntime runtime = RobotRuntime.create(actions).register(mechanism);

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
    void robotRuntimeCanUseHardwareGraphAsRuntimeActionContext() {
        RecordingMotorBackend backend = new RecordingMotorBackend();
        HardwareGraph hardware = HardwareGraph.using(BackendRegistry.of(backend));
        RobotRuntime runtime = RobotRuntime.create(hardware)
                .register(new TestMechanism(MOTOR.percent(0.55)));

        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.55, backend.handle.percent, 1.0e-9);
        assertEquals(1, backend.created);
    }

    @Test
    void controlPidLoopTransformsPositionRequestIntoMotorOutput() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        actions.encoder(ENCODER).position = 0.5;
        ControlBinding control = Controls.position(MOTOR)
                .feedback(ENCODER)
                .pid(0.2, 0.0, 0.0);
        TestMechanism mechanism = new TestMechanism(control.position(2.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.3, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void controlFeedforwardLoopTransformsVelocityRequestIntoMotorVoltage() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.velocity(MOTOR)
                .ff(0.5, 2.0, 1.0);
        TestMechanism mechanism = new TestMechanism(control.velocity(3.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(7.5, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void controlLoopsReceiveRuntimeDtSeconds() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.position(MOTOR)
                .loop(binding -> context -> ControlOutput.percent(context.dtSeconds()));
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(new MechanismContext(0.0, 0.0, 0.1, true, false, false), EventContext.empty());

        assertEquals(0.1, actions.motor(MOTOR).percent, 1.0e-9);
    }

    @Test
    void controlLoopsComposePidPercentAndFeedforwardVoltageAsVoltage() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ControlBinding control = Controls.velocity(MOTOR)
                .loop(binding -> context -> ControlOutput.percent(0.25))
                .loop(binding -> context -> ControlOutput.voltage(2.0));
        TestMechanism mechanism = new TestMechanism(control.velocity(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(5.0, actions.motor(MOTOR).voltage, 1.0e-9);
    }

    @Test
    void controlLoopRuntimeResetsWhenRequestedTargetChanges() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        ResetCountingLoop loop = new ResetCountingLoop();
        ControlBinding control = Controls.position(MOTOR).loop(loop);
        TestMechanism mechanism = new TestMechanism(control.position(1.0));
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.set(control.position(2.0));
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(2, loop.runtime.resets);
        assertEquals(3, loop.runtime.calculations);
    }

    @Test
    void robotRuntimeStepsDiscoveredSimulationModels() {
        SimulatedMechanism mechanism = new SimulatedMechanism();
        SimulationSession session = SimulationSession.create()
                .model("mechanism", mechanism.simulation);
        RobotRuntime runtime = RobotRuntime.create(session.hardwareGraph())
                .register(mechanism);

        runtime.simulationPeriodic(0.0, 0.2);
        session.step(0.2);

        EncoderHandle encoder = session.hardwareGraph().encoder(mechanism.encoder);
        assertTrue(encoder.positionRotations() > 0.0);
        assertTrue(encoder.velocityRotationsPerSecond() > 0.0);
    }

    private static MechanismContext contextAt(double nowSeconds) {
        return new MechanismContext(nowSeconds, 0.0, 0.02, true, false, false);
    }

    private static final class TestMechanism implements Mechanism {
        private final Action initial;
        private Action secondary;

        private TestMechanism(Action initial) {
            this.initial = initial;
        }
    }

    private static final class ParentMechanism implements Mechanism {
        private final Action initial = Actions.neutral();
        private final TestMechanism child;

        private ParentMechanism(TestMechanism child) {
            this.child = child;
        }
    }

    private static final class DeviceMechanism implements Mechanism {
        private final MotorDevice motor = MOTOR;
        private final Action initial = Actions.neutral();
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
        private final SimModel simulation = SimModels.motor(motor).encoder(encoder);
        private final Action initial = motor.percent(1.0);
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
    }

    private static final class RecordingEncoderHandle implements EncoderHandle {
        private final EncoderDevice device;
        private double position;
        private double velocity;

        private RecordingEncoderHandle(EncoderDevice device) {
            this.device = device;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
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
