package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

class MechanismRuntimeTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.SIM, 1);
    private static final MotorDevice CHILD_MOTOR = MotorDevice.of(MotorKinds.SIM, 2);

    @Test
    void sequenceAdvancesByTimeAndRoutesOutputsToMotorHandle() {
        RecordingActionContext actions = new RecordingActionContext(MOTOR);
        TestMechanism mechanism = new TestMechanism(
                States.sequence()
                        .forTime(0.5, States.percent(MOTOR, 0.25))
                        .then(States.voltage(MOTOR, 6.0)));
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
                States.timeout(States.percent(MOTOR, 0.4), 0.3)
                        .then(States.until(ctx -> ctx.nowSeconds() >= 1.0, States.percent(MOTOR, 0.6))
                                .then(States.voltage(MOTOR, 7.0))));
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
    void pathRuntimeInitializesOnceExecutesUntilFinishedAndEndsOnce() {
        PathState path = Paths.pathPlanner("leave-zone");
        RecordingPathRuntime pathRuntime = new RecordingPathRuntime(3);
        TestMechanism mechanism = new TestMechanism(path.then(States.percent(MOTOR, 0.9)));
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
        TestMechanism mechanism = new TestMechanism(States.sequence()
                .doOnce(() -> actions.events++)
                .forTime(0.02, States.percent(MOTOR, 0.2))
                .then(States.sequence()
                        .doOnce(() -> actions.events++)
                        .then(States.percent(MOTOR, 0.8))));
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
        TestMechanism child = new TestMechanism(States.percent(CHILD_MOTOR, 0.7));
        TestMechanism parent = new TestMechanism(States.set().set(child, child.initialState()));
        MechanismRuntime runtime = MechanismRuntime.of(parent, actions);

        runtime.periodic(contextAt(0.0), EventContext.empty());

        assertEquals(0.7, actions.motor(CHILD_MOTOR).percent, 1.0e-9);
    }

    @Test
    void robotRuntimeCanUseHardwareGraphAsRuntimeActionContext() {
        RecordingMotorBackend backend = new RecordingMotorBackend();
        HardwareGraph hardware = HardwareGraph.using(BackendRegistry.of(backend));
        RobotRuntime runtime = RobotRuntime.create(hardware)
                .register(new TestMechanism(States.percent(MOTOR, 0.55)));

        runtime.robotPeriodic(1.0, 0.02);

        assertEquals(0.55, backend.handle.percent, 1.0e-9);
        assertEquals(1, backend.created);
    }

    @Test
    void robotRuntimeStepsDiscoveredSimulationModels() {
        SimulatedMechanism mechanism = new SimulatedMechanism();
        RobotRuntime runtime = RobotRuntime.create()
                .register(mechanism)
                .bindInMemoryRuntime();

        runtime.simulationPeriodic(0.0, 0.2);

        EncoderHandle encoder = runtime.actionContext().encoder(mechanism.encoder);
        assertTrue(encoder.positionRotations() > 0.0);
        assertTrue(encoder.velocityRotationsPerSecond() > 0.0);
    }

    private static MechanismContext contextAt(double nowSeconds) {
        return new MechanismContext(nowSeconds, 0.0, 0.02, true, false, false);
    }

    private static final class TestMechanism implements Mechanism {
        private final State initial;

        private TestMechanism(State initial) {
            this.initial = initial;
        }
    }

    private static final class SimulatedMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 21);
        private final EncoderDevice encoder = EncoderDevice.of(ca.frc6390.athena.api.hardware.EncoderKinds.SIM, 21);
        private final SimModel simulation = SimModels.motor(motor).encoder(encoder);
        private final State initial = States.percent(motor, 1.0);
    }

    private static final class HookedMechanism implements Mechanism {
        private final State idle = States.neutral();
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

    private static final class RecordingPathRuntime implements PathRuntime {
        private final int finishAfterExecutes;
        private int initializes;
        private int executes;
        private int ends;

        private RecordingPathRuntime(int finishAfterExecutes) {
            this.finishAfterExecutes = finishAfterExecutes;
        }

        @Override
        public void initialize(PathState path, MechanismContext context) {
            initializes++;
        }

        @Override
        public void execute(PathState path, MechanismContext context) {
            executes++;
        }

        @Override
        public boolean isFinished(PathState path, MechanismContext context) {
            return executes >= finishAfterExecutes;
        }

        @Override
        public void end(PathState path, MechanismContext context, boolean interrupted) {
            ends++;
        }
    }

    private static final class RecordingActionContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotorHandle> motors = new HashMap<>();
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

    private static final class RecordingMotorBackend implements MotorBackend {
        private final RecordingMotorHandle handle = new RecordingMotorHandle(MOTOR);
        private int created;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.SIM;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            created++;
            return handle;
        }
    }
}
