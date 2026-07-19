package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class ActionArbitrationTest {
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice ROLLERS = MotorDevice.of(MotorKinds.KRAKEN_X44, 2);

    @Test
    void concurrentChildrenCannotSilentlyOverrideTheSameMotor() {
        DirectActions mechanism = new DirectActions();
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        for (Action conflicting : List.of(
                Actions.parallel(ARM.percent(0.25), ARM.voltage(4.0)),
                Actions.race(ARM.percent(0.25), ARM.voltage(4.0)),
                Actions.deadline(Actions.waitSeconds(1.0).then(ARM.percent(0.25)), ARM.voltage(4.0)))) {
            IllegalArgumentException error = assertThrows(
                    IllegalArgumentException.class, () -> scheduler.request(conflicting));
            String message = error.getMessage();
            assertTrue(message.contains("Concurrent action conflict at root."
                    + conflicting.getClass().getSimpleName().toLowerCase()));
            assertTrue(message.contains("child[0]"));
            assertTrue(message.contains("child[1]"));
            assertTrue(message.contains("motor '" + ARM.defaultName() + "'"));
        }
    }

    @Test
    void sequenceStartCannotSilentlyOverrideAStageUsingTheSameMotor() {
        DirectActions mechanism = new DirectActions();
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);
        Action conflicting = Actions.sequence()
                .start(ARM.percent(0.20))
                .forTime(0.5, ARM.percent(0.80));

        IllegalArgumentException error = assertThrows(
                IllegalArgumentException.class, () -> scheduler.request(conflicting));

        String message = error.getMessage();
        assertTrue(message.contains("root.sequence.step[0].parallel"));
        assertTrue(message.contains("child[0] MotorPercent"));
        assertTrue(message.contains("child[1] MotorPercent"));
        assertTrue(message.contains("motor '" + ARM.defaultName() + "'"));
    }

    @Test
    void runtimeGeneratedConflictFailsInsteadOfUsingTraversalOrder() {
        SingleMotorMechanism mechanism = new SingleMotorMechanism(ARM, 0.0);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);
        Action dynamicConflict = Actions.compute(
                () -> Actions.parallel(ARM.percent(0.20), ARM.voltage(4.0)), ARM);

        scheduler.request(dynamicConflict);
        IllegalStateException error = assertThrows(
                IllegalStateException.class, () -> scheduler.teleopPeriodic(0.0, 0.02));

        String message = error.getMessage();
        assertTrue(message.contains("Runtime output conflict in lease action ScheduledOutputs"));
        assertTrue(message.contains("output[0] MotorPercent"));
        assertTrue(message.contains("output[1] MotorVoltage"));
        assertTrue(message.contains("motor '" + ARM.defaultName() + "'"));
    }

    @Test
    void sequentialReuseAndParallelDisjointResourcesRemainValid() {
        DirectActions mechanism = new DirectActions();
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        assertDoesNotThrow(() -> scheduler.request(Actions.sequence()
                .forTime(0.1, mechanism.firstArm)
                .then(mechanism.secondArm)));
        assertDoesNotThrow(() -> scheduler.request(Actions.parallel(
                mechanism.firstArm, mechanism.rollers)));
    }

    @Test
    void disjointHeldActionsWithinOneMechanismRunTogether() {
        boolean[] arm = {false};
        boolean[] rollers = {false};
        RecordingContext hardware = new RecordingContext();
        HeldActions mechanism = new HeldActions(arm, new boolean[] {false}, rollers);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        arm[0] = true;
        scheduler.teleopPeriodic(0.0, 0.02);
        rollers[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.70, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void hooksInAControlsMechanismLeaseActionsOwnedByAnotherMechanism() {
        boolean[] arm = {true};
        boolean[] rollers = {true};
        RecordingContext hardware = new RecordingContext();
        TargetMechanism target = new TargetMechanism();
        ExternalControls controls = new ExternalControls(target, arm, rollers);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(target)
                .register(controls);

        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.70, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void persistentEdgeRequestsCoexistAndReactivationUpdatesRecency() {
        boolean[] first = {false};
        boolean[] second = {false};
        boolean[] rollers = {false};
        RecordingContext hardware = new RecordingContext();
        PersistentActions mechanism = new PersistentActions(first, second, rollers);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        first[0] = true;
        scheduler.teleopPeriodic(0.0, 0.02);
        first[0] = false;
        rollers[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.20, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.50, hardware.motor(ROLLERS).percent, 1.0e-9);

        rollers[0] = false;
        second[0] = true;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(-0.80, hardware.motor(ARM).percent, 1.0e-9);

        second[0] = false;
        first[0] = true;
        scheduler.teleopPeriodic(0.06, 0.02);
        assertEquals(0.20, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void newerSequenceReservesMotorsUsedByItsLaterSteps() {
        boolean[] armReady = {false};
        RecordingContext hardware = new RecordingContext();
        ReservedSequence mechanism = new ReservedSequence(armReady);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.reverse);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-0.5, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(-0.5, hardware.motor(ROLLERS).percent, 1.0e-9);

        scheduler.request(mechanism.collect);
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.3, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.0, hardware.motor(ROLLERS).percent, 1.0e-9);

        armReady[0] = true;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.3, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.7, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void directRequestsCoexistByResourceAndRerequestingUpdatesRecency() {
        RecordingContext hardware = new RecordingContext();
        DirectActions mechanism = new DirectActions();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.firstArm);
        scheduler.request(mechanism.rollers);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.15, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.55, hardware.motor(ROLLERS).percent, 1.0e-9);

        scheduler.request(mechanism.secondArm);
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(-0.75, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.request(mechanism.firstArm);
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.15, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void parallelRequestSpanningRegisteredMechanismsIsPartitionedByOwner() {
        RecordingContext hardware = new RecordingContext();
        SingleMotorMechanism arm = new SingleMotorMechanism(ARM, 0.35);
        SingleMotorMechanism rollers = new SingleMotorMechanism(ROLLERS, 0.65);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(arm)
                .register(rollers);

        scheduler.request(Actions.parallel(arm.run, rollers.run));
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.35, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.65, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void newestConflictWinsWithoutRenewingAndOlderHeldActionResumesOnRelease() {
        boolean[] inward = {false};
        boolean[] outward = {false};
        RecordingContext hardware = new RecordingContext();
        HeldActions mechanism = new HeldActions(inward, outward, new boolean[] {false});
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        inward[0] = true;
        scheduler.teleopPeriodic(0.0, 0.02);
        outward[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(-0.60, hardware.motor(ARM).percent, 1.0e-9);

        outward[0] = false;
        scheduler.teleopPeriodic(0.06, 0.02);
        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void persistentEdgeRequestRemainsNewerThanAnAlreadyHeldAction() {
        boolean[] held = {true};
        boolean[] stop = {false};
        RecordingContext hardware = new RecordingContext();
        HeldWithPersistentStop mechanism = new HeldWithPersistentStop(held, stop);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);

        stop[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);

        stop[0] = false;
        scheduler.teleopPeriodic(0.04, 0.02);
        scheduler.teleopPeriodic(0.06, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void heldActionRenewsAtTeleopInitAfterAutonomousOverride() {
        boolean[] held = {true};
        boolean[] stop = {false};
        RecordingContext hardware = new RecordingContext();
        HeldWithPersistentStop mechanism = new HeldWithPersistentStop(held, stop);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, true, false),
                new EventContext(0.0, 0.02, LifecycleMode.AUTONOMOUS,
                        LifecyclePhase.PERIODIC, true, false));
        stop[0] = true;
        scheduler.periodic(
                new MechanismContext(0.02, 0.0, 0.02, true, true, false),
                new EventContext(0.02, 0.02, LifecycleMode.AUTONOMOUS,
                        LifecyclePhase.PERIODIC, true, false));
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.periodic(
                new MechanismContext(0.04, 0.0, 0.02, true, false, false),
                new EventContext(0.04, 0.02, LifecycleMode.TELEOP,
                        LifecyclePhase.INIT, true, false));

        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void releasingOneLeaseNeutralizesOnlyItsMotor() {
        boolean[] arm = {true};
        boolean[] rollers = {true};
        RecordingContext hardware = new RecordingContext();
        HeldActions mechanism = new HeldActions(arm, new boolean[] {false}, rollers);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        rollers[0] = false;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0, hardware.motor(ARM).stopCalls);
        assertEquals(0.0, hardware.motor(ROLLERS).percent, 1.0e-9);
        assertEquals(1, hardware.motor(ROLLERS).stopCalls);

        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(1, hardware.motor(ROLLERS).stopCalls);
    }

    @Test
    void newerRobotCompositeOverridesHeldChildActionAcrossMechanismRuntimes() {
        boolean[] childHeld = {true};
        RecordingContext hardware = new RecordingContext();
        ChildWithHeldAction child = new ChildWithHeldAction(childHeld);
        RobotCompositeActions robot = new RobotCompositeActions();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(child)
                .register(robot);

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-0.20, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.request(robot.collect);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.65, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.75, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void releasedChildActionDoesNotNeutralizeMotorDrivenByRobotComposite() {
        boolean[] childHeld = {true};
        RecordingContext hardware = new RecordingContext();
        ChildWithHeldAction child = new ChildWithHeldAction(childHeld);
        RobotCompositeActions robot = new RobotCompositeActions();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(child)
                .register(robot);

        scheduler.teleopPeriodic(0.0, 0.02);
        childHeld[0] = false;
        scheduler.request(robot.collect);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.65, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0, hardware.motor(ARM).stopCalls);
    }

    @Test
    void robotCompositesCanRunCollectShootCollectAcrossMechanismRuntimes() {
        boolean[] childHeld = {true};
        RecordingContext hardware = new RecordingContext();
        ChildWithHeldAction child = new ChildWithHeldAction(childHeld);
        RobotCompositeActions robot = new RobotCompositeActions();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(child)
                .register(robot);

        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(robot.collect);
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.65, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.75, hardware.motor(ROLLERS).percent, 1.0e-9);

        scheduler.request(robot.shoot);
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(-0.80, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(1.0, hardware.motor(ROLLERS).percent, 1.0e-9);

        scheduler.request(robot.collect);
        scheduler.teleopPeriodic(0.06, 0.02);
        assertEquals(0.65, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.75, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void newerRequestOverridesOnlyTheConflictingCompositeChild() {
        boolean[] composite = {true};
        boolean[] rollerOverride = {false};
        RecordingContext hardware = new RecordingContext();
        CompositeActions mechanism = new CompositeActions(composite, rollerOverride);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        rollerOverride[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.30, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.90, hardware.motor(ROLLERS).percent, 1.0e-9);

        rollerOverride[0] = false;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.30, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.40, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void constraintsRunAfterArbitrationAndDoNotFallBackToAnOlderRequest() {
        boolean[] older = {true};
        boolean[] rejected = {false};
        RecordingContext hardware = new RecordingContext();
        ConstrainedActions mechanism = new ConstrainedActions(older, rejected);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-0.30, hardware.motor(ARM).percent, 1.0e-9);

        rejected[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);

        rejected[0] = false;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(-0.30, hardware.motor(ARM).percent, 1.0e-9);
    }

    private static final class HeldActions implements Mechanism {
        private final Action inward = ARM.percent(0.25);
        private final Action outward = ARM.percent(-0.60);
        private final Action intake = ROLLERS.percent(0.70);
        private final HookBinding inwardBinding;
        private final HookBinding outwardBinding;
        private final HookBinding rollerBinding;

        private HeldActions(boolean[] inward, boolean[] outward, boolean[] rollers) {
            inwardBinding = Events.when(() -> inward[0]).active().whileActive(this.inward);
            outwardBinding = Events.when(() -> outward[0]).active().whileActive(this.outward);
            rollerBinding = Events.when(() -> rollers[0]).active().whileActive(intake);
        }
    }

    private static final class ReservedSequence implements Mechanism {
        private final Action reverse = Actions.parallel(ARM.percent(-0.5), ROLLERS.percent(-0.5));
        private final Action collect;

        private ReservedSequence(boolean[] armReady) {
            collect = Actions.sequence()
                    .until(() -> armReady[0], ARM.percent(0.3))
                    .then(Actions.parallel(ARM.percent(0.3), ROLLERS.percent(0.7)));
        }
    }

    private static final class TargetMechanism implements Mechanism {
        private final Action arm = ARM.percent(0.25);
        private final Action rollers = ROLLERS.percent(0.70);
    }

    private static final class HeldWithPersistentStop implements Mechanism {
        private final Action drive = ARM.percent(0.25);
        private final Action stop = ARM.neutral();
        private final HookBinding driveBinding;
        private final HookBinding stopBinding;

        private HeldWithPersistentStop(boolean[] held, boolean[] stop) {
            driveBinding = Events.when(() -> held[0]).active().whileActive(drive);
            stopBinding = Events.when(() -> stop[0]).active().onStart(this.stop);
        }
    }

    private static final class ExternalControls implements Mechanism {
        private final HookBinding arm;
        private final HookBinding rollers;

        private ExternalControls(TargetMechanism target, boolean[] arm, boolean[] rollers) {
            this.arm = Events.when(() -> arm[0]).active().whileActive(target.arm);
            this.rollers = Events.when(() -> rollers[0]).active().whileActive(target.rollers);
        }
    }

    private static final class CompositeActions implements Mechanism {
        private final Action composite = Actions.parallel(ARM.percent(0.30), ROLLERS.percent(0.40));
        private final Action rollerOverride = ROLLERS.percent(0.90);
        private final HookBinding compositeBinding;
        private final HookBinding overrideBinding;

        private CompositeActions(boolean[] composite, boolean[] rollerOverride) {
            compositeBinding = Events.when(() -> composite[0]).active().whileActive(this.composite);
            overrideBinding = Events.when(() -> rollerOverride[0]).active().whileActive(this.rollerOverride);
        }
    }

    private static final class ChildWithHeldAction implements Mechanism {
        private final Action home = ARM.percent(-0.20);
        private final HookBinding homeBinding;

        private ChildWithHeldAction(boolean[] held) {
            homeBinding = Events.when(() -> held[0]).active().whileActive(home);
        }
    }

    private static final class RobotCompositeActions implements Mechanism {
        private final Action collect = Actions.parallel(ARM.percent(0.65), ROLLERS.percent(0.75));
        private final Action shoot = Actions.parallel(ARM.percent(-0.80), ROLLERS.percent(1.0));
    }

    private static final class PersistentActions implements Mechanism {
        private final Action first = ARM.percent(0.20);
        private final Action second = ARM.percent(-0.80);
        private final Action rollers = ROLLERS.percent(0.50);
        private final HookBinding firstBinding;
        private final HookBinding secondBinding;
        private final HookBinding rollerBinding;

        private PersistentActions(boolean[] first, boolean[] second, boolean[] rollers) {
            firstBinding = Events.when(() -> first[0]).active().onStart(this.first);
            secondBinding = Events.when(() -> second[0]).active().onStart(this.second);
            rollerBinding = Events.when(() -> rollers[0]).active().onStart(this.rollers);
        }
    }

    private static final class ConstrainedActions implements Mechanism {
        private final Action older = ARM.percent(-0.30);
        private final ControlBinding guarded = Controls.velocity(ARM)
                .constraint(Constraints.require(() -> false));
        private final Action rejected = guarded.percent(0.80);
        private final HookBinding olderBinding;
        private final HookBinding rejectedBinding;

        private ConstrainedActions(boolean[] older, boolean[] rejected) {
            olderBinding = Events.when(() -> older[0]).active().whileActive(this.older);
            rejectedBinding = Events.when(() -> rejected[0]).active().whileActive(this.rejected);
        }
    }

    private static final class DirectActions implements Mechanism {
        private final MotorDevice arm = ARM;
        private final MotorDevice rollersMotor = ROLLERS;
        private final Action firstArm = ARM.percent(0.15);
        private final Action secondArm = ARM.percent(-0.75);
        private final Action rollers = ROLLERS.percent(0.55);
    }

    private static final class SingleMotorMechanism implements Mechanism {
        private final MotorDevice motor;
        private final Action run;

        private SingleMotorMechanism(MotorDevice motor, double output) {
            this.motor = motor;
            this.run = motor.percent(output);
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new HashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double percent = Double.NaN;
        private int stopCalls;

        private RecordingMotor(MotorDevice device) {
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
        public void stop() {
            stopCalls++;
            percent = 0.0;
        }
    }
}
