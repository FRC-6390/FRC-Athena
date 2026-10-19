package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Lifecycle Actions: callback ordering, composition, and lease arbitration through the mechanism scheduler. */
class LifecycleActionTest {
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 81);

    @Test
    void startsExecutesFinishesThenEndsFalse() {
        List<String> events = new ArrayList<>();
        int[] cycles = {0};
        RecordingContext hardware = new RecordingContext();
        ArmMechanism robot = new ArmMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action action = Actions.lifecycle("calibrate")
                .onStart(() -> events.add("start"))
                .execute(() -> events.add("execute"))
                .until(() -> ++cycles[0] >= 3)
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();

        scheduler.request(action);
        for (int cycle = 0; cycle < 4; cycle++) scheduler.teleopPeriodic(cycle * 0.02, 0.02);

        assertEquals(List.of("start", "execute", "execute", "execute", "end:false"), events);
        assertTrue(scheduler.isComplete(action));
        assertFalse(scheduler.isRunning(action));
    }

    @Test
    void cancellationEndsInterruptedExactlyOnce() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        ArmMechanism robot = new ArmMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action hold = Actions.lifecycle("hold")
                .onStart(() -> events.add("start"))
                .execute(() -> events.add("execute"))
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();

        scheduler.request(hold);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.cancel(hold);
        scheduler.teleopPeriodic(0.04, 0.02);
        scheduler.cancel(hold);

        assertEquals(List.of("start", "execute", "execute", "end:true"), events);
        assertFalse(scheduler.isRunning(hold));
        assertFalse(scheduler.isComplete(hold));
    }

    @Test
    void cancelAllEndsRunningLifecycles() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        scheduler.request(Actions.lifecycle("held")
                .onStart(() -> events.add("start"))
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build());
        scheduler.teleopPeriodic(0.0, 0.02);

        scheduler.cancelAll();

        assertEquals(List.of("start", "end:true"), events);
    }

    @Test
    void disableEndsInterruptedAndRestartsWhenEnabledAgain() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action action = Actions.lifecycle("drive")
                .onStart(() -> events.add("start"))
                .execute(() -> events.add("execute"))
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();

        scheduler.request(action);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.periodic(
                new MechanismContext(0.02, 0.0, 0.02, false, false, false),
                new EventContext(0.02, 0.02, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));
        scheduler.periodic(
                new MechanismContext(0.04, 0.0, 0.02, false, false, false),
                new EventContext(0.04, 0.02, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));
        scheduler.teleopPeriodic(0.06, 0.02);

        assertEquals(List.of("start", "execute", "end:true", "start", "execute"), events);
    }

    @Test
    void sequenceAdvancesOnlyAfterLifecycleFinishes() {
        List<String> events = new ArrayList<>();
        int[] cycles = {0};
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action step = Actions.lifecycle("step")
                .execute(() -> events.add("execute"))
                .until(() -> ++cycles[0] >= 2)
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();
        Action auto = Actions.sequence().run(step).forTime(Seconds.of(0.04), ARM.percent(0.5));

        scheduler.request(auto);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertTrue(hardware.motor(ARM).history.isEmpty(), "motor driven before the lifecycle finished");
        for (int cycle = 1; cycle < 6; cycle++) scheduler.teleopPeriodic(cycle * 0.02, 0.02);

        assertEquals(List.of("execute", "execute", "end:false"), events);
        assertTrue(hardware.motor(ARM).history.contains(0.5));
    }

    @Test
    void raceEndsTheLifecycleLoserInterrupted() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action watch = Actions.lifecycle("watch")
                .execute(() -> events.add("execute"))
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();

        scheduler.request(Actions.race(watch, Actions.waitSeconds(Seconds.of(0.04))));
        for (int cycle = 0; cycle < 5; cycle++) scheduler.teleopPeriodic(cycle * 0.02, 0.02);

        // The lifecycle executes in the cycle the wait completes, before the race ends it.
        assertEquals(List.of("execute", "execute", "execute", "end:true"), events);
    }

    @Test
    void timeoutEndsTheLifecycleInterrupted() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action spin = Actions.lifecycle("spin")
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build()
                .timeout(Seconds.of(0.04));

        scheduler.request(spin);
        for (int cycle = 0; cycle < 5; cycle++) scheduler.teleopPeriodic(cycle * 0.02, 0.02);

        assertEquals(List.of("end:true"), events);
    }

    @Test
    void newerRequestForSameNamedResourceEndsTheOlderLifecycle() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action first = Actions.lifecycle("first")
                .requires("drive")
                .onStart(() -> events.add("first:start"))
                .execute(() -> events.add("first:execute"))
                .onEnd(interrupted -> events.add("first:end:" + interrupted))
                .build();
        Action second = Actions.lifecycle("second")
                .requires(NamedResource.of("drive"))
                .onStart(() -> events.add("second:start"))
                .execute(() -> events.add("second:execute"))
                .onEnd(interrupted -> events.add("second:end:" + interrupted))
                .build();

        scheduler.request(first);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.request(second);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertEquals(
                List.of("first:start", "first:execute", "first:execute", "first:end:true",
                        "second:start", "second:execute"),
                events);
        assertFalse(scheduler.isRunning(first));
        assertTrue(scheduler.isRunning(second));
    }

    @Test
    void lifecycleRequiringAMotorIsEndedByAMechanismActionOnThatMotor() {
        List<String> events = new ArrayList<>();
        RecordingContext hardware = new RecordingContext();
        ArmMechanism robot = new ArmMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action guard = Actions.lifecycle("guard")
                .requires(ARM)
                .execute(() -> events.add("execute"))
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();

        scheduler.request(guard);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(robot.overrideArm);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(List.of("execute", "end:true"), events);
        assertEquals(-0.8, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void parallelLifecyclesSharingAResourceAreRejected() {
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new ArmMechanism());
        Action left = Actions.lifecycle("left").requires("drive").build();
        Action right = Actions.lifecycle("right").requires("drive").build();

        assertThrows(IllegalArgumentException.class, () -> scheduler.request(Actions.parallel(left, right)));
    }

    @Test
    void lifecycleBoundedOnlyWhenItHasAFinishCondition() {
        assertFalse(Actions.isBounded(Actions.lifecycle("open").build()));
        assertTrue(Actions.isBounded(Actions.lifecycle("closed").until(() -> true).build()));
    }

    @Test
    void namedResourceRejectsBlankNames() {
        assertThrows(IllegalArgumentException.class, () -> NamedResource.of("  "));
        assertEquals(NamedResource.of("drive"), NamedResource.of(" drive "));
    }

    private static final class ArmMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final Action overrideArm = arm.percent(-0.80);
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new IdentityHashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private final List<Double> history = new ArrayList<>();
        private double percent = Double.NaN;

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
            history.add(percent);
        }

        @Override
        public void stop() {
            percent = 0.0;
            history.add(0.0);
        }
    }
}
