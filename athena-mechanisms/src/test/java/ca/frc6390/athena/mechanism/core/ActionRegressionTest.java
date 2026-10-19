package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Regression tests for action-system bugs found in the 2026-10 audit. */
class ActionRegressionTest {
    private static final double DT = 0.02;
    private final MotorDevice arm = MotorDevice.of(MotorKinds.KRAKEN_X60, 41);
    private final MotorDevice roller = MotorDevice.of(MotorKinds.KRAKEN_X44, 42);

    @Test
    void untilThenThenKeepsTheMiddleStage() {
        Probe probe = new Probe();
        int[] middle = {0};
        Action chain = pct(arm, 0.1).until(() -> true)
                .then(Actions.doOnce(() -> middle[0]++))
                .then(pct(arm, 0.3));
        probe.scheduler.request(chain);
        probe.tick();
        probe.tick();
        assertEquals(1, middle[0], "middle then() action was dropped");
    }

    @Test
    void untilThenDoesNotFallBackAfterTheWrappedActionCompletes() {
        Probe probe = new Probe();
        boolean[] flag = {true};
        Action chain = Actions.waitUntil(() -> flag[0]).until(() -> false).then(pct(arm, 0.5));
        probe.scheduler.request(chain);
        probe.tick();
        assertEquals(0.5, probe.percent(arm), 1e-9);
        flag[0] = false;
        probe.tick();
        assertEquals(0.5, probe.percent(arm), 1e-9, "next stage abandoned after wrapped regressed");
    }

    @Test
    void onlyIfFalseSkipsInsideASequence() {
        Probe probe = new Probe();
        Action seq = Actions.sequence()
                .run(Actions.onlyIf(() -> false, Actions.waitSeconds(0.1)))
                .run(pct(arm, 0.4).timeout(1.0));
        probe.scheduler.request(seq);
        for (int i = 0; i < 3; i++) probe.tick();
        assertEquals(0.4, probe.percent(arm), 1e-9, "false when() branch stalls the sequence forever");
    }

    @Test
    void eitherPicksItsBranchOnceAtStart() {
        Probe probe = new Probe();
        boolean[] cond = {true};
        Action choice = Actions.either(() -> cond[0],
                Actions.sequence().forTime(0.1, pct(arm, 0.2)).run(pct(arm, 0.6).timeout(1)),
                pct(arm, -0.9).timeout(1));
        probe.scheduler.request(choice);
        probe.tick();
        cond[0] = false;
        probe.tick();
        assertEquals(0.2, probe.percent(arm), 1e-9, "choice re-picked a branch mid-run");
    }

    @Test
    void disabledTimeDoesNotCountTowardWaits() {
        Probe probe = new Probe();
        Action seq = Actions.sequence().waitSeconds(1.0).run(pct(arm, 0.7).timeout(1));
        probe.scheduler.request(seq);
        probe.tick();
        for (int i = 0; i < 100; i++) probe.disabledTick();
        probe.tick();
        assertEquals(0.0, probe.percentOrZero(arm), 1e-9, "wait elapsed while disabled");
    }

    @Test
    void sequenceThenDoesNotMutateTheSharedSequence() {
        Action base = Actions.sequence().run(pct(arm, 0.1).timeout(0.1));
        Action a = ((Actions.Sequence) base).then(pct(roller, 0.2).timeout(0.1));
        assertEquals(null, ((Actions.Sequence) base).next(), "then() mutated the shared sequence");
        assertEquals(false, a == base);
    }

    @Test
    void restartingAnotherLeaseFromADoOnceDropsNoOutputs() {
        Probe probe = new Probe();
        Action rollerRun = pct(roller, 0.5);
        Action other = pct(arm, 0.25);
        probe.scheduler.request(other);
        probe.scheduler.request(rollerRun);
        probe.tick();
        // A lease that re-requests (restarts) an older lease while evaluating.
        Action kicker = Actions.cycle().run(Actions.doOnce(() -> probe.scheduler.request(other)).then(Actions.waitSeconds(0.04)));
        probe.scheduler.request(kicker);
        for (int i = 0; i < 6; i++) {
            probe.tick();
            assertEquals(0.5, probe.percent(roller), 1e-9, "roller output dropped at tick " + i);
            assertEquals(0.25, probe.percent(arm), 1e-9, "arm output dropped at tick " + i);
        }
    }


    @Test
    void hookOnStartReplaysOnEveryPress() {
        HookProbe probe = new HookProbe();
        probe.press(true);
        probe.tick();
        assertEquals(0.3, probe.percent(arm), 1e-9);
        probe.press(false);
        for (int i = 0; i < 10; i++) probe.tick();
        assertEquals(0.0, probe.percentOrZero(arm), 1e-9, "press action should finish");
        probe.press(true);
        probe.tick();
        assertEquals(0.3, probe.percent(arm), 1e-9, "second press did nothing");
    }

    @Test
    void hookWhileActiveRestartsFromTheBeginningAfterRelease() {
        HookProbe probe = new HookProbe();
        probe.hold(true);
        for (int i = 0; i < 3; i++) probe.tick();
        assertEquals(0.6, probe.percent(roller), 1e-9, "held sequence should reach its second stage");
        probe.hold(false);
        probe.tick();
        assertEquals(0.0, probe.percentOrZero(roller), 1e-9, "release should stop the held action");
        probe.hold(true);
        probe.tick();
        assertEquals(0.2, probe.percent(roller), 1e-9, "re-hold should restart at the first stage");
    }

    private final class HookProbe implements Mechanism {
        private final MotorDevice armMotor = arm;
        private final MotorDevice rollerMotor = roller;
        private final boolean[] pressed = {false};
        private final boolean[] held = {false};
        private final Action pressAction = pct(arm, 0.3).timeout(0.06);
        private final Action heldAction = Actions.sequence()
                .forTime(0.04, pct(roller, 0.2))
                .run(pct(roller, 0.6));
        private final HookBinding onPress = Events.when(() -> pressed[0]).rising().onStart(pressAction);
        private final HookBinding whileHeld = Events.when(() -> held[0]).active().whileActive(heldAction);
        final RecordingContext context = new RecordingContext();
        final MechanismScheduler scheduler = MechanismScheduler.create(context).register(this);
        double now;

        void press(boolean value) {
            pressed[0] = value;
        }

        void hold(boolean value) {
            held[0] = value;
        }

        void tick() {
            scheduler.teleopPeriodic(now, DT);
            now += DT;
        }

        double percent(MotorDevice motor) {
            return context.motor(motor).percent;
        }

        double percentOrZero(MotorDevice motor) {
            double value = percent(motor);
            return Double.isNaN(value) ? 0.0 : value;
        }
    }

    private static Action pct(MotorDevice motor, double value) {
        return motor.percent(value);
    }

    private final class Probe implements Mechanism {
        private final MotorDevice armMotor = arm;
        private final MotorDevice rollerMotor = roller;
        final RecordingContext context = new RecordingContext();
        final MechanismScheduler scheduler;
        double now;

        Probe() {
            scheduler = MechanismScheduler.create(context).register(this);
        }

        void tick() {
            scheduler.teleopPeriodic(now, DT);
            now += DT;
        }

        void disabledTick() {
            scheduler.disabledPeriodic(now, DT);
            now += DT;
        }

        double percent(MotorDevice motor) {
            return context.motor(motor).percent;
        }

        double percentOrZero(MotorDevice motor) {
            double value = percent(motor);
            return Double.isNaN(value) ? 0.0 : value;
        }
    }

    static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new HashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        double percent = Double.NaN;

        RecordingMotor(MotorDevice device) {
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
            percent = 0.0;
        }
    }
}
