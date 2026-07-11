package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.HookGroup;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
import java.time.Duration;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

class ControlActionIntegrationTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 41);
    private static final MotorDevice SECOND_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X44, 42);

    @Test
    void toggleLeaseIsOverriddenByANewerHeldActionAndResumesOnRelease() {
        boolean[] toggleRaw = {false};
        boolean[] overrideRaw = {false};
        RecordingContext hardware = new RecordingContext();
        Target target = new Target();
        LeaseControls controls = new LeaseControls(target, toggleRaw, overrideRaw);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(target)
                .register(controls);

        tick(scheduler, 0);
        toggleRaw[0] = true;
        tick(scheduler, 1);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);

        toggleRaw[0] = false;
        tick(scheduler, 2);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);

        overrideRaw[0] = true;
        tick(scheduler, 3);
        assertEquals(-0.80, hardware.primary.percent, 1.0e-9);
        tick(scheduler, 4);
        assertEquals(-0.80, hardware.primary.percent, 1.0e-9);

        overrideRaw[0] = false;
        tick(scheduler, 5);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);

        toggleRaw[0] = true;
        tick(scheduler, 6);
        assertEquals(0.0, hardware.primary.percent, 1.0e-9);
        assertEquals(1, hardware.primary.stopCalls);
        tick(scheduler, 7);
        assertEquals(1, hardware.primary.stopCalls);
    }

    @Test
    void disablingResetsToggleAndReleasesItsActionLease() {
        boolean[] toggleRaw = {false};
        RecordingContext hardware = new RecordingContext();
        Target target = new Target();
        LeaseControls controls = new LeaseControls(target, toggleRaw, new boolean[] {false});
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(target)
                .register(controls);

        tick(scheduler, 0);
        toggleRaw[0] = true;
        tick(scheduler, 1);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);

        toggleRaw[0] = false;
        scheduler.disabledPeriodic(0.04, 0.02);
        assertEquals(0.0, hardware.primary.percent, 1.0e-9);
        assertEquals(1, hardware.primary.stopCalls);

        tick(scheduler, 3);
        assertEquals(0.0, hardware.primary.percent, 1.0e-9);
        toggleRaw[0] = true;
        tick(scheduler, 4);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);
    }

    @Test
    void latchedCompositeActionPartitionsAcrossMechanismsAndSurvivesPartialOverride() {
        boolean[] toggleRaw = {false};
        boolean[] overrideRaw = {false};
        RecordingContext hardware = new RecordingContext();
        Target target = new Target();
        SecondTarget secondTarget = new SecondTarget();
        CompositeControls controls = new CompositeControls(
                target,
                secondTarget,
                toggleRaw,
                overrideRaw);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(target)
                .register(secondTarget)
                .register(controls);

        tick(scheduler, 0);
        toggleRaw[0] = true;
        tick(scheduler, 1);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);
        assertEquals(0.60, hardware.secondary.percent, 1.0e-9);

        toggleRaw[0] = false;
        overrideRaw[0] = true;
        tick(scheduler, 2);
        assertEquals(-0.80, hardware.primary.percent, 1.0e-9);
        assertEquals(0.60, hardware.secondary.percent, 1.0e-9);

        overrideRaw[0] = false;
        tick(scheduler, 3);
        assertEquals(0.35, hardware.primary.percent, 1.0e-9);
        assertEquals(0.60, hardware.secondary.percent, 1.0e-9);

        toggleRaw[0] = true;
        tick(scheduler, 4);
        assertEquals(0.0, hardware.primary.percent, 1.0e-9);
        assertEquals(0.0, hardware.secondary.percent, 1.0e-9);
        assertEquals(1, hardware.primary.stopCalls);
        assertEquals(1, hardware.secondary.stopCalls);
    }

    @Test
    void clickPulseRunsEveryHookPhaseExactlyAsDeclared() {
        boolean[] raw = {false};
        int[] starts = {0};
        int[] active = {0};
        int[] ends = {0};
        int[] inactive = {0};
        ButtonSignal button = new ButtonSignal("click", () -> raw[0]);
        ControlSignal click = button.clicks(1, Duration.ofMillis(100))
                .onTrue(() -> starts[0]++)
                .whileTrue(() -> active[0]++)
                .onFalse(() -> ends[0]++)
                .whileFalse(() -> inactive[0]++);
        SignalControls controls = new SignalControls(click);
        MechanismScheduler scheduler = MechanismScheduler.create().register(controls);

        tick(scheduler, 0);
        raw[0] = true;
        tick(scheduler, 1);
        raw[0] = false;
        tick(scheduler, 2);
        tick(scheduler, 3);
        tick(scheduler, 8);
        assertEquals(1, starts[0]);
        assertEquals(1, active[0]);

        tick(scheduler, 9);
        assertEquals(1, ends[0]);
        assertEquals(1, active[0]);
        assertEquals(1, starts[0]);
        assertEquals(5, inactive[0]);
    }

    @Test
    void thousandsOfToggleAndOverrideTransitionsPreserveLeaseRecencyAndFallback() {
        boolean[] toggleRaw = {false};
        boolean[] overrideRaw = {false};
        RecordingContext hardware = new RecordingContext();
        Target target = new Target();
        LeaseControls controls = new LeaseControls(target, toggleRaw, overrideRaw);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(target)
                .register(controls);
        Random random = new Random(6390L);
        boolean toggleState = false;
        boolean previousToggleRaw = false;
        boolean previousOverrideRaw = false;
        long sequence = 0;
        long toggleRecency = Long.MIN_VALUE;
        long overrideRecency = Long.MIN_VALUE;

        for (int iteration = 0; iteration < 4_000; iteration++) {
            if ((iteration & 1) == 0) {
                if (random.nextDouble() < 0.35) {
                    toggleRaw[0] = !toggleRaw[0];
                }
            } else if (random.nextDouble() < 0.45) {
                overrideRaw[0] = !overrideRaw[0];
            }

            if (toggleRaw[0] && !previousToggleRaw) {
                toggleState = !toggleState;
                if (toggleState) {
                    toggleRecency = ++sequence;
                }
            }
            if (overrideRaw[0] && !previousOverrideRaw) {
                overrideRecency = ++sequence;
            }
            previousToggleRaw = toggleRaw[0];
            previousOverrideRaw = overrideRaw[0];

            tick(scheduler, iteration);

            double expected;
            if (overrideRaw[0] && (!toggleState || overrideRecency > toggleRecency)) {
                expected = -0.80;
            } else if (toggleState) {
                expected = 0.35;
            } else {
                expected = 0.0;
            }
            assertEquals(expected, hardware.primary.percent, 1.0e-9, "iteration " + iteration);
        }
    }

    private static void tick(MechanismScheduler scheduler, int tick) {
        scheduler.teleopPeriodic(tick * 0.02, 0.02);
    }

    private static final class Target implements Mechanism {
        private final Action latched = MOTOR.percent(0.35);
        private final Action override = MOTOR.percent(-0.80);
    }

    private static final class SecondTarget implements Mechanism {
        private final Action run = SECOND_MOTOR.percent(0.60);
    }

    private static final class LeaseControls implements Mechanism {
        private final HookGroup bindings;

        private LeaseControls(Target target, boolean[] toggleRaw, boolean[] overrideRaw) {
            ButtonSignal toggleButton = new ButtonSignal("toggle", () -> toggleRaw[0]);
            ToggleSignal toggle = toggleButton.toggle();
            ButtonSignal overrideButton = new ButtonSignal("override", () -> overrideRaw[0]);
            toggle.whileTrue(target.latched);
            overrideButton.whileHeld(target.override);
            bindings = group(toggle, overrideButton);
        }
    }

    private static final class SignalControls implements Mechanism {
        private final HookGroup bindings;

        private SignalControls(ControlSignal signal) {
            bindings = group(signal);
        }
    }

    private static final class CompositeControls implements Mechanism {
        private final HookGroup bindings;

        private CompositeControls(
                Target target,
                SecondTarget secondTarget,
                boolean[] toggleRaw,
                boolean[] overrideRaw) {
            ToggleSignal toggle = new ButtonSignal("toggle", () -> toggleRaw[0]).toggle();
            ButtonSignal override = new ButtonSignal("override", () -> overrideRaw[0]);
            toggle.whileTrue(Actions.parallel(target.latched, secondTarget.run));
            override.whileHeld(target.override);
            bindings = group(toggle, override);
        }
    }

    private static HookGroup group(ControlSignal... signals) {
        Map<String, HookBinding> hooks = new LinkedHashMap<>();
        for (int index = 0; index < signals.length; index++) {
            hooks.put(signals[index].name() + "." + index, signals[index].binding());
        }
        Map<String, HookBinding> copy = Map.copyOf(hooks);
        return () -> copy;
    }

    private static final class RecordingContext implements ActionContext {
        private final RecordingMotor primary = new RecordingMotor(MOTOR);
        private final RecordingMotor secondary = new RecordingMotor(SECOND_MOTOR);

        @Override
        public RecordingMotor motor(MotorDevice requested) {
            if (requested == MOTOR) {
                return primary;
            }
            if (requested == SECOND_MOTOR) {
                return secondary;
            }
            throw new IllegalArgumentException("unexpected motor");
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double percent;
        private int stopCalls;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setPercentOutput(double requested) {
            percent = requested;
        }

        @Override
        public void stop() {
            percent = 0.0;
            stopCalls++;
        }
    }
}
