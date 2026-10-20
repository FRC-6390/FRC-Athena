package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Motion policies: retarget, release, interrupt holds, abort, bounded commitment, fallbacks, bumpless gains. */
class MotionPolicyTest {
    private static final double DT = 0.02;
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 81);
    private static final MotorDevice OTHER = MotorDevice.of(MotorKinds.KRAKEN_X60, 82);
    private static final EncoderDevice ARM_ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 83);

    /** P-only control on a stationary measurement: the output voltage equals kP times the reference. */
    private static ControlBinding<?> stationaryArm() {
        return Controls.position(ARM, Units.Rotations).feedback(ARM_ENCODER).pid(1.0, 0.0, 0.0);
    }

    @Test
    void bumplessGainChangeKeepsTheOutputContinuous() {
        RecordingContext hardware = new RecordingContext();
        PidGains gains = PidGains.of(2.0, 0.5, 0.0);
        ControlBinding<?> arm = Controls.position(ARM, Units.Rotations).feedback(ARM_ENCODER).pid(gains);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(arm.position(Units.Rotations.of(3.0)));
        for (int cycle = 0; cycle < 20; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        double before = hardware.motor(ARM).voltage;

        gains.telemetry().get("p").set((Object) 6.0);
        scheduler.teleopPeriodic(20 * DT, DT);
        double atChange = hardware.motor(ARM).voltage;

        assertEquals(before, atChange, 0.05, "output must not jump when kP changes");
        scheduler.teleopPeriodic(21 * DT, DT);
        double after = hardware.motor(ARM).voltage;
        assertTrue(Math.abs(after - atChange) < 1.0, "the loop keeps running after the change");
    }

    @Test
    void smoothRetargetInheritsTheReferenceVelocity() {
        double[] smooth = takeoverVoltages(Retarget.SMOOTH);
        double[] immediate = takeoverVoltages(Retarget.IMMEDIATE);
        // Voltage equals the reference position for a P-only loop on a stationary measurement.
        double smoothStep = smooth[1] - smooth[0];
        double immediateStep = immediate[1] - immediate[0];
        assertTrue(smoothStep > 0.0, "SMOOTH keeps moving forward: " + smoothStep);
        assertTrue(immediateStep < 0.0, "IMMEDIATE re-plans from rest toward the new target: " + immediateStep);
        assertTrue(Math.abs(smoothStep) < 0.1, "SMOOTH never steps: " + smoothStep);
    }

    /** References sampled before and after a takeover to a new target in the opposite direction. */
    private static double[] takeoverVoltages(Retarget retarget) {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm()
                .profile(new MotionProfile(1.0, 2.0))
                .retarget(retarget);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(arm.position(Units.Rotations.of(10.0)));
        for (int cycle = 0; cycle < 10; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        double before = hardware.motor(ARM).voltage;
        scheduler.request(arm.position(Units.Rotations.of(-10.0)));
        scheduler.teleopPeriodic(10 * DT, DT);
        return new double[] {before, hardware.motor(ARM).voltage};
    }

    @Test
    void smoothResumeAfterAnIdleGapInheritsTheReferenceVelocity() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm().profile(new MotionProfile(1.0, 2.0));
        Action move = arm.position(Units.Rotations.of(10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(move);
        for (int cycle = 0; cycle < 10; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        scheduler.cancel(move);
        scheduler.teleopPeriodic(10 * DT, DT);      // idle cycle: nothing drives the arm
        scheduler.request(move);
        scheduler.teleopPeriodic(11 * DT, DT);

        // The reference still moves at ~0.4 units/s, so the resumed reference is well past the start.
        assertTrue(hardware.motor(ARM).voltage > 0.03, "resumed at " + hardware.motor(ARM).voltage);
    }

    @Test
    void perActionRetargetOverridesTheBindingDefault() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm().profile(new MotionProfile(1.0, 2.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(arm.position(Units.Rotations.of(10.0)));
        for (int cycle = 0; cycle < 10; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        double before = hardware.motor(ARM).voltage;
        scheduler.request(arm.position(Units.Rotations.of(-10.0)).retarget(Retarget.IMMEDIATE));
        scheduler.teleopPeriodic(10 * DT, DT);
        assertTrue(hardware.motor(ARM).voltage - before < 0.0);
    }

    @Test
    void holdKeepsTheLastReferenceAfterTheLeaseEnds() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm().release(Release.HOLD);
        Action move = arm.position(Units.Rotations.of(5.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(move);
        for (int cycle = 0; cycle < 3; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        scheduler.cancel(move);
        scheduler.teleopPeriodic(3 * DT, DT);
        scheduler.teleopPeriodic(4 * DT, DT);

        assertEquals(5.0, hardware.motor(ARM).voltage, 1.0e-9, "HOLD keeps commanding the last reference");
        assertFalse(scheduler.isRunning(move));
    }

    @Test
    void neutralReleaseStopsTheMotorAfterTheLeaseEnds() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        Action move = arm.position(Units.Rotations.of(5.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(move);
        for (int cycle = 0; cycle < 3; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        scheduler.cancel(move);
        scheduler.teleopPeriodic(3 * DT, DT);
        assertEquals(0.0, hardware.motor(ARM).voltage, 1.0e-9);
    }

    @Test
    void neutralModeReleaseOverridesUntilTheMotorIsCommandedAgain() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm().release(Release.neutral(MotorNeutralMode.BRAKE));
        Action move = arm.position(Units.Rotations.of(5.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(move);
        for (int cycle = 0; cycle < 3; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        scheduler.cancel(move);
        scheduler.teleopPeriodic(3 * DT, DT);
        assertEquals(MotorNeutralMode.BRAKE, hardware.motor(ARM).runtimeNeutral);

        scheduler.request(move);
        scheduler.teleopPeriodic(4 * DT, DT);
        assertEquals(MotorRuntimeConfig.declared(ARM).neutralMode(), hardware.motor(ARM).runtimeNeutral);
    }

    @Test
    void decelerateBrakesAlongTheProfileAndThenHolds() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm()
                .profile(new MotionProfile(1.0, 2.0))
                .release(Release.DECELERATE);
        Action move = arm.position(Units.Rotations.of(10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(move);
        for (int cycle = 0; cycle < 40; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        double moving = hardware.motor(ARM).voltage;
        scheduler.cancel(move);

        for (int cycle = 40; cycle < 240; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        double settled = hardware.motor(ARM).voltage;
        scheduler.teleopPeriodic(240 * DT, DT);
        assertTrue(settled > moving, "braking keeps moving forward: " + moving + " -> " + settled);
        assertTrue(settled < 10.0, "braking stops short of the original target: " + settled);
        assertEquals(settled, hardware.motor(ARM).voltage, 1.0e-6, "the held reference settles");
        // Braking distance at 2 units/s^2 from ~0.8 units/s is 0.16; the stop lies within one profile unit.
        assertEquals(moving + 0.5 * moving * moving / 2.0, settled, 0.25);
    }

    @Test
    void afterCompleteHoldsNewerRequestsForOverlappingResources() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        Action first = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofMillis(200)).interruptible(Interrupt.AFTER_COMPLETE);
        Action second = arm.position(Units.Rotations.of(-10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(first);
        scheduler.request(second);
        for (int cycle = 0; cycle < 5; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        assertTrue(scheduler.isRunning(first));
        assertFalse(scheduler.isRunning(second), "held until the first Action completes");
        assertTrue(hardware.motor(ARM).voltage > 0.0);

        for (int cycle = 5; cycle < 20; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
        assertTrue(scheduler.isComplete(first));
        assertTrue(scheduler.isRunning(second), "the newer request proceeds after completion");
        assertTrue(hardware.motor(ARM).voltage < 0.0 || scheduler.isRunning(second));
    }

    @Test
    void afterConditionReleasesTheHeldRequestWhenItBecomesTrue() {
        RecordingContext hardware = new RecordingContext();
        boolean[] allowed = {false};
        ControlBinding<?> arm = stationaryArm();
        Action first = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofSeconds(5))
                .interruptible(Interrupt.after(() -> allowed[0]));
        Action second = arm.position(Units.Rotations.of(-10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(new Holder(arm, ARM_ENCODER));
        scheduler.request(first);
        scheduler.request(second);
        scheduler.teleopPeriodic(0.0, DT);
        assertFalse(scheduler.isRunning(second));
        allowed[0] = true;
        scheduler.teleopPeriodic(DT, DT);
        assertTrue(scheduler.isRunning(second));
        assertTrue(hardware.motor(ARM).voltage < 0.0, "the newer request now drives the arm");
    }

    @Test
    void disableOverridesAnInterruptCommitment() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        Action hold = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofSeconds(10)).interruptible(Interrupt.AFTER_COMPLETE);
        Action blocked = arm.position(Units.Rotations.of(-10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(new Holder(arm, ARM_ENCODER, hold, blocked));
        scheduler.request(hold);
        scheduler.request(blocked);
        scheduler.teleopPeriodic(0.0, DT);
        assertFalse(scheduler.isRunning(blocked));

        scheduler.disabledPeriodic(DT, DT);
        assertTrue(scheduler.isRunning(blocked), "disable overrides the running Action's commitment");
    }

    @Test
    void heldRequestOnDisjointResourcesRunsImmediately() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        ControlBinding<?> other = Controls.position(OTHER, Units.Rotations);
        Action hold = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofSeconds(1)).interruptible(Interrupt.AFTER_COMPLETE);
        Action blocked = arm.position(Units.Rotations.of(-10.0));
        Action unrelated = other.percent(0.5);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(new Holder(arm, other, ARM_ENCODER));
        scheduler.request(hold);
        scheduler.request(blocked);
        scheduler.request(unrelated);
        scheduler.teleopPeriodic(0.0, DT);
        assertFalse(scheduler.isRunning(blocked));
        assertTrue(scheduler.isRunning(unrelated));
        assertEquals(0.5, hardware.motor(OTHER).percent, 1.0e-9);
    }

    @Test
    void cancelAndAbortOverrideTheInterruptPolicy() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        Action hold = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofSeconds(10)).interruptible(Interrupt.AFTER_COMPLETE);
        Action blocked = arm.position(Units.Rotations.of(-10.0));
        Holder holder = new Holder(arm, ARM_ENCODER, hold, blocked);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(holder);
        scheduler.request(hold);
        scheduler.request(blocked);
        scheduler.teleopPeriodic(0.0, DT);
        assertTrue(scheduler.isRunning(hold));

        scheduler.request(Actions.abort(holder));
        scheduler.teleopPeriodic(DT, DT);
        assertFalse(scheduler.isRunning(hold), "abort cancels the committed Action");
        assertTrue(scheduler.isRunning(blocked), "the held request then proceeds");
    }

    @Test
    void cancelOfARunningPolicyActionStartsTheHeldRequest() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = stationaryArm();
        Action hold = arm.position(Units.Rotations.of(10.0)).atMost(Duration.ofSeconds(10)).interruptible(Interrupt.AFTER_COMPLETE);
        Action blocked = arm.position(Units.Rotations.of(-10.0));
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(new Holder(arm, ARM_ENCODER, hold, blocked));
        scheduler.request(hold);
        scheduler.request(blocked);
        scheduler.teleopPeriodic(0.0, DT);
        scheduler.cancel(hold);
        scheduler.teleopPeriodic(DT, DT);
        assertTrue(scheduler.isRunning(blocked));
    }

    @Test
    void boundedCommitmentIsCheckedAtDeclaration() {
        ControlBinding<?> arm = stationaryArm();
        Action open = arm.position(Units.Rotations.of(10.0));
        IllegalArgumentException rejected = assertThrows(IllegalArgumentException.class,
                () -> open.interruptible(Interrupt.AFTER_COMPLETE));
        assertTrue(rejected.getMessage().contains("bounded"), rejected.getMessage());
        assertThrows(IllegalArgumentException.class, () -> open.until(() -> false).interruptible(Interrupt.after(() -> true)));
        assertFalse(Actions.isBounded(open));
        assertFalse(Actions.isBounded(Actions.waitUntil(() -> true)));
        assertTrue(Actions.isBounded(open.atMost(Duration.ofMillis(300))));
        assertTrue(Actions.isBounded(Actions.timeout(open, Seconds.of(1.0))));
        assertTrue(Actions.isBounded(Actions.sequence().forTime(Seconds.of(1.0), open)));
        assertFalse(Actions.isBounded(Actions.sequence().until(() -> true, open)));
        assertTrue(Actions.isBounded(Actions.fault("stop")));
        assertTrue(Actions.isBounded(Actions.neutral()));
        assertTrue(Actions.isBounded(Actions.waitSeconds(Seconds.of(0.5))));
        assertTrue(Actions.isBounded(Actions.race(open.atMost(Duration.ofSeconds(1)), open)));
        assertFalse(Actions.isBounded(Actions.parallel(open.atMost(Duration.ofSeconds(1)), open)));
    }

    @Test
    void nestedInterruptPolicyIsRejected() {
        ControlBinding<?> arm = stationaryArm();
        Action policied = arm.position(Units.Rotations.of(1.0)).atMost(Duration.ofSeconds(1)).interruptible(Interrupt.AFTER_COMPLETE);
        Action nested = Actions.parallel(policied, Actions.waitSeconds(Seconds.of(0.1)));
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext())
                .register(new Holder(arm, ARM_ENCODER));
        assertThrows(IllegalArgumentException.class, () -> scheduler.request(nested));
    }

    @Test
    void retargetPolicyRejectsNonControlActions() {
        assertThrows(IllegalArgumentException.class,
                () -> Actions.waitSeconds(Seconds.of(1.0)).retarget(Retarget.SMOOTH));
    }

    @Test
    void attemptUsesTheFirstSuccessfulStep() {
        boolean[] ran = {false, false, false};
        Action chain = Actions.attempt(Actions.run(() -> ran[0] = true))
                .within(Seconds.of(0.5))
                .orTry(Actions.run(() -> ran[1] = true))
                .within(Seconds.of(0.5))
                .orElse(Actions.run(() -> ran[2] = true));
        runChain(chain, 0.2);
        assertTrue(ran[0]);
        assertFalse(ran[1]);
        assertFalse(ran[2]);
    }

    @Test
    void attemptMovesOnWhenAStepMissesItsTimeLimit() {
        boolean[] ran = {false, false, false};
        Action chain = Actions.attempt(Actions.waitUntil(() -> false))
                .within(Seconds.of(0.1))
                .orTry(Actions.run(() -> ran[1] = true))
                .within(Seconds.of(0.5))
                .orElse(Actions.run(() -> ran[2] = true));
        runChain(chain, 0.5);
        assertTrue(ran[1], "the second attempt runs after the first times out");
        assertFalse(ran[2], "fallback runs only when every attempt fails");
    }

    @Test
    void attemptRunsTheFallbackWhenEveryStepFails() {
        boolean[] ran = {false};
        Action chain = Actions.attempt(Actions.waitUntil(() -> false))
                .within(Seconds.of(0.1))
                .orTry(Actions.waitUntil(() -> false))
                .within(Seconds.of(0.1))
                .orElse(Actions.run(() -> ran[0] = true));
        runChain(chain, 0.5);
        assertTrue(ran[0]);
    }

    /** Runs a request for {@code seconds} of periodic cycles on a mechanism that declares the chain. */
    private static void runChain(Action chain, double seconds) {
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext())
                .register(new Holder(chain));
        scheduler.request(chain);
        int cycles = (int) Math.round(seconds / DT);
        for (int cycle = 0; cycle < cycles; cycle++) scheduler.teleopPeriodic(cycle * DT, DT);
    }

    @Test
    void faultReportsThroughTheRuntimeFaultChannelAndCompletes() {
        Action fault = Actions.fault("arm jammed");
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext())
                .register(new Holder(fault));
        scheduler.request(fault);
        scheduler.teleopPeriodic(0.0, DT);
        assertTrue(scheduler.isComplete(fault));
        List<MechanismScheduler.RuntimeFault> faults = scheduler.drainRuntimeFaults();
        assertEquals(1, faults.size());
        assertEquals("arm jammed", faults.get(0).exception().getMessage());
    }

    /** Declares arbitrary objects so actions targeting them have an owner (one typed field per object). */
    private static final class Holder implements Mechanism {
        @SuppressWarnings("unused") private final Object first;
        @SuppressWarnings("unused") private final Object second;
        @SuppressWarnings("unused") private final Object third;
        @SuppressWarnings("unused") private final Object fourth;
        @SuppressWarnings("unused") private final Object fifth;
        @SuppressWarnings("unused") private final Object sixth;

        private Holder(Object... declared) {
            Object[] values = new Object[6];
            System.arraycopy(declared, 0, values, 0, declared.length);
            first = values[0];
            second = values[1];
            third = values[2];
            fourth = values[3];
            fifth = values[4];
            sixth = values[5];
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new HashMap<>();
        private final Map<EncoderDevice, RecordingEncoder> encoders = new HashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }

        @Override
        public RecordingEncoder encoder(EncoderDevice encoder) {
            return encoders.computeIfAbsent(encoder, RecordingEncoder::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private final List<Double> history = new ArrayList<>();
        private double voltage = Double.NaN;
        private double percent = Double.NaN;
        private MotorNeutralMode runtimeNeutral;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }

        @Override public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override public void setVoltage(double volts) {
            voltage = volts;
            history.add(volts);
        }

        @Override public void stop() {
            voltage = 0.0;
            percent = 0.0;
        }

        @Override public boolean supportsRuntimeConfiguration() { return true; }

        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
            runtimeNeutral = configuration.neutralMode();
        }

        @Override public double integratedPositionRotations() { return 0.0; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
    }

    private static final class RecordingEncoder implements EncoderHandle {
        private final EncoderDevice device;
        private double position;

        private RecordingEncoder(EncoderDevice device) {
            this.device = device;
        }

        @Override public EncoderDevice device() { return device; }
        @Override public double positionRotations() { return position; }
        @Override public double absolutePositionRotations() { return position; }
        @Override public double velocityRotationsPerSecond() { return 0.0; }
        @Override public void setPositionRotations(double rotations) { position = rotations; }
        @Override public boolean supportsPositionSetting() { return true; }
    }
}
