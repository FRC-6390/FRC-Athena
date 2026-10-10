package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Robot-loop robustness: tolerance targets, fail-safe inputs, re-seeding, guards, and isolation. */
class ControlRobustnessTest {
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 61);
    private static final MotorDevice SHOOTER = MotorDevice.of(MotorKinds.KRAKEN_X60, 62);
    private static final MotorDevice HOOD = MotorDevice.of(MotorKinds.KRAKEN_X44, 63);
    private static final EncoderDevice ARM_ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 64);

    @Test
    void untilWithinComparesAgainstTheRangeConstrainedTarget() {
        RecordingContext hardware = new RecordingContext();
        hardware.motor(ARM).position = 5.0;
        ControlBinding<?> arm = Controls.position(ARM).constraint(Constraints.range(Range.of(0.0, 5.0)));
        Action raise = arm.position(10.0).untilWithin(0.1);
        MechanismScheduler scheduler = scheduler(hardware, new Holder(arm));

        scheduler.request(raise);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(5.0, hardware.motor(ARM).positionTarget, 1.0e-9);
        assertTrue(scheduler.isComplete(raise));
    }

    @Test
    void parallelLatchesAToleranceChildButKeepsHoldingItsOutput() {
        double[] speed = {100.0};
        boolean[] fed = {false};
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> shooter = Controls.velocity(SHOOTER).feedback(() -> 0.0, () -> speed[0]);
        Action shoot = Actions.parallel(
                shooter.velocity(100.0).untilWithin(5.0),
                Actions.waitUntil(() -> fed[0]));
        MechanismScheduler scheduler = scheduler(hardware, new Holder(shooter));

        scheduler.request(shoot);
        scheduler.teleopPeriodic(0.0, 0.02);
        speed[0] = 60.0;
        hardware.motor(SHOOTER).velocityTarget = Double.NaN;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(100.0, hardware.motor(SHOOTER).velocityTarget, 1.0e-9, "spin-up output must be held");
        assertFalse(scheduler.isComplete(shoot));

        fed[0] = true;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertTrue(scheduler.isComplete(shoot), "a latched tolerance child counts as complete");
    }

    @Test
    void interpolatedTargetHoldsLastValidValueAndNeverThrowsInTheLoop() {
        double[] distance = {1.0};
        double[] movingPoint = {2.0};
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> hood = Controls.position(HOOD);
        InterpolatedControlAction aim = hood.interpolate(InterpolationKinds.LINEAR, () -> distance[0])
                .at(0.0, 10.0)
                .at(() -> movingPoint[0], 30.0);
        MechanismScheduler scheduler = scheduler(hardware, new Holder(hood));
        FaultIsolation.drainWarnings();

        scheduler.request(aim);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(20.0, hardware.motor(HOOD).positionTarget, 1.0e-9);

        distance[0] = Double.NaN;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(20.0, hardware.motor(HOOD).positionTarget, 1.0e-9);

        distance[0] = 1.0;
        movingPoint[0] = 0.0;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(20.0, hardware.motor(HOOD).positionTarget, 1.0e-9);
        assertFalse(scheduler.drainRuntimeWarnings().isEmpty());
        assertThrows(IllegalArgumentException.class, aim::target, "the explicit API still reports bad data");
    }

    @Test
    void interpolatedTargetWithoutAnyValidSampleIsNeutral() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> hood = Controls.position(HOOD);
        InterpolatedControlAction aim = hood.interpolate(InterpolationKinds.LINEAR, () -> Double.NaN)
                .at(0.0, 10.0)
                .at(1.0, 30.0);
        MechanismScheduler scheduler = scheduler(hardware, new Holder(hood));

        scheduler.request(aim);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertTrue(Double.isNaN(hardware.motor(HOOD).positionTarget));
        assertEquals(1, hardware.motor(HOOD).stops);
    }

    @Test
    void dynamicControlTargetHoldsLastFiniteSample() {
        double[] target = {3.0};
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> hood = Controls.position(HOOD);
        Action track = hood.position(() -> target[0]);
        MechanismScheduler scheduler = scheduler(hardware, new Holder(hood));

        scheduler.request(track);
        scheduler.teleopPeriodic(0.0, 0.02);
        target[0] = Double.POSITIVE_INFINITY;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(3.0, hardware.motor(HOOD).positionTarget, 1.0e-9);
    }

    @Test
    void encoderSetPositionReseedsARunningProfiledControl() {
        RecordingContext hardware = new RecordingContext();
        ControlBinding<?> arm = Controls.position(ARM)
                .feedback(ARM_ENCODER)
                .pid(1.0, 0.0, 0.0)
                .profile(new MotionProfile(1.0, 100.0));
        Action move = arm.position(10.0);
        MechanismScheduler scheduler = scheduler(hardware, new Holder(arm, ARM_ENCODER));

        scheduler.request(move);
        for (int cycle = 0; cycle < 5; cycle++) scheduler.teleopPeriodic(cycle * 0.02, 0.02);
        scheduler.request(Actions.setPosition(ARM_ENCODER, 100.0));
        scheduler.teleopPeriodic(0.10, 0.02);
        scheduler.teleopPeriodic(0.12, 0.02);

        // The profile restarts from the new feedback (≈100) instead of chasing its old reference
        // (≈0.1), so the proportional output stays small instead of saturating at -12 V.
        assertEquals(100.0, hardware.encoder(ARM_ENCODER).position, 1.0e-9);
        assertTrue(Math.abs(hardware.motor(ARM).voltage) < 0.5, "voltage " + hardware.motor(ARM).voltage);
    }

    @Test
    void finalGuardProbeIsIndependentOfControlUnits() {
        RecordingContext hardware = new RecordingContext();
        hardware.motor(ARM).position = 0.5;
        ControlBinding<?> arm = Controls.position(ARM)
                .constraint(Constraints.require(context -> context.requested() < 0.9));
        MechanismScheduler scheduler = scheduler(hardware, new Holder(arm));

        scheduler.request(arm.percent(0.3));
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.3, hardware.motor(ARM).percent, 1.0e-9, "0.4 below the limit must not be blocked");

        hardware.motor(ARM).position = 0.95;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void dashboardTunedLimitsAlsoBoundThePlannerResult() {
        RecordingContext hardware = new RecordingContext();
        hardware.motor(ARM).position = 350.0;
        ControlBinding<?> turret = Controls.position(ARM)
                .constraint(Constraints.range(Range.of(0.0, 400.0)))
                .planner(MotionPlanners.boundedAngular(360.0));
        RuntimeOverrides overrides = new RuntimeOverrides();
        overrides.maximumPosition(turret, 5.0);
        OutputApplier applier = OutputApplier.using(hardware, overrides);
        Output target = Outputs.position(5.0);

        applier.apply(new ResolvedOutput(OutputRequest.of(turret, target), target));

        assertEquals(5.0, hardware.motor(ARM).positionTarget, 1.0e-9);
    }

    @Test
    void continuousControlIsNotOffloadedToADeviceClosedLoop() {
        RecordingContext hardware = new RecordingContext();
        hardware.motor(ARM).capabilities = MotorControlCapabilities.voltageClosedLoop(1);
        hardware.motor(ARM).position = 350.0;
        ControlBinding<?> wrapped = Controls.position(ARM).pid(0.01, 0.0, 0.0).continuous(0.0, 360.0);
        Output target = Outputs.position(10.0);

        OutputApplier.using(hardware).apply(new ResolvedOutput(OutputRequest.of(wrapped, target), target));

        assertFalse(hardware.motor(ARM).offloaded);
        assertEquals(0.2, hardware.motor(ARM).voltage, 1.0e-9, "software PID takes the 20 degree short way");
    }

    @Test
    void nonOverlappingRangesFailWhenTheBindingIsDeclaredAndTuningIsCached() {
        assertThrows(IllegalArgumentException.class, () -> Controls.position(ARM)
                .constraint(Constraints.range(Range.of(0.0, 1.0)))
                .constraint(Constraints.range(Range.of(2.0, 3.0))));
        ControlBinding<?> arm = Controls.position(ARM).constraint(Constraints.range(Range.of(0.0, 1.0)));
        RuntimeOverrides overrides = new RuntimeOverrides();
        assertSame(overrides.tuning(arm), overrides.tuning(arm));
    }

    @Test
    void isolatedFaultNeutralizesOnlyTheFailingLeaseAndIsReportedRateLimited() {
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(new Holder(ARM, SHOOTER))
                .isolateFaults(true);
        Action broken = Actions.compute(() -> {
            throw new IllegalStateException("boom");
        }, ARM);

        scheduler.request(SHOOTER.percent(0.7));
        scheduler.request(broken);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.7, hardware.motor(SHOOTER).percent, 1.0e-9);
        assertTrue(Double.isNaN(hardware.motor(ARM).percent));
        List<MechanismScheduler.RuntimeFault> faults = scheduler.drainRuntimeFaults();
        assertEquals(1, faults.size());
        assertTrue(faults.get(0).message().contains("boom"));
        scheduler.teleopPeriodic(0.04, 0.02);
        assertTrue(scheduler.drainRuntimeFaults().isEmpty(), "repeats are rate limited");
        scheduler.teleopPeriodic(5.1, 0.02);
        assertEquals(4, scheduler.drainRuntimeFaults().get(0).occurrences());
    }

    @Test
    void isolationStillPropagatesPanicFailures() {
        MotorDevice panicMotor = MotorDevice.of(MotorKinds.KRAKEN_X60, 65).failurePolicy(FailurePolicy.PANIC);
        RecordingContext hardware = new RecordingContext();
        hardware.motor(panicMotor).failing = true;
        MechanismScheduler scheduler = MechanismScheduler.create(hardware)
                .register(new Holder(panicMotor))
                .isolateFaults(true);

        scheduler.request(panicMotor.percent(0.5));

        assertThrows(IllegalStateException.class, () -> scheduler.teleopPeriodic(0.0, 0.02));
    }

    private static MechanismScheduler scheduler(RecordingContext hardware, Holder holder) {
        return MechanismScheduler.create(hardware).register(holder);
    }

    /** Declares arbitrary objects so actions targeting them have an owner. */
    private static final class Holder implements Mechanism {
        @SuppressWarnings("unused")
        private final Object first;
        @SuppressWarnings("unused")
        private final Object second;

        private Holder(Object first) {
            this(first, null);
        }

        private Holder(Object first, Object second) {
            this.first = first;
            this.second = second;
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
        private double percent = Double.NaN;
        private double voltage = Double.NaN;
        private double positionTarget = Double.NaN;
        private double velocityTarget = Double.NaN;
        private double position;
        private int stops;
        private boolean offloaded;
        private boolean failing;
        private MotorControlCapabilities capabilities = MotorControlCapabilities.OPEN_LOOP_ONLY;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public MotorControlCapabilities controlCapabilities() { return capabilities; }

        @Override
        public void setPercentOutput(double percent) {
            if (failing) throw new IllegalStateException("CAN failure");
            this.percent = percent;
        }

        @Override public void setVoltage(double volts) { voltage = volts; }
        @Override public void setPositionTargetRotations(double rotations) { positionTarget = rotations; }

        @Override
        public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
            offloaded = true;
            positionTarget = rotations;
        }

        @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocityTarget = rotationsPerSecond;
        }

        @Override
        public void stop() {
            stops++;
            percent = 0.0;
            voltage = 0.0;
        }

        @Override public double integratedPositionRotations() { return position; }
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
