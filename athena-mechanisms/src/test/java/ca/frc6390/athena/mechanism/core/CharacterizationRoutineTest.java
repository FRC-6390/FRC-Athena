package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.VoltageUnit;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class CharacterizationRoutineTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice FOLLOWER = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
    private static final EncoderDevice ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 2)
            .conversion(360.0)
            .units(EncoderUnit.DEGREES);

    @Test
    void heldQuasistaticActionRampsLogsNormalizedFeedbackAndEndsOnRelease() {
        boolean[] held = {false};
        RecordingContext hardware = new RecordingContext();
        hardware.encoder.positionRotations = 0.5;
        hardware.encoder.velocityRotationsPerSecond = 0.25;
        RecordingLog log = new RecordingLog();
        SysIdMechanism mechanism = new SysIdMechanism(held, log);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        held[0] = true;
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);

        scheduler.teleopPeriodic(0.5, 0.02);
        assertEquals(1.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        SysIdSample sample = log.samples.get(log.samples.size() - 1);
        assertEquals(SysIdState.QUASISTATIC_FORWARD, sample.state());
        assertEquals(0.5, sample.position(), 1.0e-9);
        assertEquals(0.25, sample.velocity(), 1.0e-9);

        held[0] = false;
        scheduler.teleopPeriodic(0.52, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
    }

    @Test
    void dynamicReverseUsesStepVoltageAndStopsAtTimeout() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization()
                .maxVoltage(Volts.of(6.0))
                .timeout(Seconds.of(0.5))
                .exportTo(log);
        TestMechanism mechanism = new TestMechanism(control, sysId.dynamicReverse());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-6.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(SysIdState.DYNAMIC_REVERSE, log.samples.get(0).state());

        scheduler.teleopPeriodic(0.5, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
    }

    @Test
    void characterizationDoesNotLogWhenANewerActionWinsTheMotor() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization().exportTo(log);
        TestMechanism mechanism = new TestMechanism(control, sysId.dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.request(control.neutral());
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(0, log.samples.size());
    }

    @Test
    void allFourTestsApplyTheConfiguredVoltageAndLogTheirState() {
        assertTestOutput(SysIdState.QUASISTATIC_FORWARD, 1.5, 3.0);
        assertTestOutput(SysIdState.QUASISTATIC_REVERSE, -1.5, 3.0);
        assertTestOutput(SysIdState.DYNAMIC_FORWARD, 5.5, 0.0);
        assertTestOutput(SysIdState.DYNAMIC_REVERSE, -5.5, 0.0);
    }

    @Test
    void heldActionDoesNotRestartAfterTimeoutAndCanRestartAfterRelease() {
        boolean[] held = {true};
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization().maxVoltage(Volts.of(4.0)).timeout(Seconds.of(0.1)).exportTo(log);
        Action action = sysId.dynamicForward();
        HeldMechanism mechanism = new HeldMechanism(control, held, action);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(4.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        scheduler.teleopPeriodic(0.1, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
        int samplesAtTimeout = log.samples.size();

        scheduler.teleopPeriodic(0.2, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(samplesAtTimeout, log.samples.size());
        assertEquals(1, log.endCalls);

        held[0] = false;
        scheduler.teleopPeriodic(0.22, 0.02);
        held[0] = true;
        scheduler.teleopPeriodic(0.24, 0.02);
        assertEquals(4.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(samplesAtTimeout + 1, log.samples.size());
    }

    @Test
    void sysIdCommandsEveryMotorInAControlGroup() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.velocity(MOTOR, FOLLOWER).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(
                control,
                control.characterization().maxVoltage(Volts.of(6.0)).exportTo(log).dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(6.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(6.0, hardware.motor(FOLLOWER).voltage, 1.0e-9);
        assertEquals(1, log.samples.size());
    }

    @Test
    void loggingFallsBackToCommandedVoltageWhenBackendTelemetryIsUnavailable() {
        RecordingContext hardware = new RecordingContext();
        hardware.motor(MOTOR).telemetryUnavailable = true;
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(
                control,
                control.characterization().maxVoltage(Volts.of(4.25)).exportTo(log).dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.teleopPeriodic(0.0, 0.02);

        SysIdSample sample = log.samples.get(0);
        assertEquals(4.25, sample.appliedVoltage(), 1.0e-9);
        assertEquals(true, Double.isNaN(sample.currentAmps()));
    }

    @Test
    void sysIdRequiresAnOutputFeedbackAndPositiveConfiguration() {
        assertThrows(IllegalStateException.class, () -> Controls.of(ControlMode.POSITION).characterization());
        assertThrows(IllegalStateException.class, () -> Controls.position(MOTOR).characterization());

        Characterization sysId = Controls.position(MOTOR).feedback(ENCODER).characterization();
        assertThrows(IllegalArgumentException.class, () -> sysId.rampRate(voltsPerSecond(0.0)));
        assertThrows(IllegalArgumentException.class, () -> sysId.maxVoltage(Volts.of(-1.0)));
        assertThrows(IllegalArgumentException.class, () -> sysId.timeout(Seconds.of(Double.NaN)));
    }

    @Test
    void nonFiniteFeedbackIsRejectedBeforeItCanCorruptTheLog() {
        RecordingContext hardware = new RecordingContext();
        hardware.encoder.positionRotations = Double.NaN;
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        TestMechanism mechanism = new TestMechanism(
                control,
                control.characterization().exportTo(log).dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);

        assertThrows(IllegalArgumentException.class, () -> scheduler.teleopPeriodic(0.0, 0.02));
        assertEquals(0, log.samples.size());
    }

    @Test
    void oneRoutineLoggerRemainsValidAcrossSequentialDirections() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization().exportTo(log);
        MultiTestMechanism mechanism = new MultiTestMechanism(control, sysId);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.forward);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.cancel(mechanism.forward);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.request(mechanism.reverse);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertEquals(2, log.samples.size());
        assertEquals(SysIdState.DYNAMIC_FORWARD, log.samples.get(0).state());
        assertEquals(SysIdState.DYNAMIC_REVERSE, log.samples.get(1).state());
        assertEquals(1, log.endCalls);
    }

    @Test
    void travelRangeEndsTheTestOneCycleAfterThePositionLeavesIt() {
        RecordingContext hardware = new RecordingContext();
        hardware.encoder.positionRotations = 0.5;
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization()
                .maxVoltage(Volts.of(4.0))
                .range(Degrees.of(-90.0), Degrees.of(90.0))
                .exportTo(log);
        TestMechanism mechanism = new TestMechanism(control, sysId.dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(4.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.0, hardware.motor(MOTOR).voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
    }

    private static Velocity<VoltageUnit> voltsPerSecond(double rate) {
        return Units.Volts.per(Units.Seconds).of(rate);
    }

    private static void assertTestOutput(SysIdState state, double expectedVoltage, double elapsedSeconds) {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        Characterization sysId = control.characterization()
                .rampRate(voltsPerSecond(0.5))
                .maxVoltage(Volts.of(5.5))
                .exportTo(log);
        Action action = switch (state) {
            case QUASISTATIC_FORWARD -> sysId.quasistaticForward();
            case QUASISTATIC_REVERSE -> sysId.quasistaticReverse();
            case DYNAMIC_FORWARD -> sysId.dynamicForward();
            case DYNAMIC_REVERSE -> sysId.dynamicReverse();
            case NONE -> throw new IllegalArgumentException("NONE is not a SysId test phase.");
        };
        TestMechanism mechanism = new TestMechanism(control, action);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(action);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(elapsedSeconds, 0.02);

        assertEquals(expectedVoltage, hardware.motor(MOTOR).voltage, 1.0e-9);
        SysIdSample sample = log.samples.get(log.samples.size() - 1);
        assertEquals(state, sample.state());
        assertEquals(expectedVoltage, sample.appliedVoltage(), 1.0e-9);
    }

    private static final class SysIdMechanism implements Mechanism {
        private final ControlBinding<?> control = Controls.position(MOTOR).feedback(ENCODER);
        private final Characterization sysId;
        private final Action quasistaticForward;
        private final HookBinding binding;

        private SysIdMechanism(boolean[] held, SysIdLog log) {
            sysId = control.characterization().rampRate(voltsPerSecond(2.0)).exportTo(log);
            quasistaticForward = sysId.quasistaticForward();
            binding = Events.when(() -> held[0]).active().whileActive(quasistaticForward);
        }
    }

    private static final class TestMechanism implements Mechanism {
        private final ControlBinding<?> control;
        private final Action test;

        private TestMechanism(ControlBinding<?> control, Action test) {
            this.control = control;
            this.test = test;
        }
    }

    private static final class HeldMechanism implements Mechanism {
        private final ControlBinding<?> control;
        private final HookBinding binding;

        private HeldMechanism(ControlBinding<?> control, boolean[] held, Action action) {
            this.control = control;
            binding = Events.when(() -> held[0]).active().whileActive(action);
        }
    }

    private static final class MultiTestMechanism implements Mechanism {
        private final ControlBinding<?> control;
        private final Action forward;
        private final Action reverse;

        private MultiTestMechanism(ControlBinding<?> control, Characterization sysId) {
            this.control = control;
            forward = sysId.dynamicForward();
            reverse = sysId.dynamicReverse();
        }
    }

    private static final class RecordingLog implements SysIdLog {
        private final List<SysIdSample> samples = new ArrayList<>();
        private int endCalls;

        @Override
        public void record(SysIdSample sample) {
            samples.add(sample);
        }

        @Override
        public void end() {
            endCalls++;
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new IdentityHashMap<>();
        private final RecordingEncoder encoder = new RecordingEncoder();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }

        @Override
        public EncoderHandle encoder(EncoderDevice encoder) {
            return this.encoder;
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double voltage;
        private boolean telemetryUnavailable;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setVoltage(double volts) {
            voltage = volts;
        }

        @Override
        public double appliedVoltage() {
            if (telemetryUnavailable) {
                throw new UnsupportedOperationException("No applied-voltage telemetry");
            }
            return voltage;
        }

        @Override
        public double statorCurrentAmps() {
            if (telemetryUnavailable) {
                throw new UnsupportedOperationException("No current telemetry");
            }
            return 4.0;
        }

        @Override
        public void stop() {
            voltage = 0.0;
        }
    }

    private static final class RecordingEncoder implements EncoderHandle {
        private double positionRotations;
        private double velocityRotationsPerSecond;

        @Override
        public EncoderDevice device() {
            return ENCODER;
        }

        @Override
        public double positionRotations() {
            return positionRotations;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocityRotationsPerSecond;
        }
    }
}
