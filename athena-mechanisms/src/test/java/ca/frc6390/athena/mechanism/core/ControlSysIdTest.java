package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;
import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ControlSysIdTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
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
        assertEquals(0.0, hardware.motor.voltage, 1.0e-9);

        scheduler.teleopPeriodic(0.5, 0.02);
        assertEquals(1.0, hardware.motor.voltage, 1.0e-9);
        SysIdSample sample = log.samples.get(log.samples.size() - 1);
        assertEquals(SysIdState.QUASISTATIC_FORWARD, sample.state());
        assertEquals(0.5, sample.position(), 1.0e-9);
        assertEquals(0.25, sample.velocity(), 1.0e-9);

        held[0] = false;
        scheduler.teleopPeriodic(0.52, 0.02);
        assertEquals(0.0, hardware.motor.voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
    }

    @Test
    void dynamicReverseUsesStepVoltageAndStopsAtTimeout() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding control = Controls.position(MOTOR).feedback(ENCODER);
        ControlSysId sysId = control.sysId()
                .stepVoltage(6.0)
                .timeout(0.5)
                .logger(log);
        TestMechanism mechanism = new TestMechanism(control, sysId.dynamicReverse());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-6.0, hardware.motor.voltage, 1.0e-9);
        assertEquals(SysIdState.DYNAMIC_REVERSE, log.samples.get(0).state());

        scheduler.teleopPeriodic(0.5, 0.02);
        assertEquals(0.0, hardware.motor.voltage, 1.0e-9);
        assertEquals(1, log.endCalls);
    }

    @Test
    void characterizationDoesNotLogWhenANewerActionWinsTheMotor() {
        RecordingContext hardware = new RecordingContext();
        RecordingLog log = new RecordingLog();
        ControlBinding control = Controls.position(MOTOR).feedback(ENCODER);
        ControlSysId sysId = control.sysId().logger(log);
        TestMechanism mechanism = new TestMechanism(control, sysId.dynamicForward());
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.test);
        scheduler.request(control.neutral());
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.0, hardware.motor.voltage, 1.0e-9);
        assertEquals(0, log.samples.size());
    }

    private static final class SysIdMechanism implements Mechanism {
        private final ControlBinding control = Controls.position(MOTOR).feedback(ENCODER);
        private final ControlSysId sysId;
        private final Action quasistaticForward;
        private final HookBinding binding;

        private SysIdMechanism(boolean[] held, SysIdLog log) {
            sysId = control.sysId().rampRate(2.0).logger(log);
            quasistaticForward = sysId.quasistaticForward();
            binding = Events.when(() -> held[0]).active().whileActive(quasistaticForward);
        }
    }

    private static final class TestMechanism implements Mechanism {
        private final ControlBinding control;
        private final Action test;

        private TestMechanism(ControlBinding control, Action test) {
            this.control = control;
            this.test = test;
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
        private final RecordingMotor motor = new RecordingMotor();
        private final RecordingEncoder encoder = new RecordingEncoder();

        @Override
        public MotorHandle motor(MotorDevice motor) {
            return this.motor;
        }

        @Override
        public EncoderHandle encoder(EncoderDevice encoder) {
            return this.encoder;
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private double voltage;

        @Override
        public MotorDevice device() {
            return MOTOR;
        }

        @Override
        public void setVoltage(double volts) {
            voltage = volts;
        }

        @Override
        public double appliedVoltage() {
            return voltage;
        }

        @Override
        public double statorCurrentAmps() {
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
