package ca.frc6390.athena.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.mechanism.config.Mechanisms;
import ca.frc6390.athena.mechanism.runtime.MechanismController;
import ca.frc6390.athena.mechanism.spec.ControlMode;
import ca.frc6390.athena.mechanism.spec.MechanismSpec;

class MechanismConfigTest {
    @Test
    void lowersPublicConfigToImmutableSpec() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor
                        .hardware(AthenaMotor.SIM, 1)
                        .brake()
                        .currentLimit(35))
                .control(control -> control.percentOutput())
                .toSpec();

        assertEquals("intake", spec.name());
        assertEquals(ControlMode.PERCENT_OUTPUT, spec.controlMode());
        assertEquals(1, spec.motors().size());
        assertEquals("intake.roller", spec.motors().get(0).path());
        assertEquals(AthenaMotor.SIM, spec.motors().get(0).kind());
    }

    @Test
    void lowersTypedInputs() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .input("beamBreak", input -> input.digital(0))
                .input("targetRpm", input -> input.runtimeNumber("dashboard/shooterTarget"))
                .toSpec();

        assertEquals(2, spec.inputs().size());
        assertEquals("intake.beamBreak", spec.inputs().get(0).path());
        assertEquals(InputSourceKind.DIGITAL_CHANNEL, spec.inputs().get(0).sourceKind());
    }

    @Test
    void lowersEncodersAndSources() {
        MechanismSpec spec = Mechanisms.simple("arm")
                .motor("pivot", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .encoder("absolute", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 2)
                        .absolutePosition()
                        .gearRatio(2.0)
                        .offset(0.25))
                .positionSource("absolute")
                .toSpec();

        assertEquals(1, spec.encoders().size());
        assertEquals("arm.absolute", spec.encoders().get(0).path());
        assertEquals("absolute", spec.positionSource());
    }

    @Test
    void lowersControlGainsAndStates() {
        MechanismSpec spec = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control
                        .velocity(pid -> pid.p(0.14).i(0.0).d(0.001))
                        .feedforward(ff -> ff.staticGain(0.2).velocity(0.12)))
                .state("idle", state -> state.target(0.0))
                .state("speaker", state -> state.target(4600.0))
                .toSpec();

        assertEquals(ControlMode.VELOCITY, spec.controlMode());
        assertEquals(0.14, spec.control().pidSpec().orElseThrow().p());
        assertEquals(2, spec.states().size());
        assertEquals(4600.0, spec.states().get(1).targetValue().orElseThrow());
    }

    @Test
    void reportsMissingBackend() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SPARK_MAX_BRUSHLESS, 3))
                .control(control -> control.percentOutput())
                .toSpec();

        var report = spec.validate(AthenaValidationContext.withBackends(BackendRegistry.of()));

        assertTrue(report.hasErrors());
        assertEquals("hardware.missing-backend", report.errors().get(0).code());
    }

    @Test
    void reportsMissingCapability() {
        MechanismSpec spec = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .control(control -> control.velocity())
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertEquals("hardware.missing-capability", report.errors().get(0).code());
    }

    @Test
    void validatesWhenBackendProvidesRequiredCapability() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertFalse(report.hasErrors());
    }

    @Test
    void reportsInvalidInputChannel() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .input("badSwitch", input -> input.digital(-1))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertEquals("input.invalid-channel", report.errors().get(0).code());
    }

    @Test
    void reportsUnknownPositionSource() {
        MechanismSpec spec = Mechanisms.simple("arm")
                .motor("pivot", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .positionSource("missing")
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertEquals("mechanism.unknown-position-source", report.errors().get(0).code());
    }

    @Test
    void reportsInvalidEncoderGearRatio() {
        MechanismSpec spec = Mechanisms.simple("arm")
                .motor("pivot", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .encoder("absolute", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 2)
                        .gearRatio(0.0))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertEquals("encoder.invalid-gear-ratio", report.errors().get(0).code());
    }

    @Test
    void reportsInvalidPidGain() {
        MechanismSpec spec = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.velocity(pid -> pid.p(Double.NaN)))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertTrue(report.errors().stream().anyMatch(error -> error.code().equals("control.invalid-pid")));
    }

    @Test
    void reportsDuplicateStates() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("idle", state -> state.target(0.0))
                .state("idle", state -> state.target(1.0))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new PercentOnlyBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertTrue(report.errors().stream().anyMatch(error -> error.code().equals("mechanism.duplicate-state")));
    }

    @Test
    void mechanismControllerAppliesPercentOutputStates() {
        MechanismSpec spec = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .state("collect", state -> state.target(0.65))
                .toSpec();
        var motor = new RecordingMotor(spec.motors().get(0));

        MechanismController.of(spec, java.util.List.of(motor)).applyState("collect");

        assertEquals(0.65, motor.percent, 1.0e-9);
    }

    @Test
    void mechanismControllerAppliesPositionAndVelocityTargets() {
        MechanismSpec arm = Mechanisms.simple("arm")
                .motor("pivot", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.position())
                .state("amp", state -> state.target(0.375))
                .toSpec();
        MechanismSpec shooter = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .control(control -> control.velocity())
                .state("speaker", state -> state.target(82.0))
                .toSpec();
        var armMotor = new RecordingMotor(arm.motors().get(0));
        var shooterMotor = new RecordingMotor(shooter.motors().get(0));

        MechanismController.of(arm, java.util.List.of(armMotor)).applyState("amp");
        MechanismController.of(shooter, java.util.List.of(shooterMotor)).applyState("speaker");

        assertEquals(0.375, armMotor.position, 1.0e-9);
        assertEquals(82.0, shooterMotor.velocity, 1.0e-9);
    }

    private static final class PercentOnlyBackend implements MotorBackend {
        @Override
        public boolean supports(ca.frc6390.athena.api.hardware.MotorKind kind) {
            return kind == AthenaMotor.SIM;
        }

        @Override
        public CapabilitySet capabilities(ca.frc6390.athena.api.hardware.MotorKind kind) {
            return CapabilitySet.of(MotorCapability.PERCENT_OUTPUT);
        }

        @Override
        public MotorDevice create(MotorSpec spec) {
            return () -> spec;
        }
    }

    private static final class RecordingMotor implements MotorDevice {
        private final MotorSpec spec;
        private double percent;
        private double position;
        private double velocity;

        private RecordingMotor(MotorSpec spec) {
            this.spec = spec;
        }

        @Override
        public MotorSpec spec() {
            return spec;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            position = rotations;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }
    }
}
