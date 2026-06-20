package ca.frc6390.athena.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.drivetrain.config.Drivetrains;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.sim.hardware.SimMotorBackend;

class SwerveDrivetrainConfigTest {
    @Test
    void lowersSwerveModules() {
        var spec = Drivetrains.swerve("drive")
                .trackWidth(TrackWidth.meters(0.58))
                .wheelBase(WheelBase.meters(0.62))
                .driveInverted(true)
                .module("frontLeft", module -> module
                        .location(0.31, 0.29)
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 11).absolutePosition())
                        .steerPid(4.5, 0.0, 0.12)
                        .driveFeedforward(0.18, 2.1, 0.3))
                .toSpec();

        assertEquals("drive", spec.name());
        assertEquals(0.58, spec.trackWidth().meters());
        assertEquals(0.62, spec.wheelBase().meters());
        assertEquals(1, spec.modules().size());
        assertEquals("frontLeft", spec.modules().get(0).name());
        assertTrue(spec.modules().get(0).driveInverted());
        assertFalse(spec.modules().get(0).steerInverted());
        assertEquals(4.5, spec.modules().get(0).control().steerP());
        assertEquals(2.1, spec.modules().get(0).control().driveKv());
    }

    @Test
    void moduleInversionOverridesDrivetrainDefault() {
        var module = Drivetrains.swerve("drive")
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(true)
                .module("frontRight", config -> config
                        .driveInverted(false)
                        .steerInverted(false)
                        .encoderInverted(false)
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 12)))
                .toSpec()
                .modules()
                .get(0);

        assertFalse(module.driveInverted());
        assertFalse(module.steerInverted());
        assertFalse(module.encoderInverted());
    }

    @Test
    void validatesWithSimulationBackend() {
        var spec = Drivetrains.swerve("drive")
                .module("frontLeft", module -> module
                        .location(0.3, 0.3)
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 11)))
                .toSpec();
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));

        assertFalse(spec.validate(context).hasErrors());
    }

    @Test
    void reportsMissingModuleParts() {
        var spec = Drivetrains.swerve("drive")
                .module("frontLeft", module -> module.driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1)))
                .toSpec();
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = spec.validate(context);

        assertTrue(report.hasErrors());
        assertFalse(report.errorsWithCode("swerve.missing-motor").isEmpty());
        assertFalse(report.errorsWithCode("swerve.missing-encoder").isEmpty());
    }

    @Test
    void reportsDuplicateModulesAndInvalidGains() {
        var spec = Drivetrains.swerve("drive")
                .module("frontLeft", module -> module
                        .location(Double.NaN, 0.0)
                        .steerPid(Double.POSITIVE_INFINITY, 0.0, 0.0)
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 11)))
                .module("frontLeft", module -> module
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 3))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 4))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 12)))
                .toSpec();
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = spec.validate(context);

        assertFalse(report.errorsWithCode("swerve.duplicate-module").isEmpty());
        assertFalse(report.errorsWithCode("swerve.invalid-location").isEmpty());
        assertFalse(report.errorsWithCode("swerve.invalid-gains").isEmpty());
    }
}
