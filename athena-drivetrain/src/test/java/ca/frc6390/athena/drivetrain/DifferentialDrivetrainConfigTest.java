package ca.frc6390.athena.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.drivetrain.config.Drivetrains;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.sim.hardware.SimMotorBackend;

class DifferentialDrivetrainConfigTest {
    @Test
    void lowersDifferentialDrivetrain() {
        var spec = Drivetrains.differential("drive")
                .leftMotor("leftLeader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .rightMotor("rightLeader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .trackWidth(TrackWidth.meters(0.71))
                .toSpec();

        assertEquals("drive", spec.name());
        assertEquals(1, spec.leftMotors().size());
        assertEquals(0.71, spec.trackWidth().meters());
    }

    @Test
    void validatesWithSimulationBackend() {
        var spec = Drivetrains.differential("drive")
                .leftMotor("leftLeader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .rightMotor("rightLeader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));

        assertFalse(spec.validate(context).hasErrors());
    }

    @Test
    void reportsMissingSide() {
        var spec = Drivetrains.differential("drive")
                .leftMotor("leftLeader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));

        assertTrue(spec.validate(context).hasErrors());
        assertEquals("drivetrain.no-right-motors", spec.validate(context).errors().get(0).code());
    }
}
