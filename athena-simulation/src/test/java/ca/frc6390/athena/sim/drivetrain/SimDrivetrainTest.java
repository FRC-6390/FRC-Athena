package ca.frc6390.athena.sim.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.drivetrain.config.Drivetrains;
import ca.frc6390.athena.sim.world.SimWorld;

class SimDrivetrainTest {
    @Test
    void differentialDriveAppliesSideVelocities() {
        SimWorld world = new SimWorld();
        var spec = Drivetrains.differential("drive")
                .leftMotor("leftLeader", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .leftMotor("leftFollower", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .rightMotor("rightLeader", motor -> motor.hardware(AthenaMotor.SIM, 3))
                .toSpec();

        var drive = new SimDifferentialDrive(world, spec).tankVelocity(2.0, -1.0);
        world.step(0.5);

        assertEquals(1.0, drive.motor("leftLeader").position(), 1.0e-9);
        assertEquals(1.0, drive.motor("leftFollower").position(), 1.0e-9);
        assertEquals(-0.5, drive.motor("rightLeader").position(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> drive.motor("missing"));
    }

    @Test
    void swerveDriveAppliesModuleVelocityAndSteerTargets() {
        SimWorld world = new SimWorld();
        var spec = Drivetrains.swerve("swerve")
                .module("frontLeft", module -> module
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 11)))
                .module("frontRight", module -> module
                        .driveInverted(true)
                        .steerInverted(true)
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 3))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 4))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 12)))
                .toSpec();

        var drive = new SimSwerveDrive(world, spec).driveAll(3.0, 45.0);
        world.step(0.25);

        assertEquals(0.75, drive.driveMotor("frontLeft").position(), 1.0e-9);
        assertEquals(45.0, drive.steerMotor("frontLeft").position(), 1.0e-9);
        assertEquals(-0.75, drive.driveMotor("frontRight").position(), 1.0e-9);
        assertEquals(-45.0, drive.steerMotor("frontRight").position(), 1.0e-9);
    }

    @Test
    void swerveDriveRejectsUnknownModules() {
        SimWorld world = new SimWorld();
        var spec = Drivetrains.swerve("swerve")
                .module("frontLeft", module -> module
                        .driveMotor(motor -> motor.hardware(AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(AthenaEncoder.SIM, 11)))
                .toSpec();
        var drive = new SimSwerveDrive(world, spec);

        assertThrows(IllegalArgumentException.class, () -> drive.module("missing", 1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> drive.driveMotor("missing"));
    }
}
