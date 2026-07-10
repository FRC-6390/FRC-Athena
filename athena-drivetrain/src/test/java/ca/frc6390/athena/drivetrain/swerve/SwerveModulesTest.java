package ca.frc6390.athena.drivetrain.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

class SwerveModulesTest {
    private static final double THREE_INCH = 0.0762;
    private static final double FOUR_INCH = 0.1016;

    @Test
    void oldSdsAndRevPresetsAreAvailableWithExpectedModels() {
        List<ExpectedPreset> presets = List.of(
                preset("SDS", "MK3_STANDARD", SwerveModules.SDS.MK3.STANDARD::new, 8.16, 12.8, FOUR_INCH),
                preset("SDS", "MK3_FAST", SwerveModules.SDS.MK3.FAST::new, 6.86, 12.8, FOUR_INCH),
                preset("SDS", "MK4_L1", SwerveModules.SDS.MK4.L1::new, 8.14, 12.8, FOUR_INCH),
                preset("SDS", "MK4_L2", SwerveModules.SDS.MK4.L2::new, 6.75, 12.8, FOUR_INCH),
                preset("SDS", "MK4_L3", SwerveModules.SDS.MK4.L3::new, 6.12, 12.8, FOUR_INCH),
                preset("SDS", "MK4_L4", SwerveModules.SDS.MK4.L4::new, 5.14, 12.8, FOUR_INCH),
                preset("SDS", "MK4I_L1", SwerveModules.SDS.MK4I.L1::new, 8.14, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4I_L2", SwerveModules.SDS.MK4I.L2::new, 6.75, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4I_L3", SwerveModules.SDS.MK4I.L3::new, 6.12, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4I_L1_PLUS", SwerveModules.SDS.MK4I.L1_PLUS::new, 7.13, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4I_L2_PLUS", SwerveModules.SDS.MK4I.L2_PLUS::new, 5.9, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4I_L3_PLUS", SwerveModules.SDS.MK4I.L3_PLUS::new, 5.36, 150.0 / 7.0, FOUR_INCH),
                preset("SDS", "MK4N_L1_PLUS", SwerveModules.SDS.MK4N.L1_PLUS::new, 7.13, 18.75, FOUR_INCH),
                preset("SDS", "MK4N_L2_PLUS", SwerveModules.SDS.MK4N.L2_PLUS::new, 5.9, 18.75, FOUR_INCH),
                preset("SDS", "MK4N_L3_PLUS", SwerveModules.SDS.MK4N.L3_PLUS::new, 5.36, 18.75, FOUR_INCH),
                preset("SDS", "MK4C_L1_PLUS", SwerveModules.SDS.MK4C.L1_PLUS::new, 7.13, 12.8, FOUR_INCH),
                preset("SDS", "MK4C_L2_PLUS", SwerveModules.SDS.MK4C.L2_PLUS::new, 5.9, 12.8, FOUR_INCH),
                preset("SDS", "MK4C_L3_PLUS", SwerveModules.SDS.MK4C.L3_PLUS::new, 5.36, 12.8, FOUR_INCH),
                preset("SDS", "MK5N_R1", SwerveModules.SDS.MK5N.R1::new, 7.03, 287.0 / 11.0, FOUR_INCH),
                preset("SDS", "MK5N_R2", SwerveModules.SDS.MK5N.R2::new, 6.03, 287.0 / 11.0, FOUR_INCH),
                preset("SDS", "MK5N_R3", SwerveModules.SDS.MK5N.R3::new, 5.27, 287.0 / 11.0, FOUR_INCH),
                preset("SDS", "MK5I_R1", SwerveModules.SDS.MK5I.R1::new, 7.03, 26.0, FOUR_INCH),
                preset("SDS", "MK5I_R2", SwerveModules.SDS.MK5I.R2::new, 6.03, 26.0, FOUR_INCH),
                preset("SDS", "MK5I_R3", SwerveModules.SDS.MK5I.R3::new, 5.27, 26.0, FOUR_INCH),
                preset("REV", "MAXSWERVE_3IN_LOW", SwerveModules.REV.MAXSWERVE_3IN.LOW::new, 5.50, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_MEDIUM", SwerveModules.REV.MAXSWERVE_3IN.MEDIUM::new, 5.08, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_HIGH", SwerveModules.REV.MAXSWERVE_3IN.HIGH::new, 4.71, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_EXTRA_HIGH_1", SwerveModules.REV.MAXSWERVE_3IN.EXTRA_HIGH_1::new, 4.50, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_EXTRA_HIGH_2", SwerveModules.REV.MAXSWERVE_3IN.EXTRA_HIGH_2::new, 4.29, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_EXTRA_HIGH_3", SwerveModules.REV.MAXSWERVE_3IN.EXTRA_HIGH_3::new, 4.00, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_EXTRA_HIGH_4", SwerveModules.REV.MAXSWERVE_3IN.EXTRA_HIGH_4::new, 3.75, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "MAXSWERVE_3IN_EXTRA_HIGH_5", SwerveModules.REV.MAXSWERVE_3IN.EXTRA_HIGH_5::new, 3.56, 9424.0 / 203.0, THREE_INCH),
                preset("REV", "EASYSWERVE_4IN_STANDARD", SwerveModules.REV.EASYSWERVE_4IN.STANDARD::new, 6.3, 20.0, FOUR_INCH));

        assertEquals(33, presets.size());
        for (ExpectedPreset preset : presets) {
            SwerveModuleModel model = preset.factory().get().model();
            assertEquals(preset.vendor(), model.vendor());
            assertEquals(preset.name(), model.name());
            assertEquals(preset.driveReduction(), model.driveReduction(), 1.0e-9);
            assertEquals(preset.steerReduction(), model.steerReduction(), 1.0e-9);
            assertEquals(preset.wheelDiameterMeters(), model.wheelDiameterMeters(), 1.0e-9);
        }
    }

    @Test
    void presetsAreMechanismTemplateModulesFilledThroughGenericSlots() {
        HardwareBus canivore = HardwareBus.can("canivore");
        SwerveModule module = new SwerveModules.SDS.MK5N.R3();

        assertThrows(IllegalStateException.class, () -> module.target(new SwerveModuleTarget(1.0, 0.25)));

        module.drive.fill(canivore.motor(MotorKinds.KRAKEN_X60, 1))
                .steer.fill(canivore.motor(MotorKinds.KRAKEN_X44, 2))
                .angle.fill(canivore.encoder(EncoderKinds.CANCODER, 3));

        assertNotNull(module.driveVelocity);
        assertNotNull(module.steerPosition);
        assertThrows(IllegalStateException.class, () -> module.target(new SwerveModuleTarget(1.0, 0.25)));

        module.driveMaxSpeedMetersPerSecond(4.0).steerPid(1.0, 0.0, 0.0);

        assertDoesNotThrow(() -> module.target(new SwerveModuleTarget(1.0, 0.25)));
    }

    @Test
    void rectangularKinematicsOwnsControlAndUnifiedSimulationModel() {
        SwerveModule frontLeft = module(1, 2, 11);
        SwerveModule frontRight = module(3, 4, 12);
        SwerveModule backLeft = module(5, 6, 13);
        SwerveModule backRight = module(7, 8, 14);

        SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.6, 0.5, 4.5, frontLeft, frontRight, backLeft, backRight);
        SimModel model = kinematics.simulationModel();

        assertEquals(4, kinematics.modules().size());
        assertEquals(0.3, kinematics.modules().get(0).xMeters(), 1.0e-9);
        assertEquals(0.25, kinematics.modules().get(0).yMeters(), 1.0e-9);
        assertEquals(8, model.physicsLeaves().size());
        assertEquals(8, model.motors().size());
        assertEquals(8, model.encoders().size());
    }

    @Test
    void productionKinematicsRoundTripsArbitraryChassisVelocity() {
        SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.6, 0.5, 4.5, module(21, 22, 31), module(23, 24, 32), module(25, 26, 33), module(27, 28, 34));
        RobotVelocity requested = new RobotVelocity(1.2, -0.7, 0.9);

        RobotVelocity measured = kinematics.velocity(kinematics.targets(requested));

        assertEquals(requested.xMetersPerSecond(), measured.xMetersPerSecond(), 1.0e-9);
        assertEquals(requested.yMetersPerSecond(), measured.yMetersPerSecond(), 1.0e-9);
        assertEquals(requested.angularRadiansPerSecond(), measured.angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void reverseTranslationFlipsDriveSpeedInsteadOfRotatingModulesHalfATurn() {
        SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.6, 0.5, 4.5, module(41, 42, 51), module(43, 44, 52), module(45, 46, 53), module(47, 48, 54));

        List<SwerveModuleTarget> targets = kinematics.targets(new RobotVelocity(-2.0, 0.0, 0.0));

        for (SwerveModuleTarget target : targets) {
            assertEquals(-2.0, target.speedMetersPerSecond(), 1.0e-9);
            assertEquals(0.0, target.angleRotations(), 1.0e-9);
        }
        RobotVelocity reconstructed = kinematics.velocity(targets);
        assertEquals(-2.0, reconstructed.xMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, reconstructed.yMetersPerSecond(), 1.0e-9);
    }

    private static ExpectedPreset preset(
            String vendor,
            String name,
            Supplier<SwerveModule> factory,
            double driveReduction,
            double steerReduction,
            double wheelDiameterMeters) {
        return new ExpectedPreset(vendor, name, factory, driveReduction, steerReduction, wheelDiameterMeters);
    }

    private static SwerveModule module(int driveMotorId, int steerMotorId, int encoderId) {
        HardwareBus canivore = HardwareBus.can("canivore");
        return new SwerveModules.SDS.MK5N.R3()
                .drive.fill(canivore.motor(MotorKinds.KRAKEN_X60, driveMotorId))
                .steer.fill(canivore.motor(MotorKinds.KRAKEN_X44, steerMotorId))
                .angle.fill(canivore.encoder(EncoderKinds.CANCODER, encoderId));
    }

    private record ExpectedPreset(
            String vendor,
            String name,
            Supplier<SwerveModule> factory,
            double driveReduction,
            double steerReduction,
            double wheelDiameterMeters) {
    }
}

