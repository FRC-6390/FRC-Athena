package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.imu.VirtualImu;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.testsupport.hardware.TestSimHardwareFactory.TestEncoderType;
import ca.frc6390.athena.testsupport.hardware.TestSimHardwareFactory.TestMotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainLoopTimingKnobsTest {

    @AfterEach
    void resetRobotTimeCache() {
        ca.frc6390.athena.core.RobotTime.resetNowSecondsForTest();
    }

    @Test
    void defaultsAreFiveMillisecondsForAllThreeLoops() {
        SwerveDrivetrain drivetrain = createDrivetrain();

        assertEquals(0.005, drivetrain.updatePeriodSeconds(), 1e-9);
        assertEquals(0.005, drivetrain.driverCommandPeriodSeconds(), 1e-9);
        assertEquals(0.005, drivetrain.autoFollowerPeriodSeconds(), 1e-9);
    }

    @Test
    void driverCommandLoopCanRunSlowerThanDrivetrainLoop() throws Exception {
        SwerveDrivetrain drivetrain = createDrivetrain();
        drivetrain.updatePeriodSeconds(0.005);
        drivetrain.driverCommandPeriodSeconds(0.02);
        drivetrain.bindDriveCommandInputs(() -> 1.0, () -> 0.0, () -> 0.0, () -> false);

        setRobotTimeSeconds(0.000);
        drivetrain.update();
        double t0 = drivetrain.robotSpeeds().getSourceLastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE);

        setRobotTimeSeconds(0.005);
        drivetrain.update();
        double t1 = drivetrain.robotSpeeds().getSourceLastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE);

        setRobotTimeSeconds(0.020);
        drivetrain.update();
        double t2 = drivetrain.robotSpeeds().getSourceLastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE);

        assertEquals(t0, t1, 1e-9);
        assertTrue(t2 > t1, "expected drive source timestamp to update on the 20ms command period");
    }

    @Test
    void autoFollowerLoopPeriodKnobIsConfigurable() {
        SwerveDrivetrain drivetrain = createDrivetrain();
        drivetrain.updatePeriodSeconds(0.005);
        drivetrain.autoFollowerPeriodSeconds(0.01);
        assertEquals(0.01, drivetrain.autoFollowerPeriodSeconds(), 1e-9);
        assertEquals(10.0, drivetrain.autoFollowerPeriodMs(), 1e-9);

        drivetrain.autoFollowerPeriodMs(7.5);
        assertEquals(0.0075, drivetrain.autoFollowerPeriodSeconds(), 1e-9);
        assertEquals(7.5, drivetrain.autoFollowerPeriodMs(), 1e-9);
    }

    private static SwerveDrivetrain createDrivetrain() {
        double trackWidthMeters = 0.57;
        double wheelbaseMeters = 0.57;
        Translation2d[] locations = SwerveModule.SwerveModuleConfig.generateModuleLocations(
                trackWidthMeters,
                wheelbaseMeters);

        SwerveModule.SwerveModuleConfig fl = createModuleConfig(locations[0], 1, 5, 9);
        SwerveModule.SwerveModuleConfig fr = createModuleConfig(locations[1], 2, 6, 10);
        SwerveModule.SwerveModuleConfig bl = createModuleConfig(locations[2], 3, 7, 11);
        SwerveModule.SwerveModuleConfig br = createModuleConfig(locations[3], 4, 8, 12);

        VirtualImu imu = new VirtualImu(new TestImu());
        return new SwerveDrivetrain(imu, fl, fr, bl, br);
    }

    private static SwerveModule.SwerveModuleConfig createModuleConfig(
            Translation2d location,
            int driveId,
            int steerId,
            int encoderId) {
        double wheelDiameterMeters = Units.inchesToMeters(4.0);
        double driveRatio = 1.0 / 6.12;
        double steerRatio = 150.0 / 7.0;
        double neoFreeSpeedRpm = 5820.0;
        double maxSpeedMetersPerSecond = (neoFreeSpeedRpm / 60.0) * driveRatio * Math.PI * wheelDiameterMeters;

        MotorControllerConfig driveMotor = MotorControllerConfig.create(TestMotorType.INSTANCE, driveId)
                .encoder(e -> e.config(EncoderConfig.create(TestEncoderType.INSTANCE, driveId)
                        .measurement(m -> m
                                .gearRatio(driveRatio)
                                .conversion(Math.PI * wheelDiameterMeters))));

        MotorControllerConfig steerMotor = MotorControllerConfig.create(TestMotorType.INSTANCE, steerId)
                .encoder(e -> e.config(EncoderConfig.create(TestEncoderType.INSTANCE, steerId)
                        .measurement(m -> m.gearRatio(steerRatio))));

        EncoderConfig moduleEncoder = EncoderConfig.create(TestEncoderType.INSTANCE, encoderId)
                .measurement(m -> m.gearRatio(1.0));

        return new SwerveModule.SwerveModuleConfig(
                location,
                wheelDiameterMeters,
                maxSpeedMetersPerSecond,
                driveMotor,
                steerMotor,
                new PIDController(40.0, 0.0, 0.0),
                moduleEncoder,
                SwerveModule.SwerveModuleSimConfig.fromMotors(AthenaMotor.NEO_V1, AthenaMotor.NEO_V1));
    }

    private static void setRobotTimeSeconds(double nowSeconds) throws Exception {
        Method updateNowSeconds = ca.frc6390.athena.core.RobotTime.class
                .getDeclaredMethod("updateNowSeconds", double.class);
        updateNowSeconds.setAccessible(true);
        updateNowSeconds.invoke(null, nowSeconds);
    }

    private static final class TestImu implements Imu {
        private static final ImuType TYPE = () -> "test";

        @Override
        public Rotation2d getRoll() {
            return Rotation2d.kZero;
        }

        @Override
        public Rotation2d getPitch() {
            return Rotation2d.kZero;
        }

        @Override
        public Rotation2d getYaw() {
            return Rotation2d.kZero;
        }

        @Override
        public Rotation2d getVelocityZ() {
            return Rotation2d.kZero;
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {
        }

        @Override
        public ImuConfig getConfig() {
            return ImuConfig.create(TYPE, 0);
        }
    }
}
