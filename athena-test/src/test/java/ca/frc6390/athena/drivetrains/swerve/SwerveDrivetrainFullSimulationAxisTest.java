package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.control.SwerveDriveCommand;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainFullSimulationAxisTest {

    private static final double DRIVER_COMMAND_PERIOD_SECONDS = 0.02;
    private static final double DRIVETRAIN_UPDATE_PERIOD_SECONDS = 0.005;

    @AfterEach
    void resetRobotTimeCache() {
        ca.frc6390.athena.core.RobotTime.resetNowSecondsForTest();
    }

    @Test
    void fullOmegaWithPureXFieldCommandStaysOnFieldXAxis() throws Exception {
        SwerveDrivetrain drivetrain = createDrivetrain();
        drivetrain.updatePeriodSeconds(DRIVETRAIN_UPDATE_PERIOD_SECONDS);
        drivetrain.fieldRelative(true);
        drivetrain.configureSimulation(
                SwerveSimulationConfig.defaults()
                        .withNominalVoltage(12.0)
                        .withWheelCoefficientOfFriction(1.2)
                        .withMaxSpeedScale(1.0));

        DoubleSupplier x = () -> 1.0;
        DoubleSupplier y = () -> 0.0;
        DoubleSupplier theta = () -> 1.0;
        SwerveDriveCommand command = new SwerveDriveCommand(drivetrain, x, y, theta, true);

        command.initialize();
        setRobotTimeSeconds(0.0);
        drivetrain.update();
        drivetrain.simulationPeriodic();

        int steps = 1000; // 5 seconds at 5ms drivetrain loop.
        int driverStepStride = Math.max(1, (int) Math.round(
                DRIVER_COMMAND_PERIOD_SECONDS / DRIVETRAIN_UPDATE_PERIOD_SECONDS));
        for (int i = 1; i <= steps; i++) {
            setRobotTimeSeconds(i * DRIVETRAIN_UPDATE_PERIOD_SECONDS);
            if ((i % driverStepStride) == 0) {
                command.execute();
            }
            drivetrain.update();
            drivetrain.simulationPeriodic();
        }
        command.end(false);

        Pose2d finalPose = drivetrain.simulation().pose();
        double traveledX = finalPose.getX();
        double driftY = Math.abs(finalPose.getY());
        double leadSeconds = drivetrain.fieldRelativeLeadSecondsForTest();

        // Under full-omega + full-translation command, path should remain mostly on commanded
        // field X-axis. Significant Y here indicates axis skew/coupling.
        assertTrue(
                traveledX > 1.0,
                "expected forward travel along X in simulation, got X=" + traveledX
                        + " Y=" + finalPose.getY()
                        + " headingDeg=" + finalPose.getRotation().getDegrees()
                        + " leadSec=" + leadSeconds);
        assertTrue(
                driftY < 0.75,
                "expected bounded Y drift under pure X field command, got " + driftY + " m at X=" + traveledX
                        + " headingDeg=" + finalPose.getRotation().getDegrees()
                        + " leadSec=" + leadSeconds);
    }

    @Test
    void fullOmegaWithPureXFieldAutoSourceStaysOnFieldXAxis() throws Exception {
        SwerveDrivetrain drivetrain = createDrivetrain();
        drivetrain.updatePeriodSeconds(DRIVETRAIN_UPDATE_PERIOD_SECONDS);
        drivetrain.fieldRelative(false);
        drivetrain.configureSimulation(
                SwerveSimulationConfig.defaults()
                        .withNominalVoltage(12.0)
                        .withWheelCoefficientOfFriction(1.2)
                        .withMaxSpeedScale(1.0));

        setRobotTimeSeconds(0.0);
        drivetrain.speeds().stop(RobotSpeeds.DRIVE_SOURCE);
        drivetrain.update();
        drivetrain.simulationPeriodic();

        int steps = 1000; // 5 seconds at 5ms drivetrain loop.
        int autoStepStride = Math.max(1, (int) Math.round(
                DRIVER_COMMAND_PERIOD_SECONDS / DRIVETRAIN_UPDATE_PERIOD_SECONDS));
        for (int i = 1; i <= steps; i++) {
            setRobotTimeSeconds(i * DRIVETRAIN_UPDATE_PERIOD_SECONDS);
            if ((i % autoStepStride) == 0) {
                drivetrain.speeds().setFieldRelative(
                        RobotSpeeds.AUTO_SOURCE,
                        drivetrain.speeds().maxVelocity(),
                        0.0,
                        drivetrain.speeds().maxAngularVelocity());
            }
            drivetrain.update();
            drivetrain.simulationPeriodic();
        }
        drivetrain.speeds().stop(RobotSpeeds.AUTO_SOURCE);

        Pose2d finalPose = drivetrain.simulation().pose();
        double traveledX = finalPose.getX();
        double driftY = Math.abs(finalPose.getY());
        double leadSeconds = drivetrain.fieldRelativeLeadSecondsForTest();

        assertTrue(
                traveledX > 1.0,
                "expected forward travel along X in simulation (auto source), got X=" + traveledX
                        + " Y=" + finalPose.getY()
                        + " headingDeg=" + finalPose.getRotation().getDegrees()
                        + " leadSec=" + leadSeconds);
        assertTrue(
                driftY < 0.8,
                "expected bounded Y drift under pure X auto-source command, got " + driftY + " m at X=" + traveledX
                        + " headingDeg=" + finalPose.getRotation().getDegrees()
                        + " leadSec=" + leadSeconds);
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
        // SDS MK4i L3 defaults used in many Athena examples.
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
        private Rotation2d yaw = Rotation2d.kZero;
        private Rotation2d velZ = Rotation2d.kZero;

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
            return yaw;
        }

        @Override
        public Rotation2d getVelocityZ() {
            return velZ;
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {
            this.yaw = yaw != null ? yaw : Rotation2d.kZero;
            this.velZ = angularVelocityZ != null ? angularVelocityZ : Rotation2d.kZero;
        }

        @Override
        public ImuConfig getConfig() {
            return ImuConfig.create(TYPE, 0);
        }
    }
}
