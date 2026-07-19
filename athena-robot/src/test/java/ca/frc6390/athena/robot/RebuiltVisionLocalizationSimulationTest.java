package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import org.junit.jupiter.api.Test;

class RebuiltVisionLocalizationSimulationTest {
    @Test
    void rebuiltFourCameraPipelineCorrectsLargeInitialOdometryError() {
        SimulationSession simulation = SimulationSession.create()
                .visionField(VisionSimulationField.of(
                        VisionSimulationTarget.aprilTag(1, 3.5, 4.0, 1.3, Math.PI)))
                .resetPose(new PoseSnapshot(2.0, 4.0, 0.0));
        RebuiltLocalization mechanism = new RebuiltLocalization();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        double initialError = Math.hypot(
                mechanism.estimatedFieldPose.pose().xMeters() - 2.0,
                mechanism.estimatedFieldPose.pose().yMeters() - 4.0);
        for (int cycle = 1; cycle <= 12; cycle++) {
            simulation.step(0.02);
            runtime.robotPeriodic(cycle * 0.02, 0.02);
        }
        double correctedError = Math.hypot(
                mechanism.estimatedFieldPose.pose().xMeters() - 2.0,
                mechanism.estimatedFieldPose.pose().yMeters() - 4.0);

        assertFalse(mechanism.filteredVision.acceptedMeasurements().isEmpty());
        assertFalse(mechanism.fusedVision.acceptedMeasurements().isEmpty());
        assertTrue(correctedError < initialError,
                () -> "Vision did not correct odometry: initial=" + initialError + ", corrected=" + correctedError);
    }

    @Test
    void movingRobotDrivesGroundTruthOdometryAndCameraFusionForwardTogether() {
        SimulationSession simulation = SimulationSession.create()
                .visionField(VisionSimulationField.of(
                        VisionSimulationTarget.aprilTag(1, 5.0, 4.0, 1.3, Math.PI)))
                .resetPose(new PoseSnapshot(2.0, 4.0, 0.0));
        RebuiltLocalization mechanism = new RebuiltLocalization();
        mechanism.odometry.reset(new PoseSnapshot(2.0, 4.0, 0.0));
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        runtime.request(mechanism.drive);

        for (int cycle = 1; cycle <= 50; cycle++) {
            double now = cycle * 0.02;
            runtime.teleopPeriodic(now, 0.02);
            runtime.simulationPeriodic(now, 0.02);
        }

        PoseSnapshot truth = simulation.pose();
        PoseSnapshot odometry = mechanism.odometry.pose();
        PoseSnapshot estimate = mechanism.estimatedFieldPose.pose();
        assertTrue(Math.hypot(truth.xMeters() - 2.0, truth.yMeters() - 4.0) > 0.5,
                () -> "Simulation truth barely moved: " + truth);
        assertTrue(Math.hypot(odometry.xMeters() - truth.xMeters(), odometry.yMeters() - truth.yMeters()) < 0.25,
                () -> "Odometry diverged from simulation truth: truth=" + truth + ", odometry=" + odometry);
        assertTrue(Math.hypot(estimate.xMeters() - truth.xMeters(), estimate.yMeters() - truth.yMeters()) < 0.5,
                () -> "Vision localization pinned or dragged the estimate: truth=" + truth + ", estimate=" + estimate);
        assertFalse(mechanism.filteredVision.acceptedMeasurements().isEmpty());
    }

    @Test
    void offseasonMk4iPooledDriveRotatesGroundTruthAndOdometry() {
        SimulationSession simulation = SimulationSession.create();
        OffseasonDrivetrain drivetrain = new OffseasonDrivetrain();
        OffseasonLocalization localization = new OffseasonLocalization(drivetrain);
        RobotRuntime runtime = RobotRuntime.simulated(simulation)
                .register(drivetrain)
                .register(localization);
        drivetrain.driver.set(RobotVelocity.robot(0.0, 0.0, Math.PI));
        runtime.request(drivetrain.drive);

        for (int cycle = 1; cycle <= 50; cycle++) {
            double now = cycle * 0.02;
            runtime.teleopPeriodic(now, 0.02);
            runtime.simulationPeriodic(now, 0.02);
        }

        assertTrue(Math.abs(simulation.pose().headingRadians()) > 3.0,
                () -> "Offseason simulation did not rotate: " + simulation.pose());
        assertTrue(Math.abs(drivetrain.odometry.pose().headingRadians()) > 3.0,
                () -> "Offseason odometry did not rotate: " + drivetrain.odometry.pose());
        assertTrue(Math.abs(localization.estimated.pose().headingRadians()) > 3.0,
                () -> "Offseason localization did not rotate: " + localization.estimated.pose());
    }

    private static final class RebuiltLocalization implements Mechanism {
        private final ImuDevice imuDevice = ImuDevice.of(ImuKinds.PIGEON_2, 90);
        private final ImuSource imu = imuDevice.relative();
        private final SwerveModule frontLeft = module(91, 101, 111);
        private final SwerveModule frontRight = module(92, 102, 112);
        private final SwerveModule backLeft = module(93, 103, 113);
        private final SwerveModule backRight = module(94, 104, 114);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.5588, 0.5588, 5.0, frontLeft, frontRight, backLeft, backRight);
        private final SwerveOdometry odometry = kinematics.odometry(imu);
        private final Action drive = kinematics.drive(() -> RobotVelocity.robot(0.8, 0.0, 0.35));

        private final CameraDevice intakeCamera = Cameras.photonVision("Intake")
                .mount(new CameraMountPose(0.2413, -0.32, 0.38989, 0.0, 10.0, 0.0))
                .failurePolicy(FailurePolicy.DISABLE_DEVICE);
        private final CameraDevice turretCamera = Cameras.photonVision("Turret")
                .mount(new CameraMountPose(-0.04, -0.335, 0.219964, 90.0, 0.0, 0.0))
                .failurePolicy(FailurePolicy.DISABLE_DEVICE);
        private final CameraDevice climberCamera = Cameras.photonVision("Climber")
                .mount(new CameraMountPose(-0.31, 0.0, 0.219964, 180.0, 0.0, 0.0))
                .failurePolicy(FailurePolicy.DISABLE_DEVICE);
        private final CameraDevice sideCamera = Cameras.photonVision("Side")
                .mount(new CameraMountPose(-0.13, 0.33, 0.199898, -90.0, 0.0, 0.0))
                .failurePolicy(FailurePolicy.DISABLE_DEVICE);

        private final PoseSignal intakePose = configured(intakeCamera);
        private final PoseSignal turretPose = configured(turretCamera);
        private final PoseSignal climberPose = configured(climberCamera);
        private final PoseSignal sidePose = configured(sideCamera);
        private final Localization filteredVision = Localizations.filter()
                .input(intakePose, turretPose, climberPose, sidePose)
                .filter(VisionFilters.finitePoseAndTimestamp())
                .filter(VisionFilters.maxLatencySeconds(0.15))
                .filter(VisionFilters.maxAmbiguity(0.20))
                .filter(VisionFilters.maxAverageTagDistanceMeters(5.0))
                .filter(LocalizationFilters.inside(Rectangle2d.field(16.541, 8.0692)))
                .name("filteredVision");
        private final Localization fusedVision = Localizations.covarianceIntersection()
                .input(filteredVision.translationOnly())
                .groupWithinSeconds(0.10)
                .maxTranslationDisagreementMeters(0.75)
                .name("fusedVision");
        private final Localization headingCandidates = Localizations.filter()
                .input(filteredVision)
                .name("headingCandidates");
        private final Localization fusedHeading = Localizations.covarianceIntersection()
                .input(headingCandidates.headingOnly())
                .groupWithinSeconds(0.10)
                .maxHeadingDisagreementRadians(Math.toRadians(20.0))
                .name("fusedHeading");
        private final Localization estimatedFieldPose = Localizations.kalman()
                .input(odometry, fusedVision, fusedHeading)
                .stateStdDevs(0.08, Math.toRadians(8.0))
                .defaultVisionStdDevs(0.9, Math.toRadians(25.0))
                .maxNormalizedVisionResidual(9.0)
                .name("estimatedFieldPose");

        private static PoseSignal configured(CameraDevice camera) {
            return camera.pose()
                    .multiTagStdDevs(0.25, 0.25, Math.toRadians(5.0))
                    .singleTagStdDevs(0.9, 0.9, Math.toRadians(12.0))
                    .distanceStdDevScaling(2.0, 2.0);
        }

        private static SwerveModule module(int driveId, int steerId, int encoderId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId).units(EncoderUnit.ROTATIONS));
            return module
                    .driveMaxSpeedMetersPerSecond(5.0)
                    .steerPid(12.0, 0.0, 0.0);
        }
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }

    private static final class OffseasonDrivetrain implements Mechanism {
        private final ImuDevice imuDevice = ImuDevice.of(ImuKinds.PIGEON_2, 190);
        private final ImuSource imu = imuDevice.relative();
        private final SwerveModule frontLeft = offseasonModule(191, 201, 211, 0.384);
        private final SwerveModule frontRight = offseasonModule(192, 202, 212, 0.171);
        private final SwerveModule backLeft = offseasonModule(193, 203, 213, 0.693);
        private final SwerveModule backRight = offseasonModule(194, 204, 214, 0.365);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.55, 0.55, 4.0, frontLeft, frontRight, backLeft, backRight);
        public final SwerveOdometry odometry = kinematics.odometry(imu);
        private final RobotVelocityPool pool = new RobotVelocityPool();
        private final RobotVelocityPool.Channel driver = pool.channel();
        private final Action drive = kinematics.drive(pool, () -> Math.toRadians(imu.yawDegrees()));

        private static SwerveModule offseasonModule(
                int driveId,
                int steerId,
                int encoderId,
                double offset) {
            SwerveModule module = new SwerveModules.SDS.MK4I.L3();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId).brake());
            module.steer.fill(MotorDevice.of(MotorKinds.FALCON_500, steerId).inverted());
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId)
                    .units(EncoderUnit.ROTATIONS)
                    .offset(offset));
            return module
                    .driveMaxSpeedMetersPerSecond(4.0)
                    .steerPid(14.0, 0.0, 0.0);
        }
    }

    private static final class OffseasonLocalization implements Mechanism {
        private final Localization estimated;

        private OffseasonLocalization(OffseasonDrivetrain drivetrain) {
            estimated = Localizations.kalman().input(drivetrain.odometry);
        }
    }
}
