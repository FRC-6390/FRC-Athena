package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.RecoveryPolicy;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public final class Localization implements Mechanism {
    private static final RecoveryPolicy CAMERA_RECOVERY =
            RecoveryPolicy.retryEvery(Seconds.of(0.1)).healthySamples(1);

    // Devices
    public final PhotonVisionDevice rightCamera = new PhotonVisionDevice("right")
            .mount(new CameraMountPose(
                    Inches.of(-5.25),
                    Inches.of(-13.75),
                    Inches.of(7.0),
                    Degrees.of(-90.0),
                    Degrees.of(0.0),
                    Degrees.of(0.0)))
            .failurePolicy(FailurePolicy.WARN);
        //     .onRecovery(CAMERA_RECOVERY);

                    public final PhotonVisionDevice leftCamera = new PhotonVisionDevice("left")
            .mount(new CameraMountPose(
                    Inches.of(-5.25),
                    Inches.of(13.75),
                    Inches.of(7.0),
                    Degrees.of(90.0),
                    Degrees.of(0.0),
                    Degrees.of(0.0)))
            .failurePolicy(FailurePolicy.WARN);
        //     .onRecovery(CAMERA_RECOVERY);

    public final PhotonVisionDevice backCamera = new PhotonVisionDevice("front")
            .mount(new CameraMountPose(
                    Inches.of(-12.5),
                    Meters.of(0.0),
                    Inches.of(15.5),
                    Degrees.of(180.0),
                    Degrees.of(0.0),
                    Degrees.of(0.0)))
            .failurePolicy(FailurePolicy.WARN);
        //     .onRecovery(CAMERA_RECOVERY);

    // Bindings and controls
    private final SwerveOdometry odometry;
    public final PoseSignal rightPose = rightCamera.pose()
            .multiTagOnCoprocessor()
            .singleTagUsesHeading()
            .multiTagStdDevs(Meters.of(0.25), Meters.of(0.25), Degrees.of(5.0))
            .singleTagStdDevs(Meters.of(0.9), Meters.of(0.9), Degrees.of(12.0))
            .distanceStdDevScaling(Meters.of(2.0), 2.0);

             public final PoseSignal leftPose = leftCamera.pose()
            .multiTagOnCoprocessor()
            .singleTagUsesHeading()
            .multiTagStdDevs(Meters.of(0.25), Meters.of(0.25), Degrees.of(5.0))
            .singleTagStdDevs(Meters.of(0.9), Meters.of(0.9), Degrees.of(12.0))
            .distanceStdDevScaling(Meters.of(2.0), 2.0);

    public final PoseSignal backPose = backCamera.pose()
            .multiTagOnCoprocessor()
            .singleTagUsesHeading()
            .multiTagStdDevs(Meters.of(0.25), Meters.of(0.25), Degrees.of(5.0))
            .singleTagStdDevs(Meters.of(0.9), Meters.of(0.9), Degrees.of(12.0))
            .distanceStdDevScaling(Meters.of(2.0), 2.0);
    // One filter stage per camera, each feeding the Kalman estimator directly: independent cameras
    // tighten the estimate, and the vision status (athena/vision/<camera>/...) names each camera.
    public final ca.frc6390.athena.localization.pipeline.Localization leftVision;
    public final ca.frc6390.athena.localization.pipeline.Localization rightVision;
    public final ca.frc6390.athena.localization.pipeline.Localization backVision;
    public final ca.frc6390.athena.localization.pipeline.Localization estimatedFieldPose;

    public Localization(Drivetrain drivetrain) {
        odometry = drivetrain.odometry;
        drivetrain.fieldPose(this::pose);
        leftVision = cameraStage(leftPose, "leftVision");
        rightVision = cameraStage(rightPose, "rightVision");
        backVision = cameraStage(backPose, "backVision");
        estimatedFieldPose = Localizations.kalman()
                .input(odometry, leftVision, rightVision, backVision)
                .stateStdDevs(Meters.of(0.08), Degrees.of(8.0))
                .defaultVisionStdDevs(Meters.of(0.9), Degrees.of(25.0))
                .maxNormalizedVisionResidual(9.0)
                // Trust vision less while driving and spinning (motion blur, latency), and ignore it
                // while spinning very fast.
                .visionStdDevSpeedScaling(0.5, 0.5)
                .maxVisionAngularVelocity(DegreesPerSecond.of(540.0))
                // While disabled, snap to multi-tag vision so the pose is right before auto starts.
                .seedFromVisionWhileDisabled(DriverStation::isDisabled)
                .name("estimatedFieldPose")
                .publishNetworkTables();
    }

    private static ca.frc6390.athena.localization.pipeline.Localization cameraStage(PoseSignal camera, String name) {
        return Localizations.filter()
                .input(camera)
                .filter(VisionFilters.finitePoseAndTimestamp())
                .filter(VisionFilters.maxLatency(Seconds.of(0.15)))
                .filter(VisionFilters.maxAmbiguity(0.20))
                .filter(VisionFilters.maxAverageTagDistance(Meters.of(5.0)))
                .filter(LocalizationFilters.inside(Rectangle2d.field(Constants.FIELD_LENGTH, Constants.FIELD_WIDTH)))
                .name(name);
    }

    // Miscellaneous state
    public Pose2d pose() {
        return estimatedFieldPose.pose2d();
    }

    @Telemetry("distanceToHubMeters")
    public double distanceToHubMeters() {
        return Constants.Field.distanceToAllianceHubMeters(pose());
    }

    @Telemetry("inTrench")
    public boolean inTrench() {
        Pose2d robotPose = pose();
        return Constants.Field.TRENCHES.contains(robotPose);
    }

    @Telemetry("field/blueLeftTrench")
    private Rectangle2d blueLeftTrench() {
        return Constants.Field.BLUE_LEFT_TRENCH;
    }

    @Telemetry("field/blueRightTrench")
    private Rectangle2d blueRightTrench() {
        return Constants.Field.BLUE_RIGHT_TRENCH;
    }

    @Telemetry("field/redLeftTrench")
    private Rectangle2d redLeftTrench() {
        return Constants.Field.RED_LEFT_TRENCH;
    }

    @Telemetry("field/redRightTrench")
    private Rectangle2d redRightTrench() {
        return Constants.Field.RED_RIGHT_TRENCH;
    }

    public Pose2d odometryPose() {
        return odometry.pose();
    }

    // Action factories
    public Action reset(Pose2d pose) {
        return Actions.doOnce(context -> {
            odometry.reset(snapshot(pose));
            estimatedFieldPose.reset(pose).apply(context);
        });
    }

    /** Resets to the pose supplied when the action runs, not when it is created. */
    public Action reset(Supplier<Pose2d> pose) {
        return Actions.doOnce(context -> {
            Pose2d resetPose = pose.get();
            odometry.reset(snapshot(resetPose));
            estimatedFieldPose.reset(resetPose).apply(context);
        });
    }

    public Action resetHeading() {
        return Actions.doOnce(context -> {
            Pose2d current = pose();
            Pose2d resetPose = new Pose2d(current.getTranslation(), Rotation2d.kZero);
            odometry.reset(snapshot(resetPose));
            estimatedFieldPose.reset(resetPose).apply(context);
        });
    }

    private static PoseSnapshot snapshot(Pose2d pose) {
        return new PoseSnapshot(
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians());
    }
}
