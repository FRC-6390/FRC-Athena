package ca.frc6390.athena.core.localization;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.devices.IMU;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class RobotLocalization<T> extends SubsystemBase implements RobotSendableSystem{

    private final PoseEstimator<T> fieldEstimator2d, relativeEstimator2d;
    private final PoseEstimator3d<T> fieldEstimator3d, relativeEstimator3d;
    private RobotVision vision;
    private final IMU imu;
    private final RobotSpeeds robotSpeeds;
    private final Supplier<T> wheelPositions;

    private Pose2d fieldPose, relativePose;
    private Pose3d fieldPose3d, relativePose3d;
    private Field2d field;
    private RobotConfig robotConfig;
    private static final double STD_EPSILON = 1e-5;

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private PIDController rotationController, translationController;
    private AutoFactory factory;

    private BiConsumer<ChassisSpeeds, DriveFeedforwards> autoDrive;

    private boolean suppressUpdates = false;
    private StructPublisher<Pose2d> fieldPosePublisher;
    private StructPublisher<Pose2d> relativePosePublisher;
    private StructPublisher<Pose3d> fieldPose3dPublisher;
    private StructPublisher<Pose3d> relativePose3dPublisher;

    private double visionMaxLatencySeconds = 0.9;
    private double visionOutlierTranslationMeters = 2.5;
    private double visionOutlierRotationRadians = Units.degreesToRadians(35.0);
    private double visionStdDevOutlierMultiplier = 4.0;
    private double visionStdDevTranslationLimit = 5.0;
    private double visionStdDevRotationLimit = Units.degreesToRadians(120.0);
    private boolean hasAcceptedVisionMeasurement = false;
    private double minVisionUpdateSeparationSeconds = 0.02;
    private double lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;
    private final RobotLocalizationConfig.PoseSpace poseSpace;

    private final RobotLocalizationPersistence persistence;
    private final RobotLocalizationCameraManager cameraManager;
    private boolean restoringPersistentState = false;

    public RobotLocalization(
            PoseEstimator<T> fieldEstimator,
            PoseEstimator<T> relativeEstimator,
            RobotLocalizationConfig config,
            RobotSpeeds robotSpeeds,
            IMU imu,
            Supplier<T> wheelPositions) {
        this(
                fieldEstimator,
                relativeEstimator,
                null,
                null,
                (config == null ? new RobotLocalizationConfig() : config).use2d(),
                robotSpeeds,
                imu,
                wheelPositions);
    }

    public RobotLocalization(
            PoseEstimator3d<T> fieldEstimator,
            PoseEstimator3d<T> relativeEstimator,
            RobotLocalizationConfig config,
            RobotSpeeds robotSpeeds,
            IMU imu,
            Supplier<T> wheelPositions) {
        this(
                null,
                null,
                fieldEstimator,
                relativeEstimator,
                (config == null ? new RobotLocalizationConfig().use3d() : config.use3d()),
                robotSpeeds,
                imu,
                wheelPositions);
    }

    private RobotLocalization(
            PoseEstimator<T> fieldEstimator2d,
            PoseEstimator<T> relativeEstimator2d,
            PoseEstimator3d<T> fieldEstimator3d,
            PoseEstimator3d<T> relativeEstimator3d,
            RobotLocalizationConfig config,
            RobotSpeeds robotSpeeds,
            IMU imu,
            Supplier<T> wheelPositions) {
        config = config == null ? new RobotLocalizationConfig() : config;
        this.localizationConfig = config;
        this.poseSpace = config.poseSpace();
        this.robotSpeeds = robotSpeeds;
        this.imu = imu;
        this.wheelPositions = wheelPositions;
        this.fieldPose = new Pose2d();
        this.relativePose = new Pose2d();
        this.fieldPose3d = new Pose3d(fieldPose);
        this.relativePose3d = new Pose3d(relativePose);
        this.autoDrive = (speeds, feed) -> robotSpeeds.setSpeeds("auto", speeds);

        this.field = new Field2d();
        this.fieldPosePublisher = null;
        this.relativePosePublisher = null;
        this.fieldPose3dPublisher = null;
        this.relativePose3dPublisher = null;
        this.cameraManager = new RobotLocalizationCameraManager(STD_EPSILON, field);

        this.fieldEstimator2d = fieldEstimator2d;
        this.relativeEstimator2d = relativeEstimator2d;
        this.fieldEstimator3d = fieldEstimator3d;
        this.relativeEstimator3d = relativeEstimator3d;

        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D
                && (fieldEstimator3d == null || relativeEstimator3d == null)) {
            throw new IllegalArgumentException("3D localization requires PoseEstimator3d instances.");
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.TWO_D
                && (fieldEstimator2d == null || relativeEstimator2d == null)) {
            throw new IllegalArgumentException("2D localization requires PoseEstimator instances.");
        }

        this.visionEnabled = config.useVision();

        imu.addVirtualAxis("relative", imu::getYaw);
        imu.addVirtualAxis("field", imu::getYaw);
        imu.setVirtualAxis("relative", new Rotation2d());
        imu.setVirtualAxis("field", new Rotation2d());

        if (config.rotation() != null && config.translation() != null) {
            configurePathPlanner(config.translation(), config.rotation());
        }

        this.persistence = new RobotLocalizationPersistence(this.poseSpace);
        persistence.updateSnapshot(fieldPose, fieldPose3d, imu.getVirtualAxis("driver"));
        loadPersistentState();
    }

    public RobotLocalization<T> enableVisionForLocalization(boolean visionEnabled){
        this.visionEnabled = visionEnabled;
        return this;
    }

    public RobotLocalization<T> setRobotVision(RobotVision vision){
        this.vision = vision;
        if (vision != null) {
            vision.attachLocalization(this);
            vision.setLocalizationStdDevs(localizationConfig.getVisionStd(), localizationConfig.getVisionMultitagStd());
        }
        cameraManager.ensureCameraShuffleboardEntries(vision);
        return this;
    }

    public RobotLocalization<T> setVisionOutlierThresholds(double translationMeters, double rotationDegrees) {
        this.visionOutlierTranslationMeters = Math.max(0.0, translationMeters);
        this.visionOutlierRotationRadians = Units.degreesToRadians(Math.max(0.0, rotationDegrees));
        return this;
    }

    public RobotLocalization<T> setVisionStdDevOutlierMultiplier(double multiplier) {
        if (Double.isFinite(multiplier) && multiplier >= 1.0) {
            this.visionStdDevOutlierMultiplier = multiplier;
        }
        return this;
    }

    public RobotLocalization<T> setVisionStdDevLimits(double translationMeters, double rotationDegrees) {
        this.visionStdDevTranslationLimit = Math.max(0.0, translationMeters);
        this.visionStdDevRotationLimit = Units.degreesToRadians(Math.max(0.0, rotationDegrees));
        return this;
    }

    public RobotLocalization<T> setVisionUpdateRateLimit(double minSeparationSeconds) {
        if (Double.isFinite(minSeparationSeconds) && minSeparationSeconds >= 0.0) {
            this.minVisionUpdateSeparationSeconds = minSeparationSeconds;
        }
        return this;
    }

    public RobotLocalization<T> setVisionLatencyLimit(double maxLatencySeconds) {
        if (Double.isFinite(maxLatencySeconds) && maxLatencySeconds >= 0.0) {
            this.visionMaxLatencySeconds = maxLatencySeconds;
        }
        return this;
    }

    public void setAutoDrive(BiConsumer<RobotSpeeds, ChassisSpeeds> autoDrive) {
        this.autoDrive = (speeds, feeds) -> autoDrive.accept(robotSpeeds, speeds);
    }

    private Matrix<N3, N1> sanitizeVisionStdDevs(Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) {
            return null;
        }
        double sanitizedX = sanitizeStdDevEntry(stdDevs.get(0, 0), visionStdDevTranslationLimit);
        double sanitizedY = sanitizeStdDevEntry(stdDevs.get(1, 0), visionStdDevTranslationLimit);
        double sanitizedTheta = sanitizeStdDevEntry(stdDevs.get(2, 0), visionStdDevRotationLimit);
        if (Double.isNaN(sanitizedX) || Double.isNaN(sanitizedY) || Double.isNaN(sanitizedTheta)) {
            return null;
        }
        return VecBuilder.fill(sanitizedX, sanitizedY, sanitizedTheta);
    }

    private double sanitizeStdDevEntry(double value, double maxValue) {
        double sanitized = Math.abs(value);
        if (!Double.isFinite(sanitized) || sanitized < STD_EPSILON) {
            return Double.NaN;
        }
        if (maxValue > 0.0 && sanitized >= maxValue) {
            return Double.NaN;
        }
        return sanitized;
    }

    private boolean shouldApplyVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        if (!isFinitePose(visionPose)) {
            return false;
        }
        if (!Double.isFinite(timestampSeconds)) {
            return false;
        }
        if (hasAcceptedVisionMeasurement) {
            if (timestampSeconds <= lastVisionMeasurementTimestamp) {
                return false;
            }
            if (minVisionUpdateSeparationSeconds > 0.0
                    && (timestampSeconds - lastVisionMeasurementTimestamp) < minVisionUpdateSeparationSeconds) {
                return false;
            }
        }
        double ageSeconds = Timer.getFPGATimestamp() - timestampSeconds;
        if (visionMaxLatencySeconds > 0.0) {
            // Reject frames that are too old or unexpectedly in the future (>100 ms lead).
            if (ageSeconds > visionMaxLatencySeconds || ageSeconds < -0.1) {
                return false;
            }
        }
        if (!hasAcceptedVisionMeasurement) {
            return true;
        }
        Pose2d referencePose = fieldPose != null ? fieldPose : new Pose2d();
        Pose2d delta = visionPose.relativeTo(referencePose);
        double translationError = delta.getTranslation().getNorm();
        double rotationError = Math.abs(delta.getRotation().getRadians());

        double translationThreshold = visionOutlierTranslationMeters;
        double rotationThreshold = visionOutlierRotationRadians;
        if (stdDevs != null) {
            double translationStd = Math.max(stdDevs.get(0, 0), stdDevs.get(1, 0));
            double rotationStd = stdDevs.get(2, 0);
            if (Double.isFinite(translationStd) && translationStd > STD_EPSILON) {
                translationThreshold = Math.max(translationThreshold, translationStd * visionStdDevOutlierMultiplier);
            }
            if (Double.isFinite(rotationStd) && rotationStd > STD_EPSILON) {
                rotationThreshold = Math.max(rotationThreshold, rotationStd * visionStdDevOutlierMultiplier);
            }
        }

        return translationError <= translationThreshold && rotationError <= rotationThreshold;
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private Matrix<N3, N1> adjustVisionStdDevsForDisagreement(Matrix<N3, N1> stdDevs, Pose2d visionPose) {
        if (stdDevs == null) {
            return null;
        }
        Pose2d referencePose = fieldPose != null ? fieldPose : new Pose2d();
        Pose2d delta = visionPose.relativeTo(referencePose);
        double translationError = delta.getTranslation().getNorm();
        double rotationError = Math.abs(delta.getRotation().getRadians());

        double translationDenominator = Math.max(visionOutlierTranslationMeters, STD_EPSILON);
        double rotationDenominator = Math.max(visionOutlierRotationRadians, STD_EPSILON);

        double translationScale = 1.0 + Math.pow(translationError / translationDenominator, 2.0);
        double rotationScale = 1.0 + Math.pow(rotationError / rotationDenominator, 2.0);

        translationScale = Math.min(translationScale, 6.0);
        rotationScale = Math.min(rotationScale, 6.0);

        double scaledX = clampStdDev(stdDevs.get(0, 0) * translationScale, visionStdDevTranslationLimit);
        double scaledY = clampStdDev(stdDevs.get(1, 0) * translationScale, visionStdDevTranslationLimit);
        double scaledTheta = clampStdDev(stdDevs.get(2, 0) * rotationScale, visionStdDevRotationLimit);
        return VecBuilder.fill(scaledX, scaledY, scaledTheta);
    }

    private static double clampStdDev(double value, double limit) {
        if (limit > 0.0) {
            return Math.min(value, limit);
        }
        return value;
    }

    private Matrix<N4, N1> expandStdDevsTo3d(Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) {
            return null;
        }
        double zStd = clampStdDev(localizationConfig.getVisionStd3d().get(2, 0), visionStdDevTranslationLimit);
        return VecBuilder.fill(stdDevs.get(0, 0), stdDevs.get(1, 0), zStd, stdDevs.get(2, 0));
    }

    private void persistRobotState(boolean force) {
        Pose2d currentFieldPose = fieldPose != null ? fieldPose : new Pose2d();
        Pose3d currentFieldPose3d =
                poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D && fieldPose3d != null
                        ? fieldPose3d
                        : new Pose3d(currentFieldPose);
        Rotation2d currentDriverYaw = imu != null ? imu.getVirtualAxis("driver") : new Rotation2d();

        persistence.persist(currentFieldPose, currentFieldPose3d, currentDriverYaw, force, restoringPersistentState);
    }

    private void loadPersistentState() {
        restoringPersistentState = true;
        try {
            persistence.readPersistentState().ifPresent(state -> {
                if (imu != null) {
                    imu.update();
                    imu.setYaw(state.driverYaw());
                }
                if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
                    resetFieldPose(state.pose3d());
                } else {
                    resetFieldPose(state.pose2d());
                }
            });
        } finally {
            restoringPersistentState = false;
        }
    }

    private Rotation3d getRotation3d(String axisId) {
        Rotation2d yaw = imu.getVirtualAxis(axisId);
        Rotation2d roll = imu.getRoll();
        Rotation2d pitch = imu.getPitch();
        return new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants){
      return configurePathPlanner(translationConstants, rotationConstants, autoDrive);
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants, BiConsumer<ChassisSpeeds, DriveFeedforwards> output){
        try{
            robotConfig = RobotConfig.fromGUISettings();  
        }catch(Exception e){
            DriverStation.reportWarning("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
            return this;
        }
        AutoBuilder.configure(
            this::getFieldPose, 
            this::resetFieldPose, 
            () -> robotSpeeds.getSpeeds("drive"), 
            output, 
            new PPHolonomicDriveController(
            translationConstants,
            rotationConstants
            ),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }            
      );
      FollowPathCommand.warmupCommand().schedule();
      return this;
    }

    public RobotLocalization<T> configureChoreo(Subsystem drivetrain){
        PIDConstants rotationConstants = localizationConfig.rotation();
        PIDConstants translationConstants = localizationConfig.translation();

        rotationController = new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD);
        rotationController.setIZone(rotationConstants.iZone);

        translationController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        translationController.setIZone(translationConstants.iZone);

        factory = new AutoFactory(this::getFieldPose, this::resetFieldPose, 
        (sample) -> {
            Pose2d botpose = getFieldPose();
            Pose2d trajpose = sample.getPose();
            
            ChassisSpeeds speeds = sample.getChassisSpeeds();

            speeds.vxMetersPerSecond = translationController.calculate(botpose.getX(), trajpose.getX());
            speeds.vyMetersPerSecond = translationController.calculate(botpose.getY(), trajpose.getY());
            speeds.omegaRadiansPerSecond = rotationController.calculate(botpose.getRotation().getRadians(), trajpose.getRotation().getRadians());

            robotSpeeds.setSpeeds("auto", speeds);
        }, 
        true,
        drivetrain);
      return this;
    }

    public AutoFactory getChoreoAutoFactory(){
        return factory;
    }

    public void resetFieldPose(Pose2d pose) {
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetFieldPose(new Pose3d(pose));
            return;
        }
        fieldEstimator2d.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("field", pose.getRotation());
        this.fieldPose = pose;
        this.fieldPose3d = new Pose3d(pose);
        this.hasAcceptedVisionMeasurement = false;
        this.lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;
        persistRobotState(true);
    }

    public void resetFieldPose(Pose3d pose) {
        if (pose == null) {
            return;
        }
        if (poseSpace != RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetFieldPose(pose.toPose2d());
            return;
        }
        fieldEstimator3d.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        Pose2d pose2d = pose.toPose2d();
        imu.setVirtualAxis("field", pose2d.getRotation());
        this.fieldPose3d = pose;
        this.fieldPose = pose2d;
        this.hasAcceptedVisionMeasurement = false;
        this.lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;
        persistRobotState(true);
    }

    public void resetFieldPose(double x, double y) {
        resetFieldPose(new Pose2d(x, y, imu.getVirtualAxis("field")));
    }

    public void resetFieldPose(double x, double y, double thetaDegree) {
        resetFieldPose(new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegree)));
    }

    public void resetFieldPose() {
        resetFieldPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void resetRelativePose(Pose2d pose) {
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetRelativePose(new Pose3d(pose));
            return;
        }
        relativeEstimator2d.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("relative", pose.getRotation());
        this.relativePose = pose;
        this.relativePose3d = new Pose3d(pose);
    }

    public void resetRelativePose(Pose3d pose) {
        if (pose == null) {
            return;
        }
        if (poseSpace != RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetRelativePose(pose.toPose2d());
            return;
        }
        relativeEstimator3d.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        Pose2d pose2d = pose.toPose2d();
        imu.setVirtualAxis("relative", pose2d.getRotation());
        this.relativePose3d = pose;
        this.relativePose = pose2d;
    }

    public void resetRelativePose(double x, double y) {
        resetRelativePose(new Pose2d(x, y, imu.getVirtualAxis("relative")));
    }

    public void resetRelativePose(double x, double y, double r) {
        resetRelativePose(new Pose2d(x, y, new Rotation2d(r)));
    }

    public void resetRelativePose() {
        resetRelativePose(0,0, 0);
    }

    public void setVisionStd(Matrix<N3, N1> matrix){
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Matrix<N4, N1> expanded = expandStdDevsTo3d(matrix);
            if (expanded != null) {
                fieldEstimator3d.setVisionMeasurementStdDevs(expanded);
            }
        } else {
            fieldEstimator2d.setVisionMeasurementStdDevs(matrix);
        }
    }

    public void setVisionStd(double x, double y, double theta){
        setVisionStd(VecBuilder.fill(x,y,Units.degreesToRadians(theta)));
    }

    public void update() {
        if (suppressUpdates) {
            return;
        }

        if (vision != null && visionEnabled) {
            vision.setRobotOrientation(fieldPose);
            vision.getBestVisionMeasurement().ifPresent(measurement -> {
                Matrix<N3, N1> rawStdDevs = measurement.stdDevs();
                Matrix<N3, N1> sanitizedStdDevs = sanitizeVisionStdDevs(rawStdDevs);
                if (rawStdDevs != null && sanitizedStdDevs == null) {
                    return;
                }
                Pose2d measurementPose = measurement.pose2d();
                double timestampSeconds = measurement.timestampSeconds();
                if (!shouldApplyVisionMeasurement(measurementPose, timestampSeconds, sanitizedStdDevs)) {
                    return;
                }
                Matrix<N3, N1> adjustedStdDevs2d =
                        adjustVisionStdDevsForDisagreement(sanitizedStdDevs, measurementPose);
                if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
                    Matrix<N4, N1> adjustedStdDevs3d = expandStdDevsTo3d(adjustedStdDevs2d);
                    if (adjustedStdDevs3d != null) {
                        fieldEstimator3d.addVisionMeasurement(measurement.pose3d(), timestampSeconds, adjustedStdDevs3d);
                    } else {
                        fieldEstimator3d.addVisionMeasurement(measurement.pose3d(), timestampSeconds);
                    }
                } else {
                    if (adjustedStdDevs2d != null) {
                        fieldEstimator2d.addVisionMeasurement(measurementPose, timestampSeconds, adjustedStdDevs2d);
                    } else {
                        fieldEstimator2d.addVisionMeasurement(measurementPose, timestampSeconds);
                    }
                }
                hasAcceptedVisionMeasurement = true;
                lastVisionMeasurementTimestamp = timestampSeconds;
            });
        }

        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Rotation3d fieldRotation = getRotation3d("field");
            fieldPose3d = fieldEstimator3d.update(fieldRotation, wheelPositions.get());
            fieldPose = fieldPose3d.toPose2d();
            Rotation3d relativeRotation = getRotation3d("relative");
            relativePose3d = relativeEstimator3d.update(relativeRotation, wheelPositions.get());
            relativePose = relativePose3d.toPose2d();
        } else {
            fieldPose = fieldEstimator2d.update(imu.getVirtualAxis("field"), wheelPositions.get());
            fieldPose3d = new Pose3d(fieldPose);
            relativePose = relativeEstimator2d.update(imu.getVirtualAxis("relative"), wheelPositions.get());
            relativePose3d = new Pose3d(relativePose);
        }

        field.setRobotPose(fieldPose);
        if (fieldPosePublisher != null) {
            fieldPosePublisher.set(fieldPose);
        }
        if (relativePosePublisher != null) {
            relativePosePublisher.set(relativePose);
        }
        if (fieldPose3dPublisher != null) {
            fieldPose3dPublisher.set(fieldPose3d);
        }
        if (relativePose3dPublisher != null) {
            relativePose3dPublisher.set(relativePose3d);
        }
        cameraManager.updateCameraVisualizations(vision, fieldPose);
        persistRobotState(false);
    }

    public FieldObject2d getField2dObject(String id){
        return field.getObject(id);
    }

    public Pose2d getFieldPose(){
        return fieldPose;
    }

    public Pose3d getFieldPose3d() {
        return fieldPose3d;
    }

    public boolean isSuppressUpdates() {
        return suppressUpdates;
    }

    public void setSuppressUpdates(boolean suppressUpdates) {
        this.suppressUpdates = suppressUpdates;
    }

    public Pose2d getRelativePose(){
        return relativePose;
    }

    public Pose3d getRelativePose3d() {
        return relativePose3d;
    }

    public Field2d getField2d() {
        return field;
    }

    public Field2d getVisionField2d() {
        return cameraManager.getVisionField();
    }

    public void registerVisionShuffleboardTab(ShuffleboardTab tab) {
        cameraManager.setVisionShuffleboardTab(tab);
        cameraManager.ensureCameraShuffleboardEntries(vision);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        cameraManager.setLocalizationShuffleboardTab(tab);
        tab.add("Estimator", field).withPosition( 0,0).withSize(4, 3);

        String baseTopic = "/Shuffleboard/" + tab.getTitle();

        if (fieldPosePublisher == null) {
            String fieldTopic = baseTopic + "/EstimatorPose";
            fieldPosePublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(fieldTopic, Pose2d.struct)
                    .publish();
            fieldPosePublisher.set(fieldPose);
        }

        if (relativePosePublisher == null) {
            String relativeTopic = baseTopic + "/RelativePose";
            relativePosePublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(relativeTopic, Pose2d.struct)
                    .publish();
            relativePosePublisher.set(relativePose);
        }

        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            if (fieldPose3dPublisher == null) {
                String field3dTopic = baseTopic + "/EstimatorPose3d";
                fieldPose3dPublisher = NetworkTableInstance.getDefault()
                        .getStructTopic(field3dTopic, Pose3d.struct)
                        .publish();
                fieldPose3dPublisher.set(fieldPose3d);
            }
            if (relativePose3dPublisher == null) {
                String relative3dTopic = baseTopic + "/RelativePose3d";
                relativePose3dPublisher = NetworkTableInstance.getDefault()
                        .getStructTopic(relative3dTopic, Pose3d.struct)
                        .publish();
                relativePose3dPublisher.set(relativePose3d);
            }
        }

        if (level == SendableLevel.DEBUG) {
            // Debug-specific struct publishers are already configured above.
        }
        cameraManager.ensureCameraShuffleboardEntries(vision);
        return tab;
    }

    @Override
    public void periodic() {
       update();
    }

} 
