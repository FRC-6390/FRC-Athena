package ca.frc6390.athena.core.localization;

import java.lang.reflect.Field;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.ChoreoAutoFactory;
import ca.frc6390.athena.core.auto.ChoreoBinding;
import ca.frc6390.athena.core.auto.HolonomicDriveBinding;
import ca.frc6390.athena.core.auto.HolonomicFeedforward;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class RobotLocalization<T> extends SubsystemBase implements RobotSendableSystem{

    private final PoseEstimator<T> fieldEstimator2d, relativeEstimator2d;
    private final PoseEstimator3d<T> fieldEstimator3d, relativeEstimator3d;
    private RobotVision vision;
    private final Imu imu;
    private final RobotSpeeds robotSpeeds;
    private final Supplier<T> wheelPositions;

    private Pose2d fieldPose, relativePose;
    private Pose3d fieldPose3d, relativePose3d;
    private Field2d field;
    private static final double STD_EPSILON = 1e-5;

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private ChoreoAutoFactory choreoFactory;

    private BiConsumer<ChassisSpeeds, HolonomicFeedforward> autoDrive;

    private boolean suppressUpdates = false;
    private StructPublisher<Pose2d> fieldPosePublisher;
    private StructPublisher<Pose2d> relativePosePublisher;
    private StructPublisher<Pose3d> fieldPose3dPublisher;
    private StructPublisher<Pose3d> relativePose3dPublisher;
    private NetworkTable backendTable;
    private NetworkTableEntry backendOverrideEnabledEntry;
    private NetworkTableEntry backendSlipStrategyEntry;
    private NetworkTableEntry backendImuStrategyEntry;
    private NetworkTableEntry backendVisionStrategyEntry;

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
    private final Map<String, VirtualPose2d> virtualPoses2d = new HashMap<>();
    private final Map<String, VirtualPose3d> virtualPoses3d = new HashMap<>();
    private long virtualPoseRevision = 0;
    private boolean slipActive = false;
    private double slipActiveUntilSeconds = Double.NEGATIVE_INFINITY;
    private double lastUpdateTimestamp = Double.NaN;
    private Pose2d lastFieldPoseForSlip = new Pose2d();
    private Translation2d lastFieldVelocityForSlip = new Translation2d();
    private double poseJumpGuardUntilSeconds = Double.NEGATIVE_INFINITY;
    private Pose2d lastHealthPose = new Pose2d();
    private double lastHealthTimestamp = Double.NaN;
    private double lastPoseJumpMeters = 0.0;
    private double driftRateMetersPerSec = 0.0;
    private int visionMeasurementCount = 0;
    private int visionMeasurementAccepted = 0;
    private static final int VISION_ACCEPT_WINDOW = 50;
    private final boolean[] visionAcceptWindow = new boolean[VISION_ACCEPT_WINDOW];
    private int visionAcceptIndex = 0;
    private int visionAcceptCount = 0;
    private int visionAcceptAccepted = 0;
    private double visionAcceptRateWindow = 0.0;
    private GenericEntry poseJumpEntry;
    private GenericEntry driftRateEntry;
    private GenericEntry visionAcceptanceEntry;
    private GenericEntry slipActiveEntry;
    private NetworkTableEntry poseJumpNtEntry;
    private NetworkTableEntry driftRateNtEntry;
    private NetworkTableEntry visionAcceptanceNtEntry;
    private NetworkTableEntry slipActiveNtEntry;
    private GenericEntry backendOverrideToggleEntry;
    private SendableChooser<RobotLocalizationConfig.BackendConfig.SlipStrategy> backendSlipStrategyChooser;
    private SendableChooser<RobotLocalizationConfig.BackendConfig.ImuStrategy> backendImuStrategyChooser;
    private SendableChooser<RobotLocalizationConfig.BackendConfig.VisionStrategy> backendVisionStrategyChooser;

    private final RobotLocalizationPersistence persistence;
    private final RobotLocalizationCameraManager cameraManager;
    private boolean restoringPersistentState = false;
    private final Matrix<N3, N1> baseStateStdDevs2d;
    private final Matrix<N4, N1> baseStateStdDevs3d;
    private double processStdDevScale = 1.0;
    private boolean processStdDevUpdateFailed = false;
    private static Field poseEstimatorQField;
    private static Field poseEstimator3dQField;
    private FieldObject2d plannedPathObject;
    private FieldObject2d actualPathObject;
    private final java.util.List<Pose2d> actualPathPoses = new java.util.ArrayList<>();
    private double actualPathSpacingMeters = 0.1;
    private double actualPathMinIntervalSeconds = 0.1;
    private int actualPathMaxPoints = 600;
    private double plannedPathSpacingMeters = 0.2;
    private Double fieldLengthMetersOverride = null;
    private Double fieldWidthMetersOverride = null;
    private double lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
    private Pose2d lastActualPathPose = new Pose2d();
    private String lastPlannedAutoId;
    private boolean fieldPublished;

    public RobotLocalization(
            PoseEstimator<T> fieldEstimator,
            PoseEstimator<T> relativeEstimator,
            RobotLocalizationConfig config,
            RobotSpeeds robotSpeeds,
            Imu imu,
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
            Imu imu,
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
            Imu imu,
            Supplier<T> wheelPositions) {
        config = config == null ? new RobotLocalizationConfig() : config;
        this.localizationConfig = config;
        this.poseSpace = config.poseSpace();
        this.robotSpeeds = robotSpeeds;
        this.imu = imu;
        this.wheelPositions = wheelPositions;
        this.baseStateStdDevs2d = config.getStd2d();
        this.baseStateStdDevs3d = config.getStd3d();
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
        this.plannedPathObject = field.getObject("AutoPlan");
        this.actualPathObject = field.getObject("ActualPath");
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

        this.visionEnabled = config.isVisionEnabled();

        imu.addVirtualAxis("relative", imu::getYaw);
        imu.addVirtualAxis("field", imu::getYaw);
        imu.setVirtualAxis("relative", new Rotation2d());
        imu.setVirtualAxis("field", new Rotation2d());

        if (config.rotation() != null && config.translation() != null
                && AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).isPresent()) {
            configurePathPlanner(config.translation(), config.rotation());
        }

        this.persistence = new RobotLocalizationPersistence(this.poseSpace);
        persistence.updateSnapshot(fieldPose, fieldPose3d, imu.getVirtualAxis("driver"));
        loadPersistentState();
        initBackendOverrides();
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
        Rotation2d yaw = getYaw(axisId);
        Rotation2d roll = imu.getRoll();
        Rotation2d pitch = imu.getPitch();
        return new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
    }

    private Rotation2d getYaw(String axisId) {
        RobotLocalizationConfig.BackendConfig backend = localizationConfig.backend();
        if (backend != null && backend.imuStrategy() == RobotLocalizationConfig.BackendConfig.ImuStrategy.RAW_YAW) {
            return imu.getYaw();
        }
        return imu.getVirtualAxis(axisId);
    }

    private RobotLocalizationConfig.BackendConfig backendConfig() {
        RobotLocalizationConfig.BackendConfig base =
                localizationConfig != null ? localizationConfig.backend() : RobotLocalizationConfig.BackendConfig.defualt();
        if (backendOverrideEnabledEntry != null && backendOverrideEnabledEntry.getBoolean(false)) {
            return applyBackendOverride(base);
        }
        return base;
    }

    private boolean isVisionEnabled() {
        return backendConfig().resolveVisionEnabled(visionEnabled);
    }

    private void initBackendOverrides() {
        backendTable = NetworkTableInstance.getDefault().getTable("Athena/Localization/Backend");
        backendOverrideEnabledEntry = backendTable.getEntry("EnableOverride");
        backendSlipStrategyEntry = backendTable.getEntry("SlipStrategy");
        backendImuStrategyEntry = backendTable.getEntry("ImuStrategy");
        backendVisionStrategyEntry = backendTable.getEntry("VisionStrategy");

        backendOverrideEnabledEntry.setBoolean(false);
        RobotLocalizationConfig.BackendConfig base = backendConfig();
        backendSlipStrategyEntry.setString(base.slipStrategy().name());
        backendImuStrategyEntry.setString(base.imuStrategy().name());
        backendVisionStrategyEntry.setString(base.visionStrategy().name());
    }

    private void syncBackendOverridesFromShuffleboard() {
        if (backendOverrideEnabledEntry == null || backendOverrideToggleEntry == null) {
            return;
        }
        backendOverrideEnabledEntry.setBoolean(backendOverrideToggleEntry.getBoolean(false));
        if (backendSlipStrategyEntry != null && backendSlipStrategyChooser != null) {
            RobotLocalizationConfig.BackendConfig.SlipStrategy selected = backendSlipStrategyChooser.getSelected();
            if (selected != null) {
                backendSlipStrategyEntry.setString(selected.name());
            }
        }
        if (backendImuStrategyEntry != null && backendImuStrategyChooser != null) {
            RobotLocalizationConfig.BackendConfig.ImuStrategy selected = backendImuStrategyChooser.getSelected();
            if (selected != null) {
                backendImuStrategyEntry.setString(selected.name());
            }
        }
        if (backendVisionStrategyEntry != null && backendVisionStrategyChooser != null) {
            RobotLocalizationConfig.BackendConfig.VisionStrategy selected = backendVisionStrategyChooser.getSelected();
            if (selected != null) {
                backendVisionStrategyEntry.setString(selected.name());
            }
        }
    }

    private RobotLocalizationConfig.BackendConfig applyBackendOverride(RobotLocalizationConfig.BackendConfig base) {
        RobotLocalizationConfig.BackendConfig.SlipStrategy slipStrategy =
                parseEnum(backendSlipStrategyEntry.getString(base.slipStrategy().name()), base.slipStrategy());
        RobotLocalizationConfig.BackendConfig.ImuStrategy imuStrategy =
                parseEnum(backendImuStrategyEntry.getString(base.imuStrategy().name()), base.imuStrategy());
        RobotLocalizationConfig.BackendConfig.VisionStrategy visionStrategy =
                parseEnum(backendVisionStrategyEntry.getString(base.visionStrategy().name()), base.visionStrategy());

        return base
                .withSlipStrategy(slipStrategy)
                .withImuStrategy(imuStrategy)
                .withVisionStrategy(visionStrategy);
    }

    private static <E extends Enum<E>> E parseEnum(String value, E fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return Enum.valueOf(fallback.getDeclaringClass(), value.trim().toUpperCase());
        } catch (IllegalArgumentException ex) {
            return fallback;
        }
    }

    public RobotLocalization<T> configurePathPlanner(HolonomicPidConstants translationConstants, HolonomicPidConstants rotationConstants){
      return configurePathPlanner(translationConstants, rotationConstants, autoDrive);
    }

    public RobotLocalization<T> configurePathPlanner(HolonomicPidConstants translationConstants, HolonomicPidConstants rotationConstants, BiConsumer<ChassisSpeeds, HolonomicFeedforward> output){
        AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).ifPresentOrElse(backend -> {
            HolonomicDriveBinding binding = new HolonomicDriveBinding(
                    this::getFieldPose,
                    this::resetFieldPose,
                    () -> robotSpeeds.getSpeeds("drive"),
                    output,
                    translationConstants,
                    rotationConstants,
                    () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false));

            boolean configured = backend.configureHolonomic(binding);
            backend.warmupCommand(RobotAuto.AutoSource.PATH_PLANNER)
                    .ifPresent(CommandScheduler.getInstance()::schedule);
            if (!configured) {
                DriverStation.reportWarning("Auto backend failed to configure holonomic path following.", false);
            }
        }, () -> DriverStation.reportWarning("No auto backend found for PATH_PLANNER; skipping configuration.", false));
      return this;
    }

    public RobotLocalization<T> configureChoreo(Subsystem drivetrain){
        HolonomicPidConstants rotationConstants = localizationConfig.rotation();
        HolonomicPidConstants translationConstants = localizationConfig.translation();

        PIDController rotationController = new PIDController(rotationConstants.kP(), rotationConstants.kI(), rotationConstants.kD());
        rotationController.setIZone(rotationConstants.iZone());
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController translationController = new PIDController(translationConstants.kP(), translationConstants.kI(), translationConstants.kD());
        translationController.setIZone(translationConstants.iZone());

        ChoreoBinding binding = new ChoreoBinding(
                translationConstants,
                rotationConstants,
                this::getFieldPose,
                this::resetFieldPose,
                (desiredPose, desiredSpeeds) -> {
                    Pose2d botpose = getFieldPose();
                    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond,
                            desiredSpeeds.vyMetersPerSecond,
                            desiredSpeeds.omegaRadiansPerSecond);

                    fieldSpeeds.vxMetersPerSecond = translationController.calculate(botpose.getX(), desiredPose.getX());
                    fieldSpeeds.vyMetersPerSecond = translationController.calculate(botpose.getY(), desiredPose.getY());
                    fieldSpeeds.omegaRadiansPerSecond = rotationController.calculate(
                            botpose.getRotation().getRadians(),
                            desiredPose.getRotation().getRadians());

                    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldSpeeds,
                            botpose.getRotation());

                    robotSpeeds.setSpeeds("auto", robotRelative);
                },
                true,
                drivetrain);

        AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).ifPresentOrElse(backend -> {
            Optional<ChoreoAutoFactory> factory = backend.createChoreoFactory(binding);
            if (factory.isPresent()) {
                choreoFactory = factory.get();
            } else {
                DriverStation.reportWarning("Choreo backend is present but did not provide a factory.", false);
            }
        }, () -> DriverStation.reportWarning("No auto backend found for CHOREO; skipping configuration.", false));

      return this;
    }

    public Optional<ChoreoAutoFactory> getChoreoAutoFactory(){
        return Optional.ofNullable(choreoFactory);
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
        advanceVirtualPoseRevision();
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
        advanceVirtualPoseRevision();
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
        advanceVirtualPoseRevision();
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
        advanceVirtualPoseRevision();
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
        syncBackendOverridesFromShuffleboard();

        if (vision != null && isVisionEnabled()) {
            vision.setRobotOrientation(fieldPose);
            RobotLocalizationConfig.BackendConfig backend = backendConfig();
            if (backend.useMultiVision()) {
                fuseVisionMeasurements(vision.getVisionMeasurements(), backend).ifPresent(measurement -> {
                    if (applyVisionMeasurement(measurement)) {
                        hasAcceptedVisionMeasurement = true;
                        lastVisionMeasurementTimestamp = measurement.timestampSeconds();
                    }
                });
            } else {
                vision.getBestVisionMeasurement().ifPresent(measurement -> {
                    if (applyVisionMeasurement(measurement)) {
                        hasAcceptedVisionMeasurement = true;
                        lastVisionMeasurementTimestamp = measurement.timestampSeconds();
                    }
                });
            }
        }

        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Rotation3d fieldRotation = getRotation3d("field");
            fieldPose3d = fieldEstimator3d.update(fieldRotation, wheelPositions.get());
            fieldPose = fieldPose3d.toPose2d();
            Rotation3d relativeRotation = getRotation3d("relative");
            relativePose3d = relativeEstimator3d.update(relativeRotation, wheelPositions.get());
            relativePose = relativePose3d.toPose2d();
        } else {
            fieldPose = fieldEstimator2d.update(getYaw("field"), wheelPositions.get());
            fieldPose3d = new Pose3d(fieldPose);
            relativePose = relativeEstimator2d.update(getYaw("relative"), wheelPositions.get());
            relativePose3d = new Pose3d(relativePose);
        }

        field.setRobotPose(fieldPose);
        updateActualPath();
        publishFieldOnce();
        updateHealthMetrics();
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
        updateSlipState();
        advanceVirtualPoseRevision();
        persistRobotState(false);
    }

    public void updateAutoVisualization(RobotAuto autos) {
        if (autos == null) {
            clearPlannedPath();
            return;
        }
        String selectedId = autos.getSelectedAuto()
                .map(routine -> routine.key().id())
                .orElse(null);
        if (Objects.equals(selectedId, lastPlannedAutoId) && !isPlannedPathEmpty()) {
            return;
        }
        lastPlannedAutoId = selectedId;
        autos.getSelectedAutoPoses()
                .filter(list -> !list.isEmpty())
                .ifPresentOrElse(this::setPlannedPath, this::clearPlannedPath);
    }

    public void setPlannedPath(java.util.List<Pose2d> poses) {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        if (poses == null || poses.isEmpty()) {
            plannedPathObject.setPoses(java.util.List.of());
            return;
        }
        java.util.List<Pose2d> resolved = applyAllianceFlip(poses);
        resolved = downsamplePoses(resolved, plannedPathSpacingMeters);
        plannedPathObject.setPoses(resolved);
    }

    public void clearPlannedPath() {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        plannedPathObject.setPoses(java.util.List.of());
    }

    public void resetActualPath() {
        actualPathPoses.clear();
        lastActualPathPose = new Pose2d();
        lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
        if (actualPathObject == null) {
            actualPathObject = field.getObject("ActualPath");
        }
        actualPathObject.setPoses(java.util.List.of());
    }

    public void setPlannedPathSpacingMeters(double spacingMeters) {
        if (Double.isFinite(spacingMeters) && spacingMeters > 0.0) {
            plannedPathSpacingMeters = spacingMeters;
        }
    }

    public void setFieldDimensionsMeters(double lengthMeters, double widthMeters) {
        if (Double.isFinite(lengthMeters) && lengthMeters > 0.0) {
            fieldLengthMetersOverride = lengthMeters;
        }
        if (Double.isFinite(widthMeters) && widthMeters > 0.0) {
            fieldWidthMetersOverride = widthMeters;
        }
    }

    public void setActualPathSpacingMeters(double spacingMeters) {
        if (Double.isFinite(spacingMeters) && spacingMeters > 0.0) {
            actualPathSpacingMeters = spacingMeters;
        }
    }

    public void setActualPathMinIntervalSeconds(double minIntervalSeconds) {
        if (Double.isFinite(minIntervalSeconds) && minIntervalSeconds >= 0.0) {
            actualPathMinIntervalSeconds = minIntervalSeconds;
        }
    }

    public void setActualPathMaxPoints(int maxPoints) {
        if (maxPoints > 0) {
            actualPathMaxPoints = maxPoints;
        }
    }

    private void updateActualPath() {
        if (fieldPose == null) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (actualPathPoses.isEmpty()) {
            appendActualPose(fieldPose, now);
            return;
        }
        double distance = fieldPose.getTranslation().getDistance(lastActualPathPose.getTranslation());
        if (distance < actualPathSpacingMeters
                && now - lastActualPathTimestamp < actualPathMinIntervalSeconds) {
            return;
        }
        appendActualPose(fieldPose, now);
    }

    private void appendActualPose(Pose2d pose, double timestampSeconds) {
        if (actualPathObject == null) {
            actualPathObject = field.getObject("ActualPath");
        }
        actualPathPoses.add(pose);
        if (actualPathPoses.size() > actualPathMaxPoints) {
            actualPathPoses.remove(0);
        }
        lastActualPathPose = pose;
        lastActualPathTimestamp = timestampSeconds;
        actualPathObject.setPoses(actualPathPoses);
    }

    private boolean isPlannedPathEmpty() {
        return plannedPathObject == null
                || plannedPathObject.getPoses() == null
                || plannedPathObject.getPoses().isEmpty();
    }

    private void publishFieldOnce() {
        if (fieldPublished) {
            return;
        }
        SmartDashboard.putData("Field", field);
        fieldPublished = true;
    }

    private java.util.List<Pose2d> applyAllianceFlip(java.util.List<Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            return java.util.List.of();
        }
        boolean isRed =
                DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
        if (!isRed) {
            return poses;
        }
        double fieldLength = resolveFieldLengthMeters();
        double fieldWidth = resolveFieldWidthMeters();
        java.util.List<Pose2d> flipped = new java.util.ArrayList<>(poses.size());
        for (Pose2d pose : poses) {
            if (pose == null) {
                continue;
            }
            double x = fieldLength - pose.getX();
            double y = fieldWidth - pose.getY();
            Rotation2d rotation = pose.getRotation().plus(Rotation2d.fromDegrees(180.0));
            flipped.add(new Pose2d(x, y, rotation));
        }
        return flipped;
    }

    private double resolveFieldLengthMeters() {
        if (fieldLengthMetersOverride != null) {
            return fieldLengthMetersOverride;
        }
        if (vision != null) {
            for (LocalizationCamera camera : vision.getCameras().values()) {
                var layout = camera != null ? camera.getConfig().getFieldLayout() : null;
                if (layout != null) {
                    return layout.getFieldLength();
                }
            }
        }
        return 16.54;
    }

    private double resolveFieldWidthMeters() {
        if (fieldWidthMetersOverride != null) {
            return fieldWidthMetersOverride;
        }
        if (vision != null) {
            for (LocalizationCamera camera : vision.getCameras().values()) {
                var layout = camera != null ? camera.getConfig().getFieldLayout() : null;
                if (layout != null) {
                    return layout.getFieldWidth();
                }
            }
        }
        return 8.21;
    }

    private static java.util.List<Pose2d> downsamplePoses(java.util.List<Pose2d> poses, double spacingMeters) {
        if (poses == null || poses.isEmpty()) {
            return java.util.List.of();
        }
        if (!Double.isFinite(spacingMeters) || spacingMeters <= 0.0) {
            return poses;
        }
        java.util.List<Pose2d> result = new java.util.ArrayList<>();
        Pose2d last = null;
        for (Pose2d pose : poses) {
            if (pose == null) {
                continue;
            }
            if (last == null
                    || pose.getTranslation().getDistance(last.getTranslation()) >= spacingMeters) {
                result.add(pose);
                last = pose;
            }
        }
        return result;
    }

    private void updateHealthMetrics() {
        ensureHealthNetworkEntries();
        double now = Timer.getFPGATimestamp();
        if (Double.isFinite(lastHealthTimestamp)) {
            double dt = now - lastHealthTimestamp;
            if (dt > 0.0) {
                lastPoseJumpMeters = fieldPose.getTranslation().getDistance(lastHealthPose.getTranslation());
                driftRateMetersPerSec = lastPoseJumpMeters / dt;
            }
        }
        lastHealthPose = fieldPose;
        lastHealthTimestamp = now;

        double acceptanceRate = visionAcceptRateWindow;
        if (poseJumpEntry != null) {
            poseJumpEntry.setDouble(lastPoseJumpMeters);
        }
        if (poseJumpNtEntry != null) {
            poseJumpNtEntry.setDouble(lastPoseJumpMeters);
        }
        if (driftRateEntry != null) {
            driftRateEntry.setDouble(driftRateMetersPerSec);
        }
        if (driftRateNtEntry != null) {
            driftRateNtEntry.setDouble(driftRateMetersPerSec);
        }
        if (visionAcceptanceEntry != null) {
            visionAcceptanceEntry.setDouble(acceptanceRate);
        }
        if (visionAcceptanceNtEntry != null) {
            visionAcceptanceNtEntry.setDouble(acceptanceRate);
        }
        if (slipActiveEntry != null) {
            slipActiveEntry.setBoolean(slipActive);
        }
        if (slipActiveNtEntry != null) {
            slipActiveNtEntry.setBoolean(slipActive);
        }
    }

    private void updateSlipState() {
        RobotLocalizationConfig.BackendConfig backend = backendConfig();
        double now = Timer.getFPGATimestamp();
        if (Double.isFinite(lastUpdateTimestamp)) {
            double dt = now - lastUpdateTimestamp;
            if (dt > 0.0) {
                Rotation2d imuYawRate = imu.getVelocityZ();
                double imuYawRateRad = imuYawRate != null ? imuYawRate.getRadians() : Double.NaN;
                double estimatedYawRate =
                        fieldPose.getRotation()
                                .minus(lastFieldPoseForSlip.getRotation())
                                .getRadians() / dt;
                double yawRateRad = Double.isFinite(imuYawRateRad) ? imuYawRateRad : estimatedYawRate;
                double disagreement = Math.abs(yawRateRad - estimatedYawRate);
                boolean yawSlip =
                        Math.abs(yawRateRad) > backend.slipYawRateThreshold()
                                && disagreement > backend.slipYawRateDisagreement();

                Translation2d deltaTranslation =
                        fieldPose.getTranslation().minus(lastFieldPoseForSlip.getTranslation());
                Translation2d odomVelocity = deltaTranslation.times(1.0 / dt);
                Translation2d odomAccel = odomVelocity.minus(lastFieldVelocityForSlip).times(1.0 / dt);
                double odomAccelMag = odomAccel.getNorm();
                double imuAccelMag =
                        Math.hypot(imu.getAccelX(), imu.getAccelY());
                boolean accelSlip =
                        Double.isFinite(imuAccelMag)
                                && imuAccelMag > backend.slipAccelThreshold()
                                && Math.abs(imuAccelMag - odomAccelMag) > backend.slipAccelDisagreement();

                if (yawSlip || accelSlip) {
                    slipActiveUntilSeconds = now + backend.slipHoldSeconds();
                }
                lastFieldVelocityForSlip = odomVelocity;
            }
        }
        slipActive = now <= slipActiveUntilSeconds;
        lastUpdateTimestamp = now;
        lastFieldPoseForSlip = fieldPose;
        updateProcessStdDevScale(slipActive);
    }

    private void updateProcessStdDevScale(boolean slipActive) {
        if (processStdDevUpdateFailed) {
            return;
        }
        RobotLocalizationConfig.BackendConfig backend = backendConfig();
        double targetScale = slipActive ? backend.slipProcessStdDevScale() : 1.0;
        if (!Double.isFinite(targetScale) || targetScale <= 0.0) {
            targetScale = 1.0;
        }
        if (Math.abs(targetScale - processStdDevScale) < 1e-3) {
            return;
        }
        processStdDevScale = targetScale;
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            applyProcessStdDevs(fieldEstimator3d, baseStateStdDevs3d, targetScale);
            applyProcessStdDevs(relativeEstimator3d, baseStateStdDevs3d, targetScale);
            refreshVisionStdDevs3d();
        } else {
            applyProcessStdDevs(fieldEstimator2d, baseStateStdDevs2d, targetScale);
            applyProcessStdDevs(relativeEstimator2d, baseStateStdDevs2d, targetScale);
            refreshVisionStdDevs2d();
        }
    }

    private void refreshVisionStdDevs2d() {
        Matrix<N3, N1> std = localizationConfig.getVisionStd2d();
        if (fieldEstimator2d != null) {
            fieldEstimator2d.setVisionMeasurementStdDevs(std);
        }
        if (relativeEstimator2d != null) {
            relativeEstimator2d.setVisionMeasurementStdDevs(std);
        }
    }

    private void refreshVisionStdDevs3d() {
        Matrix<N4, N1> std = localizationConfig.getVisionStd3d();
        if (fieldEstimator3d != null) {
            fieldEstimator3d.setVisionMeasurementStdDevs(std);
        }
        if (relativeEstimator3d != null) {
            relativeEstimator3d.setVisionMeasurementStdDevs(std);
        }
    }

    private void applyProcessStdDevs(PoseEstimator<?> estimator, Matrix<N3, N1> base, double scale) {
        if (estimator == null || base == null || processStdDevUpdateFailed) {
            return;
        }
        try {
            Field field = poseEstimatorQField;
            if (field == null) {
                field = PoseEstimator.class.getDeclaredField("m_q");
                field.setAccessible(true);
                poseEstimatorQField = field;
            }
            Matrix<?, ?> q = (Matrix<?, ?>) field.get(estimator);
            for (int i = 0; i < 3; i++) {
                double value = base.get(i, 0) * scale;
                q.set(i, 0, value * value);
            }
        } catch (ReflectiveOperationException | RuntimeException ex) {
            processStdDevUpdateFailed = true;
            DriverStation.reportWarning("Failed to update pose estimator process noise: " + ex.getMessage(), false);
        }
    }

    private void applyProcessStdDevs(PoseEstimator3d<?> estimator, Matrix<N4, N1> base, double scale) {
        if (estimator == null || base == null || processStdDevUpdateFailed) {
            return;
        }
        try {
            Field field = poseEstimator3dQField;
            if (field == null) {
                field = PoseEstimator3d.class.getDeclaredField("m_q");
                field.setAccessible(true);
                poseEstimator3dQField = field;
            }
            Matrix<?, ?> q = (Matrix<?, ?>) field.get(estimator);
            for (int i = 0; i < 4; i++) {
                double value = base.get(i, 0) * scale;
                q.set(i, 0, value * value);
            }
        } catch (ReflectiveOperationException | RuntimeException ex) {
            processStdDevUpdateFailed = true;
            DriverStation.reportWarning("Failed to update pose estimator process noise: " + ex.getMessage(), false);
        }
    }

    public void addVirtualPose(String name, Supplier<Pose2d> supplier) {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(supplier, "supplier");
        virtualPoses2d.put(name, new VirtualPose2d(supplier));
    }

    public void addVirtualPose3d(String name, Supplier<Pose3d> supplier) {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(supplier, "supplier");
        virtualPoses3d.put(name, new VirtualPose3d(supplier));
    }

    public Pose2d getVirtualPose(String name) {
        VirtualPose2d virtualPose = virtualPoses2d.get(name);
        return virtualPose != null ? virtualPose.get(virtualPoseRevision) : new Pose2d();
    }

    public Pose3d getVirtualPose3d(String name) {
        VirtualPose3d virtualPose = virtualPoses3d.get(name);
        return virtualPose != null ? virtualPose.get(virtualPoseRevision) : new Pose3d();
    }

    public void setVirtualPose(String name, Pose2d pose) {
        VirtualPose2d virtualPose = virtualPoses2d.get(name);
        if (virtualPose != null && pose != null) {
            virtualPose.set(pose, virtualPoseRevision);
        }
    }

    public void setVirtualPose3d(String name, Pose3d pose) {
        VirtualPose3d virtualPose = virtualPoses3d.get(name);
        if (virtualPose != null && pose != null) {
            virtualPose.set(pose, virtualPoseRevision);
        }
    }

    public void invalidateVirtualPoses() {
        advanceVirtualPoseRevision();
    }

    private void advanceVirtualPoseRevision() {
        virtualPoseRevision++;
    }

    public FieldObject2d getField2dObject(String id){
        return field.getObject(id);
    }

    public Pose2d getFieldPose(){
        return fieldPose;
    }

    public RobotLocalizationConfig getLocalizationConfig() {
        return localizationConfig;
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
        if (poseJumpEntry == null) {
            var healthLayout = tab.getLayout("Localization Health", BuiltInLayouts.kList);
            poseJumpEntry = healthLayout.add("Pose Jump (m)", lastPoseJumpMeters)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
            driftRateEntry = healthLayout.add("Drift Rate (m/s)", driftRateMetersPerSec)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
            visionAcceptanceEntry = healthLayout.add("Vision Accept Rate", 0.0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
            slipActiveEntry = healthLayout.add("Slip Active", slipActive)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .getEntry();
        }
        if (backendOverrideToggleEntry == null && backendOverrideEnabledEntry != null) {
            var backendLayout = tab.getLayout("Localization Backend", BuiltInLayouts.kList);
            backendOverrideToggleEntry = backendLayout.add(
                            "Override Enabled", backendOverrideEnabledEntry.getBoolean(false))
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .getEntry();
            ensureBackendChoosers(backendConfig());
            backendLayout.add("Slip Strategy", backendSlipStrategyChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser);
            backendLayout.add("IMU Strategy", backendImuStrategyChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser);
            backendLayout.add("Vision Strategy", backendVisionStrategyChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser);
        }
        cameraManager.ensureCameraShuffleboardEntries(vision);
        return tab;
    }

    private void ensureHealthNetworkEntries() {
        if (poseJumpNtEntry != null) {
            return;
        }
        NetworkTable healthTable = NetworkTableInstance.getDefault().getTable("Athena/Localization/Health");
        poseJumpNtEntry = healthTable.getEntry("PoseJumpMeters");
        driftRateNtEntry = healthTable.getEntry("DriftRateMetersPerSec");
        visionAcceptanceNtEntry = healthTable.getEntry("VisionAcceptRate");
        slipActiveNtEntry = healthTable.getEntry("SlipActive");
    }

    private void ensureBackendChoosers(RobotLocalizationConfig.BackendConfig base) {
        RobotLocalizationConfig.BackendConfig resolved =
                base != null ? base : RobotLocalizationConfig.BackendConfig.defualt();
        if (backendSlipStrategyChooser == null) {
            backendSlipStrategyChooser = new SendableChooser<>();
            for (RobotLocalizationConfig.BackendConfig.SlipStrategy strategy :
                    RobotLocalizationConfig.BackendConfig.SlipStrategy.values()) {
                if (strategy == resolved.slipStrategy()) {
                    backendSlipStrategyChooser.setDefaultOption(strategy.name(), strategy);
                } else {
                    backendSlipStrategyChooser.addOption(strategy.name(), strategy);
                }
            }
        }
        if (backendImuStrategyChooser == null) {
            backendImuStrategyChooser = new SendableChooser<>();
            for (RobotLocalizationConfig.BackendConfig.ImuStrategy strategy :
                    RobotLocalizationConfig.BackendConfig.ImuStrategy.values()) {
                if (strategy == resolved.imuStrategy()) {
                    backendImuStrategyChooser.setDefaultOption(strategy.name(), strategy);
                } else {
                    backendImuStrategyChooser.addOption(strategy.name(), strategy);
                }
            }
        }
        if (backendVisionStrategyChooser == null) {
            backendVisionStrategyChooser = new SendableChooser<>();
            for (RobotLocalizationConfig.BackendConfig.VisionStrategy strategy :
                    RobotLocalizationConfig.BackendConfig.VisionStrategy.values()) {
                if (strategy == resolved.visionStrategy()) {
                    backendVisionStrategyChooser.setDefaultOption(strategy.name(), strategy);
                } else {
                    backendVisionStrategyChooser.addOption(strategy.name(), strategy);
                }
            }
        }
    }

    @Override
    public void periodic() {
       update();
    }

    private boolean applyVisionMeasurement(LocalizationCamera.VisionMeasurement measurement) {
        if (measurement == null) {
            return false;
        }
        visionMeasurementCount++;
        boolean accepted = false;
        Matrix<N3, N1> rawStdDevs = measurement.stdDevs();
        Matrix<N3, N1> sanitizedStdDevs = sanitizeVisionStdDevs(rawStdDevs);
        if (rawStdDevs != null && sanitizedStdDevs == null) {
            recordVisionSample(false);
            return false;
        }
        Pose2d measurementPose = measurement.pose2d();
        if (!passesPoseJumpGuard(measurementPose, false)) {
            recordVisionSample(false);
            return false;
        }
        double timestampSeconds = measurement.timestampSeconds();
        if (!shouldApplyVisionMeasurement(measurementPose, timestampSeconds, sanitizedStdDevs)) {
            recordVisionSample(false);
            return false;
        }
        Matrix<N3, N1> adjustedStdDevs2d =
                adjustVisionStdDevsForDisagreement(sanitizedStdDevs, measurementPose);
        if (slipActive) {
            double scale = backendConfig().slipVisionStdDevScale();
            if (scale > 0.0 && scale < 1.0) {
                adjustedStdDevs2d = scaleStdDevs(adjustedStdDevs2d, scale);
            }
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Matrix<N4, N1> adjustedStdDevs3d = expandStdDevsTo3d(adjustedStdDevs2d);
            Pose3d measurementPose3d = measurement.pose3d() != null
                    ? measurement.pose3d()
                    : new Pose3d(measurementPose);
            if (adjustedStdDevs3d != null) {
                fieldEstimator3d.addVisionMeasurement(measurementPose3d, timestampSeconds, adjustedStdDevs3d);
            } else {
                fieldEstimator3d.addVisionMeasurement(measurementPose3d, timestampSeconds);
            }
        } else {
            if (adjustedStdDevs2d != null) {
                fieldEstimator2d.addVisionMeasurement(measurementPose, timestampSeconds, adjustedStdDevs2d);
            } else {
                fieldEstimator2d.addVisionMeasurement(measurementPose, timestampSeconds);
            }
        }
        accepted = true;
        recordVisionSample(true);
        visionMeasurementAccepted++;
        return true;
    }

    private boolean passesPoseJumpGuard(Pose2d measurementPose, boolean hasAgreement) {
        RobotLocalizationConfig.BackendConfig backend = backendConfig();
        double now = Timer.getFPGATimestamp();
        if (!hasAgreement && now < poseJumpGuardUntilSeconds) {
            return false;
        }
        double threshold = backend.poseJumpMeters();
        if (threshold <= 0.0) {
            return true;
        }
        Pose2d referencePose = fieldPose != null ? fieldPose : new Pose2d();
        double distance = measurementPose.getTranslation().getDistance(referencePose.getTranslation());
        if (!hasAgreement && distance > threshold) {
            poseJumpGuardUntilSeconds = now + backend.poseJumpHoldSeconds();
            return false;
        }
        return true;
    }

    private void recordVisionSample(boolean accepted) {
        if (visionAcceptCount < VISION_ACCEPT_WINDOW) {
            visionAcceptCount++;
        } else if (visionAcceptWindow[visionAcceptIndex]) {
            visionAcceptAccepted--;
        }
        visionAcceptWindow[visionAcceptIndex] = accepted;
        if (accepted) {
            visionAcceptAccepted++;
        }
        visionAcceptIndex = (visionAcceptIndex + 1) % VISION_ACCEPT_WINDOW;
        visionAcceptRateWindow =
                visionAcceptCount > 0 ? (double) visionAcceptAccepted / visionAcceptCount : 0.0;
    }

    private Matrix<N3, N1> scaleStdDevs(Matrix<N3, N1> stdDevs, double scale) {
        if (stdDevs == null) {
            return null;
        }
        return VecBuilder.fill(
                stdDevs.get(0, 0) * scale,
                stdDevs.get(1, 0) * scale,
                stdDevs.get(2, 0) * scale);
    }

    private Optional<LocalizationCamera.VisionMeasurement> fuseVisionMeasurements(
            List<LocalizationCamera.VisionMeasurement> measurements,
            RobotLocalizationConfig.BackendConfig backend) {
        if (measurements == null || measurements.isEmpty()) {
            return Optional.empty();
        }
        boolean hasAgreement = hasPoseAgreement(measurements, backend.poseJumpAgreementMeters());
        measurements.sort(Comparator.comparingDouble(LocalizationCamera.VisionMeasurement::timestampSeconds));
        double latestTimestamp = measurements.get(measurements.size() - 1).timestampSeconds();
        double maxSeparation = backend.visionFusionMaxSeparationSeconds();
        double minWeight = backend.visionFusionMinWeight();

        double sumWeight = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumCos = 0.0;
        double sumSin = 0.0;
        double sumTimestamp = 0.0;
        double sumLatency = 0.0;
        double sumConfidence = 0.0;
        double sumDistance = 0.0;
        double sumStdX2 = 0.0;
        double sumStdY2 = 0.0;
        double sumStdTheta2 = 0.0;

        for (LocalizationCamera.VisionMeasurement measurement : measurements) {
            if (latestTimestamp - measurement.timestampSeconds() > maxSeparation) {
                continue;
            }
            Matrix<N3, N1> stdDevs = sanitizeVisionStdDevs(measurement.stdDevs());
            double baseWeight = 1.0;
            if (stdDevs != null) {
                double stdX = Math.max(stdDevs.get(0, 0), STD_EPSILON);
                double stdY = Math.max(stdDevs.get(1, 0), STD_EPSILON);
                double stdTheta = Math.max(stdDevs.get(2, 0), STD_EPSILON);
                baseWeight = 1.0 / (stdX + stdY + stdTheta);
            }
            double confidence = measurement.confidence();
            double confidenceWeight = Double.isFinite(confidence)
                    ? Math.pow(Math.max(0.0, Math.min(1.0, confidence)), backend.visionFusionConfidenceExponent())
                    : 1.0;
            double latencyWeight = 1.0;
            if (backend.visionFusionLatencyWeight() > 0.0) {
                latencyWeight = 1.0 / (1.0 + measurement.latencySeconds() * backend.visionFusionLatencyWeight());
            }
            double distanceWeight = 1.0;
            if (backend.visionFusionDistanceWeight() > 0.0 && Double.isFinite(measurement.distanceMeters())) {
                distanceWeight = 1.0 / (1.0 + measurement.distanceMeters() * backend.visionFusionDistanceWeight());
            }
            double cameraWeight = measurement.weightMultiplier();
            if (!Double.isFinite(cameraWeight) || cameraWeight <= 0.0) {
                cameraWeight = 1.0;
            }
            double weight = baseWeight * confidenceWeight * latencyWeight * distanceWeight * cameraWeight;
            if (!Double.isFinite(weight) || weight < minWeight) {
                continue;
            }

            Pose2d pose = measurement.pose2d();
            double theta = pose.getRotation().getRadians();
            sumWeight += weight;
            sumX += pose.getX() * weight;
            sumY += pose.getY() * weight;
            sumCos += Math.cos(theta) * weight;
            sumSin += Math.sin(theta) * weight;
            sumTimestamp += measurement.timestampSeconds() * weight;
            sumLatency += measurement.latencySeconds() * weight;
            sumConfidence += (Double.isFinite(confidence) ? confidence : 1.0) * weight;
            sumDistance += (Double.isFinite(measurement.distanceMeters()) ? measurement.distanceMeters() : 0.0) * weight;
            if (stdDevs != null) {
                sumStdX2 += stdDevs.get(0, 0) * stdDevs.get(0, 0) * weight;
                sumStdY2 += stdDevs.get(1, 0) * stdDevs.get(1, 0) * weight;
                sumStdTheta2 += stdDevs.get(2, 0) * stdDevs.get(2, 0) * weight;
            }
        }

        if (sumWeight <= 0.0) {
            return Optional.empty();
        }

        Pose2d fusedPose = new Pose2d(
                sumX / sumWeight,
                sumY / sumWeight,
                new Rotation2d(Math.atan2(sumSin / sumWeight, sumCos / sumWeight)));
        if (!passesPoseJumpGuard(fusedPose, hasAgreement)) {
            return Optional.empty();
        }
        double fusedTimestamp = sumTimestamp / sumWeight;
        double fusedLatency = sumLatency / sumWeight;
        double fusedConfidence = sumConfidence / sumWeight;
        double fusedDistance = sumDistance / sumWeight;

        Matrix<N3, N1> fusedStdDevs = null;
        if (sumStdX2 > 0.0 || sumStdY2 > 0.0 || sumStdTheta2 > 0.0) {
            fusedStdDevs = VecBuilder.fill(
                    Math.sqrt(sumStdX2 / sumWeight),
                    Math.sqrt(sumStdY2 / sumWeight),
                    Math.sqrt(sumStdTheta2 / sumWeight));
        }

        return Optional.of(new LocalizationCamera.VisionMeasurement(
                fusedPose,
                new Pose3d(fusedPose),
                fusedTimestamp,
                fusedLatency,
                fusedStdDevs,
                fusedConfidence,
                fusedDistance,
                1.0));
    }

    private boolean hasPoseAgreement(
            List<LocalizationCamera.VisionMeasurement> measurements,
            double agreementMeters) {
        if (measurements == null || measurements.size() < 2 || agreementMeters <= 0.0) {
            return false;
        }
        for (int i = 0; i < measurements.size(); i++) {
            Pose2d poseA = measurements.get(i).pose2d();
            if (poseA == null) {
                continue;
            }
            for (int j = i + 1; j < measurements.size(); j++) {
                Pose2d poseB = measurements.get(j).pose2d();
                if (poseB == null) {
                    continue;
                }
                double distance =
                        poseA.getTranslation().getDistance(poseB.getTranslation());
                if (distance <= agreementMeters) {
                    return true;
                }
            }
        }
        return false;
    }

    private static final class VirtualPose2d {
        private final Supplier<Pose2d> baseSupplier;
        private Transform2d offset = new Transform2d();
        private Pose2d cachedPose = new Pose2d();
        private long cachedRevision = Long.MIN_VALUE;

        private VirtualPose2d(Supplier<Pose2d> baseSupplier) {
            this.baseSupplier = baseSupplier;
        }

        private Pose2d get(long revision) {
            if (cachedRevision != revision) {
                Pose2d base = safePose(baseSupplier.get());
                cachedPose = base.transformBy(offset);
                cachedRevision = revision;
            }
            return cachedPose;
        }

        private void set(Pose2d desiredPose, long revision) {
            Pose2d base = safePose(baseSupplier.get());
            Pose2d safeDesired = safePose(desiredPose);
            offset = new Transform2d(base, safeDesired);
            cachedPose = safeDesired;
            cachedRevision = revision;
        }

        private static Pose2d safePose(Pose2d pose) {
            return pose != null ? pose : new Pose2d();
        }
    }

    private static final class VirtualPose3d {
        private final Supplier<Pose3d> baseSupplier;
        private Transform3d offset = new Transform3d();
        private Pose3d cachedPose = new Pose3d();
        private long cachedRevision = Long.MIN_VALUE;

        private VirtualPose3d(Supplier<Pose3d> baseSupplier) {
            this.baseSupplier = baseSupplier;
        }

        private Pose3d get(long revision) {
            if (cachedRevision != revision) {
                Pose3d base = safePose(baseSupplier.get());
                cachedPose = base.transformBy(offset);
                cachedRevision = revision;
            }
            return cachedPose;
        }

        private void set(Pose3d desiredPose, long revision) {
            Pose3d base = safePose(baseSupplier.get());
            Pose3d safeDesired = safePose(desiredPose);
            offset = new Transform3d(base, safeDesired);
            cachedPose = safeDesired;
            cachedRevision = revision;
        }

        private static Pose3d safePose(Pose3d pose) {
            return pose != null ? pose : new Pose3d();
        }
    }

}
