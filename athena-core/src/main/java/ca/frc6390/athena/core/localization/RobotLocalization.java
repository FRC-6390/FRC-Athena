package ca.frc6390.athena.core.localization;

import java.lang.reflect.Field;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotTime;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.ChoreoAutoFactory;
import ca.frc6390.athena.core.auto.ChoreoBinding;
import ca.frc6390.athena.core.auto.HolonomicDriveBinding;
import ca.frc6390.athena.core.auto.HolonomicFeedforward;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class RobotLocalization<T> extends SubsystemBase implements RobotSendableSystem{

    private final PoseEstimatorFactory<T> estimatorFactory;
    private RobotVision vision;
    private final Imu imu;
    private final RobotSpeeds robotSpeeds;
    private final Supplier<T> wheelPositions;

    private Pose2d fieldPose;
    private Pose3d fieldPose3d;
    private static final double STD_EPSILON = 1e-5;
    private static final Rotation2d ZERO_ROTATION = new Rotation2d();
    private static final Pose2d ZERO_POSE_2D = new Pose2d();
    private static final Pose3d ZERO_POSE_3D = new Pose3d();
    private static final String AUTO_PLANNER_PID_AUTOTUNER_OUTPUT_SOURCE = "auto_tuner";
    private static final double CHOREO_HEADING_DEADBAND_RAD = Units.degreesToRadians(1.5);
    private static final double CHOREO_OMEGA_DEADBAND_RAD_PER_SEC = 0.05;
    private static final double SLIP_DT_MAX_SECONDS = 0.25;
    private static final double SLIP_YAW_TRANSLATION_GATE_MPS = 0.25;
    private static final double SLIP_ACCEL_TRANSLATION_GATE_MPS = 0.15;
    private static final double IMU_LINEAR_OBSERVED_SPEED_MPS = 0.15;
    private static final double IMU_LINEAR_OBSERVED_ACCEL_MPS2 = 0.75;
    private static final double IMU_ACCEL_NOISE_FLOOR_MPS2 = 0.10;
    private static final double REVERSAL_COMMAND_THRESHOLD_MPS = 0.8;
    private static final double REVERSAL_ODOM_ACCEL_THRESHOLD_MPS2 = 2.5;
    private static final double REVERSAL_IMU_ACCEL_MIN_MPS2 = 0.75;
    private static final double REVERSAL_COMMAND_FULL_SCALE_MPS = 2.2;
    private static final double WHEELSPIN_COMMAND_SPEED_MPS = 0.8;
    private static final double WHEELSPIN_COMMAND_SUSTAIN_MPS = 0.18;
    private static final double WHEELSPIN_ODOM_SPEED_MPS = 1.0;
    private static final double WHEELSPIN_ODOM_SUSTAIN_MPS = 0.15;
    private static final double WHEELSPIN_IMU_SPEED_RATIO = 0.30;
    private static final double WHEELSPIN_IMU_SPEED_FLOOR_MPS = 0.25;
    private static final double WHEELSPIN_DEFICIT_RATIO_START = 0.80;
    private static final double WHEELSPIN_DEFICIT_RATIO_FULL = 0.40;
    private static final double SLIP_SCORE_FILTER_RISE_TIME_SECONDS = 0.06;
    private static final double SLIP_SCORE_FILTER_FALL_TIME_SECONDS = 0.20;
    private static final double SLIP_EVIDENCE_FILTER_RISE_TIME_SECONDS = 0.05;
    private static final double SLIP_CONTACT_EVIDENCE_FALL_TIME_SECONDS = 0.42;
    private static final double SLIP_REVERSAL_EVIDENCE_FALL_TIME_SECONDS = 0.36;
    private static final double SLIP_TRANSIENT_ENTER_SCORE = 0.35;
    private static final double SLIP_TRANSIENT_EXIT_SCORE = 0.18;
    private static final double SLIP_ACTIVE_ENTER_SCORE = 0.62;
    private static final double SLIP_ACTIVE_EXIT_SCORE = 0.28;
    private static final double SLIP_WEIGHT_YAW = 0.22;
    private static final double SLIP_WEIGHT_ACCEL = 0.20;
    private static final double SLIP_WEIGHT_REVERSAL = 0.23;
    private static final double SLIP_WEIGHT_CONTACT = 0.35;
    private static final double REVERSAL_OPPOSING_VELOCITY_MAX_MPS = 0.20;
    private static final double REVERSAL_OPPOSING_VELOCITY_MIN_MPS = 0.02;
    private static final double CONTACT_LOCK_IMU_SPEED_MPS = 0.16;
    private static final double CONTACT_ASSIST_COMPONENT = 0.45;
    private static final double CONTACT_LOCK_ENTER_COMPONENT = 0.55;
    private static final double CONTACT_LOCK_MAX_ODOM_WEIGHT = 0.08;
    private static final double REVERSAL_INERTIA_IMU_OPPOSE_MPS = 0.05;
    private static final double REVERSAL_NEW_DIRECTION_ALLOWANCE_MPS = 0.06;
    private static final double SLIP_REVERSAL_SUSTAIN_COMPONENT = 0.20;
    private static final int POSE_STRUCT_PRUNE_PER_CYCLE = 8;
    private static final int BOUNDING_BOX_STATIC_PUBLISH_PERIOD = 10;

    private enum SlipMode {
        NOMINAL,
        TRANSIENT,
        SLIP,
        RECOVER
    }

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private ChoreoAutoFactory choreoFactory;

    private BiConsumer<ChassisSpeeds, HolonomicFeedforward> autoDrive;

    private boolean suppressUpdates = false;
    private NetworkTable backendTable;
    private NetworkTableEntry backendOverrideEnabledEntry;
    private NetworkTableEntry backendImuStrategyEntry;
    private NetworkTableEntry backendVisionStrategyEntry;

    private double visionMaxLatencySeconds = 0.9;
    private double visionOutlierTranslationMeters = 2.5;
    private double visionOutlierRotationRadians = Units.degreesToRadians(35.0);
    private double visionStdDevOutlierMultiplier = 4.0;
    private double visionStdDevTranslationLimit = 5.0;
    private double visionStdDevRotationLimit = Units.degreesToRadians(120.0);
    private double minVisionUpdateSeparationSeconds = 0.02;
    private final RobotLocalizationConfig.PoseSpace poseSpace;
    private final Map<String, PoseConfig> poseConfigs = new HashMap<>();
    private final Map<String, PoseEstimatorState> poseStates = new HashMap<>();
    private final Map<String, StructPublisher<Pose2d>> poseStructPublishers = new HashMap<>();
    private final ArrayDeque<String> poseStructPruneQueue = new ArrayDeque<>();
    private final LinkedHashMap<String, PoseBoundingBox2d> namedBoundingBoxes = new LinkedHashMap<>();
    private final Map<String, StructArrayPublisher<Pose2d>> boundingBoxPublishers = new HashMap<>();
    private final Map<String, Pose2d[]> boundingBoxTrajectoryCache = new HashMap<>();
    private final Map<String, PoseBoundingBox2d> boundingBoxTrajectorySources = new HashMap<>();
    private final Map<String, PoseBoundingBox2d> boundingBoxMetadataPublished = new HashMap<>();
    private final Map<String, Boolean> boundingBoxContainsPublished = new HashMap<>();
    private final Map<String, Pose2d[]> boundingBoxTrajectoryPublished = new HashMap<>();
    private final Map<String, Pose2d> fieldPoseObjectsBuffer = new LinkedHashMap<>();
    private int boundingBoxPublishCycle = 0;
    private String primaryPoseName;
    private boolean slipActive = false;
    private double lastUpdateTimestamp = Double.NaN;
    private Pose2d lastFieldPoseForSlip = new Pose2d();
    private Translation2d lastFieldVelocityForSlip = new Translation2d();
    private double lastCommandedVxForSlip = 0.0;
    private double lastCommandedVyForSlip = 0.0;
    private boolean imuLinearSignalObserved = false;
    private double slipScore = 0.0;
    private double slipScoreRaw = 0.0;
    private double slipContactComponent = 0.0;
    private double slipReversalComponent = 0.0;
    private SlipMode slipMode = SlipMode.NOMINAL;
    private Pose2d lastRawEstimatorPoseForSlipFusion = new Pose2d();
    private Pose2d lastCorrectedPoseForSlipFusion = new Pose2d();
    private double lastSlipFusionTimestamp = Double.NaN;
    private String lastSlipCause = "none";
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
    private NetworkTableEntry poseJumpNtEntry;
    private NetworkTableEntry driftRateNtEntry;
    private NetworkTableEntry visionAcceptanceNtEntry;
    private NetworkTableEntry slipActiveNtEntry;
    private NetworkTableEntry slipCauseNtEntry;
    private NetworkTableEntry slipScoreNtEntry;
    private NetworkTableEntry slipModeNtEntry;
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private final ChassisSpeeds commandedSpeedsScratchA = new ChassisSpeeds();
    private final ChassisSpeeds commandedSpeedsScratchB = new ChassisSpeeds();
    private double normalizedMovementSpeed = 0.0;
    private Pose2d lastVelocityPose = new Pose2d();
    private double lastVelocityTimestamp = Double.NaN;

    private final RobotLocalizationPersistence persistence;
    private final RobotVisionCameraManager cameraManager;
    private RobotNetworkTables robotNetworkTables;
    private boolean restoringPersistentState = false;
    private final Matrix<N3, N1> baseStateStdDevs2d;
    private final Matrix<N4, N1> baseStateStdDevs3d;
    private double processStdDevScale = 1.0;
    private boolean processStdDevUpdateFailed = false;
    private static Field poseEstimatorQField;
    private static Field poseEstimator3dQField;
    private final RobotLocalizationFieldPublisher fieldPublisher;
    private DiagnosticsChannel diagnosticsChannel;
    private final DiagnosticsView diagnosticsView = new DiagnosticsView();
    private Subsystem autoPlannerPidAutotunerRequirement;

    public RobotLocalization(
            PoseEstimatorFactory<T> estimatorFactory,
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
        this.estimatorFactory = Objects.requireNonNull(estimatorFactory, "estimatorFactory");
        this.baseStateStdDevs2d = config.getStd2d();
        this.baseStateStdDevs3d = config.getStd3d();
        this.fieldPose = ZERO_POSE_2D;
        this.fieldPose3d = ZERO_POSE_3D;
        this.autoDrive = (speeds, feed) -> robotSpeeds.setSpeeds("auto", speeds);

        this.fieldPublisher = new RobotLocalizationFieldPublisher(() -> vision);
        this.cameraManager = new RobotVisionCameraManager(STD_EPSILON, fieldPublisher.getField());
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            // No extra initialization required.
        }

        this.visionEnabled = config.isVisionEnabled();

        imu.addVirtualAxis("field", imu::getYaw);
        imu.setVirtualAxis("field", new Rotation2d());

        if (config.rotation() != null && config.translation() != null
                && AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).isPresent()) {
            configurePathPlanner(config.translation(), config.rotation());
        }

        this.persistence = new RobotLocalizationPersistence(this.poseSpace);
        persistence.updateSnapshot(fieldPose, fieldPose3d, imu.getVirtualAxis("driver"));
        resetVelocityTracking(fieldPose);
        List<PoseConfig> configs = config.poseConfigs();
        if (configs.isEmpty()) {
            registerPoseConfig(defaultPoseConfig());
        } else {
            for (PoseConfig poseConfig : configs) {
                registerPoseConfig(poseConfig);
            }
        }
        for (RobotLocalizationConfig.NamedBoundingBox namedBox : config.boundingBoxes()) {
            if (namedBox != null) {
                zone(namedBox.name(), namedBox.box());
            }
        }
        loadPersistentState();
        initBackendOverrides();
        publishAutoPlannerPidAutotuner();
    }

    public RobotLocalization<T> enableVisionForLocalization(boolean visionEnabled){
        boolean changed = this.visionEnabled != visionEnabled;
        this.visionEnabled = visionEnabled;
        if (changed) {
            appendDiagnosticLog(
                    "INFO",
                    "config",
                    visionEnabled ? "vision localization enabled" : "vision localization disabled");
        }
        return this;
    }

    public RobotLocalization<T> setRobotVision(RobotVision vision){
        this.vision = vision;
        appendDiagnosticLog(
                "INFO",
                "lifecycle",
                vision != null ? "vision attached to localization" : "vision detached from localization");
        if (vision != null) {
            vision.attachLocalization(this);
            vision.measurements().localizationStdDevs(
                    localizationConfig.getVisionStd(),
                    localizationConfig.getVisionMultitagStd());
        }
        cameraManager.ensureCameraEntries(vision);
        return this;
    }

    public RobotLocalization<T> diagnostics(DiagnosticsChannel channel) {
        this.diagnosticsChannel = channel;
        if (channel != null) {
            appendDiagnosticLog("INFO", "lifecycle", "diagnostics channel attached");
        }
        return this;
    }

    public DiagnosticsView diagnostics() {
        return diagnosticsView;
    }

    public RobotLocalization<T> setAutoPlannerPidAutotunerRequirement(Subsystem requirement) {
        this.autoPlannerPidAutotunerRequirement = requirement;
        publishAutoPlannerPidAutotuner();
        return this;
    }

    /**
     * Attaches the RobotCore-owned NetworkTables router so localization can honor the same runtime
     * gating flags and publish config topics under Athena/NetworkTableConfig.
     */
    public void attachRobotNetworkTables(RobotNetworkTables robotNetworkTables) {
        this.robotNetworkTables = robotNetworkTables;
        cameraManager.attachRobotNetworkTables(robotNetworkTables);
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
        return RobotLocalizationVisionUtil.sanitizeVisionStdDevs(
                stdDevs,
                visionStdDevTranslationLimit,
                visionStdDevRotationLimit,
                STD_EPSILON);
    }

    private boolean shouldApplyVisionMeasurement(
            PoseEstimatorState state,
            Pose2d referencePose,
            Pose2d visionPose,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs) {
        return RobotLocalizationVisionUtil.shouldApplyVisionMeasurement(
                visionPose,
                timestampSeconds,
                stdDevs,
                state.hasAcceptedVisionMeasurement,
                state.lastVisionMeasurementTimestamp,
                minVisionUpdateSeparationSeconds,
                visionMaxLatencySeconds,
                referencePose,
                visionOutlierTranslationMeters,
                visionOutlierRotationRadians,
                visionStdDevOutlierMultiplier,
                STD_EPSILON);
    }

    private Matrix<N3, N1> adjustVisionStdDevsForDisagreement(
            Matrix<N3, N1> stdDevs,
            Pose2d visionPose,
            Pose2d referencePose) {
        return RobotLocalizationVisionUtil.adjustVisionStdDevsForDisagreement(
                stdDevs,
                visionPose,
                referencePose,
                visionOutlierTranslationMeters,
                visionOutlierRotationRadians,
                visionStdDevTranslationLimit,
                visionStdDevRotationLimit,
                STD_EPSILON);
    }

    private Matrix<N4, N1> expandStdDevsTo3d(Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) {
            return null;
        }
        double zStd = RobotLocalizationVisionUtil.clampStdDev(
                localizationConfig.getVisionStd3d().get(2, 0),
                visionStdDevTranslationLimit);
        return RobotLocalizationVisionUtil.expandStdDevsTo3d(stdDevs, zStd);
    }

    private void persistRobotState(boolean force) {
        Pose2d currentFieldPose = fieldPose != null ? fieldPose : ZERO_POSE_2D;
        Pose3d currentFieldPose3d = fieldPose3d != null ? fieldPose3d : new Pose3d(currentFieldPose);
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
                    resetPose(localizationConfig.autoPoseName(), state.pose3d());
                } else {
                    resetPose(localizationConfig.autoPoseName(), state.pose2d());
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

    private Rotation3d getRotation3d(String axisId, RobotLocalizationConfig.BackendConfig backend) {
        Rotation2d yaw = getYaw(axisId, backend);
        Rotation2d roll = imu.getRoll();
        Rotation2d pitch = imu.getPitch();
        return new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
    }

    private Rotation2d getYaw(String axisId, RobotLocalizationConfig.BackendConfig backend) {
        RobotLocalizationConfig.BackendConfig resolved =
                backend != null ? backend : RobotLocalizationConfig.BackendConfig.defaults();
        if (resolved.imuStrategy() == RobotLocalizationConfig.BackendConfig.ImuStrategy.RAW_YAW) {
            return imu.getYaw();
        }
        return imu.getVirtualAxis(axisId);
    }

    private RobotLocalizationConfig.BackendConfig backendConfig() {
        RobotLocalizationConfig.BackendConfig base =
                localizationConfig != null ? localizationConfig.backend() : RobotLocalizationConfig.BackendConfig.defaults();
        if (backendOverrideEnabledEntry != null && backendOverrideEnabledEntry.getBoolean(false)) {
            return applyBackendOverride(base);
        }
        return base;
    }

    private boolean isVisionEnabled() {
        return backendConfig().resolveVisionEnabled(visionEnabled);
    }

    private boolean isVisionEnabled(RobotLocalizationConfig.BackendConfig backend) {
        RobotLocalizationConfig.BackendConfig resolved =
                backend != null ? backend : RobotLocalizationConfig.BackendConfig.defaults();
        return resolved.resolveVisionEnabled(visionEnabled);
    }

    private void initBackendOverrides() {
        backendTable = NetworkTableInstance.getDefault().getTable("Athena/Localization/Backend");
        backendOverrideEnabledEntry = backendTable.getEntry("EnableOverride");
        backendImuStrategyEntry = backendTable.getEntry("ImuStrategy");
        backendVisionStrategyEntry = backendTable.getEntry("VisionStrategy");

        backendOverrideEnabledEntry.setBoolean(false);
        RobotLocalizationConfig.BackendConfig base = backendConfig();
        backendImuStrategyEntry.setString(base.imuStrategy().name());
        backendVisionStrategyEntry.setString(base.visionStrategy().name());
    }

    // Backend override values are edited directly via NetworkTables under Athena/Localization/Backend.

    private RobotLocalizationConfig.BackendConfig applyBackendOverride(RobotLocalizationConfig.BackendConfig base) {
        RobotLocalizationConfig.BackendConfig.ImuStrategy imuStrategy =
                parseEnum(backendImuStrategyEntry.getString(base.imuStrategy().name()), base.imuStrategy());
        RobotLocalizationConfig.BackendConfig.VisionStrategy visionStrategy =
                parseEnum(backendVisionStrategyEntry.getString(base.visionStrategy().name()), base.visionStrategy());

        return base
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

    private void publishAutoPlannerPidAutotuner() {
        RobotLocalizationConfig.AutoPlannerPidAutotunerConfig tunerConfig = localizationConfig.autoPlannerPidAutotuner();
        if (!tunerConfig.enabled()) {
            return;
        }

        Subsystem requirement = autoPlannerPidAutotunerRequirement != null
                ? autoPlannerPidAutotunerRequirement
                : this;
        String dashboardPath = tunerConfig.dashboardPath();
        AutoPlannerPidAutotuners.initializeDashboard(dashboardPath);

        AutoPlannerPidAutotunerProgram configuredProgram = tunerConfig.program();
        AutoPlannerPidAutotunerProgram program = configuredProgram != null
                ? configuredProgram
                : AutoPlannerPidAutotunerContext::relayTheta;

        AutoPlannerPidAutotunerContext ctx = new AutoPlannerPidAutotunerContext() {
            @Override
            public RobotLocalization<?> localization() {
                return RobotLocalization.this;
            }

            @Override
            public Subsystem requirement() {
                return requirement;
            }

            @Override
            public String dashboardPath() {
                return dashboardPath;
            }

            @Override
            public String outputSource() {
                return AUTO_PLANNER_PID_AUTOTUNER_OUTPUT_SOURCE;
            }

            @Override
            public HolonomicPidConstants translationPid() {
                return localizationConfig.translation();
            }

            @Override
            public HolonomicPidConstants rotationPid() {
                return localizationConfig.rotation();
            }

            @Override
            public void output(ChassisSpeeds speeds) {
                robotSpeeds.setSpeeds(AUTO_PLANNER_PID_AUTOTUNER_OUTPUT_SOURCE, speeds);
            }

            @Override
            public void stopOutput() {
                robotSpeeds.stopSpeeds(AUTO_PLANNER_PID_AUTOTUNER_OUTPUT_SOURCE);
            }
        };

        Command built = program.build(ctx);
        if (built == null) {
            DriverStation.reportWarning(
                    "AutoPlanner PID autotuner program returned null command; skipping publish.",
                    false);
            return;
        }

        Command guarded = Commands.defer(
                () -> Commands.either(
                        built,
                        Commands.runOnce(() -> DriverStation.reportWarning(
                                "AutoPlanner PID autotuner can only run in Test mode.",
                                false)),
                        DriverStation::isTest),
                java.util.Set.of(requirement));
        SmartDashboard.putData(dashboardPath + "/Run", guarded);
    }

    public RobotLocalization<T> configurePathPlanner(HolonomicPidConstants translationConstants, HolonomicPidConstants rotationConstants){
      return configurePathPlanner(translationConstants, rotationConstants, autoDrive);
    }

    public RobotLocalization<T> configurePathPlanner(HolonomicPidConstants translationConstants, HolonomicPidConstants rotationConstants, BiConsumer<ChassisSpeeds, HolonomicFeedforward> output){
        AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).ifPresentOrElse(backend -> {
            HolonomicDriveBinding binding = new HolonomicDriveBinding(
                    this::getFieldPose,
                    pose -> resetPose(localizationConfig.autoPoseName(), pose),
                    this::getRobotRelativeSpeeds,
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
        String autoHeadingAxis = localizationConfig.autoPoseName();
        if (autoHeadingAxis == null || autoHeadingAxis.isBlank()) {
            autoHeadingAxis = "field";
        }
        final String headingAxis = autoHeadingAxis;

        PIDController rotationController = new PIDController(rotationConstants.kP(), rotationConstants.kI(), rotationConstants.kD());
        rotationController.setIZone(rotationConstants.iZone());
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xController = new PIDController(
                translationConstants.kP(),
                translationConstants.kI(),
                translationConstants.kD());
        xController.setIZone(translationConstants.iZone());
        PIDController yController = new PIDController(
                translationConstants.kP(),
                translationConstants.kI(),
                translationConstants.kD());
        yController.setIZone(translationConstants.iZone());

        ChoreoBinding binding = new ChoreoBinding(
                translationConstants,
                rotationConstants,
                this::getFieldPose,
                pose -> resetPose(localizationConfig.autoPoseName(), pose),
                (desiredPose, desiredSpeeds) -> {
                    Pose2d botpose = getFieldPose();
                    Rotation2d measuredHeading = getYaw(headingAxis, backendConfig());
                    double desiredHeadingRadians = desiredPose.getRotation().getRadians();
                    double measuredHeadingRadians = measuredHeading.getRadians();
                    double headingErrorRadians = MathUtil.inputModulus(
                            desiredHeadingRadians - measuredHeadingRadians,
                            -Math.PI,
                            Math.PI);
                    double xFeedback = xController.calculate(botpose.getX(), desiredPose.getX());
                    double yFeedback = yController.calculate(botpose.getY(), desiredPose.getY());
                    double thetaFeedback = 0.0;
                    if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > CHOREO_OMEGA_DEADBAND_RAD_PER_SEC
                            || Math.abs(headingErrorRadians) > CHOREO_HEADING_DEADBAND_RAD) {
                        thetaFeedback = rotationController.calculate(measuredHeadingRadians, desiredHeadingRadians);
                    }
                    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
                            desiredSpeeds.vxMetersPerSecond + xFeedback,
                            desiredSpeeds.vyMetersPerSecond + yFeedback,
                            desiredSpeeds.omegaRadiansPerSecond + thetaFeedback);

                    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldSpeeds,
                            measuredHeading);

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

    public void resetPose(String name, Pose2d pose) {
        if (pose == null) {
            return;
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetPose(name, new Pose3d(pose));
            return;
        }
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return;
        }
        PoseEstimatorState state = ensurePoseState(config);
        PoseEstimator<T> estimator = state.estimator2d;
        if (estimator == null) {
            return;
        }
        estimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        state.pose2d = pose;
        state.pose3d = new Pose3d(pose);
        if (isPrimaryPose(config)) {
            imu.setVirtualAxis("field", pose.getRotation());
            setPrimaryPose(state);
            resetVelocityTracking(pose);
            resetVisionTracking(state);
            persistRobotState(true);
        }
        publishPoseStruct(config, pose);
    }

    public void resetPose(String name, Pose3d pose) {
        if (pose == null) {
            return;
        }
        if (poseSpace != RobotLocalizationConfig.PoseSpace.THREE_D) {
            resetPose(name, pose.toPose2d());
            return;
        }
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return;
        }
        PoseEstimatorState state = ensurePoseState(config);
        PoseEstimator3d<T> estimator = state.estimator3d;
        if (estimator == null) {
            return;
        }
        estimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        Pose2d pose2d = pose.toPose2d();
        state.pose3d = pose;
        state.pose2d = pose2d;
        if (isPrimaryPose(config)) {
            imu.setVirtualAxis("field", pose2d.getRotation());
            setPrimaryPose(state);
            resetVelocityTracking(pose2d);
            resetVisionTracking(state);
            persistRobotState(true);
        }
        publishPoseStruct(config, pose2d);
    }

    public void resetPose(String name, double x, double y) {
        resetPose(name, new Pose2d(x, y, new Rotation2d()));
    }

    public void resetPose(String name, double x, double y, double thetaDegrees) {
        resetPose(name, new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees)));
    }

    public void setVisionStd(Matrix<N3, N1> matrix){
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Matrix<N4, N1> expanded = expandStdDevsTo3d(matrix);
            if (expanded != null) {
                for (PoseEstimatorState state : poseStates.values()) {
                    PoseEstimator3d<T> estimator = state.estimator3d;
                    if (estimator != null) {
                        estimator.setVisionMeasurementStdDevs(expanded);
                    }
                }
            }
        } else {
            for (PoseEstimatorState state : poseStates.values()) {
                PoseEstimator<T> estimator = state.estimator2d;
                if (estimator != null) {
                    estimator.setVisionMeasurementStdDevs(matrix);
                }
            }
        }
    }

    public void setVisionStd(double x, double y, double theta){
        setVisionStd(VecBuilder.fill(x,y,Units.degreesToRadians(theta)));
    }


    public void updateAutoVisualization(RobotAuto autos) {
        fieldPublisher.updateAutoVisualization(autos);
    }

    public void setPlannedPath(java.util.List<Pose2d> poses) {
        fieldPublisher.setPlannedPath(poses);
    }

    public void clearPlannedPath() {
        fieldPublisher.clearPlannedPath();
    }

    public void resetActualPath() {
        fieldPublisher.resetActualPath();
    }

    public void setPlannedPathSpacingMeters(double spacingMeters) {
        fieldPublisher.setPlannedPathSpacingMeters(spacingMeters);
    }

    public void setFieldDimensionsMeters(double lengthMeters, double widthMeters) {
        fieldPublisher.setFieldDimensionsMeters(lengthMeters, widthMeters);
    }

    public void setActualPathSpacingMeters(double spacingMeters) {
        fieldPublisher.setActualPathSpacingMeters(spacingMeters);
    }

    public void setActualPathMinIntervalSeconds(double minIntervalSeconds) {
        fieldPublisher.setActualPathMinIntervalSeconds(minIntervalSeconds);
    }

    public void setActualPathMaxPoints(int maxPoints) {
        fieldPublisher.setActualPathMaxPoints(maxPoints);
    }

    private void updateHealthMetrics(double now) {
        ensureHealthNetworkEntries();
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
        if (poseJumpNtEntry != null) {
            poseJumpNtEntry.setDouble(lastPoseJumpMeters);
        }
        if (driftRateNtEntry != null) {
            driftRateNtEntry.setDouble(driftRateMetersPerSec);
        }
        if (visionAcceptanceNtEntry != null) {
            visionAcceptanceNtEntry.setDouble(acceptanceRate);
        }
        if (slipActiveNtEntry != null) {
            slipActiveNtEntry.setBoolean(slipActive);
        }
        if (slipCauseNtEntry != null) {
            slipCauseNtEntry.setString(lastSlipCause);
        }
        if (slipScoreNtEntry != null) {
            slipScoreNtEntry.setDouble(slipScore);
        }
        if (slipModeNtEntry != null) {
            slipModeNtEntry.setString(slipMode.name());
        }
    }

    private void updateSlipState(double now) {
        RobotLocalizationConfig.BackendConfig backend = backendConfig();
        SlipMode previousMode = slipMode;
        boolean previousSlipActive = slipActive;
        if (Double.isFinite(lastUpdateTimestamp)) {
            double dt = now - lastUpdateTimestamp;
            if (dt > 1e-6 && dt <= SLIP_DT_MAX_SECONDS) {
                Rotation2d imuYawRate = imu.getVelocityZ();
                double imuYawRateRad = imuYawRate != null ? imuYawRate.getRadians() : Double.NaN;
                double estimatedYawRate =
                        fieldPose.getRotation()
                                .minus(lastFieldPoseForSlip.getRotation())
                                .getRadians() / dt;
                if (!Double.isFinite(estimatedYawRate)) {
                    estimatedYawRate = 0.0;
                }
                double yawRateRad = Double.isFinite(imuYawRateRad) ? imuYawRateRad : estimatedYawRate;
                double comparableYawRate = unwrapRateNear(yawRateRad, estimatedYawRate);
                double yawDisagreement = Math.abs(comparableYawRate - estimatedYawRate);

                Translation2d deltaTranslation =
                        fieldPose.getTranslation().minus(lastFieldPoseForSlip.getTranslation());
                Translation2d odomVelocity = deltaTranslation.times(1.0 / dt);
                Translation2d odomAccel = odomVelocity.minus(lastFieldVelocityForSlip).times(1.0 / dt);
                double odomSpeed = odomVelocity.getNorm();
                double odomAccelMag = odomAccel.getNorm();

                ChassisSpeeds commandedRobot = combinedCommandedRobotSpeeds(commandedSpeedsScratchA);
                double commandedSpeed = Math.hypot(
                        commandedRobot.vxMetersPerSecond,
                        commandedRobot.vyMetersPerSecond);
                double reversalFlipX = axisSignFlipComponent(
                        lastCommandedVxForSlip,
                        commandedRobot.vxMetersPerSecond,
                        REVERSAL_COMMAND_THRESHOLD_MPS,
                        REVERSAL_COMMAND_FULL_SCALE_MPS);
                double reversalFlipY = axisSignFlipComponent(
                        lastCommandedVyForSlip,
                        commandedRobot.vyMetersPerSecond,
                        REVERSAL_COMMAND_THRESHOLD_MPS,
                        REVERSAL_COMMAND_FULL_SCALE_MPS);
                double reversalFlipComponent = Math.max(reversalFlipX, reversalFlipY);
                double headingCos = fieldPose.getRotation().getCos();
                double headingSin = fieldPose.getRotation().getSin();
                Translation2d commandedFieldVelocity = new Translation2d(
                        commandedRobot.vxMetersPerSecond * headingCos
                                - commandedRobot.vyMetersPerSecond * headingSin,
                        commandedRobot.vxMetersPerSecond * headingSin
                                + commandedRobot.vyMetersPerSecond * headingCos);

                double imuAx = finiteOrZero(imu.getAccelerationX());
                double imuAy = finiteOrZero(imu.getAccelerationY());
                double imuAccelMag = Math.hypot(imuAx, imuAy);
                double imuSpeedX = finiteOrZero(imu.getXSpeedMetersPerSecond());
                double imuSpeedY = finiteOrZero(imu.getYSpeedMetersPerSecond());
                double imuLinearSpeed = Math.hypot(imuSpeedX, imuSpeedY);
                Translation2d imuFieldVelocity = new Translation2d(
                        imuSpeedX * headingCos - imuSpeedY * headingSin,
                        imuSpeedX * headingSin + imuSpeedY * headingCos);
                if (imuLinearSpeed >= IMU_LINEAR_OBSERVED_SPEED_MPS
                        || imuAccelMag >= IMU_LINEAR_OBSERVED_ACCEL_MPS2) {
                    imuLinearSignalObserved = true;
                }

                boolean translatingForYaw =
                        Math.max(odomSpeed, commandedSpeed) > SLIP_YAW_TRANSLATION_GATE_MPS;
                double yawMagnitudeExcess = translatingForYaw
                        ? clamp01((Math.abs(comparableYawRate) - backend.slipYawRateThreshold())
                                / Math.max(backend.slipYawRateThreshold(), 1e-6))
                        : 0.0;
                double yawResidualNorm = translatingForYaw
                        ? clamp01(yawDisagreement / Math.max(backend.slipYawRateDisagreement(), 1e-6))
                        : 0.0;
                double yawComponent = yawMagnitudeExcess * yawResidualNorm;

                boolean translatingForAccel =
                        Math.max(odomSpeed, commandedSpeed) > SLIP_ACCEL_TRANSLATION_GATE_MPS;
                double accelThreshold = Math.max(backend.slipAccelThreshold(), IMU_ACCEL_NOISE_FLOOR_MPS2);
                double accelMagnitudeExcess = translatingForAccel
                        ? clamp01((imuAccelMag - accelThreshold) / Math.max(accelThreshold, 1e-6))
                        : 0.0;
                double accelResidualNorm = translatingForAccel
                        ? clamp01(Math.abs(imuAccelMag - odomAccelMag)
                                / Math.max(backend.slipAccelDisagreement(), 1e-6))
                        : 0.0;
                double accelComponent = accelMagnitudeExcess * accelResidualNorm;

                double reversalComponentInstant = 0.0;
                if (reversalFlipComponent > 0.0) {
                    double odomExcess = clamp01((odomAccelMag - REVERSAL_ODOM_ACCEL_THRESHOLD_MPS2)
                            / Math.max(REVERSAL_ODOM_ACCEL_THRESHOLD_MPS2, 1e-6));
                    double imuDeficit = clamp01((REVERSAL_IMU_ACCEL_MIN_MPS2 - imuAccelMag)
                            / Math.max(REVERSAL_IMU_ACCEL_MIN_MPS2, 1e-6));
                    reversalComponentInstant = reversalFlipComponent * odomExcess * imuDeficit;
                }
                double commandedFieldSpeed = commandedFieldVelocity.getNorm();
                if (commandedFieldSpeed > 0.20) {
                    Translation2d commandDirection = commandedFieldVelocity.times(1.0 / commandedFieldSpeed);
                    double odomParallel = odomVelocity.getX() * commandDirection.getX()
                            + odomVelocity.getY() * commandDirection.getY();
                    double imuParallel = imuFieldVelocity.getX() * commandDirection.getX()
                            + imuFieldVelocity.getY() * commandDirection.getY();
                    double parallelDeficit = clamp01(
                            (Math.abs(odomParallel) - Math.abs(imuParallel) - 0.08)
                                    / Math.max(Math.abs(odomParallel), 0.20));
                    double opposingEvidence = 0.0;
                    if (odomParallel > 0.15 && imuParallel < 0.0) {
                        opposingEvidence =
                                clamp01((-imuParallel) / Math.max(Math.abs(odomParallel), 0.20));
                    }
                    double directionalMismatch = Math.max(parallelDeficit, opposingEvidence);
                    if (reversalFlipComponent > 0.0) {
                        reversalComponentInstant = Math.max(
                                reversalComponentInstant,
                                directionalMismatch * MathUtil.interpolate(0.45, 1.0, reversalFlipComponent));
                    } else if (slipReversalComponent >= SLIP_REVERSAL_SUSTAIN_COMPONENT * 0.75) {
                        reversalComponentInstant = Math.max(
                                reversalComponentInstant,
                                directionalMismatch * 0.85);
                    }
                }
                double reversalComponent = filterSlipEvidenceComponent(
                        slipReversalComponent,
                        reversalComponentInstant,
                        dt,
                        SLIP_EVIDENCE_FILTER_RISE_TIME_SECONDS,
                        SLIP_REVERSAL_EVIDENCE_FALL_TIME_SECONDS);
                slipReversalComponent = reversalComponent;

                double contactComponentInstant = 0.0;
                boolean sustainingSlipEvidence =
                        slipMode != SlipMode.NOMINAL
                                || slipScore >= SLIP_TRANSIENT_EXIT_SCORE
                                || slipContactComponent >= CONTACT_ASSIST_COMPONENT * 0.40
                                || slipReversalComponent >= SLIP_REVERSAL_SUSTAIN_COMPONENT * 0.75;
                double commandGate = sustainingSlipEvidence
                        ? WHEELSPIN_COMMAND_SUSTAIN_MPS
                        : WHEELSPIN_COMMAND_SPEED_MPS;
                double odomGate = sustainingSlipEvidence
                        ? WHEELSPIN_ODOM_SUSTAIN_MPS
                        : WHEELSPIN_ODOM_SPEED_MPS;
                boolean wheelspinCandidate = commandedSpeed > commandGate && odomSpeed > odomGate;
                boolean contactDetectionEnabled =
                        imuLinearSignalObserved
                                || wheelspinCandidate
                                || slipContactComponent >= CONTACT_ASSIST_COMPONENT * 0.30
                                || slipMode != SlipMode.NOMINAL;
                if (contactDetectionEnabled
                        && commandedSpeed > commandGate
                        && (odomSpeed > odomGate || sustainingSlipEvidence)) {
                    double expectedLinearSpeed = Math.max(Math.max(commandedSpeed, odomSpeed), commandGate);
                    double allowedImuSpeed =
                            Math.max(WHEELSPIN_IMU_SPEED_FLOOR_MPS, expectedLinearSpeed * WHEELSPIN_IMU_SPEED_RATIO);
                    contactComponentInstant =
                            clamp01((allowedImuSpeed - imuLinearSpeed) / Math.max(allowedImuSpeed, 0.05));

                    if (expectedLinearSpeed > 1e-6) {
                        double imuSpeedRatio = imuLinearSpeed / expectedLinearSpeed;
                        double deficitComponent = clamp01(
                                (WHEELSPIN_DEFICIT_RATIO_START - imuSpeedRatio)
                                        / Math.max(
                                                WHEELSPIN_DEFICIT_RATIO_START - WHEELSPIN_DEFICIT_RATIO_FULL,
                                                1e-6));
                        contactComponentInstant = Math.max(contactComponentInstant, deficitComponent);
                    }
                }
                double contactComponent = filterSlipEvidenceComponent(
                        slipContactComponent,
                        contactComponentInstant,
                        dt,
                        SLIP_EVIDENCE_FILTER_RISE_TIME_SECONDS,
                        SLIP_CONTACT_EVIDENCE_FALL_TIME_SECONDS);
                slipContactComponent = contactComponent;

                slipScoreRaw = clamp01(
                        SLIP_WEIGHT_YAW * yawComponent
                                + SLIP_WEIGHT_ACCEL * accelComponent
                                + SLIP_WEIGHT_REVERSAL * reversalComponent
                                + SLIP_WEIGHT_CONTACT * contactComponent);
                double filterTime = slipScoreRaw > slipScore
                        ? SLIP_SCORE_FILTER_RISE_TIME_SECONDS
                        : SLIP_SCORE_FILTER_FALL_TIME_SECONDS;
                double alpha = dt / (filterTime + dt);
                slipScore += alpha * (slipScoreRaw - slipScore);
                slipScore = clamp01(slipScore);

                updateSlipMode(slipScore, contactComponent);
                if (slipMode == SlipMode.NOMINAL && slipScore < 0.08) {
                    lastSlipCause = "none";
                } else {
                    lastSlipCause = dominantSlipCause(yawComponent, accelComponent, reversalComponent, contactComponent);
                }

                lastFieldVelocityForSlip = odomVelocity;
                lastCommandedVxForSlip = commandedRobot.vxMetersPerSecond;
                lastCommandedVyForSlip = commandedRobot.vyMetersPerSecond;
            } else {
                lastCommandedVxForSlip = 0.0;
                lastCommandedVyForSlip = 0.0;
                slipScoreRaw = slipScore;
            }
        }
        slipActive = slipMode == SlipMode.TRANSIENT || slipMode == SlipMode.SLIP;
        if (slipMode != previousMode) {
            appendDiagnosticLog(
                    slipActive ? "WARN" : "INFO",
                    "slip",
                    "slip mode " + previousMode.name() + " -> " + slipMode.name()
                            + " cause=" + lastSlipCause
                            + " score=" + String.format("%.3f", slipScore));
        } else if (slipActive != previousSlipActive) {
            appendDiagnosticLog(
                    slipActive ? "WARN" : "INFO",
                    "slip",
                    slipActive ? "slip detection active" : "slip detection cleared");
        }
        lastUpdateTimestamp = now;
        lastFieldPoseForSlip = fieldPose;
        updateProcessStdDevScale(slipScore);
    }

    private ChassisSpeeds combinedCommandedRobotSpeeds(ChassisSpeeds target) {
        ChassisSpeeds resolved = target != null ? target : new ChassisSpeeds();
        if (robotSpeeds == null) {
            resolved.vxMetersPerSecond = 0.0;
            resolved.vyMetersPerSecond = 0.0;
            resolved.omegaRadiansPerSecond = 0.0;
            return resolved;
        }
        robotSpeeds.calculate(resolved);
        resolved.vxMetersPerSecond = finiteOrZero(resolved.vxMetersPerSecond);
        resolved.vyMetersPerSecond = finiteOrZero(resolved.vyMetersPerSecond);
        resolved.omegaRadiansPerSecond = finiteOrZero(resolved.omegaRadiansPerSecond);
        return resolved;
    }

    private static double axisSignFlipComponent(
            double previous,
            double current,
            double minimumMagnitude,
            double fullScaleMagnitude) {
        if (!Double.isFinite(previous) || !Double.isFinite(current)) {
            return 0.0;
        }
        double minAbs = Math.min(Math.abs(previous), Math.abs(current));
        if (minAbs < minimumMagnitude) {
            return 0.0;
        }
        if (Math.signum(previous) == Math.signum(current)) {
            return 0.0;
        }
        double maxScale = Math.max(fullScaleMagnitude, minimumMagnitude + 1e-6);
        return clamp01((minAbs - minimumMagnitude) / Math.max(maxScale - minimumMagnitude, 1e-6));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static double clamp01(double value) {
        return MathUtil.clamp(value, 0.0, 1.0);
    }

    private static double filterSlipEvidenceComponent(
            double previousComponent,
            double currentComponent,
            double dtSeconds,
            double riseTimeSeconds,
            double fallTimeSeconds) {
        if (!Double.isFinite(dtSeconds) || dtSeconds <= 1e-6) {
            return clamp01(currentComponent);
        }
        double previous = clamp01(previousComponent);
        double current = clamp01(currentComponent);
        double timeConstant = current >= previous ? riseTimeSeconds : fallTimeSeconds;
        if (!Double.isFinite(timeConstant) || timeConstant <= 1e-6) {
            return current;
        }
        double alpha = dtSeconds / (timeConstant + dtSeconds);
        return clamp01(previous + alpha * (current - previous));
    }

    private void updateSlipMode(double score, double contactComponent) {
        switch (slipMode) {
            case NOMINAL:
                if (score >= SLIP_ACTIVE_ENTER_SCORE || contactComponent >= 0.90) {
                    slipMode = SlipMode.SLIP;
                } else if (score >= SLIP_TRANSIENT_ENTER_SCORE
                        || contactComponent >= CONTACT_ASSIST_COMPONENT
                        || slipReversalComponent >= SLIP_REVERSAL_SUSTAIN_COMPONENT) {
                    slipMode = SlipMode.TRANSIENT;
                }
                break;
            case TRANSIENT:
                if (score >= SLIP_ACTIVE_ENTER_SCORE || contactComponent >= 0.90) {
                    slipMode = SlipMode.SLIP;
                } else if (score <= SLIP_TRANSIENT_EXIT_SCORE
                        && contactComponent < CONTACT_ASSIST_COMPONENT
                        && slipReversalComponent < SLIP_REVERSAL_SUSTAIN_COMPONENT) {
                    slipMode = SlipMode.NOMINAL;
                }
                break;
            case SLIP:
                if (score <= SLIP_ACTIVE_EXIT_SCORE
                        && contactComponent < CONTACT_ASSIST_COMPONENT
                        && slipReversalComponent < SLIP_REVERSAL_SUSTAIN_COMPONENT) {
                    slipMode = SlipMode.RECOVER;
                }
                break;
            case RECOVER:
                if (score >= SLIP_ACTIVE_ENTER_SCORE || contactComponent >= 0.90) {
                    slipMode = SlipMode.SLIP;
                } else if (score >= SLIP_TRANSIENT_ENTER_SCORE
                        || contactComponent >= CONTACT_ASSIST_COMPONENT
                        || slipReversalComponent >= SLIP_REVERSAL_SUSTAIN_COMPONENT) {
                    slipMode = SlipMode.TRANSIENT;
                } else if (score <= SLIP_TRANSIENT_EXIT_SCORE * 0.5
                        && contactComponent < CONTACT_ASSIST_COMPONENT * 0.70
                        && slipReversalComponent < SLIP_REVERSAL_SUSTAIN_COMPONENT * 0.70) {
                    slipMode = SlipMode.NOMINAL;
                }
                break;
            default:
                slipMode = SlipMode.NOMINAL;
                break;
        }
    }

    private static String dominantSlipCause(
            double yawComponent,
            double accelComponent,
            double reversalComponent,
            double contactComponent) {
        double best = yawComponent;
        String cause = "yawMismatch";
        if (accelComponent > best) {
            best = accelComponent;
            cause = "accelMismatch";
        }
        if (reversalComponent > best) {
            best = reversalComponent;
            cause = "reversal";
        }
        if (contactComponent > best) {
            best = contactComponent;
            cause = "contact";
        }
        return best < 0.05 ? "none" : cause;
    }

    private static double unwrapRateNear(double wrappedRate, double referenceRate) {
        if (!Double.isFinite(wrappedRate) || !Double.isFinite(referenceRate)) {
            return wrappedRate;
        }
        double best = wrappedRate;
        double bestError = Math.abs(wrappedRate - referenceRate);
        double twoPi = 2.0 * Math.PI;
        for (int k = -2; k <= 2; k++) {
            double candidate = wrappedRate + k * twoPi;
            double error = Math.abs(candidate - referenceRate);
            if (error < bestError) {
                best = candidate;
                bestError = error;
            }
        }
        return best;
    }

    private void updateProcessStdDevScale(double slipScoreValue) {
        if (processStdDevUpdateFailed) {
            return;
        }
        RobotLocalizationConfig.BackendConfig backend = backendConfig();
        double slipTargetScale = backend.slipProcessStdDevScale();
        if (!Double.isFinite(slipTargetScale) || slipTargetScale <= 0.0) {
            slipTargetScale = 1.0;
        }
        double normalizedScore = clamp01(slipScoreValue);
        double targetScale = 1.0 + normalizedScore * (slipTargetScale - 1.0);
        if (!Double.isFinite(targetScale) || targetScale <= 0.0) {
            targetScale = 1.0;
        }
        if (Math.abs(targetScale - processStdDevScale) < 1e-3) {
            return;
        }
        processStdDevScale = targetScale;
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            for (PoseEstimatorState state : poseStates.values()) {
                applyProcessStdDevs(state.estimator3d, baseStateStdDevs3d, targetScale);
            }
            refreshVisionStdDevs3d();
        } else {
            for (PoseEstimatorState state : poseStates.values()) {
                applyProcessStdDevs(state.estimator2d, baseStateStdDevs2d, targetScale);
            }
            refreshVisionStdDevs2d();
        }
    }

    private void refreshVisionStdDevs2d() {
        Matrix<N3, N1> std = localizationConfig.getVisionStd2d();
        for (PoseEstimatorState state : poseStates.values()) {
            PoseEstimator<T> estimator = state.estimator2d;
            if (estimator != null) {
                estimator.setVisionMeasurementStdDevs(std);
            }
        }
    }

    private void refreshVisionStdDevs3d() {
        Matrix<N4, N1> std = localizationConfig.getVisionStd3d();
        for (PoseEstimatorState state : poseStates.values()) {
            PoseEstimator3d<T> estimator = state.estimator3d;
            if (estimator != null) {
                estimator.setVisionMeasurementStdDevs(std);
            }
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

    private PoseConfig defaultPoseConfig() {
        return PoseConfig.defaults("field");
    }

    public boolean addPoseConfig(PoseConfig config) {
        Objects.requireNonNull(config, "config");
        if (poseConfigs.containsKey(config.name())) {
            return false;
        }
        registerPoseConfig(config);
        return true;
    }

    public void registerPoseConfig(PoseConfig config) {
        Objects.requireNonNull(config, "config");
        poseConfigs.put(config.name(), config);
        PoseEstimatorState state = poseStates.computeIfAbsent(config.name(), key -> new PoseEstimatorState());
        ensurePoseEstimator(config, state);
        state.active = config.active();
        ensurePrimaryPose(config);
        if (!isPrimaryPose(config)) {
            Pose2d pose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
            publishPoseStruct(config, pose);
        }
    }

    public Optional<PoseConfig> getPoseConfig(String name) {
        return Optional.ofNullable(poseConfigs.get(name));
    }

    public boolean hasPoseConfig(String name) {
        return name != null && poseConfigs.containsKey(name);
    }

    public boolean removePoseConfigIfPresent(String name) {
        if (!poseConfigs.containsKey(name)) {
            return false;
        }
        removePoseConfig(name);
        return true;
    }

    public boolean setPoseConfigPose(String name, Pose2d pose) {
        if (name == null || pose == null) {
            return false;
        }
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return false;
        }
        poseConfigs.put(name, config.withStartPose(pose));
        resetPose(name, pose);
        return true;
    }

    public boolean setPoseConfigPose(String name, Pose3d pose) {
        if (name == null || pose == null) {
            return false;
        }
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return false;
        }
        poseConfigs.put(name, config.withStartPose(pose));
        resetPose(name, pose);
        return true;
    }

    public boolean updatePoseConfig(String name, UnaryOperator<PoseConfig> updater) {
        if (name == null || updater == null) {
            return false;
        }
        PoseConfig existing = poseConfigs.get(name);
        if (existing == null) {
            return false;
        }
        PoseConfig updated = updater.apply(existing);
        if (updated == null || !Objects.equals(name, updated.name())) {
            return false;
        }
        applyPoseConfigUpdate(existing, updated);
        return true;
    }

    public boolean updatePoseConfig(String name, PoseConfig updated) {
        if (name == null || updated == null || !Objects.equals(name, updated.name())) {
            return false;
        }
        PoseConfig existing = poseConfigs.get(name);
        if (existing == null) {
            return false;
        }
        applyPoseConfigUpdate(existing, updated);
        return true;
    }

    public void removePoseConfig(String name) {
        poseConfigs.remove(name);
        poseStates.remove(name);
        poseStructPruneQueue.remove(name);
        StructPublisher<Pose2d> publisher = poseStructPublishers.remove(name);
        if (publisher != null) {
            publisher.close();
        }
        if (Objects.equals(primaryPoseName, name)) {
            primaryPoseName = null;
            for (PoseConfig config : poseConfigs.values()) {
                ensurePrimaryPose(config);
            }
        }
    }

    public void setPoseActive(String name, boolean active) {
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return;
        }
        applyPoseConfigUpdate(config, config.withActive(active));
    }

    public boolean setPoseConfigNetworkTablesPublishing(String name, boolean publishToNetworkTables) {
        return updatePoseConfig(name, config -> config.withNetworkTablesPublishing(publishToNetworkTables));
    }

    public boolean isPoseActive(String name) {
        PoseEstimatorState state = poseStates.get(name);
        return state != null && state.active;
    }

    private void applyPoseConfigUpdate(PoseConfig existing, PoseConfig updated) {
        if (existing == null || updated == null) {
            return;
        }
        poseConfigs.put(updated.name(), updated);
        PoseEstimatorState state = poseStates.get(updated.name());
        if (state != null) {
            state.active = updated.active();
        }
        if (existing.publishToNetworkTables() && !updated.publishToNetworkTables()) {
            poseStructPruneQueue.remove(updated.name());
            StructPublisher<Pose2d> publisher = poseStructPublishers.remove(updated.name());
            if (publisher != null) {
                publisher.close();
            }
        }
        ensurePrimaryPose(updated);
        if (updated.startPose2d() != null && !Objects.equals(existing.startPose2d(), updated.startPose2d())) {
            resetPose(updated.name(), updated.startPose2d());
        } else if (updated.startPose3d() != null && !Objects.equals(existing.startPose3d(), updated.startPose3d())) {
            resetPose(updated.name(), updated.startPose3d());
        }
    }

    public Pose2d getPose2d(String name) {
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return ZERO_POSE_2D;
        }
        PoseEstimatorState state = ensurePoseState(config);
        updatePoseFromConfig(config, state, currentTimestampSeconds());
        return state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
    }

    public Pose3d getPose3d(String name) {
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return ZERO_POSE_3D;
        }
        PoseEstimatorState state = ensurePoseState(config);
        updatePoseFromConfig(config, state, currentTimestampSeconds());
        return state.pose3d != null ? state.pose3d : ZERO_POSE_3D;
    }
 
    private PoseEstimatorState ensurePoseState(PoseConfig config) {
        PoseEstimatorState state = poseStates.computeIfAbsent(config.name(), key -> new PoseEstimatorState());
        ensurePoseEstimator(config, state);
        return state;
    }

    private void ensurePoseEstimator(PoseConfig config, PoseEstimatorState state) {
        if (config == null || state == null) {
            return;
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            if (state.estimator3d == null) {
                Pose3d startPose = resolveInitialPose3d(config);
                PoseEstimator3d<T> estimator =
                        estimatorFactory.create3d(startPose, baseStateStdDevs3d, localizationConfig.getVisionStd3d());
                if (estimator == null) {
                    throw new IllegalArgumentException("3D localization requires a PoseEstimator3d instance.");
                }
                state.estimator3d = estimator;
                state.pose3d = startPose;
                state.pose2d = startPose.toPose2d();
            }
        } else if (state.estimator2d == null) {
            Pose2d startPose = resolveInitialPose2d(config);
            PoseEstimator<T> estimator =
                    estimatorFactory.create2d(startPose, baseStateStdDevs2d, localizationConfig.getVisionStd2d());
            if (estimator == null) {
                throw new IllegalArgumentException("2D localization requires a PoseEstimator instance.");
            }
            state.estimator2d = estimator;
            state.pose2d = startPose;
            state.pose3d = new Pose3d(startPose);
        }
    }

    private void ensurePrimaryPose(PoseConfig config) {
        if (config == null) {
            return;
        }
        if (primaryPoseName == null || "field".equals(config.name())) {
            primaryPoseName = config.name();
            PoseEstimatorState state = poseStates.get(primaryPoseName);
            if (state != null) {
                setPrimaryPose(state);
            }
        }
    }

    private Pose2d resolveInitialPose2d(PoseConfig config) {
        if (config == null) {
            return ZERO_POSE_2D;
        }
        if (config.startPose2d() != null) {
            return config.startPose2d();
        }
        if (config.startPose3d() != null) {
            return config.startPose3d().toPose2d();
        }
        PoseEstimatorState primary = getPrimaryPoseState();
        if (primary != null && primary.pose2d != null) {
            return primary.pose2d;
        }
        return ZERO_POSE_2D;
    }

    private Pose3d resolveInitialPose3d(PoseConfig config) {
        if (config == null) {
            return ZERO_POSE_3D;
        }
        if (config.startPose3d() != null) {
            return config.startPose3d();
        }
        if (config.startPose2d() != null) {
            return new Pose3d(config.startPose2d());
        }
        PoseEstimatorState primary = getPrimaryPoseState();
        if (primary != null && primary.pose3d != null) {
            return primary.pose3d;
        }
        return ZERO_POSE_3D;
    }

    private PoseEstimatorState getPrimaryPoseState() {
        if (primaryPoseName == null) {
            return null;
        }
        return poseStates.get(primaryPoseName);
    }

    private boolean isPrimaryPose(PoseConfig config) {
        return config != null && Objects.equals(primaryPoseName, config.name());
    }

    private PoseConfig findConfigForState(PoseEstimatorState state) {
        if (state == null) {
            return null;
        }
        for (Map.Entry<String, PoseEstimatorState> entry : poseStates.entrySet()) {
            if (entry.getValue() == state) {
                return poseConfigs.get(entry.getKey());
            }
        }
        return null;
    }

    private void setPrimaryPose(PoseEstimatorState state) {
        if (state == null) {
            return;
        }
        fieldPose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
        fieldPose3d = state.pose3d != null ? state.pose3d : new Pose3d(fieldPose);
    }

    private void resetVelocityTracking(Pose2d pose) {
        lastVelocityPose = pose != null ? pose : ZERO_POSE_2D;
        lastVelocityTimestamp = currentTimestampSeconds();
        fieldRelativeSpeeds.vxMetersPerSecond = 0.0;
        fieldRelativeSpeeds.vyMetersPerSecond = 0.0;
        fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0;
        robotRelativeSpeeds.vxMetersPerSecond = 0.0;
        robotRelativeSpeeds.vyMetersPerSecond = 0.0;
        robotRelativeSpeeds.omegaRadiansPerSecond = 0.0;
        normalizedMovementSpeed = 0.0;
        resetSlipFusionTracking(lastVelocityPose, lastVelocityTimestamp);
    }

    private void resetSlipFusionTracking(Pose2d pose, double timestampSeconds) {
        Pose2d resolvedPose = pose != null ? pose : ZERO_POSE_2D;
        double resolvedTimestamp = Double.isFinite(timestampSeconds) ? timestampSeconds : currentTimestampSeconds();
        lastRawEstimatorPoseForSlipFusion = resolvedPose;
        lastCorrectedPoseForSlipFusion = resolvedPose;
        lastSlipFusionTimestamp = resolvedTimestamp;
        slipScore = 0.0;
        slipScoreRaw = 0.0;
        slipContactComponent = 0.0;
        slipReversalComponent = 0.0;
        slipMode = SlipMode.NOMINAL;
        slipActive = false;
        lastUpdateTimestamp = resolvedTimestamp;
        lastFieldPoseForSlip = resolvedPose;
        lastFieldVelocityForSlip = new Translation2d();
        lastCommandedVxForSlip = 0.0;
        lastCommandedVyForSlip = 0.0;
        lastSlipCause = "none";
        updateProcessStdDevScale(0.0);
    }

    private void updateVelocityTracking(Pose2d pose, double nowSeconds) {
        Pose2d currentPose = pose != null ? pose : ZERO_POSE_2D;
        if (!Double.isFinite(lastVelocityTimestamp)) {
            resetVelocityTracking(currentPose);
            return;
        }

        double dt = nowSeconds - lastVelocityTimestamp;
        if (!Double.isFinite(dt) || dt <= 1e-6) {
            lastVelocityPose = currentPose;
            lastVelocityTimestamp = nowSeconds;
            return;
        }

        double fieldVx = (currentPose.getX() - lastVelocityPose.getX()) / dt;
        double fieldVy = (currentPose.getY() - lastVelocityPose.getY()) / dt;
        double deltaTheta =
                currentPose.getRotation().minus(lastVelocityPose.getRotation()).getRadians();
        double omega = deltaTheta / dt;

        if (!Double.isFinite(fieldVx)) {
            fieldVx = 0.0;
        }
        if (!Double.isFinite(fieldVy)) {
            fieldVy = 0.0;
        }
        if (!Double.isFinite(omega)) {
            omega = 0.0;
        }

        fieldRelativeSpeeds.vxMetersPerSecond = fieldVx;
        fieldRelativeSpeeds.vyMetersPerSecond = fieldVy;
        fieldRelativeSpeeds.omegaRadiansPerSecond = omega;
        double cos = currentPose.getRotation().getCos();
        double sin = currentPose.getRotation().getSin();
        robotRelativeSpeeds.vxMetersPerSecond = fieldVx * cos + fieldVy * sin;
        robotRelativeSpeeds.vyMetersPerSecond = -fieldVx * sin + fieldVy * cos;
        robotRelativeSpeeds.omegaRadiansPerSecond = omega;
        normalizedMovementSpeed = normalizeMovementSpeed(Math.hypot(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond));

        lastVelocityPose = currentPose;
        lastVelocityTimestamp = nowSeconds;
    }

    private double normalizeMovementSpeed(double movementSpeedMetersPerSecond) {
        if (!Double.isFinite(movementSpeedMetersPerSecond) || movementSpeedMetersPerSecond <= 0.0) {
            return 0.0;
        }
        double maxSpeedMetersPerSecond = robotSpeeds != null ? robotSpeeds.getMaxVelocity() : Double.NaN;
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 1e-9) {
            return 1.0;
        }
        double normalized = movementSpeedMetersPerSecond / maxSpeedMetersPerSecond;
        if (!Double.isFinite(normalized)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, normalized));
    }

    private void resetVisionTracking(PoseEstimatorState state) {
        if (state == null) {
            return;
        }
        state.hasAcceptedVisionMeasurement = false;
        state.lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;
        state.poseJumpGuardUntilSeconds = Double.NEGATIVE_INFINITY;
    }

    public FieldObject2d getField2dObject(String id){
        return fieldPublisher.getObject(id);
    }

    /**
     * Registers or updates a named localization bounding box for visualization and telemetry.
     */
    public RobotLocalization<T> zone(String name, PoseBoundingBox2d box) {
        String resolvedName = sanitizeBoundingBoxName(name);
        Objects.requireNonNull(box, "box");
        PoseBoundingBox2d previous = namedBoundingBoxes.put(resolvedName, box);
        if (!box.equals(previous)) {
            boundingBoxTrajectoryCache.remove(resolvedName);
            boundingBoxTrajectorySources.remove(resolvedName);
            boundingBoxMetadataPublished.remove(resolvedName);
            boundingBoxContainsPublished.remove(resolvedName);
            boundingBoxTrajectoryPublished.remove(resolvedName);
        }
        fieldPublisher.setBoundingBoxes(namedBoundingBoxes);
        return this;
    }

    /**
     * Registers or updates a named localization bounding box from opposite corners.
     */
    public RobotLocalization<T> zone(String name, Translation2d cornerA, Translation2d cornerB) {
        return zone(name, PoseBoundingBox2d.fromCorners(cornerA, cornerB));
    }

    /**
     * Updates an existing named localization bounding box from opposite corners.
     *
     * <p>Returns {@code false} if the zone is not currently registered.</p>
     */
    public boolean updateZoneCorners(String name, Translation2d cornerA, Translation2d cornerB) {
        if (cornerA == null || cornerB == null) {
            return false;
        }
        String resolvedName = sanitizeBoundingBoxName(name);
        PoseBoundingBox2d existing = namedBoundingBoxes.get(resolvedName);
        if (existing == null) {
            return false;
        }
        zone(resolvedName, existing.updateCorners(cornerA, cornerB));
        return true;
    }

    /**
     * Removes a named localization bounding box from visualization and telemetry.
     */
    public boolean removeZone(String name) {
        String resolvedName = sanitizeBoundingBoxName(name);
        PoseBoundingBox2d removed = namedBoundingBoxes.remove(resolvedName);
        if (removed == null) {
            return false;
        }
        StructArrayPublisher<Pose2d> publisher = boundingBoxPublishers.remove(resolvedName);
        if (publisher != null) {
            publisher.close();
        }
        boundingBoxTrajectoryCache.remove(resolvedName);
        boundingBoxTrajectorySources.remove(resolvedName);
        boundingBoxMetadataPublished.remove(resolvedName);
        boundingBoxContainsPublished.remove(resolvedName);
        boundingBoxTrajectoryPublished.remove(resolvedName);
        fieldPublisher.setBoundingBoxes(namedBoundingBoxes);
        return true;
    }

    /**
     * Clears all named localization bounding boxes from visualization and telemetry.
     */
    public void clearZones() {
        namedBoundingBoxes.clear();
        for (StructArrayPublisher<Pose2d> publisher : boundingBoxPublishers.values()) {
            if (publisher != null) {
                publisher.close();
            }
        }
        boundingBoxPublishers.clear();
        boundingBoxTrajectoryCache.clear();
        boundingBoxTrajectorySources.clear();
        boundingBoxMetadataPublished.clear();
        boundingBoxContainsPublished.clear();
        boundingBoxTrajectoryPublished.clear();
        fieldPublisher.clearBoundingBoxes();
    }

    /**
     * Returns an immutable view of registered localization bounding boxes.
     */
    private Map<String, PoseBoundingBox2d> zonesMap() {
        return Map.copyOf(namedBoundingBoxes);
    }

    /**
     * Convenience primary pose alias: {@code robot.localization().pose()}.
     */
    public Pose2d pose() {
        return getFieldPose();
    }

    public Pose3d pose3d() {
        return getFieldPose3d();
    }

    public SpeedsView speeds() {
        return new SpeedsView(this);
    }

    public ZonesView zones() {
        return new ZonesView(this);
    }

    public Pose2d getFieldPose(){
        PoseEstimatorState primary = getPrimaryPoseState();
        if (primary != null) {
            setPrimaryPose(primary);
        }
        return fieldPose;
    }

    /**
     * Returns estimated chassis speeds in the robot frame derived from primary pose deltas.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speeds = robotRelativeSpeeds;
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    /**
     * Returns estimated chassis speeds in the field frame derived from primary pose deltas.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds speeds = fieldRelativeSpeeds;
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    public double getXSpeedMetersPerSecond() {
        return robotRelativeSpeeds.vxMetersPerSecond;
    }

    public double getYSpeedMetersPerSecond() {
        return robotRelativeSpeeds.vyMetersPerSecond;
    }

    public double getThetaSpeedRadiansPerSecond() {
        return robotRelativeSpeeds.omegaRadiansPerSecond;
    }

    public double getMovementSpeedMetersPerSecond() {
        return Math.hypot(getXSpeedMetersPerSecond(), getYSpeedMetersPerSecond());
    }

    public double getNormalizedMovementSpeed() {
        return normalizedMovementSpeed;
    }

    public double getNormalizedSpeed() {
        return getNormalizedMovementSpeed();
    }

    /**
     * Checks whether the primary field pose is inside the provided axis-aligned bounding box.
     */
    public boolean inZone(PoseBoundingBox2d box) {
        return box != null && box.contains(getFieldPose());
    }

    /**
     * Checks whether the primary field pose is inside the axis-aligned box formed by two corners.
     */
    public boolean inZone(Translation2d cornerA, Translation2d cornerB) {
        if (cornerA == null || cornerB == null) {
            return false;
        }
        return inZone(PoseBoundingBox2d.fromCorners(cornerA, cornerB));
    }

    /**
     * Checks whether a named pose config is inside the provided axis-aligned bounding box.
     */
    public boolean poseInZone(String name, PoseBoundingBox2d box) {
        if (name == null || box == null || !hasPoseConfig(name)) {
            return false;
        }
        return box.contains(getPose2d(name));
    }

    /**
     * Checks whether a named pose config is inside the axis-aligned box formed by two corners.
     */
    public boolean poseInZone(String name, Translation2d cornerA, Translation2d cornerB) {
        if (name == null || cornerA == null || cornerB == null || !hasPoseConfig(name)) {
            return false;
        }
        return poseInZone(name, PoseBoundingBox2d.fromCorners(cornerA, cornerB));
    }

    public RobotLocalizationConfig getLocalizationConfig() {
        return localizationConfig;
    }

    public Pose3d getFieldPose3d() {
        PoseEstimatorState primary = getPrimaryPoseState();
        if (primary != null) {
            setPrimaryPose(primary);
        }
        return fieldPose3d;
    }

    public boolean isSuppressUpdates() {
        return suppressUpdates;
    }

    public void setSuppressUpdates(boolean suppressUpdates) {
        if (this.suppressUpdates != suppressUpdates) {
            appendDiagnosticLog(
                    "INFO",
                    "control",
                    suppressUpdates ? "localization updates suppressed" : "localization updates resumed");
        }
        this.suppressUpdates = suppressUpdates;
    }

    public Field2d getField2d() {
        return fieldPublisher.getField();
    }

    public Field2d getVisionField2d() {
        return cameraManager.getVisionField();
    }

    public static final class SpeedsView {
        private final RobotLocalization<?> owner;

        private SpeedsView(RobotLocalization<?> owner) {
            this.owner = owner;
        }

        public ChassisSpeeds robotRelative() {
            return owner.getRobotRelativeSpeeds();
        }

        public ChassisSpeeds fieldRelative() {
            return owner.getFieldRelativeSpeeds();
        }

        public double xMetersPerSecond() {
            return owner.getXSpeedMetersPerSecond();
        }

        public double yMetersPerSecond() {
            return owner.getYSpeedMetersPerSecond();
        }

        public double thetaRadiansPerSecond() {
            return owner.getThetaSpeedRadiansPerSecond();
        }

        public double movementMetersPerSecond() {
            return owner.getMovementSpeedMetersPerSecond();
        }

        public double normalizedMovement() {
            return owner.getNormalizedMovementSpeed();
        }
    }

    public static final class ZonesView {
        private final RobotLocalization<?> owner;

        private ZonesView(RobotLocalization<?> owner) {
            this.owner = owner;
        }

        public Map<String, PoseBoundingBox2d> all() {
            return owner.zonesMap();
        }

        public boolean contains(String name) {
            return name != null && owner.zonesMap().containsKey(name);
        }

        public boolean in(String name) {
            PoseBoundingBox2d box = owner.zonesMap().get(name);
            return box != null && owner.inZone(box);
        }

        public boolean in(PoseBoundingBox2d box) {
            return owner.inZone(box);
        }

        public boolean updateCorners(String name, Translation2d cornerA, Translation2d cornerB) {
            return owner.updateZoneCorners(name, cornerA, cornerB);
        }
    }

    public final class DiagnosticsView {
        public DiagnosticsView log(String level, String category, String message) {
            appendDiagnosticLog(level, category, message);
            return this;
        }

        public DiagnosticsView info(String category, String message) {
            appendDiagnosticLog("INFO", category, message);
            return this;
        }

        public DiagnosticsView warn(String category, String message) {
            appendDiagnosticLog("WARN", category, message);
            return this;
        }

        public DiagnosticsView error(String category, String message) {
            appendDiagnosticLog("ERROR", category, message);
            return this;
        }

        public List<DiagnosticsChannel.Event> events(int limit) {
            DiagnosticsChannel channel = diagnosticsChannel;
            return channel != null ? channel.events(limit) : List.of();
        }

        public int count() {
            DiagnosticsChannel channel = diagnosticsChannel;
            return channel != null ? channel.eventCount() : 0;
        }

        public DiagnosticsView clear() {
            clearDiagnosticsLog();
            return this;
        }

        public Map<String, Object> summary() {
            return getDiagnosticsSummary();
        }

        public Map<String, Object> snapshot(int limit) {
            return getDiagnosticsSnapshot(limit);
        }
    }

    private void appendDiagnosticLog(String level, String category, String message) {
        DiagnosticsChannel channel = diagnosticsChannel;
        if (channel == null || message == null || message.isBlank()) {
            return;
        }
        try {
            channel.log(level, category, message);
        } catch (RuntimeException ignored) {
            // Diagnostics should never break localization updates.
        }
    }

    public Map<String, Object> getDiagnosticsSummary() {
        Map<String, Object> summary = new LinkedHashMap<>();
        summary.put("poseSpace", poseSpace != null ? poseSpace.name() : "");
        summary.put("poseCount", poseConfigs.size());
        summary.put("primaryPose", primaryPoseName != null ? primaryPoseName : "");
        summary.put("visionEnabled", isVisionEnabled());
        summary.put("suppressUpdates", suppressUpdates);
        summary.put("slipActive", slipActive);
        summary.put("slipMode", slipMode.name());
        summary.put("slipCause", lastSlipCause);
        summary.put("slipScore", slipScore);
        summary.put("visionAcceptRate", visionAcceptRateWindow);
        summary.put("driftRateMetersPerSec", driftRateMetersPerSec);
        summary.put("poseJumpMeters", lastPoseJumpMeters);
        DiagnosticsChannel channel = diagnosticsChannel;
        if (channel != null) {
            summary.put("channel", channel.summary());
        }
        return summary;
    }

    public Map<String, Object> getDiagnosticsSnapshot(int limit) {
        int resolvedLimit = limit > 0 ? Math.min(limit, 2048) : 120;
        Map<String, Object> snapshot = new LinkedHashMap<>(getDiagnosticsSummary());
        Pose2d pose = getFieldPose();
        Map<String, Object> poseNode = new LinkedHashMap<>();
        poseNode.put("xM", pose.getX());
        poseNode.put("yM", pose.getY());
        poseNode.put("headingDeg", pose.getRotation().getDegrees());
        snapshot.put("pose", poseNode);
        ChassisSpeeds robot = getRobotRelativeSpeeds();
        Map<String, Object> speedsNode = new LinkedHashMap<>();
        speedsNode.put("vxMps", robot.vxMetersPerSecond);
        speedsNode.put("vyMps", robot.vyMetersPerSecond);
        speedsNode.put("omegaRadPerSec", robot.omegaRadiansPerSecond);
        snapshot.put("robotSpeeds", speedsNode);
        DiagnosticsChannel channel = diagnosticsChannel;
        if (channel != null) {
            snapshot.put("events", channel.events(resolvedLimit));
        }
        return snapshot;
    }

    public void clearDiagnosticsLog() {
        DiagnosticsChannel channel = diagnosticsChannel;
        if (channel != null) {
            channel.clear();
        }
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        Pose2d pose = getFieldPose();
        RobotNetworkTables.Node posesNode = node.child("Poses").child("Primary");
        posesNode.putDouble("xM", pose.getX());
        posesNode.putDouble("yM", pose.getY());
        posesNode.putDouble("rotDeg", pose.getRotation().getDegrees());

        RobotNetworkTables.Node speedsNode = node.child("Speeds");
        speedsNode.putDouble("robotVxMps", robotRelativeSpeeds.vxMetersPerSecond);
        speedsNode.putDouble("robotVyMps", robotRelativeSpeeds.vyMetersPerSecond);
        speedsNode.putDouble("robotOmegaRadPerSec", robotRelativeSpeeds.omegaRadiansPerSecond);
        speedsNode.putDouble("fieldVxMps", fieldRelativeSpeeds.vxMetersPerSecond);
        speedsNode.putDouble("fieldVyMps", fieldRelativeSpeeds.vyMetersPerSecond);
        speedsNode.putDouble("fieldOmegaRadPerSec", fieldRelativeSpeeds.omegaRadiansPerSecond);
        speedsNode.putDouble("movementMps", getMovementSpeedMetersPerSecond());
        speedsNode.putDouble("movementNormalized", normalizedMovementSpeed);
        speedsNode.putDouble("normalizedSpeed", getNormalizedSpeed());

        publishBoundingBoxes(node, pose);

        // Field2d is a localization concern (not vision). Publish it only when explicitly enabled.
        if (nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET)) {
            if (!fieldPublisher.isFieldPublished()) {
                // Elastic/Glass can bind a Field2d widget to this SmartDashboard entry.
                SmartDashboard.putData("Athena Localization Field2d", fieldPublisher.getField());
                fieldPublisher.publishFieldOnce();
            }
            fieldPublisher.setRobotPose(pose);
            fieldPublisher.setBoundingBoxes(namedBoundingBoxes);
            fieldPublisher.setNamedPoses(collectFieldPoses());
        } else {
            fieldPublisher.clearBoundingBoxes();
            fieldPublisher.clearNamedPoses();
        }

        if (nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_HEALTH_WIDGETS)) {
            RobotNetworkTables.Node health = node.child("Health");
            health.putDouble("poseJumpMeters", lastPoseJumpMeters);
            health.putDouble("driftRateMetersPerSec", driftRateMetersPerSec);
            health.putDouble("visionAcceptRate", visionAcceptRateWindow);
            health.putBoolean("slipActive", slipActive);
            health.putString("slipMode", slipMode.name());
            health.putString("slipCause", lastSlipCause);
            health.putDouble("slipScore", slipScore);
        }

        return node;
    }

    private Map<String, Pose2d> collectFieldPoses() {
        fieldPoseObjectsBuffer.clear();
        for (Map.Entry<String, PoseConfig> entry : poseConfigs.entrySet()) {
            if (entry == null || entry.getKey() == null || entry.getValue() == null) {
                continue;
            }
            PoseEstimatorState state = poseStates.get(entry.getKey());
            if (state == null || state.pose2d == null) {
                continue;
            }
            fieldPoseObjectsBuffer.put(entry.getKey(), state.pose2d);
        }
        return fieldPoseObjectsBuffer;
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
        slipModeNtEntry = healthTable.getEntry("SlipMode");
        slipCauseNtEntry = healthTable.getEntry("SlipCause");
        slipScoreNtEntry = healthTable.getEntry("SlipScore");
    }

    private static double currentTimestampSeconds() {
        double nowSeconds = RobotTime.nowSeconds();
        if (Double.isFinite(nowSeconds)) {
            return nowSeconds;
        }
        return Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        if (suppressUpdates) {
            return;
        }
        double nowSeconds = currentTimestampSeconds();
        updateActivePoseConfigs(nowSeconds);
        publishPoseObjects();
    }

    private void updateActivePoseConfigs(double nowSeconds) {
        for (PoseConfig config : poseConfigs.values()) {
            PoseEstimatorState state = ensurePoseState(config);
            if (state.active) {
                updatePoseFromConfig(config, state, nowSeconds);
            }
        }
    }

    private void publishPoseObjects() {
        prunePoseStructPublishers();
        fieldPublisher.clearRobotPose();
        for (PoseConfig config : poseConfigs.values()) {
            PoseEstimatorState state = poseStates.get(config.name());
            if (state == null) {
                continue;
            }
            Pose2d pose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
            publishPoseStruct(config, pose);
        }
    }

    private void prunePoseStructPublishers() {
        if (poseStructPublishers.isEmpty()) {
            poseStructPruneQueue.clear();
            return;
        }
        if (poseStructPruneQueue.isEmpty()) {
            poseStructPruneQueue.addAll(poseStructPublishers.keySet());
        }
        int budget = Math.min(POSE_STRUCT_PRUNE_PER_CYCLE, poseStructPruneQueue.size());
        for (int i = 0; i < budget; i++) {
            String name = poseStructPruneQueue.pollFirst();
            if (name == null) {
                continue;
            }
            StructPublisher<Pose2d> publisher = poseStructPublishers.get(name);
            if (publisher == null) {
                continue;
            }
            PoseConfig config = poseConfigs.get(name);
            boolean keep = config != null && config.publishToNetworkTables();
            if (!keep) {
                publisher.close();
                poseStructPublishers.remove(name);
            }
        }
    }

    private void publishPoseStruct(PoseConfig config, Pose2d pose) {
        RobotNetworkTables nt = robotNetworkTables;
        if (nt == null || !nt.isPublishingEnabled() || !nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_POSE_TOPICS)) {
            return;
        }
        if (config == null || !config.publishToNetworkTables()) {
            return;
        }
        String name = config.name();
        if (name == null || name.isBlank()) {
            return;
        }
        StructPublisher<Pose2d> publisher = poseStructPublishers.get(name);
        if (publisher == null) {
            String topic = "Athena/Localization/Poses/" + name;
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic(topic, Pose2d.struct)
                    .publish();
            poseStructPublishers.put(name, publisher);
            poseStructPruneQueue.addLast(name);
        }
        publisher.set(pose);
    }

    private void publishBoundingBoxes(RobotNetworkTables.Node localizationNode, Pose2d currentPose) {
        RobotNetworkTables nt = robotNetworkTables;
        if (nt == null || !nt.isPublishingEnabled()) {
            return;
        }

        boolean publishStaticThisCycle =
                (boundingBoxPublishCycle++ % BOUNDING_BOX_STATIC_PUBLISH_PERIOD) == 0;
        RobotNetworkTables.Node boxesNode = localizationNode.child("BoundingBoxes");
        boundingBoxPublishers.entrySet().removeIf(entry -> {
            String name = entry.getKey();
            if (name != null && namedBoundingBoxes.containsKey(name)) {
                return false;
            }
            StructArrayPublisher<Pose2d> publisher = entry.getValue();
            if (publisher != null) {
                publisher.close();
            }
            boundingBoxTrajectoryCache.remove(name);
            boundingBoxTrajectorySources.remove(name);
            boundingBoxMetadataPublished.remove(name);
            boundingBoxContainsPublished.remove(name);
            boundingBoxTrajectoryPublished.remove(name);
            return true;
        });

        for (Map.Entry<String, PoseBoundingBox2d> entry : namedBoundingBoxes.entrySet()) {
            String name = entry.getKey();
            PoseBoundingBox2d box = entry.getValue();
            if (name == null || name.isBlank() || box == null) {
                continue;
            }
            RobotNetworkTables.Node boxNode = boxesNode.child(name);
            PoseBoundingBox2d publishedBox = boundingBoxMetadataPublished.get(name);
            boolean boxChanged = !box.equals(publishedBox);
            if (publishStaticThisCycle || boxChanged) {
                boxNode.putDouble("minX", box.minX());
                boxNode.putDouble("minY", box.minY());
                boxNode.putDouble("maxX", box.maxX());
                boxNode.putDouble("maxY", box.maxY());
                boundingBoxMetadataPublished.put(name, snapshotBoundingBox(box));
            }
            boolean containsPrimaryPose = box.contains(currentPose);
            Boolean previousContains = boundingBoxContainsPublished.get(name);
            if (publishStaticThisCycle || previousContains == null || previousContains.booleanValue() != containsPrimaryPose) {
                boxNode.putBoolean("containsPrimaryPose", containsPrimaryPose);
                boundingBoxContainsPublished.put(name, containsPrimaryPose);
            }

            StructArrayPublisher<Pose2d> publisher = boundingBoxPublishers.computeIfAbsent(name, key ->
                    NetworkTableInstance.getDefault()
                            .getStructArrayTopic("Athena/Localization/BoundingBoxes/" + key + "/Trajectory", Pose2d.struct)
                            .publish());
            Pose2d[] trajectory = getOrBuildBoundingBoxTrajectory(name, box);
            Pose2d[] publishedTrajectory = boundingBoxTrajectoryPublished.get(name);
            if (publishStaticThisCycle || boxChanged || publishedTrajectory == null) {
                publisher.set(trajectory);
                boundingBoxTrajectoryPublished.put(name, trajectory);
            }
        }
    }

    private Pose2d[] getOrBuildBoundingBoxTrajectory(String name, PoseBoundingBox2d box) {
        PoseBoundingBox2d previousBox = boundingBoxTrajectorySources.get(name);
        Pose2d[] cached = boundingBoxTrajectoryCache.get(name);
        if (cached != null && box.equals(previousBox)) {
            return cached;
        }
        Pose2d[] rebuilt = toBoundingBoxTrajectory(box);
        boundingBoxTrajectoryCache.put(name, rebuilt);
        boundingBoxTrajectorySources.put(name, snapshotBoundingBox(box));
        return rebuilt;
    }

    private static Pose2d[] toBoundingBoxTrajectory(PoseBoundingBox2d box) {
        Pose2d bottomLeft = new Pose2d(box.minX(), box.minY(), ZERO_ROTATION);
        Pose2d bottomRight = new Pose2d(box.maxX(), box.minY(), ZERO_ROTATION);
        Pose2d topRight = new Pose2d(box.maxX(), box.maxY(), ZERO_ROTATION);
        Pose2d topLeft = new Pose2d(box.minX(), box.maxY(), ZERO_ROTATION);
        return new Pose2d[] { bottomLeft, bottomRight, topRight, topLeft, bottomLeft };
    }

    private static PoseBoundingBox2d snapshotBoundingBox(PoseBoundingBox2d box) {
        return new PoseBoundingBox2d(box.minX(), box.minY(), box.maxX(), box.maxY());
    }

    private static String sanitizeBoundingBoxName(String name) {
        if (name == null) {
            throw new IllegalArgumentException("bounding box name must not be null");
        }
        String resolved = name.trim();
        if (resolved.isEmpty()) {
            throw new IllegalArgumentException("bounding box name must not be blank");
        }
        if (resolved.indexOf('/') >= 0 || resolved.indexOf('\\') >= 0) {
            throw new IllegalArgumentException("bounding box name must not contain '/' or '\\'");
        }
        return resolved;
    }

    private void updatePoseFromConfig(PoseConfig config, PoseEstimatorState state, double now) {
        if (config == null || state == null) {
            return;
        }
        if (!state.active) {
            return;
        }
        if (suppressUpdates) {
            return;
        }
        RobotLocalizationConfig.BackendConfig backend = resolveBackendForPose(config);
        boolean hasFreshVision = hasFreshVision(now, config.constraints(), state);
        boolean supportsVisionInput =
                config.continuousInputs().contains(PoseInput.VISION)
                        || config.onDemandInputs().contains(PoseInput.VISION);
        if (!config.constraints().canUpdate(now, state.lastUpdateSeconds, hasFreshVision, slipActive)) {
            return;
        }

        boolean appliedVision = false;
        if (supportsVisionInput) {
            appliedVision = applyVisionForConfig(config, state, backend, now);
        }
        if (config.constraints().requireVision() && !appliedVision) {
            return;
        }

        boolean updatedEstimator = false;
        if (config.continuousInputs().contains(PoseInput.ODOMETRY)
                || config.continuousInputs().contains(PoseInput.IMU_YAW)) {
            updatedEstimator = updateEstimator(state, backend, now);
        } else if (appliedVision) {
            updatedEstimator = refreshPoseFromEstimator(state, now);
        }

        if (updatedEstimator || appliedVision) {
            state.lastUpdateSeconds = now;
        }
    }

    private boolean hasFreshVision(double nowSeconds, PoseConstraints constraints, PoseEstimatorState state) {
        if (!state.hasAcceptedVisionMeasurement) {
            return false;
        }
        double maxLatency = constraints.maxLatencySeconds();
        if (maxLatency > 0.0) {
            return nowSeconds - state.lastVisionMeasurementTimestamp <= maxLatency;
        }
        return true;
    }

    private RobotLocalizationConfig.BackendConfig resolveBackendForPose(PoseConfig config) {
        if (config == null || config.backendOverride() == null) {
            return backendConfig();
        }
        return config.backendOverride();
    }

    private boolean applyVisionForConfig(
            PoseConfig config,
            PoseEstimatorState state,
            RobotLocalizationConfig.BackendConfig backend,
            double nowSeconds) {
        if (vision == null || !isVisionEnabled(backend)) {
            return false;
        }
        if (Double.isFinite(state.lastVisionFusionUpdateSeconds)
                && Math.abs(nowSeconds - state.lastVisionFusionUpdateSeconds) < 1e-6) {
            return false;
        }
        state.lastVisionFusionUpdateSeconds = nowSeconds;
        Pose2d referencePose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
        vision.measurements().robotOrientation(referencePose);
        boolean accepted = false;
        if (backend.useMultiVision()) {
            Optional<VisionCamera.VisionMeasurement> measurement =
                    fuseVisionMeasurements(state, referencePose, vision.measurements().measurements(), backend);
            if (measurement.isPresent() && applyVisionMeasurement(config, state, measurement.get(), backend, nowSeconds)) {
                state.hasAcceptedVisionMeasurement = true;
                state.lastVisionMeasurementTimestamp = measurement.get().timestampSeconds();
                accepted = true;
            }
        } else {
            Optional<VisionCamera.VisionMeasurement> measurement = vision.measurements().bestMeasurement();
            if (measurement.isPresent() && applyVisionMeasurement(config, state, measurement.get(), backend, nowSeconds)) {
                state.hasAcceptedVisionMeasurement = true;
                state.lastVisionMeasurementTimestamp = measurement.get().timestampSeconds();
                accepted = true;
            }
        }
        return accepted;
    }

    private boolean updateEstimator(
            PoseEstimatorState state,
            RobotLocalizationConfig.BackendConfig backend,
            double now) {
        if (Math.abs(now - state.lastEstimatorUpdateSeconds) < 1e-6) {
            return false;
        }
        boolean isPrimary = state == getPrimaryPoseState();
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Rotation3d rotation = getRotation3d("field", backend);
            PoseEstimator3d<T> estimator = state.estimator3d;
            if (estimator == null) {
                return false;
            }
            state.pose3d = estimator.update(rotation, wheelPositions.get());
            state.pose2d = state.pose3d.toPose2d();
            if (isPrimary) {
                applySlipAwareEstimatorCorrection3d(state, estimator, rotation, now);
            }
        } else {
            PoseEstimator<T> estimator = state.estimator2d;
            if (estimator == null) {
                return false;
            }
            Rotation2d yaw = getYaw("field", backend);
            state.pose2d = estimator.update(yaw, wheelPositions.get());
            state.pose3d = new Pose3d(state.pose2d);
            if (isPrimary) {
                applySlipAwareEstimatorCorrection2d(state, estimator, yaw, now);
            }
        }
        state.lastEstimatorUpdateSeconds = now;
        onPoseUpdated(state, now);
        return true;
    }

    private void applySlipAwareEstimatorCorrection2d(
            PoseEstimatorState state,
            PoseEstimator<T> estimator,
            Rotation2d yaw,
            double nowSeconds) {
        Pose2d rawPose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
        Translation2d correctedTranslation = computeSlipAwareFieldTranslation(rawPose, yaw, nowSeconds);
        if (correctedTranslation == null) {
            return;
        }
        if (correctedTranslation.getDistance(rawPose.getTranslation()) < 1e-5) {
            return;
        }
        Pose2d correctedPose = new Pose2d(correctedTranslation, yaw);
        estimator.resetPosition(yaw, wheelPositions.get(), correctedPose);
        state.pose2d = correctedPose;
        state.pose3d = new Pose3d(correctedPose);
    }

    private void applySlipAwareEstimatorCorrection3d(
            PoseEstimatorState state,
            PoseEstimator3d<T> estimator,
            Rotation3d rotation,
            double nowSeconds) {
        Pose3d rawPose3d = state.pose3d != null ? state.pose3d : new Pose3d();
        Pose2d rawPose2d = rawPose3d.toPose2d();
        Translation2d correctedTranslation =
                computeSlipAwareFieldTranslation(rawPose2d, rawPose2d.getRotation(), nowSeconds);
        if (correctedTranslation == null) {
            return;
        }
        if (correctedTranslation.getDistance(rawPose2d.getTranslation()) < 1e-5) {
            return;
        }
        Pose3d correctedPose = new Pose3d(
                correctedTranslation.getX(),
                correctedTranslation.getY(),
                rawPose3d.getZ(),
                rotation);
        estimator.resetPosition(rotation, wheelPositions.get(), correctedPose);
        state.pose3d = correctedPose;
        state.pose2d = correctedPose.toPose2d();
    }

    private Translation2d computeSlipAwareFieldTranslation(Pose2d rawPose, Rotation2d heading, double nowSeconds) {
        Pose2d resolvedRawPose = rawPose != null ? rawPose : ZERO_POSE_2D;
        Rotation2d resolvedHeading = heading != null ? heading : resolvedRawPose.getRotation();
        boolean contactAssistActive = slipContactComponent >= CONTACT_ASSIST_COMPONENT;
        boolean reversalAssistActive = slipReversalComponent >= SLIP_REVERSAL_SUSTAIN_COMPONENT * 0.75;
        boolean tractionAssistActive =
                slipMode == SlipMode.TRANSIENT
                        || slipMode == SlipMode.SLIP
                        || slipMode == SlipMode.RECOVER
                        || contactAssistActive
                        || reversalAssistActive;
        if (!tractionAssistActive
                || (slipScore < SLIP_TRANSIENT_EXIT_SCORE && !contactAssistActive && !reversalAssistActive)) {
            lastRawEstimatorPoseForSlipFusion = resolvedRawPose;
            lastCorrectedPoseForSlipFusion = resolvedRawPose;
            lastSlipFusionTimestamp = nowSeconds;
            return resolvedRawPose.getTranslation();
        }
        if (!Double.isFinite(lastSlipFusionTimestamp)) {
            lastRawEstimatorPoseForSlipFusion = resolvedRawPose;
            lastCorrectedPoseForSlipFusion = resolvedRawPose;
            lastSlipFusionTimestamp = nowSeconds;
            return resolvedRawPose.getTranslation();
        }
        double dt = nowSeconds - lastSlipFusionTimestamp;
        if (!Double.isFinite(dt) || dt <= 1e-6 || dt > SLIP_DT_MAX_SECONDS) {
            lastRawEstimatorPoseForSlipFusion = resolvedRawPose;
            lastCorrectedPoseForSlipFusion = resolvedRawPose;
            lastSlipFusionTimestamp = nowSeconds;
            return resolvedRawPose.getTranslation();
        }

        Translation2d rawOdomVelocity =
                resolvedRawPose.getTranslation()
                        .minus(lastRawEstimatorPoseForSlipFusion.getTranslation())
                        .times(1.0 / dt);
        Translation2d imuFieldVelocity = computeImuFieldVelocity(resolvedHeading);
        double maxTrustedImuSpeed = 6.0;
        if (robotSpeeds != null) {
            double configuredMax = robotSpeeds.getMaxVelocity();
            if (Double.isFinite(configuredMax) && configuredMax > 0.0) {
                maxTrustedImuSpeed = Math.max(0.75, configuredMax * 1.35);
            }
        }
        if (imuFieldVelocity.getNorm() > maxTrustedImuSpeed) {
            imuFieldVelocity = new Translation2d();
        }
        Translation2d commandedFieldVelocity = computeCommandedFieldVelocity(resolvedHeading);
        double commandedNorm = commandedFieldVelocity.getNorm();
        Translation2d commandDirection = null;
        if (commandedNorm > 0.2) {
            commandDirection = commandedFieldVelocity.times(1.0 / commandedNorm);
        }

        double odomWeight = 1.0 - slipScore;
        if (!imuLinearSignalObserved) {
            odomWeight = 1.0;
        }
        switch (slipMode) {
            case SLIP:
                odomWeight = Math.max(odomWeight, 0.50);
                break;
            case TRANSIENT:
                odomWeight = Math.max(odomWeight, 0.85);
                break;
            case RECOVER:
                odomWeight = Math.max(odomWeight, 0.92);
                break;
            default:
                break;
        }

        if (commandDirection != null) {
            double imuParallel = imuFieldVelocity.getX() * commandDirection.getX()
                    + imuFieldVelocity.getY() * commandDirection.getY();
            if (reversalAssistActive && imuParallel < -REVERSAL_INERTIA_IMU_OPPOSE_MPS) {
                double opposeNorm = clamp01(
                        (-imuParallel - REVERSAL_INERTIA_IMU_OPPOSE_MPS) / Math.max(commandedNorm, 0.2));
                double reversalBlend = Math.max(
                        opposeNorm,
                        clamp01((slipReversalComponent - 0.15) / 0.85));
                odomWeight = Math.min(odomWeight, MathUtil.interpolate(0.45, 0.08, reversalBlend));
            }
        }

        if (slipContactComponent >= CONTACT_LOCK_ENTER_COMPONENT
                && imuFieldVelocity.getNorm() < CONTACT_LOCK_IMU_SPEED_MPS) {
            double contactLockBlend = clamp01(
                    (slipContactComponent - CONTACT_LOCK_ENTER_COMPONENT)
                            / (1.0 - CONTACT_LOCK_ENTER_COMPONENT));
            odomWeight = Math.min(
                    odomWeight,
                    MathUtil.interpolate(0.25, CONTACT_LOCK_MAX_ODOM_WEIGHT, contactLockBlend));
        }

        odomWeight = clamp01(odomWeight);
        Translation2d fusedVelocity =
                rawOdomVelocity.times(odomWeight).plus(imuFieldVelocity.times(1.0 - odomWeight));

        if (commandedNorm > 0.2) {
            Translation2d direction = commandedFieldVelocity.times(1.0 / commandedNorm);
            double fusedParallel = fusedVelocity.getX() * direction.getX() + fusedVelocity.getY() * direction.getY();
            Translation2d fusedPerpendicular = fusedVelocity.minus(direction.times(fusedParallel));
            double odomParallel = rawOdomVelocity.getX() * direction.getX() + rawOdomVelocity.getY() * direction.getY();
            double imuParallel = imuFieldVelocity.getX() * direction.getX() + imuFieldVelocity.getY() * direction.getY();

            if (slipContactComponent > 0.35) {
                double contactBlend = clamp01((slipContactComponent - 0.35) / 0.65);
                double imuOrZeroParallel = Math.abs(imuParallel) < 0.25 ? 0.0 : imuParallel;
                double targetParallel = MathUtil.interpolate(imuParallel, imuOrZeroParallel, contactBlend);
                targetParallel = MathUtil.interpolate(targetParallel, 0.0, contactBlend * 0.85);
                fusedParallel = MathUtil.interpolate(fusedParallel, targetParallel, contactBlend);
            }

            if (slipContactComponent >= CONTACT_LOCK_ENTER_COMPONENT
                    && Math.abs(imuParallel) < CONTACT_LOCK_IMU_SPEED_MPS) {
                double lockBlend = clamp01(
                        (slipContactComponent - CONTACT_LOCK_ENTER_COMPONENT)
                                / (1.0 - CONTACT_LOCK_ENTER_COMPONENT));
                fusedParallel = MathUtil.interpolate(fusedParallel, 0.0, lockBlend);
                fusedPerpendicular = fusedPerpendicular.times(1.0 - (0.80 * lockBlend));
            }

            if (slipReversalComponent > 0.20) {
                double reversalBlend = clamp01((slipReversalComponent - 0.20) / 0.80);
                double opposingClamp = MathUtil.interpolate(
                        REVERSAL_OPPOSING_VELOCITY_MAX_MPS,
                        REVERSAL_OPPOSING_VELOCITY_MIN_MPS,
                        reversalBlend);
                if (fusedParallel < -opposingClamp) {
                    fusedParallel = -opposingClamp;
                }
                if (odomParallel < -0.15 && Math.abs(imuParallel) < 0.20) {
                    fusedParallel = Math.max(fusedParallel, 0.0);
                }
            }

            if (reversalAssistActive && imuParallel < -REVERSAL_INERTIA_IMU_OPPOSE_MPS) {
                double opposeBlend = clamp01(
                        (-imuParallel - REVERSAL_INERTIA_IMU_OPPOSE_MPS) / Math.max(commandedNorm, 0.2));
                double positiveAllowance = MathUtil.interpolate(
                        REVERSAL_NEW_DIRECTION_ALLOWANCE_MPS,
                        0.0,
                        opposeBlend);
                if (fusedParallel > positiveAllowance) {
                    fusedParallel = positiveAllowance;
                }
            }

            if (slipScore > 0.55 && fusedParallel < 0.0) {
                double opposingClamp = MathUtil.interpolate(
                        0.15,
                        0.03,
                        clamp01((slipScore - 0.55) / 0.45));
                fusedParallel = Math.max(fusedParallel, -opposingClamp);
            }

            fusedVelocity = fusedPerpendicular.plus(direction.times(fusedParallel));
        } else if (slipContactComponent > 0.55 || slipReversalComponent > 0.45) {
            double damp = MathUtil.interpolate(
                    1.0,
                    0.25,
                    Math.max(slipContactComponent, slipReversalComponent));
            fusedVelocity = fusedVelocity.times(damp);
        }

        Translation2d correctedTranslation =
                lastCorrectedPoseForSlipFusion.getTranslation().plus(fusedVelocity.times(dt));
        Pose2d correctedPose = new Pose2d(correctedTranslation, resolvedHeading);
        lastRawEstimatorPoseForSlipFusion = resolvedRawPose;
        lastCorrectedPoseForSlipFusion = correctedPose;
        lastSlipFusionTimestamp = nowSeconds;
        return correctedTranslation;
    }

    private Translation2d computeImuFieldVelocity(Rotation2d heading) {
        double imuRobotVx = finiteOrZero(imu.getXSpeedMetersPerSecond());
        double imuRobotVy = finiteOrZero(imu.getYSpeedMetersPerSecond());
        double cos = heading.getCos();
        double sin = heading.getSin();
        return new Translation2d(
                imuRobotVx * cos - imuRobotVy * sin,
                imuRobotVx * sin + imuRobotVy * cos);
    }

    private Translation2d computeCommandedFieldVelocity(Rotation2d heading) {
        ChassisSpeeds commandedRobot = combinedCommandedRobotSpeeds(commandedSpeedsScratchB);
        double cos = heading.getCos();
        double sin = heading.getSin();
        return new Translation2d(
                commandedRobot.vxMetersPerSecond * cos - commandedRobot.vyMetersPerSecond * sin,
                commandedRobot.vxMetersPerSecond * sin + commandedRobot.vyMetersPerSecond * cos);
    }

    private boolean refreshPoseFromEstimator(PoseEstimatorState state, double nowSeconds) {
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            PoseEstimator3d<T> estimator = state.estimator3d;
            if (estimator == null) {
                return false;
            }
            state.pose3d = estimator.getEstimatedPosition();
            state.pose2d = state.pose3d.toPose2d();
        } else {
            PoseEstimator<T> estimator = state.estimator2d;
            if (estimator == null) {
                return false;
            }
            state.pose2d = estimator.getEstimatedPosition();
            state.pose3d = new Pose3d(state.pose2d);
        }
        onPoseUpdated(state, nowSeconds);
        return true;
    }

    private void onPoseUpdated(PoseEstimatorState state, double nowSeconds) {
        if (state == null) {
            return;
        }
        boolean isPrimary = state == getPrimaryPoseState();
        Pose2d pose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
        if (isPrimary) {
            setPrimaryPose(state);
            updateVelocityTracking(pose, nowSeconds);
            fieldPublisher.updateActualPath(pose);
            RobotNetworkTables nt = robotNetworkTables;
            boolean fieldEnabled = nt != null && nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET);
            if (fieldEnabled) {
                fieldPublisher.setRobotPose(pose);
            }
            PoseConfig config = primaryPoseName != null ? poseConfigs.get(primaryPoseName) : null;
            publishPoseStruct(config, pose);
        } else {
            PoseConfig config = findConfigForState(state);
            if (config != null) {
                publishPoseStruct(config, pose);
            }
        }
        if (isPrimary) {
            updateHealthMetrics(nowSeconds);
            RobotNetworkTables nt = robotNetworkTables;
            if (nt != null && nt.enabled(RobotNetworkTables.Flag.VISION_CAMERA_WIDGETS)) {
                cameraManager.updateCameraVisualizations(vision, fieldPose);
            }
            updateSlipState(nowSeconds);
            persistRobotState(false);
        }
    }

    private boolean applyVisionMeasurement(
            PoseConfig config,
            PoseEstimatorState state,
            VisionCamera.VisionMeasurement measurement,
            RobotLocalizationConfig.BackendConfig backend,
            double nowSeconds) {
        if (measurement == null) {
            return false;
        }
        boolean accepted = false;
        Matrix<N3, N1> rawStdDevs = measurement.stdDevs();
        Matrix<N3, N1> sanitizedStdDevs = sanitizeVisionStdDevs(rawStdDevs);
        if (rawStdDevs != null && sanitizedStdDevs == null) {
            recordVisionSample(config, false);
            return false;
        }
        Pose2d measurementPose = measurement.pose2d();
        Pose2d referencePose = state.pose2d != null ? state.pose2d : ZERO_POSE_2D;
        if (!passesPoseJumpGuard(state, referencePose, measurementPose, false, backend, nowSeconds)) {
            recordVisionSample(config, false);
            return false;
        }
        double timestampSeconds = measurement.timestampSeconds();
        if (!shouldApplyVisionMeasurement(state, referencePose, measurementPose, timestampSeconds, sanitizedStdDevs)) {
            recordVisionSample(config, false);
            return false;
        }
        Matrix<N3, N1> adjustedStdDevs2d =
                adjustVisionStdDevsForDisagreement(sanitizedStdDevs, measurementPose, referencePose);
        if (slipActive) {
            double scale = backend.slipVisionStdDevScale();
            if (scale > 0.0 && scale < 1.0) {
                adjustedStdDevs2d = scaleStdDevs(adjustedStdDevs2d, scale);
            }
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            PoseEstimator3d<?> estimator = state.estimator3d;
            if (estimator == null) {
                return false;
            }
            Matrix<N4, N1> adjustedStdDevs3d = expandStdDevsTo3d(adjustedStdDevs2d);
            Pose3d measurementPose3d = measurement.pose3d() != null
                    ? measurement.pose3d()
                    : new Pose3d(measurementPose);
            if (adjustedStdDevs3d != null) {
                estimator.addVisionMeasurement(measurementPose3d, timestampSeconds, adjustedStdDevs3d);
            } else {
                estimator.addVisionMeasurement(measurementPose3d, timestampSeconds);
            }
        } else {
            PoseEstimator<T> estimator = state.estimator2d;
            if (estimator == null) {
                return false;
            }
            if (adjustedStdDevs2d != null) {
                estimator.addVisionMeasurement(measurementPose, timestampSeconds, adjustedStdDevs2d);
            } else {
                estimator.addVisionMeasurement(measurementPose, timestampSeconds);
            }
        }
        accepted = true;
        recordVisionSample(config, true);
        return true;
    }

    private boolean passesPoseJumpGuard(
            PoseEstimatorState state,
            Pose2d referencePose,
            Pose2d measurementPose,
            boolean hasAgreement,
            RobotLocalizationConfig.BackendConfig backend,
            double now) {
        if (!hasAgreement && now < state.poseJumpGuardUntilSeconds) {
            return false;
        }
        double threshold = backend.poseJumpMeters();
        if (threshold <= 0.0) {
            return true;
        }
        double distance = measurementPose.getTranslation().getDistance(referencePose.getTranslation());
        if (!hasAgreement && distance > threshold) {
            state.poseJumpGuardUntilSeconds = now + backend.poseJumpHoldSeconds();
            return false;
        }
        return true;
    }

    private void recordVisionSample(PoseConfig config, boolean accepted) {
        if (!isPrimaryPose(config)) {
            return;
        }
        visionMeasurementCount++;
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
        if (accepted) {
            visionMeasurementAccepted++;
        }
    }

    private Matrix<N3, N1> scaleStdDevs(Matrix<N3, N1> stdDevs, double scale) {
        return RobotLocalizationVisionUtil.scaleStdDevs(stdDevs, scale);
    }

    private Optional<VisionCamera.VisionMeasurement> fuseVisionMeasurements(
            PoseEstimatorState state,
            Pose2d referencePose,
            List<VisionCamera.VisionMeasurement> measurements,
            RobotLocalizationConfig.BackendConfig backend) {
        if (measurements == null || measurements.isEmpty()) {
            return Optional.empty();
        }
        boolean hasAgreement = RobotLocalizationVisionUtil.hasPoseAgreement(
                measurements,
                backend.poseJumpAgreementMeters());
        measurements.sort(Comparator.comparingDouble(VisionCamera.VisionMeasurement::timestampSeconds));
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

        for (VisionCamera.VisionMeasurement measurement : measurements) {
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
        Pose2d basePose = referencePose != null ? referencePose : new Pose2d();
        if (!passesPoseJumpGuard(
                state,
                basePose,
                fusedPose,
                hasAgreement,
                backend,
                currentTimestampSeconds())) {
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

        return Optional.of(new VisionCamera.VisionMeasurement(
                fusedPose,
                new Pose3d(fusedPose),
                fusedTimestamp,
                fusedLatency,
                fusedStdDevs,
                fusedConfidence,
                fusedDistance,
                1.0));
    }

    private final class PoseEstimatorState {
        private PoseEstimator<T> estimator2d;
        private PoseEstimator3d<T> estimator3d;
        private Pose2d pose2d = new Pose2d();
        private Pose3d pose3d = new Pose3d();
        private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;
        private double lastEstimatorUpdateSeconds = Double.NEGATIVE_INFINITY;
        private double lastVisionFusionUpdateSeconds = Double.NEGATIVE_INFINITY;
        private boolean hasAcceptedVisionMeasurement = false;
        private double lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;
        private double poseJumpGuardUntilSeconds = Double.NEGATIVE_INFINITY;
        private boolean active = true;
    }

}
