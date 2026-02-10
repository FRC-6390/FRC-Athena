package ca.frc6390.athena.core.localization;

import java.lang.reflect.Field;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import ca.frc6390.athena.core.RobotAuto;
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
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.core.RobotNetworkTables;
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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private ChoreoAutoFactory choreoFactory;

    private BiConsumer<ChassisSpeeds, HolonomicFeedforward> autoDrive;

    private boolean suppressUpdates = false;
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
    private double minVisionUpdateSeparationSeconds = 0.02;
    private final RobotLocalizationConfig.PoseSpace poseSpace;
    private final Map<String, PoseConfig> poseConfigs = new HashMap<>();
    private final Map<String, PoseEstimatorState> poseStates = new HashMap<>();
    private final Map<String, StructPublisher<Pose2d>> poseStructPublishers = new HashMap<>();
    private String primaryPoseName;
    private boolean slipActive = false;
    private double slipActiveUntilSeconds = Double.NEGATIVE_INFINITY;
    private double lastUpdateTimestamp = Double.NaN;
    private Pose2d lastFieldPoseForSlip = new Pose2d();
    private Translation2d lastFieldVelocityForSlip = new Translation2d();
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
        this.fieldPose = new Pose2d();
        this.fieldPose3d = new Pose3d(fieldPose);
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
        List<PoseConfig> configs = config.poseConfigs();
        if (configs.isEmpty()) {
            registerPoseConfig(defaultPoseConfig());
        } else {
            for (PoseConfig poseConfig : configs) {
                registerPoseConfig(poseConfig);
            }
        }
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
        cameraManager.ensureCameraEntries(vision);
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
                backend != null ? backend : RobotLocalizationConfig.BackendConfig.defualt();
        if (resolved.imuStrategy() == RobotLocalizationConfig.BackendConfig.ImuStrategy.RAW_YAW) {
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

    private boolean isVisionEnabled(RobotLocalizationConfig.BackendConfig backend) {
        RobotLocalizationConfig.BackendConfig resolved =
                backend != null ? backend : RobotLocalizationConfig.BackendConfig.defualt();
        return resolved.resolveVisionEnabled(visionEnabled);
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

    // Backend override values are edited directly via NetworkTables under Athena/Localization/Backend.

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
                    pose -> resetPose(localizationConfig.autoPoseName(), pose),
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
                pose -> resetPose(localizationConfig.autoPoseName(), pose),
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
                        Math.hypot(imu.getAccelerationX(), imu.getAccelerationY());
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
            Pose2d pose = state.pose2d != null ? state.pose2d : new Pose2d();
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
        applyPoseConfigUpdate(config, config.setActive(active));
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
            return new Pose2d();
        }
        PoseEstimatorState state = ensurePoseState(config);
        updatePoseFromConfig(config, state);
        return state.pose2d != null ? state.pose2d : new Pose2d();
    }

    public Pose3d getPose3d(String name) {
        PoseConfig config = poseConfigs.get(name);
        if (config == null) {
            return new Pose3d();
        }
        PoseEstimatorState state = ensurePoseState(config);
        updatePoseFromConfig(config, state);
        return state.pose3d != null ? state.pose3d : new Pose3d();
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
            return new Pose2d();
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
        return new Pose2d();
    }

    private Pose3d resolveInitialPose3d(PoseConfig config) {
        if (config == null) {
            return new Pose3d();
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
        return new Pose3d();
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
        fieldPose = state.pose2d != null ? state.pose2d : new Pose2d();
        fieldPose3d = state.pose3d != null ? state.pose3d : new Pose3d(fieldPose);
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

    public Pose2d getFieldPose(){
        PoseEstimatorState primary = getPrimaryPoseState();
        if (primary != null) {
            setPrimaryPose(primary);
        }
        return fieldPose;
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
        this.suppressUpdates = suppressUpdates;
    }

    public Field2d getField2d() {
        return fieldPublisher.getField();
    }

    public Field2d getVisionField2d() {
        return cameraManager.getVisionField();
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

        // Field2d is a localization concern (not vision). Publish it only when explicitly enabled.
        if (nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET)) {
            if (!fieldPublisher.isFieldPublished()) {
                // Elastic/Glass can bind a Field2d widget to this SmartDashboard entry.
                SmartDashboard.putData("Athena Localization Field2d", fieldPublisher.getField());
                fieldPublisher.publishFieldOnce();
            }
            fieldPublisher.setRobotPose(pose);
        }

        if (nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_HEALTH_WIDGETS)) {
            RobotNetworkTables.Node health = node.child("Health");
            health.putDouble("poseJumpMeters", lastPoseJumpMeters);
            health.putDouble("driftRateMetersPerSec", driftRateMetersPerSec);
            health.putDouble("visionAcceptRate", visionAcceptRateWindow);
            health.putBoolean("slipActive", slipActive);
        }

        return node;
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

    @Override
    public void periodic() {
        if (suppressUpdates) {
            return;
        }
        updateActivePoseConfigs();
        publishPoseObjects();
    }

    private void updateActivePoseConfigs() {
        for (PoseConfig config : poseConfigs.values()) {
            PoseEstimatorState state = ensurePoseState(config);
            if (state.active) {
                updatePoseFromConfig(config, state);
            }
        }
    }

    private void publishPoseObjects() {
        fieldPublisher.clearRobotPose();
        for (PoseConfig config : poseConfigs.values()) {
            PoseEstimatorState state = poseStates.get(config.name());
            if (state == null) {
                continue;
            }
            Pose2d pose = state.pose2d != null ? state.pose2d : new Pose2d();
            publishPoseStruct(config, pose);
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
        }
        publisher.set(pose);
    }

    private void updatePoseFromConfig(PoseConfig config, PoseEstimatorState state) {
        if (config == null || state == null) {
            return;
        }
        if (!state.active) {
            return;
        }
        if (suppressUpdates) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        RobotLocalizationConfig.BackendConfig backend = resolveBackendForPose(config);
        boolean hasFreshVision = hasFreshVision(now, config.constraints(), state);
        if (!config.constraints().canUpdate(now, state.lastUpdateSeconds, hasFreshVision, slipActive)) {
            return;
        }

        boolean appliedVision = false;
        if (config.onDemandInputs().contains(PoseInput.VISION)) {
            appliedVision = applyVisionForConfig(config, state, backend, now);
        }
        if (config.constraints().requireVision() && !appliedVision) {
            return;
        }

        boolean updatedEstimator = false;
        if (config.continuousInputs().contains(PoseInput.ODOMETRY)
                || config.continuousInputs().contains(PoseInput.IMU_YAW)) {
            updatedEstimator = updateEstimator(state, backend);
        } else if (appliedVision) {
            updatedEstimator = refreshPoseFromEstimator(state);
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
        Pose2d referencePose = state.pose2d != null ? state.pose2d : new Pose2d();
        vision.setRobotOrientation(referencePose);
        boolean accepted = false;
        if (backend.useMultiVision()) {
            Optional<VisionCamera.VisionMeasurement> measurement =
                    fuseVisionMeasurements(state, referencePose, vision.getVisionMeasurements(), backend);
            if (measurement.isPresent() && applyVisionMeasurement(config, state, measurement.get(), backend)) {
                state.hasAcceptedVisionMeasurement = true;
                state.lastVisionMeasurementTimestamp = measurement.get().timestampSeconds();
                accepted = true;
            }
        } else {
            Optional<VisionCamera.VisionMeasurement> measurement = vision.getBestVisionMeasurement();
            if (measurement.isPresent() && applyVisionMeasurement(config, state, measurement.get(), backend)) {
                state.hasAcceptedVisionMeasurement = true;
                state.lastVisionMeasurementTimestamp = measurement.get().timestampSeconds();
                accepted = true;
            }
        }
        return accepted;
    }

    private boolean updateEstimator(PoseEstimatorState state, RobotLocalizationConfig.BackendConfig backend) {
        double now = Timer.getFPGATimestamp();
        if (Math.abs(now - state.lastEstimatorUpdateSeconds) < 1e-6) {
            return false;
        }
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Rotation3d rotation = getRotation3d("field", backend);
            PoseEstimator3d<T> estimator = state.estimator3d;
            if (estimator == null) {
                return false;
            }
            state.pose3d = estimator.update(rotation, wheelPositions.get());
            state.pose2d = state.pose3d.toPose2d();
        } else {
            PoseEstimator<T> estimator = state.estimator2d;
            if (estimator == null) {
                return false;
            }
            state.pose2d = estimator.update(getYaw("field", backend), wheelPositions.get());
            state.pose3d = new Pose3d(state.pose2d);
        }
        state.lastEstimatorUpdateSeconds = now;
        onPoseUpdated(state);
        return true;
    }

    private boolean refreshPoseFromEstimator(PoseEstimatorState state) {
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
        onPoseUpdated(state);
        return true;
    }

    private void onPoseUpdated(PoseEstimatorState state) {
        if (state == null) {
            return;
        }
        boolean isPrimary = state == getPrimaryPoseState();
        Pose2d pose = state.pose2d != null ? state.pose2d : new Pose2d();
        if (isPrimary) {
            setPrimaryPose(state);
            RobotNetworkTables nt = robotNetworkTables;
            boolean fieldEnabled = nt != null && nt.enabled(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET);
            if (fieldEnabled) {
                fieldPublisher.setRobotPose(pose);
                fieldPublisher.updateActualPath(pose);
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
            updateHealthMetrics();
            RobotNetworkTables nt = robotNetworkTables;
            if (nt != null && nt.enabled(RobotNetworkTables.Flag.VISION_CAMERA_WIDGETS)) {
                cameraManager.updateCameraVisualizations(vision, fieldPose);
            }
            updateSlipState();
            persistRobotState(false);
        }
    }

    private boolean applyVisionMeasurement(
            PoseConfig config,
            PoseEstimatorState state,
            VisionCamera.VisionMeasurement measurement,
            RobotLocalizationConfig.BackendConfig backend) {
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
        Pose2d referencePose = state.pose2d != null ? state.pose2d : new Pose2d();
        if (!passesPoseJumpGuard(state, referencePose, measurementPose, false, backend)) {
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
            RobotLocalizationConfig.BackendConfig backend) {
        double now = Timer.getFPGATimestamp();
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
        if (!passesPoseJumpGuard(state, basePose, fusedPose, hasAgreement, backend)) {
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
