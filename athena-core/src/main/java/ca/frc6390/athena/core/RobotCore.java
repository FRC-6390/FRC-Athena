package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.concurrent.ConcurrentHashMap;
import java.io.File;
import java.io.IOException;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.lang.management.RuntimeMXBean;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.concurrent.TimeUnit;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.loop.TimedRunner;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.localization.RobotDrivetrainLocalizationFactory;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanismFactory;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCore<T extends RobotDrivetrain<T>> extends TimedRobot {
    private static volatile RobotCore<?> activeInstance;
    private static final String AUTO_PROGRAM_CHOOSER_KEY = "ProgramChooser";
    private static final Pose2d[] EMPTY_POSE2D_ARRAY = new Pose2d[0];
    private static final String EMPTY_AUTO_TRAJECTORY_SIGNATURE = "";

    public enum RuntimeMode {
        AUTO,
        TELEOP,
        DISABLED,
        TEST
    }

    public record AutoConfig(
            HolonomicPidConstants translationPid,
            HolonomicPidConstants rotationPid,
            String poseName,
            RobotLocalizationConfig.AutoPlannerPidAutotunerConfig pidAutotunerConfig,
            List<Consumer<RobotAuto.RegistrySection>> registryBindings) {

        public AutoConfig {
            poseName = normalizePoseName(poseName);
            registryBindings = registryBindings != null ? List.copyOf(registryBindings) : List.of();
        }

        public static AutoConfig defaults() {
            return new AutoConfig(null, null, null, null, List.of());
        }

        public AutoConfig pid(
                double tP,
                double tI,
                double tD,
                double rP,
                double rI,
                double rD) {
            return pid(new HolonomicPidConstants(tP, tI, tD), new HolonomicPidConstants(rP, rI, rD));
        }

        public AutoConfig pid(HolonomicPidConstants translation, HolonomicPidConstants rotation) {
            return new AutoConfig(translation, rotation, poseName, pidAutotunerConfig, registryBindings);
        }

        public AutoConfig pid(Consumer<RobotLocalizationConfig.AutoPlannerPidSection> section) {
            RobotLocalizationConfig seed = RobotLocalizationConfig.defaults();
            if (translationPid != null || rotationPid != null || pidAutotunerConfig != null) {
                seed.planner().autoPlannerPid(spec -> {
                    if (translationPid != null) {
                        setPidAxis(spec.translation(), translationPid);
                    }
                    if (rotationPid != null) {
                        setPidAxis(spec.rotation(), rotationPid);
                    }
                    if (pidAutotunerConfig != null) {
                        spec.autotunerConfig(cfg -> cfg
                                .enabled(pidAutotunerConfig.enabled())
                                .dashboardPath(pidAutotunerConfig.dashboardPath())
                                .program(pidAutotunerConfig.program()));
                    }
                });
            }
            seed.planner().autoPlannerPid(section);
            return new AutoConfig(
                    seed.translation(),
                    seed.rotation(),
                    poseName,
                    seed.autoPlannerPidAutotuner(),
                    registryBindings);
        }

        public AutoConfig pose(String poseName) {
            return new AutoConfig(translationPid, rotationPid, poseName, pidAutotunerConfig, registryBindings);
        }

        public AutoConfig registry(Consumer<RobotAuto.RegistrySection> section) {
            if (section == null) {
                return this;
            }
            List<Consumer<RobotAuto.RegistrySection>> merged = new ArrayList<>(registryBindings);
            merged.add(section);
            return new AutoConfig(translationPid, rotationPid, poseName, pidAutotunerConfig, merged);
        }

        private static String normalizePoseName(String poseName) {
            if (poseName == null) {
                return null;
            }
            String trimmed = poseName.trim();
            return trimmed.isEmpty() ? null : trimmed;
        }

        private static void setPidAxis(
                RobotLocalizationConfig.PidAxisSection axis,
                HolonomicPidConstants constants) {
            if (axis == null || constants == null) {
                return;
            }
            axis.kp(constants.kP()).ki(constants.kI()).kd(constants.kD()).iZone(constants.iZone());
        }
    }

    public record SystemConfig(
            boolean tweaksEnabled,
            int vmOvercommitMode,
            int vmOvercommitRatio,
            int vmSwappiness,
            boolean loopSwapEnabled,
            int loopSwapSizeMiB,
            boolean systemWebServerEnabled,
            boolean configServerEnabled,
            boolean telemetryEnabled,
            boolean networkTablesDataLogEnabled) {

        public static SystemConfig defaults() {
            return new SystemConfig(
                    true,
                    1,
                    95,
                    120,
                    true,
                    32,
                    false,
                    false,
                    false,
                    false);
        }

        public static SystemConfig rioDefaults() {
            return new SystemConfig(
                    false,
                    RIO_DEFAULT_VM_OVERCOMMIT_MODE,
                    RIO_DEFAULT_VM_OVERCOMMIT_RATIO,
                    RIO_DEFAULT_VM_SWAPPINESS,
                    false,
                    32,
                    true,
                    true,
                    true,
                    false);
        }
    }

    public record RobotCoreConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain,
            RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig,
            boolean autoInitResetEnabled, TelemetryRegistry.TelemetryConfig telemetryConfig,
            List<RegisterableMechanism> mechanisms, boolean performanceMode,
            boolean timingDebugEnabled, boolean telemetryEnabled, AutoConfig autoConfig, RobotCoreHooks<T> hooks,
            SystemConfig systemConfig) {

        public RobotCoreConfig {
            mechanisms = mechanisms != null ? List.copyOf(mechanisms) : List.of();
            autoConfig = autoConfig != null ? autoConfig : AutoConfig.defaults();
            hooks = hooks != null ? hooks : RobotCoreHooks.<T>empty();
            systemConfig = systemConfig != null ? systemConfig : SystemConfig.defaults();
        }

        public RobotCoreConfig(
                RobotDrivetrainConfig<T> driveTrain,
                RobotLocalizationConfig localizationConfig,
                RobotVisionConfig visionConfig,
                boolean autoInitResetEnabled,
                TelemetryRegistry.TelemetryConfig telemetryConfig,
                List<RegisterableMechanism> mechanisms,
                boolean performanceMode,
                boolean timingDebugEnabled,
                boolean telemetryEnabled,
                AutoConfig autoConfig,
                RobotCoreHooks<T> hooks) {
            this(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    SystemConfig.defaults());
        }

        public static RobotCoreConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config) {
            return new RobotCoreConfig<>(
                    config,
                    RobotLocalizationConfig.defaults(),
                    RobotVisionConfig.defaults(),
                    true,
                    TelemetryRegistry.TelemetryConfig.defaults(),
                    List.of(),
                    false,
                    false,
                    true,
                    AutoConfig.defaults(),
                    RobotCoreHooks.empty(),
                    SystemConfig.defaults());
        }

        public static RobotCoreConfig<DifferentialDrivetrain> differential(DifferentialDrivetrainConfig config) {
            return new RobotCoreConfig<>(
                    config,
                    RobotLocalizationConfig.defaults(),
                    RobotVisionConfig.defaults(),
                    true,
                    TelemetryRegistry.TelemetryConfig.defaults(),
                    List.of(),
                    false,
                    false,
                    true,
                    AutoConfig.defaults(),
                    RobotCoreHooks.empty(),
                    SystemConfig.defaults());
        }

        public RobotCoreConfig<T> localization(RobotLocalizationConfig localizationConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> vision(RobotVisionConfig visionConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> cameras(ConfigurableCamera... cameras) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    RobotVisionConfig.defaults().addCameras(cameras),
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> autoInitResetEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    enabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> telemetry(TelemetryRegistry.TelemetryConfig telemetryConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        /**
         * Adds mechanisms to be registered automatically during {@link RobotCore#robotInit()}.
         */
        public RobotCoreConfig<T> addMechanisms(RegisterableMechanism... entries) {
            if (entries == null || entries.length == 0) {
                return this;
            }
            List<RegisterableMechanism> merged = new ArrayList<>(mechanisms != null ? mechanisms : List.of());
            merged.addAll(Arrays.asList(entries));
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    merged,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> performanceMode(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    enabled,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> timingDebugEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    enabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> telemetryEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    enabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> auto(AutoConfig autoConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCoreConfig<T> hooks(java.util.function.Consumer<RobotCoreHooks.HooksSection<T>> section) {
            if (section == null) {
                return this;
            }
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks.hooks(section),
                    systemConfig);
        }

        public RobotCoreConfig<T> inputs(java.util.function.Consumer<RobotCoreHooks.InputsSection<T>> section) {
            if (section == null) {
                return this;
            }
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks.inputs(section),
                    systemConfig);
        }

        public RobotCoreConfig<T> system(SystemConfig systemConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig);
        }

        public RobotCore<T> create() {
            return new RobotCore<>(this);
        }
    }

    private final T drivetrain;
    private final RobotLocalization<?> localization;
    private final RobotVision vision;
    private final RobotNetworkTables robotNetworkTables = new RobotNetworkTables();
    private final RobotNetworkTables.Node mechanismsRootNode;
    private final RobotNetworkTables.Node autoRootNode;
    private RobotVisionSim visionSim;
    private Notifier visionSimNotifier;
    private final RobotAuto autos;
    private final RobotCopilot copilot;
    private AthenaRuntimeServer configServer;
    private volatile boolean configServerEnabled = true;
    private String configServerBaseUrl;
    private String cachedConfigExportBaseUrl = "";
    private ConfigExportUrls cachedConfigExportUrls = ConfigExportUrls.empty();
    private final ConfigServerSection configServerSection;
    private final SystemSection systemSection;
    private final DiagnosticsSection diagnosticsSection;
    private final Map<String, DiagnosticsChannel> diagnosticsChannels;
    private final HashMap<String, Mechanism> mechanisms;
    private final List<SuperstructureMechanism<?, ?>> registeredSuperstructures;
    private final HashMap<String, SuperstructureMechanism<?, ?>> superstructuresByName;
    private final RobotMechanisms mechanismView;
    private final Set<Mechanism> scheduledCustomPidMechanisms;
    private Command autonomousCommand;
    private final TelemetryRegistry telemetry;
    private boolean autoInitResetEnabled;
    private NetworkTableEntry autoInitResetEntry;
    private final StructArrayPublisher<Pose2d> selectedAutoTrajectoryPublisher;
    private NtSendablePublisher autoProgramChooserPublisher;
    private final List<Consumer<RobotAuto.RegistrySection>> configuredAutoRegistryBindings;
    private final List<RegisterableMechanism> configuredMechanisms;
    private boolean configuredMechanismsRegistered;
    private final boolean timingDebugEnabled;
    private final boolean telemetryEnabled;
    private boolean mechanismsNetworkTablesPublished;
    private boolean coreNetworkTablesPublished;
    private final Set<String> publishedMechanismsComp;
    private final Set<String> publishedSuperstructuresComp;
    private final List<String> mechanismPublishKeysBuffer;
    private final List<String> superstructurePublishKeysBuffer;
    private boolean mechanismPublishKeysDirty = true;
    private boolean superstructurePublishKeysDirty = true;
    private int mechanismPublishCursor = 0;
    private int superstructurePublishCursor = 0;
    private int remainingMechanismRefreshCount = 0;
    private int remainingSuperstructureRefreshCount = 0;
    private long lastMechanismsConfigRevision = -1;
    private double lastMechanismAutoPublishSeconds = Double.NaN;
    private long lastCoreNetworkTablesConfigRevision = -1;
    private double lastCoreNetworkTablesPublishSeconds = Double.NaN;
    private String lastPublishedAutoTrajectorySignature = EMPTY_AUTO_TRAJECTORY_SIGNATURE;
    private int lastPublishedAutoTrajectoryPointCount = -1;
    private final RuntimeMXBean runtimeMxBean;
    private final MemoryMXBean memoryMxBean;
    private final java.lang.management.OperatingSystemMXBean osMxBean;
    private final com.sun.management.OperatingSystemMXBean sunOsMxBean;
    private final List<GarbageCollectorMXBean> gcMxBeans;
    private double lastSlowPerformanceMetricsPublishSeconds = Double.NaN;
    private double lastVerySlowPerformanceMetricsPublishSeconds = Double.NaN;
    private double lastLoopSchedulerMs = Double.NaN;
    private double lastLoopTelemetryMs = Double.NaN;
    private double lastLoopLocalizationMs = Double.NaN;
    private double lastLoopUserMs = Double.NaN;
    private double lastLoopTotalMs = Double.NaN;
    private final RobotCoreHooks<T> coreHooks;
    private final RobotCoreContextImpl coreHookContext;
    private final Consumer<RobotCoreHooks.Binding<T>> timedBindingRunnerInvoker;
    private final TypedInputResolver hookInputResolver;
    private final List<TimedRunner<RobotCoreHooks.Binding<T>>> periodicHookRunners;
    private final List<TimedRunner<RobotCoreHooks.Binding<T>>> disabledPeriodicHookRunners;
    private final List<TimedRunner<RobotCoreHooks.Binding<T>>> teleopPeriodicHookRunners;
    private final List<TimedRunner<RobotCoreHooks.Binding<T>>> autonomousPeriodicHookRunners;
    private final List<TimedRunner<RobotCoreHooks.Binding<T>>> testPeriodicHookRunners;
    private final List<TimedRunner<RobotCoreHooks.ControlLoop<T>>> coreControlLoopRunners;
    private final Map<String, PIDController> coreControlLoopPids;
    private final Map<String, OutputType> coreControlLoopPidOutputTypes;
    private final Map<String, SimpleMotorFeedforward> coreControlLoopFeedforwards;
    private final Map<String, OutputType> coreControlLoopFeedforwardOutputTypes;
    private volatile RuntimeMode runtimeMode = RuntimeMode.DISABLED;
    private volatile boolean competitionStreamSheddingApplied = false;
    private static final double MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS = 0.05;
    private static final int MECHANISM_AUTO_PUBLISH_BATCH_SIZE = 2;
    private static final double PERFORMANCE_SLOW_METRICS_PERIOD_SECONDS = 1.0;
    private static final double PERFORMANCE_VERY_SLOW_METRICS_PERIOD_SECONDS = 5.0;
    private static final double COMPETITION_AUTO_PUBLISH_PERIOD_SECONDS = 0.2;
    private static final double STARTUP_LOG_THRESHOLD_SECONDS = 0.05;
    private static final String SYSTEM_WEBSERVER_BINARY_PATH = "/usr/local/natinst/share/NIWebServer/SystemWebServer";
    private static final String SYSTEM_WEBSERVER_PROCESS_NAME = "SystemWebServer";
    private static final String SYSTEM_WEBCONTAINER_PROCESS_NAME = "NIWebServiceContainer";
    private static final String SYSTEM_LOOP_SWAP_FILE_PATH = "/home/lvuser/athena-swap.img";
    private static final int SYSTEM_LOOP_SWAP_MIN_SIZE_MIB = 8;
    private static final int SYSTEM_LOOP_SWAP_MAX_SIZE_MIB = 256;
    private static final int SYSTEM_LOOP_SWAP_DEFAULT_SIZE_MIB = 32;
    private static final int SYSTEM_SWAPPINESS_MIN = 0;
    private static final int SYSTEM_SWAPPINESS_MAX = 200;
    private static final int SYSTEM_OVERCOMMIT_MODE_MIN = 0;
    private static final int SYSTEM_OVERCOMMIT_MODE_MAX = 2;
    private static final int SYSTEM_OVERCOMMIT_RATIO_MIN = 0;
    private static final int SYSTEM_OVERCOMMIT_RATIO_MAX = 100;
    private static final int RIO_DEFAULT_VM_OVERCOMMIT_MODE = 2;
    private static final int RIO_DEFAULT_VM_OVERCOMMIT_RATIO = 86;
    private static final int RIO_DEFAULT_VM_SWAPPINESS = 60;
    private static final long SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS = 12L;
    private static final long SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS = 20L;

    private record ConfigExportUrls(
            String baseUrl,
            String indexUrlJson,
            String indexUrlToml,
            String allZipUrl,
            String mechanismsBaseUrl,
            String customConfigUrl,
            String customConfigBaseUrl,
            String diagnosticsUrl,
            String diagnosticsBaseUrl,
            String diagnosticsHistoryUrl,
            String autoLogUrl,
            String mechanismLogUrl,
            String mechanismLogBaseUrl) {
        static ConfigExportUrls empty() {
            return new ConfigExportUrls("", "", "", "", "", "", "", "", "", "", "", "", "");
        }
    }

    public RobotCore(RobotCoreConfig<T> config) {
        double constructorStart = Timer.getFPGATimestamp();
        activeInstance = this;
        AutoConfig autoConfig = config.autoConfig() != null ? config.autoConfig() : AutoConfig.defaults();
        RobotLocalizationConfig resolvedLocalizationConfig =
                applyAutoConfigToLocalization(config.localizationConfig(), autoConfig);
        drivetrain = timedStartupStep("constructor.driveTrain.build", () -> config.driveTrain.build());
        localization = timedStartupStep(
                "constructor.localization.create",
                () -> createLocalizationForDrivetrain(drivetrain, resolvedLocalizationConfig));
        vision = timedStartupStep("constructor.vision.create", () -> RobotVision.fromConfig(config.visionConfig));
        autos = new RobotAuto().attachRobotCore(this);
        copilot = new RobotCopilot(drivetrain, localization, RobotCopilot.inferDriveStyle(drivetrain));
        mechanismsRootNode = robotNetworkTables.root().child("Mechanisms");
        autoRootNode = robotNetworkTables.root().child("Auto");
        selectedAutoTrajectoryPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Athena/Auto/Selected/Trajectory", Pose2d.struct)
                .publish();
        mechanisms = new HashMap<>();
        registeredSuperstructures = new ArrayList<>();
        superstructuresByName = new HashMap<>();
        mechanismView = new RobotMechanisms(mechanisms, superstructuresByName, registeredSuperstructures);
        configServerSection = new ConfigServerSection(this);
        systemSection = new SystemSection();
        diagnosticsChannels = new ConcurrentHashMap<>();
        diagnosticsSection = new DiagnosticsSection(this);
        diagnosticsSection.core().info("lifecycle", "robot core constructed");
        scheduledCustomPidMechanisms = new HashSet<>();
        publishedMechanismsComp = new HashSet<>();
        publishedSuperstructuresComp = new HashSet<>();
        mechanismPublishKeysBuffer = new ArrayList<>();
        superstructurePublishKeysBuffer = new ArrayList<>();
        runtimeMxBean = ManagementFactory.getRuntimeMXBean();
        memoryMxBean = ManagementFactory.getMemoryMXBean();
        osMxBean = ManagementFactory.getOperatingSystemMXBean();
        sunOsMxBean = osMxBean instanceof com.sun.management.OperatingSystemMXBean sun ? sun : null;
        gcMxBeans = ManagementFactory.getGarbageCollectorMXBeans();
        autonomousCommand = null;
        telemetry = TelemetryRegistry.create(config.telemetryConfig());
        autoInitResetEnabled = config.autoInitResetEnabled();
        configuredAutoRegistryBindings = autoConfig.registryBindings();
        configuredMechanisms = config.mechanisms() != null ? List.copyOf(config.mechanisms()) : List.of();
        configuredMechanismsRegistered = false;
        timingDebugEnabled = config.timingDebugEnabled();
        telemetryEnabled = config.telemetryEnabled();
        if (RobotBase.isReal()) {
            SystemConfig systemConfig = config.systemConfig() != null ? config.systemConfig() : SystemConfig.defaults();
            if (systemConfig.tweaksEnabled()) {
                systemSection.overcommitMode(systemConfig.vmOvercommitMode());
                systemSection.overcommitRatio(systemConfig.vmOvercommitRatio());
                systemSection.swappiness(systemConfig.vmSwappiness());
                systemSection.setLoopSwapEnabled(systemConfig.loopSwapEnabled(), systemConfig.loopSwapSizeMiB());
                systemSection.setSystemWebServerEnabled(systemConfig.systemWebServerEnabled());
                systemSection.configServerEnabled(systemConfig.configServerEnabled());
                systemSection.telemetryEnabled(systemConfig.telemetryEnabled());
                systemSection.networkTablesDataLogEnabled(systemConfig.networkTablesDataLogEnabled());
            } else {
                SystemConfig rioDefaults = SystemConfig.rioDefaults();
                systemSection.overcommitMode(rioDefaults.vmOvercommitMode());
                systemSection.overcommitRatio(rioDefaults.vmOvercommitRatio());
                systemSection.swappiness(rioDefaults.vmSwappiness());
                systemSection.setLoopSwapEnabled(false, rioDefaults.loopSwapSizeMiB());
                systemSection.setSystemWebServerEnabled(true);
                systemSection.configServerEnabled(rioDefaults.configServerEnabled());
                systemSection.telemetryEnabled(rioDefaults.telemetryEnabled());
                systemSection.networkTablesDataLogEnabled(rioDefaults.networkTablesDataLogEnabled());
            }
        }
        if (config.performanceMode()) {
            setConfigServerEnabled(false);
            robotNetworkTables.setPublishingEnabled(false);
            telemetry.setEnabled(false);
        }
        if (!telemetryEnabled) {
            telemetry.setEnabled(false);
        }
        LoopTiming.setDebugAlways(timingDebugEnabled);
        mechanismsNetworkTablesPublished = false;
        coreNetworkTablesPublished = false;
        coreHooks = config.hooks() != null ? config.hooks() : RobotCoreHooks.<T>empty();
        hookInputResolver = new TypedInputResolver(
                "RobotCore",
                TypedInputResolver.ValueMode.LENIENT,
                TypedInputResolver.NO_MUTABLES,
                coreHooks.inputs(),
                coreHooks.doubleInputs(),
                coreHooks.intInputs(),
                coreHooks.stringInputs(),
                coreHooks.pose2dInputs(),
                coreHooks.pose3dInputs(),
                coreHooks.objectInputs());
        coreHookContext = new RobotCoreContextImpl();
        timedBindingRunnerInvoker = hook -> hook.apply(coreHookContext);
        periodicHookRunners = createPeriodicHookRunners(coreHooks.periodicLoopBindings());
        disabledPeriodicHookRunners = createPeriodicHookRunners(coreHooks.disabledPeriodicLoopBindings());
        teleopPeriodicHookRunners = createPeriodicHookRunners(coreHooks.teleopPeriodicLoopBindings());
        autonomousPeriodicHookRunners = createPeriodicHookRunners(coreHooks.autonomousPeriodicLoopBindings());
        testPeriodicHookRunners = createPeriodicHookRunners(coreHooks.testPeriodicLoopBindings());
        coreControlLoopRunners = createCoreControlLoopRunners(coreHooks.controlLoopBindings());
        coreControlLoopPids = new HashMap<>();
        coreControlLoopPidOutputTypes = new HashMap<>();
        coreControlLoopFeedforwards = new HashMap<>();
        coreControlLoopFeedforwardOutputTypes = new HashMap<>();
        for (Map.Entry<String, RobotCoreHooks.PidProfile> entry : coreHooks.pidProfiles().entrySet()) {
            if (entry == null) {
                continue;
            }
            String name = entry.getKey();
            RobotCoreHooks.PidProfile profile = entry.getValue();
            if (name == null || name.isBlank() || profile == null) {
                continue;
            }
            OutputType outputType = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
            if (outputType != OutputType.PERCENT && outputType != OutputType.VOLTAGE) {
                throw new IllegalStateException("Core PID profile output type must be PERCENT or VOLTAGE");
            }
            PIDController controller = new PIDController(profile.kP(), profile.kI(), profile.kD());
            if (Double.isFinite(profile.iZone()) && profile.iZone() > 0.0) {
                controller.setIZone(profile.iZone());
            }
            if (Double.isFinite(profile.tolerance()) && profile.tolerance() > 0.0) {
                controller.setTolerance(profile.tolerance());
            }
            coreControlLoopPids.put(name, controller);
            coreControlLoopPidOutputTypes.put(name, outputType);
        }
        for (Map.Entry<String, RobotCoreHooks.FeedforwardProfile> entry : coreHooks.feedforwardProfiles().entrySet()) {
            if (entry == null) {
                continue;
            }
            String name = entry.getKey();
            RobotCoreHooks.FeedforwardProfile profile = entry.getValue();
            SimpleMotorFeedforward ff = profile != null ? profile.feedforward() : null;
            if (name == null || name.isBlank() || ff == null) {
                continue;
            }
            OutputType outputType = profile.outputType() != null ? profile.outputType() : OutputType.VOLTAGE;
            if (outputType != OutputType.VOLTAGE) {
                throw new IllegalStateException("Core feedforward profile output type must be VOLTAGE");
            }
            coreControlLoopFeedforwards.put(name, new SimpleMotorFeedforward(ff.getKs(), ff.getKv(), ff.getKa()));
            coreControlLoopFeedforwardOutputTypes.put(name, outputType);
        }

        if (vision != null) {
            vision.diagnostics(diagnosticsSection.vision());
        }
        if (localization != null) {
            localization.diagnostics(diagnosticsSection.localization());
            localization.attachRobotNetworkTables(robotNetworkTables);
            localization.setAutoPlannerPidAutotunerRequirement(drivetrain);
            if (vision != null) {
                localization.setRobotVision(vision);
            }
        }
        // Choreo auto configuration should not depend on vision. Only attempt configuration when the
        // backend exists to avoid warning spam for projects that do not include Athena Choreo.
        if (localization != null && AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).isPresent()) {
            timedStartupStep("constructor.localization.configureChoreo", () -> localization.configureChoreo(drivetrain));
        }
        if (localization != null) {
            String autoPose = resolvedLocalizationConfig.autoPoseName();
            autos.reset().poseResetter(pose -> Commands.runOnce(() -> localization.resetPose(autoPose, pose)));
        }

        if (vision != null && edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            AprilTagFieldLayout layout = vision.deriveSimulationLayout();
            visionSim = vision.createSimulation(layout);
            visionSimNotifier = new Notifier(() -> {
                Pose2d pose = localization != null ? localization.getFieldPose() : new Pose2d();
                visionSim.update(pose);
            });
            visionSimNotifier.startPeriodic(0.02);
        }

        // Make declared mechanisms available immediately after RobotCore construction (before subclass
        // constructors run). robotInit() still calls this, but it is guarded to prevent duplicates.
        timedStartupStep("constructor.registerConfiguredMechanisms", this::registerConfiguredMechanisms);
        logStartupDuration("constructor.total", Timer.getFPGATimestamp() - constructorStart);
    }

    @Override
    public final void robotInit() {
        diagnosticsSection.core().info("lifecycle", "robotInit");
        double robotInitStart = Timer.getFPGATimestamp();
        timedStartupStep("robotInit.registerConfiguredMechanisms", this::registerConfiguredMechanisms);
        timedStartupStep("robotInit.startConfigServerIfNeeded", this::startConfigServerIfNeeded);
        timedStartupStep("robotInit.configureAutoRegistry", this::configureAutoRegistry);
        timedStartupStep("robotInit.configureAutos", () -> configureAutos(autos));
        timedStartupStep("robotInit.autos.finalizeRegistration", () -> autos.execution().prepare());
        timedStartupStep("robotInit.ensureAutoChooserPublished", this::ensureAutoChooserPublished);
        timedStartupStep("robotInit.publishConfig", robotNetworkTables::publishConfig);
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE)) {
            timedStartupStep("robotInit.publishNetworkTables", () -> {
                publishNetworkTables();
            });
        }
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS)) {
            timedStartupStep("robotInit.publishNetworkTablesMechanisms", () -> {
                publishNetworkTablesMechanisms();
            });
        }
        timedStartupStep("robotInit.runInitHooks", this::runInitHooks);
        timedStartupStep("robotInit.runCorePhaseBindings",
                () -> runCorePhaseBindings(RobotCoreHooks.Phase.ROBOT_INIT, coreHooks.initBindings()));
        timedStartupStep("robotInit.onRobotInit", this::onRobotInit);
        timedStartupStep("robotInit.registerPIDCycles", this::registerPIDCycles);
        logStartupDuration("robotInit.total", Timer.getFPGATimestamp() - robotInitStart);
    }

    private void runInitHooks() {
        runMechanismInitHooks();
        runSuperstructureInitHooks();
    }

    private void runRegisteredPhaseHooks(RobotCoreHooks.Phase phase) {
        runMechanismPhaseHooks(phase);
        runSuperstructurePhaseHooks(phase);
    }

    private void runMechanismInitHooks() {
        if (mechanisms == null || mechanisms.isEmpty()) {
            return;
        }
        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            var cfg = mech.getSourceConfig();
            if (cfg == null) {
                continue;
            }
            cfg.runInitHooks(mech);
        }
    }

    private void runMechanismPhaseHooks(RobotCoreHooks.Phase phase) {
        if (phase == null || mechanisms == null || mechanisms.isEmpty()) {
            return;
        }
        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            mech.runLifecycleHooks(phase);
        }
    }

    private void runSuperstructureInitHooks() {
        if (superstructuresByName == null || superstructuresByName.isEmpty()) {
            return;
        }
        Set<SuperstructureMechanism<?, ?>> visited = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (SuperstructureMechanism<?, ?> top : superstructuresByName.values()) {
            if (top == null) {
                continue;
            }
            for (SuperstructureMechanism<?, ?> superstructure : top.children().superstructures()) {
                if (superstructure == null || !visited.add(superstructure)) {
                    continue;
                }
                superstructure.runInitHooks();
            }
        }
    }

    private void runSuperstructurePhaseHooks(RobotCoreHooks.Phase phase) {
        if (phase == null || superstructuresByName == null || superstructuresByName.isEmpty()) {
            return;
        }
        Set<SuperstructureMechanism<?, ?>> visited = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (SuperstructureMechanism<?, ?> top : superstructuresByName.values()) {
            if (top == null) {
                continue;
            }
            for (SuperstructureMechanism<?, ?> superstructure : top.children().superstructures()) {
                if (superstructure == null || !visited.add(superstructure)) {
                    continue;
                }
                superstructure.runLifecycleHooks(phase);
            }
        }
    }

    private void startConfigServerIfNeeded() {
        if (!configServerEnabled) {
            return;
        }
        if (configServer != null) {
            return;
        }
        try {
            configServer = AthenaRuntimeServer.start(this, 5806);
            configServerBaseUrl = configServer.baseUrl();
            registerDiagnosticsProvidersWithServer();
        } catch (Exception e) {
            // Config export is a convenience feature; do not fail robot init if it cannot bind.
            System.out.println("[Athena][ConfigServer] Failed to start: " + e.getMessage());
            configServer = null;
            configServerBaseUrl = null;
        }
    }

    private void stopConfigServerIfRunning() {
        AthenaRuntimeServer server = configServer;
        if (server == null) {
            configServerBaseUrl = null;
            cachedConfigExportBaseUrl = "";
            cachedConfigExportUrls = ConfigExportUrls.empty();
            return;
        }
        try {
            server.stop();
        } catch (RuntimeException ex) {
            System.out.println("[Athena][ConfigServer] Failed to stop cleanly: " + ex.getMessage());
        } finally {
            configServer = null;
            configServerBaseUrl = null;
            cachedConfigExportBaseUrl = "";
            cachedConfigExportUrls = ConfigExportUrls.empty();
        }
    }

    private void setConfigServerEnabled(boolean enabled) {
        configServerEnabled = enabled;
        if (!enabled) {
            stopConfigServerIfRunning();
        }
    }

    /**
     * Runtime API for publishing user-defined content via the Athena config server.
     */
    public ConfigServerSection configServer() {
        return configServerSection;
    }

    /**
     * Runtime controls for memory-sensitive core services that are not managed by
     * Athena/NetworkTableConfig.
     */
    public SystemSection system() {
        return systemSection;
    }

    public RobotCore<T> system(Consumer<SystemSection> section) {
        if (section != null) {
            section.accept(systemSection);
        }
        return this;
    }

    @Override
    public final void robotPeriodic() {
        LoopTiming.beginCycle();
        updateRuntimeModeCache();
        maybeApplyCompetitionStreamShedding();
        robotNetworkTables.refresh();
        robotNetworkTables.beginPublishCycle();
        updateAutoChooserPublishers();

        // Auto publish when enabled at runtime from Athena/NetworkTableConfig.
        if (robotNetworkTables.isPublishingEnabled()) {
            double now = nowSeconds();
            if (robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE)) {
                double period = robotNetworkTables.getDefaultPeriodSeconds();
                boolean due = !Double.isFinite(lastCoreNetworkTablesPublishSeconds)
                        || !Double.isFinite(now)
                        || (now - lastCoreNetworkTablesPublishSeconds) >= period
                        || robotNetworkTables.revision() != lastCoreNetworkTablesConfigRevision;
                if (due) {
                    publishNetworkTables();
                    lastCoreNetworkTablesPublishSeconds = now;
                    lastCoreNetworkTablesConfigRevision = robotNetworkTables.revision();
                }
            }
            if (robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS)) {
                autoPublishMechanismsIncremental(now);
            }
        }

        double cycleStartSeconds = nowSeconds();

        long t0Ns = System.nanoTime();
        CommandScheduler.getInstance().run();
        long t1Ns = System.nanoTime();
        telemetry.tick();
        AthenaNT.tick();
        long t2Ns = System.nanoTime();
        if (localization != null) {
            localization.updateAutoVisualization(autos);
        }
        long t3Ns = System.nanoTime();
        runCoreControlLoops(cycleStartSeconds);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.ROBOT_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.ROBOT_PERIODIC,
                coreHooks.periodicBindings(),
                periodicHookRunners);
        onRobotPeriodic();
        long t4Ns = System.nanoTime();

        double schedulerSeconds = (t1Ns - t0Ns) * 1e-9;
        double telemetrySeconds = (t2Ns - t1Ns) * 1e-9;
        double localizationSeconds = (t3Ns - t2Ns) * 1e-9;
        double userSeconds = (t4Ns - t3Ns) * 1e-9;
        double totalSeconds = (t4Ns - t0Ns) * 1e-9;

        double t0 = cycleStartSeconds;
        double t1 = t0 + schedulerSeconds;
        double t2 = t1 + telemetrySeconds;
        double t3 = t2 + localizationSeconds;
        double t4 = t3 + userSeconds;

        lastLoopSchedulerMs = schedulerSeconds * 1000.0;
        lastLoopTelemetryMs = telemetrySeconds * 1000.0;
        lastLoopLocalizationMs = localizationSeconds * 1000.0;
        lastLoopUserMs = userSeconds * 1000.0;
        lastLoopTotalMs = totalSeconds * 1000.0;
        LoopTiming.endCycle(t0, t1, t2, t3, t4);
    }

    private void autoPublishMechanismsIncremental(double now) {
        if (Double.isFinite(lastMechanismAutoPublishSeconds)
                && (now - lastMechanismAutoPublishSeconds) < MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS) {
            return;
        }
        lastMechanismAutoPublishSeconds = now;

        mechanismsNetworkTablesPublished = true;
        RobotNetworkTables.Node mechanismsNode = mechanismsRootNode;

        long revision = robotNetworkTables.revision();
        if (revision != lastMechanismsConfigRevision) {
            lastMechanismsConfigRevision = revision;
            remainingMechanismRefreshCount = Math.max(remainingMechanismRefreshCount, mechanisms.size());
            remainingSuperstructureRefreshCount =
                    Math.max(remainingSuperstructureRefreshCount, superstructuresByName.size());
        }

        publishMechanismBatch(mechanismsNode);
        publishSuperstructureBatch(mechanismsNode);
    }

    private void publishMechanismBatch(RobotNetworkTables.Node mechanismsNode) {
        if (mechanismPublishKeysDirty || mechanismPublishKeysBuffer.size() != mechanisms.size()) {
            mechanismPublishKeysBuffer.clear();
            mechanismPublishKeysBuffer.addAll(mechanisms.keySet());
            mechanismPublishKeysDirty = false;
            publishedMechanismsComp.retainAll(mechanismPublishKeysBuffer);
        }
        if (mechanismPublishKeysBuffer.isEmpty()) {
            publishedMechanismsComp.clear();
            mechanismPublishCursor = 0;
            remainingMechanismRefreshCount = 0;
            return;
        }

        int batchSize = Math.min(MECHANISM_AUTO_PUBLISH_BATCH_SIZE, mechanismPublishKeysBuffer.size());
        for (int i = 0; i < batchSize; i++) {
            if (mechanismPublishCursor >= mechanismPublishKeysBuffer.size()) {
                mechanismPublishCursor = 0;
            }
            String name = mechanismPublishKeysBuffer.get(mechanismPublishCursor++);
            Mechanism mech = mechanisms.get(name);
            if (mech == null) {
                continue;
            }
            boolean needsRefresh = remainingMechanismRefreshCount > 0;
            boolean needsInitialPublish = !publishedMechanismsComp.contains(name);
            if (!needsRefresh && !needsInitialPublish) {
                continue;
            }
            mech.networkTables(mech.networkTables().resolveNode(mechanismsNode));
            publishedMechanismsComp.add(name);
            if (needsRefresh) {
                remainingMechanismRefreshCount--;
            }
        }
        if (remainingMechanismRefreshCount < 0) {
            remainingMechanismRefreshCount = 0;
        }
    }

    private void publishSuperstructureBatch(RobotNetworkTables.Node mechanismsNode) {
        if (superstructurePublishKeysDirty || superstructurePublishKeysBuffer.size() != superstructuresByName.size()) {
            superstructurePublishKeysBuffer.clear();
            superstructurePublishKeysBuffer.addAll(superstructuresByName.keySet());
            superstructurePublishKeysDirty = false;
            publishedSuperstructuresComp.retainAll(superstructurePublishKeysBuffer);
        }
        if (superstructurePublishKeysBuffer.isEmpty()) {
            publishedSuperstructuresComp.clear();
            superstructurePublishCursor = 0;
            remainingSuperstructureRefreshCount = 0;
            return;
        }

        int batchSize = Math.min(MECHANISM_AUTO_PUBLISH_BATCH_SIZE, superstructurePublishKeysBuffer.size());
        for (int i = 0; i < batchSize; i++) {
            if (superstructurePublishCursor >= superstructurePublishKeysBuffer.size()) {
                superstructurePublishCursor = 0;
            }
            String key = superstructurePublishKeysBuffer.get(superstructurePublishCursor++);
            SuperstructureMechanism<?, ?> superstructure = superstructuresByName.get(key);
            if (superstructure == null || key == null) {
                continue;
            }
            boolean needsRefresh = remainingSuperstructureRefreshCount > 0;
            boolean needsInitialPublish = !publishedSuperstructuresComp.contains(key);
            if (!needsRefresh && !needsInitialPublish) {
                continue;
            }
            superstructure.networkTables(mechanismsNode.child(key));
            publishedSuperstructuresComp.add(key);
            if (needsRefresh) {
                remainingSuperstructureRefreshCount--;
            }
        }
        if (remainingSuperstructureRefreshCount < 0) {
            remainingSuperstructureRefreshCount = 0;
        }
    }

    @Override
    public final void autonomousInit() {
        diagnosticsSection.core().info("mode", "autonomousInit");
        prepareDrivetrainForModeTransition(false, true);
        resetAutoInitPoseIfConfigured();
        scheduleAutonomousCommand();
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.AUTONOMOUS_INIT);
        resetPeriodicRunners(autonomousPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.AUTONOMOUS_INIT, coreHooks.autonomousInitBindings());
        onAutonomousInit();
    }

    @Override
    public final void autonomousExit() {
        diagnosticsSection.core().info("mode", "autonomousExit");
        prepareDrivetrainForModeTransition(false, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.AUTONOMOUS_EXIT);
        runCoreExitBindings(RobotCoreHooks.Phase.AUTONOMOUS_EXIT, coreHooks.autonomousExitBindings());
        onAutonomousExit();
    }

    @Override
    public final void autonomousPeriodic() {
        if (autonomousCommand != null
                && !CommandScheduler.getInstance().isScheduled(autonomousCommand)) {
            prepareDrivetrainForModeTransition(false, true);
        }
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.AUTONOMOUS_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.AUTONOMOUS_PERIODIC,
                coreHooks.autonomousPeriodicBindings(),
                autonomousPeriodicHookRunners);
        onAutonomousPeriodic();
    }

    @Override
    public final void teleopInit() {
        diagnosticsSection.core().info("mode", "teleopInit");
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TELEOP_INIT);
        resetPeriodicRunners(teleopPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.TELEOP_INIT, coreHooks.teleopInitBindings());
        onTeleopInit();
    }

    @Override
    public final void teleopExit() {
        diagnosticsSection.core().info("mode", "teleopExit");
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TELEOP_EXIT);
        runCoreExitBindings(RobotCoreHooks.Phase.TELEOP_EXIT, coreHooks.teleopExitBindings());
        onTeleopExit();
    }

    @Override
    public final void teleopPeriodic() {
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TELEOP_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.TELEOP_PERIODIC,
                coreHooks.teleopPeriodicBindings(),
                teleopPeriodicHookRunners);
        onTeleopPeriodic();
    }

    @Override
    public final void disabledInit() {
        diagnosticsSection.core().info("mode", "disabledInit");
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.DISABLED_INIT);
        resetPeriodicRunners(disabledPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.DISABLED_INIT, coreHooks.disabledInitBindings());
        onDisabledInit();
    }

    @Override
    public final void disabledExit() {
        diagnosticsSection.core().info("mode", "disabledExit");
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.DISABLED_EXIT);
        runCoreExitBindings(RobotCoreHooks.Phase.DISABLED_EXIT, coreHooks.disabledExitBindings());
        onDisabledExit();
    }

    @Override
    public final void disabledPeriodic() {
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.DISABLED_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.DISABLED_PERIODIC,
                coreHooks.disabledPeriodicBindings(),
                disabledPeriodicHookRunners);
        onDisabledPeriodic();
    }

    @Override
    public final void testInit() {
        diagnosticsSection.core().info("mode", "testInit");
        CommandScheduler.getInstance().cancelAll();
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(false, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TEST_INIT);
        resetPeriodicRunners(testPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.TEST_INIT, coreHooks.testInitBindings());
        onTestInit();
    }

    @Override
    public final void testExit() {
        diagnosticsSection.core().info("mode", "testExit");
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TEST_EXIT);
        runCoreExitBindings(RobotCoreHooks.Phase.TEST_EXIT, coreHooks.testExitBindings());
        onTestExit();
    }

    @Override
    public final void testPeriodic() {
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TEST_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.TEST_PERIODIC,
                coreHooks.testPeriodicBindings(),
                testPeriodicHookRunners);
        onTestPeriodic();
    }

    @Override
    public final void simulationInit() {
        onSimulationInit();
    }

    @Override
    public final void simulationPeriodic() {
        onSimulationPeriodic();
    }

    private List<TimedRunner<RobotCoreHooks.Binding<T>>> createPeriodicHookRunners(
            List<RobotCoreHooks.PeriodicHookBinding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return new ArrayList<>();
        }
        List<TimedRunner<RobotCoreHooks.Binding<T>>> runners = new ArrayList<>(bindings.size());
        for (RobotCoreHooks.PeriodicHookBinding<T> binding : bindings) {
            if (binding == null || binding.hook() == null) {
                continue;
            }
            runners.add(TimedRunner.periodicMs(binding.hook(), binding.periodMs()));
        }
        return runners;
    }

    private List<TimedRunner<RobotCoreHooks.ControlLoop<T>>> createCoreControlLoopRunners(
            List<RobotCoreHooks.ControlLoopBinding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return new ArrayList<>();
        }
        List<TimedRunner<RobotCoreHooks.ControlLoop<T>>> runners = new ArrayList<>(bindings.size());
        for (RobotCoreHooks.ControlLoopBinding<T> binding : bindings) {
            if (binding == null || binding.loop() == null || binding.name() == null || binding.name().isBlank()) {
                continue;
            }
            runners.add(new TimedRunner<>(binding.name(), binding.periodSeconds(), binding.loop()));
        }
        return runners;
    }

    private void runCoreControlLoops(double nowSeconds) {
        if (coreControlLoopRunners == null || coreControlLoopRunners.isEmpty()) {
            return;
        }
        coreHookContext.setPhase(RobotCoreHooks.Phase.ROBOT_PERIODIC);
        for (TimedRunner<RobotCoreHooks.ControlLoop<T>> runner : coreControlLoopRunners) {
            if (runner == null || !runner.shouldRunSeconds(nowSeconds)) {
                continue;
            }
            double dtSeconds;
            if (Double.isNaN(runner.lastRunSeconds())) {
                dtSeconds = runner.periodSeconds();
            } else {
                dtSeconds = nowSeconds - runner.lastRunSeconds();
            }
            if (!Double.isFinite(dtSeconds) || dtSeconds < 0.0) {
                dtSeconds = Double.NaN;
            }
            coreHookContext.setControlLoopDtSeconds(dtSeconds);
            runner.setLastOutput(runner.task().calculate(coreHookContext));
            runner.setLastRunSeconds(nowSeconds);
        }
        coreHookContext.setControlLoopDtSeconds(Double.NaN);
    }

    private void runCorePhaseBindings(
            RobotCoreHooks.Phase phase,
            List<RobotCoreHooks.Binding<T>> bindings) {
        coreHookContext.setPhase(phase);
        runCoreBindings(bindings);
    }

    private void runCoreExitBindings(
            RobotCoreHooks.Phase phase,
            List<RobotCoreHooks.Binding<T>> bindings) {
        coreHookContext.setPhase(phase);
        runCoreBindings(coreHooks.exitBindings());
        runCoreBindings(bindings);
    }

    private void runCorePeriodicBindings(
            RobotCoreHooks.Phase phase,
            List<RobotCoreHooks.Binding<T>> bindings,
            List<TimedRunner<RobotCoreHooks.Binding<T>>> runners) {
        coreHookContext.setPhase(phase);
        runCoreBindings(bindings);
        if (runners == null || runners.isEmpty()) {
            return;
        }
        double nowSeconds = hookNowSeconds();
        for (TimedRunner<RobotCoreHooks.Binding<T>> runner : runners) {
            if (runner == null || !runner.shouldRunSeconds(nowSeconds)) {
                continue;
            }
            runner.run(timedBindingRunnerInvoker, nowSeconds);
        }
    }

    private void runCoreBindings(List<RobotCoreHooks.Binding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return;
        }
        for (int i = 0; i < bindings.size(); i++) {
            RobotCoreHooks.Binding<T> binding = bindings.get(i);
            if (binding == null) {
                continue;
            }
            try {
                binding.apply(coreHookContext);
            } catch (Throwable throwable) {
                DriverStation.reportError(
                        String.format(
                                java.util.Locale.US,
                                "[Athena] Hook binding failure in phase %s at index %d: %s",
                                coreHookContext.phase(),
                                i,
                                throwable.toString()),
                        throwable.getStackTrace());
            }
        }
    }

    private void resetPeriodicRunners(List<TimedRunner<RobotCoreHooks.Binding<T>>> runners) {
        if (runners == null || runners.isEmpty()) {
            return;
        }
        for (TimedRunner<RobotCoreHooks.Binding<T>> runner : runners) {
            if (runner != null) {
                runner.reset();
            }
        }
    }

    private static double hookNowSeconds() {
        return nowSeconds();
    }

    private static double nowSeconds() {
        double now = RobotTime.nowSeconds();
        if (!Double.isFinite(now)) {
            now = Timer.getFPGATimestamp();
        }
        return now;
    }

    private static void logStartupDuration(String step, double elapsedSeconds) {
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds < STARTUP_LOG_THRESHOLD_SECONDS) {
            return;
        }
        System.out.printf(
                java.util.Locale.US,
                "[Athena][Startup] %s took %.3fs%n",
                step,
                elapsedSeconds);
    }

    private static void timedStartupStep(String step, Runnable action) {
        if (action == null) {
            return;
        }
        double start = Timer.getFPGATimestamp();
        action.run();
        logStartupDuration(step, Timer.getFPGATimestamp() - start);
    }

    private static <R> R timedStartupStep(String step, Supplier<R> action) {
        if (action == null) {
            return null;
        }
        double start = Timer.getFPGATimestamp();
        R result = action.get();
        logStartupDuration(step, Timer.getFPGATimestamp() - start);
        return result;
    }

    private boolean hookInput(String key) {
        return hookInputResolver.boolVal(key);
    }

    private BooleanSupplier hookInputSupplier(String key) {
        return hookInputResolver.boolSupplier(key);
    }

    private double hookDoubleInput(String key) {
        return hookInputResolver.doubleVal(key);
    }

    private DoubleSupplier hookDoubleInputSupplier(String key) {
        return hookInputResolver.doubleSupplier(key);
    }

    private int hookIntVal(String key) {
        return hookInputResolver.intVal(key);
    }

    private IntSupplier hookIntValSupplier(String key) {
        return hookInputResolver.intSupplier(key);
    }

    private String hookStringVal(String key) {
        return hookInputResolver.stringVal(key);
    }

    private Supplier<String> hookStringValSupplier(String key) {
        return hookInputResolver.stringSupplier(key);
    }

    private Pose2d hookPose2dVal(String key) {
        return hookInputResolver.pose2dVal(key);
    }

    private Supplier<Pose2d> hookPose2dValSupplier(String key) {
        return hookInputResolver.pose2dSupplier(key);
    }

    private Pose3d hookPose3dVal(String key) {
        return hookInputResolver.pose3dVal(key);
    }

    private Supplier<Pose3d> hookPose3dValSupplier(String key) {
        return hookInputResolver.pose3dSupplier(key);
    }

    private <V> V hookObjectInput(String key, Class<V> type) {
        return hookInputResolver.objectVal(key, type);
    }

    private <V> Supplier<V> hookObjectInputSupplier(String key, Class<V> type) {
        return hookInputResolver.objectSupplier(key, type);
    }

    private PIDController hookPid(String name) {
        PIDController pid = coreControlLoopPids.get(name);
        if (pid == null) {
            throw new IllegalArgumentException("No PID profile found for name " + name);
        }
        return pid;
    }

    private SimpleMotorFeedforward hookFeedforward(String name) {
        SimpleMotorFeedforward ff = coreControlLoopFeedforwards.get(name);
        if (ff == null) {
            throw new IllegalArgumentException("No feedforward profile found for name " + name);
        }
        return ff;
    }

    private OutputType hookPidOutputType(String name) {
        OutputType outputType = coreControlLoopPidOutputTypes.get(name);
        if (outputType == null) {
            throw new IllegalArgumentException("No PID output type found for profile " + name);
        }
        return outputType;
    }

    private OutputType hookFeedforwardOutputType(String name) {
        OutputType outputType = coreControlLoopFeedforwardOutputTypes.get(name);
        if (outputType == null) {
            throw new IllegalArgumentException("No feedforward output type found for profile " + name);
        }
        return outputType;
    }

    private final class RobotCoreContextImpl implements RobotCoreContext<T> {
        private RobotCoreHooks.Phase phase = RobotCoreHooks.Phase.ROBOT_INIT;
        private double controlLoopDtSeconds = Double.NaN;

        private void setPhase(RobotCoreHooks.Phase phase) {
            this.phase = phase != null ? phase : RobotCoreHooks.Phase.ROBOT_INIT;
        }

        private void setControlLoopDtSeconds(double dtSeconds) {
            this.controlLoopDtSeconds = dtSeconds;
        }

        @Override
        public RobotCore<T> robotCore() {
            return RobotCore.this;
        }

        @Override
        public RobotCoreHooks.Phase phase() {
            return phase;
        }

        @Override
        public double controlLoopDtSeconds() {
            return controlLoopDtSeconds;
        }

        @Override
        public boolean input(String key) {
            return hookInput(key);
        }

        @Override
        public BooleanSupplier inputSupplier(String key) {
            return hookInputSupplier(key);
        }

        @Override
        public double doubleInput(String key) {
            return hookDoubleInput(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return hookDoubleInputSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return hookIntVal(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return hookIntValSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return hookStringVal(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return hookStringValSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return hookPose2dVal(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return hookPose2dValSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return hookPose3dVal(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return hookPose3dValSupplier(key);
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            return hookObjectInput(key, type);
        }

        @Override
        public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            return hookObjectInputSupplier(key, type);
        }

        @Override
        public PIDController pid(String name) {
            return hookPid(name);
        }

        @Override
        public SimpleMotorFeedforward feedforward(String name) {
            return hookFeedforward(name);
        }

        @Override
        public OutputType pidOutputType(String name) {
            return hookPidOutputType(name);
        }

        @Override
        public OutputType feedforwardOutputType(String name) {
            return hookFeedforwardOutputType(name);
        }
    }

    protected void onRobotInit() {}

    protected void onRobotPeriodic() {}

    public RobotCore<T> autoInitResetEnabled(boolean enabled) {
        autoInitResetEnabled = enabled;
        if (autoInitResetEntry != null) {
            autoInitResetEntry.setBoolean(enabled);
        }
        return this;
    }

    public static RobotCore<?> activeInstance() {
        return activeInstance;
    }

    public TelemetryRegistry telemetry() {
        return telemetry;
    }

    protected void onAutonomousInit() {}

    protected void onAutonomousExit() {}

    protected void onAutonomousPeriodic() {}

    protected void onTeleopInit() {}

    protected void onTeleopExit() {}

    protected void onTeleopPeriodic() {}

    protected void onDisabledInit() {}

    protected void onDisabledExit() {}

    protected void onDisabledPeriodic() {}

    protected void onTestInit() {}

    protected void onTestExit() {}

    protected void onTestPeriodic() {}

    protected void onSimulationInit() {}

    protected void onSimulationPeriodic() {}

    /**
     * Hook for registering autonomous routines and named commands.
     * Called once during {@link #robotInit()} after all subsystems are constructed.
     */
    protected void configureAutos(RobotAuto auto) {}

    private void configureAutoRegistry() {
        if (configuredAutoRegistryBindings == null || configuredAutoRegistryBindings.isEmpty()) {
            return;
        }
        RobotAuto.RegistrySection registry = autos.registry();
        for (Consumer<RobotAuto.RegistrySection> binding : configuredAutoRegistryBindings) {
            if (binding == null) {
                continue;
            }
            binding.accept(registry);
        }
    }

    protected Command createAutonomousCommand() {
        return autos.execution().selectedCommand().orElse(null);
    }

    private static RobotLocalizationConfig applyAutoConfigToLocalization(
            RobotLocalizationConfig localizationConfig,
            AutoConfig autoConfig) {
        RobotLocalizationConfig resolved =
                localizationConfig != null ? localizationConfig : RobotLocalizationConfig.defaults();
        AutoConfig auto = autoConfig != null ? autoConfig : AutoConfig.defaults();

        if (auto.translationPid() != null || auto.rotationPid() != null || auto.pidAutotunerConfig() != null) {
            resolved.planner().autoPlannerPid(section -> {
                if (auto.translationPid() != null) {
                    setPidAxis(section.translation(), auto.translationPid());
                }
                if (auto.rotationPid() != null) {
                    setPidAxis(section.rotation(), auto.rotationPid());
                }
                if (auto.pidAutotunerConfig() != null) {
                    RobotLocalizationConfig.AutoPlannerPidAutotunerConfig autotuner = auto.pidAutotunerConfig();
                    section.autotunerConfig(cfg -> cfg
                            .enabled(autotuner.enabled())
                            .dashboardPath(autotuner.dashboardPath())
                            .program(autotuner.program()));
                }
            });
        }

        if (auto.poseName() != null && !auto.poseName().isBlank()) {
            resolved.poses().autoPoseName(auto.poseName());
        }
        return resolved;
    }

    private static void setPidAxis(
            RobotLocalizationConfig.PidAxisSection axis,
            HolonomicPidConstants constants) {
        if (axis == null || constants == null) {
            return;
        }
        axis.kp(constants.kP()).ki(constants.kI()).kd(constants.kD()).iZone(constants.iZone());
    }

    private record SystemCommandResult(int exitCode, String output, boolean timedOut) {}

    private SystemCommandResult runCommand(long timeoutSeconds, String... command) {
        Process process = null;
        try {
            process = new ProcessBuilder(command).redirectErrorStream(true).start();
            boolean exited = process.waitFor(timeoutSeconds, TimeUnit.SECONDS);
            if (!exited) {
                process.destroyForcibly();
                return new SystemCommandResult(-1, "", true);
            }
            String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
            return new SystemCommandResult(process.exitValue(), output, false);
        } catch (IOException e) {
            return new SystemCommandResult(-1, e.getMessage(), false);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return new SystemCommandResult(-1, "interrupted", false);
        } finally {
            if (process != null) {
                process.destroy();
            }
        }
    }

    private SystemCommandResult runSudoCommand(long timeoutSeconds, String... args) {
        String[] command = new String[2 + args.length];
        command[0] = "sudo";
        command[1] = "-n";
        System.arraycopy(args, 0, command, 2, args.length);
        return runCommand(timeoutSeconds, command);
    }

    private static String shellQuote(String value) {
        if (value == null || value.isEmpty()) {
            return "''";
        }
        return "'" + value.replace("'", "'\"'\"'") + "'";
    }

    private SystemCommandResult runSuAdminCommand(long timeoutSeconds, String... args) {
        if (args == null || args.length == 0) {
            return new SystemCommandResult(-1, "no command", false);
        }
        StringBuilder commandBuilder = new StringBuilder();
        for (String arg : args) {
            if (arg == null) {
                continue;
            }
            if (!commandBuilder.isEmpty()) {
                commandBuilder.append(' ');
            }
            commandBuilder.append(shellQuote(arg));
        }
        if (commandBuilder.isEmpty()) {
            return new SystemCommandResult(-1, "no command", false);
        }
        String suInvocation = "su admin -c " + shellQuote(commandBuilder.toString());

        Process process = null;
        try {
            process = new ProcessBuilder("/usr/bin/script", "-q", "-c", suInvocation, "/dev/null")
                    .redirectErrorStream(true)
                    .start();
            try {
                process.getOutputStream().write('\n');
                process.getOutputStream().flush();
            } catch (IOException ignored) {
                // best effort only: blank password is sent when possible
            } finally {
                try {
                    process.getOutputStream().close();
                } catch (IOException ignored) {
                    // no-op
                }
            }
            boolean exited = process.waitFor(timeoutSeconds, TimeUnit.SECONDS);
            if (!exited) {
                process.destroyForcibly();
                return new SystemCommandResult(-1, "", true);
            }
            String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
            return new SystemCommandResult(process.exitValue(), output, false);
        } catch (IOException e) {
            return new SystemCommandResult(-1, e.getMessage(), false);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return new SystemCommandResult(-1, "interrupted", false);
        } finally {
            if (process != null) {
                process.destroy();
            }
        }
    }

    private SystemCommandResult runPrivilegedCommand(long timeoutSeconds, String... args) {
        SystemCommandResult sudoResult = runSudoCommand(timeoutSeconds, args);
        if (!sudoResult.timedOut() && sudoResult.exitCode() == 0) {
            return sudoResult;
        }
        SystemCommandResult suResult = runSuAdminCommand(timeoutSeconds, args);
        if (!suResult.timedOut() && suResult.exitCode() == 0) {
            return suResult;
        }
        String mergedOutput = "";
        if (sudoResult.output() != null && !sudoResult.output().isBlank()) {
            mergedOutput = "sudo: " + sudoResult.output();
        }
        if (suResult.output() != null && !suResult.output().isBlank()) {
            mergedOutput = mergedOutput.isBlank()
                    ? "su-admin: " + suResult.output()
                    : mergedOutput + " | su-admin: " + suResult.output();
        }
        return new SystemCommandResult(
                suResult.exitCode() != 0 ? suResult.exitCode() : sudoResult.exitCode(),
                mergedOutput,
                sudoResult.timedOut() && suResult.timedOut());
    }

    private boolean runSudoCommandAndReport(String operation, long timeoutSeconds, String... args) {
        SystemCommandResult result = runPrivilegedCommand(timeoutSeconds, args);
        if (result.timedOut()) {
            String message = operation + " timed out";
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
        if (result.exitCode() != 0) {
            String message = operation + " failed (exit " + result.exitCode() + ")"
                    + (!result.output().isEmpty() ? ": " + result.output() : "");
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
        if (!result.output().isEmpty()) {
            diagnosticsSection.core().info("system", operation + ": " + result.output());
        }
        return true;
    }

    private static int clampInt(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean setVmSysctl(String key, int value) {
        if (key == null || key.isBlank()) {
            return false;
        }
        String procPath = "/proc/sys/" + key.replace('.', '/');
        Integer current = readIntFromFile(procPath);
        if (current != null && current.intValue() == value) {
            return true;
        }
        return runSudoCommandAndReport(
                "sysctl " + key + "=" + value,
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                "/sbin/sysctl",
                "-w",
                key + "=" + value);
    }

    private Integer readIntFromFile(String path) {
        if (path == null || path.isBlank()) {
            return null;
        }
        try {
            String raw = java.nio.file.Files.readString(java.nio.file.Path.of(path), StandardCharsets.UTF_8);
            String trimmed = raw != null ? raw.trim() : "";
            if (trimmed.isEmpty()) {
                return null;
            }
            return Integer.parseInt(trimmed);
        } catch (IOException | NumberFormatException e) {
            return null;
        }
    }

    private boolean processRunningByName(String processName) {
        if (processName == null || processName.isBlank()) {
            return false;
        }
        SystemCommandResult result = runPrivilegedCommand(
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                "/usr/bin/pgrep",
                "-x",
                processName);
        if (result.timedOut()) {
            return false;
        }
        return result.exitCode() == 0;
    }

    private boolean killProcessesByName(String processName) {
        if (processName == null || processName.isBlank()) {
            return false;
        }
        SystemCommandResult pidsResult = runPrivilegedCommand(
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                "/usr/bin/pgrep",
                "-x",
                processName);
        if (pidsResult.timedOut()) {
            String message = "system process lookup timed out: " + processName;
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
        if (pidsResult.exitCode() == 1 || pidsResult.output().isBlank()) {
            return true;
        }
        if (pidsResult.exitCode() != 0) {
            String message = "system process lookup failed (" + processName + "): " + pidsResult.output();
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
        String[] pids = pidsResult.output()
                .lines()
                .map(String::trim)
                .filter(line -> !line.isBlank() && line.chars().allMatch(Character::isDigit))
                .toArray(String[]::new);
        if (pids.length == 0) {
            return true;
        }

        String[] termCommand = new String[2 + pids.length];
        termCommand[0] = "/bin/kill";
        termCommand[1] = "-TERM";
        System.arraycopy(pids, 0, termCommand, 2, pids.length);
        if (!runSudoCommandAndReport(
                "system process terminate " + processName,
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                termCommand)) {
            return false;
        }
        if (!processRunningByName(processName)) {
            return true;
        }

        String[] killCommand = new String[2 + pids.length];
        killCommand[0] = "/bin/kill";
        killCommand[1] = "-KILL";
        System.arraycopy(pids, 0, killCommand, 2, pids.length);
        return runSudoCommandAndReport(
                "system process kill " + processName,
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                killCommand);
    }

    private boolean runSystemWebServerControlCommand(String action) {
        if (action == null || action.isBlank()) {
            return false;
        }
        if (!RobotBase.isReal()) {
            return true;
        }
        return switch (action) {
            case "stop" -> {
                if (!isSystemWebServerRunning()) {
                    yield true;
                }
                boolean stopped = killProcessesByName(SYSTEM_WEBCONTAINER_PROCESS_NAME)
                        && killProcessesByName(SYSTEM_WEBSERVER_PROCESS_NAME);
                yield stopped && !isSystemWebServerRunning();
            }
            case "start" -> {
                if (isSystemWebServerRunning()) {
                    yield true;
                }
                boolean started = runSudoCommandAndReport(
                        "systemWebServer start",
                        SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                        "/sbin/start-stop-daemon",
                        "-S",
                        "-b",
                        "-x",
                        SYSTEM_WEBSERVER_BINARY_PATH,
                        "--",
                        "-timeout",
                        "50",
                        "-child-timeout",
                        "20",
                        "-system");
                if (!started) {
                    yield false;
                }
                yield isSystemWebServerRunning();
            }
            case "restart" -> runSystemWebServerControlCommand("stop")
                    && runSystemWebServerControlCommand("start");
            default -> false;
        };
    }

    private boolean isSystemWebServerRunning() {
        if (!RobotBase.isReal()) {
            return false;
        }
        return processRunningByName(SYSTEM_WEBSERVER_PROCESS_NAME)
                || processRunningByName(SYSTEM_WEBCONTAINER_PROCESS_NAME);
    }

    private static int clampLoopSwapSizeMiB(int sizeMiB) {
        return clampInt(sizeMiB, SYSTEM_LOOP_SWAP_MIN_SIZE_MIB, SYSTEM_LOOP_SWAP_MAX_SIZE_MIB);
    }

    private boolean ensureLoopSwapFileSized(int sizeMiB) {
        File file = new File(SYSTEM_LOOP_SWAP_FILE_PATH);
        long expectedBytes = (long) sizeMiB * 1024L * 1024L;
        try (java.io.RandomAccessFile raf = new java.io.RandomAccessFile(file, "rw")) {
            if (raf.length() != expectedBytes) {
                raf.setLength(expectedBytes);
            }
        } catch (IOException e) {
            String message = "loop swap file setup failed: " + e.getMessage();
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
        file.setReadable(false, false);
        file.setWritable(false, false);
        file.setReadable(true, true);
        file.setWritable(true, true);
        return true;
    }

    private String findLoopDeviceForSwapFile() {
        SystemCommandResult result = runPrivilegedCommand(
                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                "/sbin/losetup",
                "-j",
                SYSTEM_LOOP_SWAP_FILE_PATH);
        if (result.timedOut() || result.exitCode() != 0 || result.output().isBlank()) {
            return null;
        }
        String firstLine = result.output().lines().findFirst().orElse("").trim();
        if (firstLine.isEmpty()) {
            return null;
        }
        int colon = firstLine.indexOf(':');
        if (colon <= 0) {
            return null;
        }
        return firstLine.substring(0, colon).trim();
    }

    private String findFreeLoopDevice() {
        SystemCommandResult result = runPrivilegedCommand(
                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                "/sbin/losetup",
                "-f");
        if (result.timedOut() || result.exitCode() != 0 || result.output().isBlank()) {
            return null;
        }
        return result.output().lines().findFirst().orElse("").trim();
    }

    private boolean isSwapDeviceActive(String loopDevice) {
        if (loopDevice == null || loopDevice.isBlank()) {
            return false;
        }
        try {
            List<String> lines = java.nio.file.Files.readAllLines(java.nio.file.Path.of("/proc/swaps"));
            for (int i = 1; i < lines.size(); i++) {
                String line = lines.get(i);
                if (line != null && line.startsWith(loopDevice)) {
                    return true;
                }
            }
        } catch (IOException ignored) {
            return false;
        }
        return false;
    }

    private boolean configureLoopSwapEnabled(boolean enabled, int sizeMiB) {
        if (!RobotBase.isReal()) {
            return true;
        }
        int resolvedSizeMiB = clampLoopSwapSizeMiB(sizeMiB);
        long expectedBytes = (long) resolvedSizeMiB * 1024L * 1024L;
        long existingBytes = loopSwapFileSizeBytes();
        String loopDevice = findLoopDeviceForSwapFile();
        boolean active = isSwapDeviceActive(loopDevice);

        if (!enabled) {
            if (loopDevice == null || loopDevice.isBlank()) {
                return true;
            }
            boolean ok = true;
            if (active) {
                ok = runSudoCommandAndReport(
                                "loop swapoff",
                                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                                "/sbin/swapoff",
                                loopDevice)
                        && ok;
            }
            ok = runSudoCommandAndReport(
                            "loop detach",
                            SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                            "/sbin/losetup",
                            "-d",
                            loopDevice)
                    && ok;
            return ok && !isLoopSwapRunning();
        }

        if (loopDevice != null && !loopDevice.isBlank() && active && existingBytes == expectedBytes) {
            return true;
        }

        if (loopDevice != null && !loopDevice.isBlank() && (active || existingBytes != expectedBytes)) {
            boolean ok = true;
            if (active) {
                ok = runSudoCommandAndReport(
                                "loop swapoff",
                                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                                "/sbin/swapoff",
                                loopDevice)
                        && ok;
            }
            ok = runSudoCommandAndReport(
                            "loop detach",
                            SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                            "/sbin/losetup",
                            "-d",
                            loopDevice)
                    && ok;
            if (!ok) {
                return false;
            }
            loopDevice = null;
        }

        if (!ensureLoopSwapFileSized(resolvedSizeMiB)) {
            return false;
        }
        if (loopDevice == null || loopDevice.isBlank()) {
            loopDevice = findFreeLoopDevice();
            if (loopDevice == null || loopDevice.isBlank()) {
                String message = "loop device allocation failed";
                DriverStation.reportWarning("[Athena][System] " + message, false);
                diagnosticsSection.core().warn("system", message);
                return false;
            }
            if (!runSudoCommandAndReport(
                    "loop attach",
                    SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                    "/sbin/losetup",
                    loopDevice,
                    SYSTEM_LOOP_SWAP_FILE_PATH)) {
                return false;
            }
        }

        if (isSwapDeviceActive(loopDevice)) {
            return true;
        }
        if (!runSudoCommandAndReport(
                "loop mkswap",
                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                "/sbin/mkswap",
                loopDevice)) {
            return false;
        }
        return runSudoCommandAndReport(
                "loop swapon",
                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                "/sbin/swapon",
                loopDevice);
    }

    private long loopSwapFileSizeBytes() {
        File file = new File(SYSTEM_LOOP_SWAP_FILE_PATH);
        return file.exists() ? file.length() : -1L;
    }

    private boolean isLoopSwapRunning() {
        if (!RobotBase.isReal()) {
            return false;
        }
        String loopDevice = findLoopDeviceForSwapFile();
        return isSwapDeviceActive(loopDevice);
    }

    private static RobotLocalization<?> createLocalizationForDrivetrain(
            RobotDrivetrain<?> drivetrain,
            RobotLocalizationConfig localizationConfig) {
        if (drivetrain instanceof RobotDrivetrainLocalizationFactory factory) {
            return factory.createLocalization(localizationConfig);
        }
        throw new IllegalStateException(
                "Drivetrain " + drivetrain.getClass().getName()
                        + " does not implement RobotDrivetrainLocalizationFactory.");
    }

    private void prepareDrivetrainForModeTransition(boolean driveEnabled, boolean autoEnabled) {
        drivetrain.control().reset();
        RobotSpeeds speeds = drivetrain.robotSpeeds();
        speeds.setSpeedSourceState(RobotSpeeds.DRIVE_SOURCE, driveEnabled);
        speeds.setSpeedSourceState(RobotSpeeds.AUTO_SOURCE, autoEnabled);
        speeds.stop();
    }

    private void scheduleAutonomousCommand() {
        cancelAutonomousCommand();
        Command selected = createAutonomousCommand();
        if (selected != null) {
            CommandScheduler.getInstance().schedule(selected);
            autonomousCommand = selected;
        }
    }

    private void cancelAutonomousCommand() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        drivetrain().speeds().stop("auto");
    }

    private void scheduleCustomPidCycle(Mechanism mechanism) {
        if (!mechanism.customPidCycle()) {
            return;
        }
        if (!scheduledCustomPidMechanisms.add(mechanism)) {
            return;
        }
        double period = mechanism.pidPeriodSeconds();
        if (!(period > 0.0)) {
            period = 0.02;
        }
        addPeriodic(mechanism::updatePID, period);
    }

    public RobotCore<T> registerMechanism(Mechanism... mechs) {
        if (mechs == null || mechs.length == 0) {
            return this;
        }
        for (Mechanism mech : mechs) {
            if (mech == null) {
                continue;
            }
            registerMechanismInternal(mech);
            // Ensure per-mechanism NetworkTableConfig entries exist under the mechanism itself.
            RobotNetworkTables.Node mechNode =
                    mech.networkTables().resolveNode(mechanismsRootNode);
            robotNetworkTables.mechanismConfig(mechNode);
            mech.setRobotCore(this);
            scheduleCustomPidCycle(mech);
        }
        return this;
    }

    /**
     * Registers all child mechanisms contained within the given superstructures.
     */
    public RobotCore<T> registerMechanism(SuperstructureMechanism<?, ?>... supers) {
        if (supers == null || supers.length == 0) {
            return this;
        }
        for (SuperstructureMechanism<?, ?> s : supers) {
            if (s == null) {
                continue;
            }
            registerSuperstructureInternal(s);
            // Ensure per-superstructure NetworkTableConfig entries exist under Athena/Mechanisms/<SuperstructureName>.
            RobotNetworkTables.Node superNode =
                    mechanismsRootNode.child(s.getName() != null ? s.getName() : "Superstructure");
            robotNetworkTables.superstructureConfig(superNode);
            s.setRobotCore(this);
            s.diagnostics().info("lifecycle", "superstructure registered");
            registerSuperstructureDiagnosticsProviderIfReady(s);
            // Publish mechanisms under the owning superstructure name to avoid duplicates and
            // keep dashboards navigable at scale.
            s.networkTables().ownerPath(s.getName());
            registerMechanism(s.children().all().toArray(Mechanism[]::new));
        }
        return this;
    }

    /**
     * Registers any registerable mechanism (plain or composite).
     */
    public RobotCore<T> registerMechanism(RegisterableMechanism... entries) {
        if (entries == null || entries.length == 0) {
            return this;
        }
        for (RegisterableMechanism entry : entries) {
            if (entry == null) {
                continue;
            }
            if (entry instanceof RegisterableMechanismFactory factory) {
                RegisterableMechanism built = factory.build();
                if (built == null) {
                    continue;
                }
                if (built == entry) {
                    throw new IllegalStateException("RegisterableMechanismFactory returned itself.");
                }
                registerMechanism(built);
                continue;
            }
            if (entry instanceof SuperstructureMechanism<?, ?> superstructure) {
                registerSuperstructureInternal(superstructure);
                RobotNetworkTables.Node superNode =
                        mechanismsRootNode.child(superstructure.getName() != null ? superstructure.getName() : "Superstructure");
                robotNetworkTables.superstructureConfig(superNode);
                superstructure.setRobotCore(this);
                superstructure.diagnostics().info("lifecycle", "superstructure registered");
                registerSuperstructureDiagnosticsProviderIfReady(superstructure);
                superstructure.networkTables().ownerPath(superstructure.getName());
            }
            registerMechanism(entry.flattenForRegistration().toArray(Mechanism[]::new));
        }
        return this;
    }

    private void registerConfiguredMechanisms() {
        if (configuredMechanismsRegistered) {
            return;
        }
        configuredMechanismsRegistered = true;
        if (configuredMechanisms.isEmpty()) {
            return;
        }
        registerMechanism(configuredMechanisms.toArray(RegisterableMechanism[]::new));
    }

    public Mechanism mechanism(String name) {
        return mechanisms.get(name);
    }

    public <M extends Mechanism> M mechanism(Class<M> type) {
        if (type == null) {
            return null;
        }
        for (Mechanism mechanism : mechanisms.values()) {
            if (type.isInstance(mechanism)) {
                return type.cast(mechanism);
            }
        }
        return null;
    }

    /**
     * Sectioned runtime interaction API over registered mechanisms/superstructures.
     *
     * <p>This mirrors the section-lambda style used by config builders but operates on live
     * subsystem instances after registration.</p>
     */
    public RobotCore<T> mechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        mechanismView.use(section);
        return this;
    }

    public RobotMechanisms mechanisms() {
        return mechanismView;
    }

    private String registerMechanismInternal(Mechanism mech) {
        String name = mech.getName();
        String trimmed = name != null ? name.trim() : "";
        String simple = mech.getClass().getSimpleName();

        // Treat blank or default SubsystemBase name as "unnamed" and assign a stable unique name.
        boolean unnamed = trimmed.isEmpty() || trimmed.equals(simple);
        String base = trimmed.isEmpty() ? simple : trimmed;

        if (!mechanisms.containsKey(base) && !superstructuresByName.containsKey(base)) {
            if (unnamed && !base.equals(trimmed)) {
                mech.setName(base);
            }
            mechanisms.put(base, mech);
            mechanismPublishKeysDirty = true;
            return base;
        }

        if (!unnamed) {
            throw new IllegalArgumentException(
                    "Duplicate mechanism name '" + base + "'. Mechanism names must be unique.");
        }

        // Auto-suffix unnamed/default mechanisms: TurretMechanism-2, TurretMechanism-3, ...
        int idx = 2;
        String candidate;
        do {
            candidate = base + "-" + idx;
            idx++;
        } while (mechanisms.containsKey(candidate) || superstructuresByName.containsKey(candidate));

        mech.setName(candidate);
        mechanisms.put(candidate, mech);
        mechanismPublishKeysDirty = true;
        return candidate;
    }

    private String registerSuperstructureInternal(SuperstructureMechanism<?, ?> superstructure) {
        String name = superstructure.getName();
        String trimmed = name != null ? name.trim() : "";
        String simple = superstructure.getClass().getSimpleName();

        // Treat blank or default SubsystemBase name as "unnamed" and assign a stable unique name.
        boolean unnamed = trimmed.isEmpty() || trimmed.equals(simple);
        String base = trimmed.isEmpty() ? simple : trimmed;

        if (!superstructuresByName.containsKey(base) && !mechanisms.containsKey(base)) {
            if (unnamed && !base.equals(trimmed)) {
                superstructure.setName(base);
            }
            superstructuresByName.put(base, superstructure);
            registeredSuperstructures.add(superstructure);
            superstructurePublishKeysDirty = true;
            return base;
        }

        if (!unnamed) {
            throw new IllegalArgumentException(
                    "Duplicate superstructure name '" + base + "'. Names must be unique across mechanisms and superstructures.");
        }

        int idx = 2;
        String candidate;
        do {
            candidate = base + "-" + idx;
            idx++;
        } while (superstructuresByName.containsKey(candidate) || mechanisms.containsKey(candidate));

        superstructure.setName(candidate);
        superstructuresByName.put(candidate, superstructure);
        registeredSuperstructures.add(superstructure);
        superstructurePublishKeysDirty = true;
        return candidate;
    }

    public RuntimeMode runtimeMode() {
        return runtimeMode;
    }

    private void updateRuntimeModeCache() {
        if (DriverStation.isAutonomousEnabled()) {
            runtimeMode = RuntimeMode.AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            runtimeMode = RuntimeMode.TELEOP;
        } else if (DriverStation.isTestEnabled()) {
            runtimeMode = RuntimeMode.TEST;
        } else {
            runtimeMode = RuntimeMode.DISABLED;
        }
    }

    private void maybeApplyCompetitionStreamShedding() {
        if (competitionStreamSheddingApplied || !DriverStation.isFMSAttached()) {
            return;
        }
        competitionStreamSheddingApplied = true;

        robotNetworkTables.disable(RobotNetworkTables.Flag.LOCALIZATION_POSE_TOPICS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET);
        robotNetworkTables.disable(RobotNetworkTables.Flag.VISION_CAMERA_WIDGETS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.HW_MOTOR_TUNING_WIDGETS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.HW_ENCODER_TUNING_WIDGETS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.HW_IMU_TUNING_WIDGETS);
        robotNetworkTables.disable(RobotNetworkTables.Flag.SWERVE_MODULE_DEBUG);

        double configuredPeriod = robotNetworkTables.getDefaultPeriodSeconds();
        if (!Double.isFinite(configuredPeriod) || configuredPeriod < COMPETITION_AUTO_PUBLISH_PERIOD_SECONDS) {
            robotNetworkTables.setDefaultPeriodSeconds(COMPETITION_AUTO_PUBLISH_PERIOD_SECONDS);
        }

        DataLogManager.logNetworkTables(false);
        diagnosticsSection.core().warn(
                "networktables",
                "fms attached: disabled high-bandwidth debug streams and NT datalogging");
    }

    public RobotLocalization<?> localization() {
        return localization;
    }

    public RobotCopilot copilot() {
        return copilot;
    }

    public T drivetrain() {
        return drivetrain;
    }

    public RobotVision vision() {
        return vision;
    }

    public RobotAuto autos() {
        return autos;
    }

    /**
     * IntelliSense-friendly facade over Athena NetworkTables publishing controls (global flags).
     *
     * <p>Per-mechanism controls live on each mechanism under
     * {@code Athena/Mechanisms/.../<MechName>/NetworkTableConfig/...} and can be accessed via
     * {@code mech.networkTables().toggles()}.</p>
     */
    public final RobotNetworkTables networkTables() {
        return robotNetworkTables;
    }

    /**
     * Publishes drivetrain/localization/vision topics under {@code Athena/...}.
     */
    public RobotCore<T> publishNetworkTables() {
        return publishNetworkTables("Drivetrain", "Localization");
    }

    public RobotCore<T> publishNetworkTables(String drive, String local) {
        if (!robotNetworkTables.isPublishingEnabled()) {
            return this;
        }

        RobotNetworkTables.Node root = robotNetworkTables.root();
        publishConfigExportUrls(root);
        drivetrain.networkTables(root.child(drive));
        if (localization != null) {
            localization.networkTables(root.child(local));
        }
        if (vision != null) {
            vision.networkTables(root.child("Vision"));
        }
        publishAutoNetworkTables(autoRootNode);
        publishPerformanceMetrics(root);

        coreNetworkTablesPublished = true;
        return this;
    }

    private void publishConfigExportUrls(RobotNetworkTables.Node root) {
        if (root == null) {
            return;
        }
        ConfigExportUrls urls = resolveConfigExportUrls();
        RobotNetworkTables.Node cfg = root.child("Config");
        cfg.putString("baseUrl", urls.baseUrl());
        root.putString("configBaseUrl", urls.baseUrl());
        cfg.putString("indexUrlJson", urls.indexUrlJson());
        cfg.putString("indexUrlToml", urls.indexUrlToml());
        cfg.putString("allZipUrl", urls.allZipUrl());
        cfg.putString("mechanismsBaseUrl", urls.mechanismsBaseUrl());
        cfg.putString("customConfigUrl", urls.customConfigUrl());
        cfg.putString("customConfigBaseUrl", urls.customConfigBaseUrl());
        cfg.putString("diagnosticsUrl", urls.diagnosticsUrl());
        cfg.putString("diagnosticsBaseUrl", urls.diagnosticsBaseUrl());
        cfg.putString("diagnosticsHistoryUrl", urls.diagnosticsHistoryUrl());
        cfg.putString("autoLogUrl", urls.autoLogUrl());
        cfg.putString("mechanismLogUrl", urls.mechanismLogUrl());
        cfg.putString("mechanismLogBaseUrl", urls.mechanismLogBaseUrl());
        root.putString("configIndexUrlJson", urls.indexUrlJson());
        root.putString("configIndexUrlToml", urls.indexUrlToml());
        root.putString("configAllZipUrl", urls.allZipUrl());
        root.putString("configMechanismsBaseUrl", urls.mechanismsBaseUrl());
        root.putString("configCustomUrl", urls.customConfigUrl());
        root.putString("configCustomBaseUrl", urls.customConfigBaseUrl());
        root.putString("configDiagnosticsUrl", urls.diagnosticsUrl());
        root.putString("configDiagnosticsBaseUrl", urls.diagnosticsBaseUrl());
        root.putString("configDiagnosticsHistoryUrl", urls.diagnosticsHistoryUrl());
        root.putString("configAutoLogUrl", urls.autoLogUrl());
        root.putString("configMechanismLogUrl", urls.mechanismLogUrl());
        root.putString("configMechanismLogBaseUrl", urls.mechanismLogBaseUrl());
    }

    private ConfigExportUrls resolveConfigExportUrls() {
        String base = configServerBaseUrl;
        String normalizedBase = (base == null || base.isBlank()) ? "" : base;
        if (cachedConfigExportUrls != null && normalizedBase.equals(cachedConfigExportBaseUrl)) {
            return cachedConfigExportUrls;
        }
        ConfigExportUrls resolved;
        if (normalizedBase.isEmpty()) {
            resolved = ConfigExportUrls.empty();
        } else {
            String prefix = normalizedBase + "/Athena";
            resolved = new ConfigExportUrls(
                    normalizedBase,
                    prefix + "/config/index.json",
                    prefix + "/config/index.toml",
                    prefix + "/config/all.zip",
                    prefix + "/config/mechanisms/",
                    prefix + "/config/custom",
                    prefix + "/config/custom/",
                    prefix + "/diagnostics",
                    prefix + "/diagnostics/",
                    prefix + "/diagnostics/history.json",
                    prefix + "/auto/log",
                    prefix + "/mechanisms/log",
                    prefix + "/mechanisms/log/");
        }
        cachedConfigExportBaseUrl = normalizedBase;
        cachedConfigExportUrls = resolved;
        return resolved;
    }

    private AthenaRuntimeServer ensureConfigServerReadyForCustomContent() {
        if (configServer == null) {
            startConfigServerIfNeeded();
        }
        return configServer;
    }

    private static String normalizeDiagnosticsChannelKey(String rawKey) {
        if (rawKey == null) {
            throw new IllegalArgumentException("diagnostics channel key must not be null");
        }
        String key = rawKey.trim();
        while (key.startsWith("/")) {
            key = key.substring(1);
        }
        while (key.endsWith("/")) {
            key = key.substring(0, key.length() - 1);
        }
        if (key.isEmpty()) {
            throw new IllegalArgumentException("diagnostics channel key must not be blank");
        }
        if (key.contains("..")) {
            throw new IllegalArgumentException("diagnostics channel key must not contain '..'");
        }
        if (key.indexOf('\\') >= 0) {
            throw new IllegalArgumentException("diagnostics channel key must not contain '\\'");
        }
        return key;
    }

    private DiagnosticsChannel ensureDiagnosticsChannel(String key, int capacity) {
        String normalized = normalizeDiagnosticsChannelKey(key);
        DiagnosticsChannel existing = diagnosticsChannels.get(normalized);
        if (existing != null) {
            return existing;
        }
        DiagnosticsChannel created = new DiagnosticsChannel(normalized, capacity);
        DiagnosticsChannel winner = diagnosticsChannels.putIfAbsent(normalized, created);
        DiagnosticsChannel channel = winner != null ? winner : created;
        registerDiagnosticsChannelWithServerIfReady(normalized, channel);
        return channel;
    }

    private void registerDiagnosticsChannelWithServerIfReady(String key, DiagnosticsChannel channel) {
        AthenaRuntimeServer server = configServer;
        if (server == null || key == null || channel == null) {
            return;
        }
        server.registerDiagnosticsProvider(key, channel::summary, channel::snapshot, channel::clear);
    }

    private void registerSuperstructureDiagnosticsProviderIfReady(SuperstructureMechanism<?, ?> superstructure) {
        AthenaRuntimeServer server = configServer;
        if (server == null || superstructure == null) {
            return;
        }
        String name = superstructure.getName();
        if (name == null || name.isBlank()) {
            name = superstructure.getClass().getSimpleName();
        }
        String key = "superstructures/" + normalizeDiagnosticsChannelKey(name);
        final SuperstructureMechanism<?, ?> tracked = superstructure;
        server.registerDiagnosticsProvider(
                key,
                () -> buildSuperstructureDiagnosticsSummary(tracked),
                limit -> buildSuperstructureDiagnosticsSnapshot(tracked, limit),
                () -> tracked.diagnostics().clear());
    }

    private Map<String, Object> buildSuperstructureDiagnosticsSummary(SuperstructureMechanism<?, ?> superstructure) {
        if (superstructure == null) {
            return Map.of("error", "superstructure unavailable");
        }
        return superstructure.diagnostics().summary();
    }

    private Map<String, Object> buildSuperstructureDiagnosticsSnapshot(
            SuperstructureMechanism<?, ?> superstructure,
            int logLimit) {
        Map<String, Object> snapshot = new HashMap<>(buildSuperstructureDiagnosticsSummary(superstructure));
        if (superstructure == null) {
            return snapshot;
        }
        int effectiveLimit = logLimit > 0 ? Math.min(logLimit, 512) : 120;
        snapshot.put("events", superstructure.diagnostics().events(effectiveLimit));
        List<Map<String, Object>> children = new ArrayList<>();
        for (Mechanism mechanism : superstructure.children().all()) {
            if (mechanism == null) {
                continue;
            }
            Map<String, Object> child = new HashMap<>();
            child.put("name", mechanism.getName());
            child.put("atSetpoint", mechanism.atSetpoint());
            child.put("position", mechanism.position());
            child.put("setpoint", mechanism.setpoint());
            child.put("output", mechanism.output());
            int childLimit = logLimit > 0 ? Math.min(logLimit, 128) : 0;
            child.put("diagnostics", mechanism.diagnostics().snapshot(childLimit));
            children.add(child);
        }
        snapshot.put("children", children);
        return snapshot;
    }

    private void registerDiagnosticsProvidersWithServer() {
        AthenaRuntimeServer server = configServer;
        if (server == null) {
            return;
        }
        if (localization != null) {
            server.registerDiagnosticsProvider(
                    "localization",
                    localization::getDiagnosticsSummary,
                    localization::getDiagnosticsSnapshot,
                    localization::clearDiagnosticsLog);
        }
        if (vision != null) {
            server.registerDiagnosticsProvider(
                    "vision",
                    () -> vision.diagnostics().summary(),
                    limit -> vision.diagnostics().snapshot(limit),
                    () -> vision.diagnostics().clear());
        }
        for (Map.Entry<String, DiagnosticsChannel> entry : diagnosticsChannels.entrySet()) {
            if (entry == null || entry.getKey() == null || entry.getValue() == null) {
                continue;
            }
            registerDiagnosticsChannelWithServerIfReady(entry.getKey(), entry.getValue());
        }
        for (SuperstructureMechanism<?, ?> superstructure : superstructuresByName.values()) {
            registerSuperstructureDiagnosticsProviderIfReady(superstructure);
        }
    }

    private final class DiagnosticsSection {
        private final RobotCore<T> owner;
        private static final int DEFAULT_CHANNEL_CAPACITY = 256;
        private static final int CORE_CHANNEL_CAPACITY = 512;
        private static final int LOCALIZATION_CHANNEL_CAPACITY = 384;
        private static final int VISION_CHANNEL_CAPACITY = 384;
        private static final int SUPERSTRUCTURE_CHANNEL_CAPACITY = 256;

        private DiagnosticsSection(RobotCore<T> owner) {
            this.owner = owner;
        }

        public DiagnosticsChannel channel(String key) {
            return owner.ensureDiagnosticsChannel(key, DEFAULT_CHANNEL_CAPACITY);
        }

        public DiagnosticsChannel channel(String key, int capacity) {
            int resolvedCapacity = capacity > 0 ? capacity : DEFAULT_CHANNEL_CAPACITY;
            return owner.ensureDiagnosticsChannel(key, resolvedCapacity);
        }

        public DiagnosticsChannel core() {
            return owner.ensureDiagnosticsChannel("core", CORE_CHANNEL_CAPACITY);
        }

        public DiagnosticsChannel localization() {
            return owner.ensureDiagnosticsChannel("localization", LOCALIZATION_CHANNEL_CAPACITY);
        }

        public DiagnosticsChannel vision() {
            return owner.ensureDiagnosticsChannel("vision", VISION_CHANNEL_CAPACITY);
        }

        public DiagnosticsChannel superstructure(SuperstructureMechanism<?, ?> superstructure) {
            if (superstructure == null) {
                throw new IllegalArgumentException("superstructure must not be null");
            }
            String name = superstructure.getName();
            if (name == null || name.isBlank()) {
                name = superstructure.getClass().getSimpleName();
            }
            return owner.ensureDiagnosticsChannel("superstructures/" + name, SUPERSTRUCTURE_CHANNEL_CAPACITY);
        }

        public boolean remove(String key) {
            String normalized = normalizeDiagnosticsChannelKey(key);
            DiagnosticsChannel removed = diagnosticsChannels.remove(normalized);
            AthenaRuntimeServer server = configServer;
            if (server != null) {
                server.removeDiagnosticsProvider(normalized);
            }
            return removed != null;
        }

        public void clear() {
            diagnosticsChannels.clear();
            AthenaRuntimeServer server = configServer;
            if (server != null) {
                server.clearDiagnosticsProviders();
                registerDiagnosticsProvidersWithServer();
            }
        }
    }

    public final class SystemSection {
        private SystemSection() {}

        public SystemSection configServerEnabled(boolean enabled) {
            setConfigServerEnabled(enabled);
            return this;
        }

        public boolean configServerEnabled() {
            return configServerEnabled;
        }

        public SystemSection telemetryEnabled(boolean enabled) {
            telemetry.setEnabled(enabled);
            return this;
        }

        public boolean telemetryEnabled() {
            return telemetry.isEnabled();
        }

        public SystemSection networkTablesDataLogEnabled(boolean enabled) {
            DataLogManager.logNetworkTables(enabled);
            return this;
        }

        /**
         * Attempts to control NI systemWebServer using sudo.
         */
        public boolean setSystemWebServerEnabled(boolean enabled) {
            return runSystemWebServerControlCommand(enabled ? "start" : "stop");
        }

        public SystemSection systemWebServerEnabled(boolean enabled) {
            setSystemWebServerEnabled(enabled);
            return this;
        }

        public boolean restartSystemWebServer() {
            return runSystemWebServerControlCommand("restart");
        }

        public boolean systemWebServerRunning() {
            return isSystemWebServerRunning();
        }

        /**
         * Conservative defaults for low-RAM targets without changing NT-controlled publish flags/period.
         */
        public SystemSection applyLowMemoryDefaults() {
            setConfigServerEnabled(false);
            telemetry.setEnabled(false);
            DataLogManager.logNetworkTables(false);
            return this;
        }

        public boolean overcommitMode(int mode) {
            int resolvedMode = clampInt(mode, SYSTEM_OVERCOMMIT_MODE_MIN, SYSTEM_OVERCOMMIT_MODE_MAX);
            return setVmSysctl("vm.overcommit_memory", resolvedMode);
        }

        public boolean overcommitRatio(int ratio) {
            int resolvedRatio = clampInt(ratio, SYSTEM_OVERCOMMIT_RATIO_MIN, SYSTEM_OVERCOMMIT_RATIO_MAX);
            return setVmSysctl("vm.overcommit_ratio", resolvedRatio);
        }

        public boolean swappiness(int value) {
            int resolved = clampInt(value, SYSTEM_SWAPPINESS_MIN, SYSTEM_SWAPPINESS_MAX);
            return setVmSysctl("vm.swappiness", resolved);
        }

        /**
         * Aggressive mode: reduces optional services, enables swap, and relaxes strict commit limits.
         */
        public SystemSection applyAggressiveLowMemoryMode() {
            setSystemWebServerEnabled(false);
            setLoopSwapEnabled(true, SYSTEM_LOOP_SWAP_DEFAULT_SIZE_MIB);
            overcommitMode(1);
            overcommitRatio(95);
            swappiness(120);
            applyLowMemoryDefaults();
            return this;
        }

        /**
         * Enables/disables loopback swap via direct sudo commands.
         */
        public boolean setLoopSwapEnabled(boolean enabled, int sizeMiB) {
            return configureLoopSwapEnabled(enabled, sizeMiB);
        }

        public boolean setLoopSwapEnabled(boolean enabled) {
            return configureLoopSwapEnabled(enabled, SYSTEM_LOOP_SWAP_DEFAULT_SIZE_MIB);
        }

        public SystemSection loopSwapEnabled(boolean enabled, int sizeMiB) {
            configureLoopSwapEnabled(enabled, sizeMiB);
            return this;
        }

        public SystemSection loopSwapEnabled(boolean enabled) {
            configureLoopSwapEnabled(enabled, SYSTEM_LOOP_SWAP_DEFAULT_SIZE_MIB);
            return this;
        }

        public boolean loopSwapRunning() {
            return isLoopSwapRunning();
        }
    }

    public final class ConfigServerSection {
        private final RobotCore<T> owner;

        private ConfigServerSection(RobotCore<T> owner) {
            this.owner = owner;
        }

        public boolean available() {
            return owner.configServer != null || owner.ensureConfigServerReadyForCustomContent() != null;
        }

        public String baseUrl() {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            return server != null ? server.baseUrl() : null;
        }

        public String customUrl(String key) {
            String base = baseUrl();
            if (base == null || base.isBlank()) {
                return null;
            }
            String encoded = java.net.URLEncoder.encode(
                    key != null ? key : "",
                    java.nio.charset.StandardCharsets.UTF_8);
            return base + "/Athena/config/custom/" + encoded;
        }

        public String diagnosticsUrl() {
            String base = baseUrl();
            if (base == null || base.isBlank()) {
                return null;
            }
            return base + "/Athena/diagnostics";
        }

        public String diagnosticsHistoryUrl() {
            String base = baseUrl();
            if (base == null || base.isBlank()) {
                return null;
            }
            return base + "/Athena/diagnostics/history.json";
        }

        public String diagnosticsUrl(String key) {
            String base = baseUrl();
            if (base == null || base.isBlank()) {
                return null;
            }
            String encoded = java.net.URLEncoder.encode(
                    key != null ? key : "",
                    java.nio.charset.StandardCharsets.UTF_8);
            return base + "/Athena/diagnostics/" + encoded + ".json";
        }

        public ConfigServerSection json(String key, Object payload) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.putCustomJson(key, payload);
            }
            return this;
        }

        public ConfigServerSection text(String key, String text) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.putCustomText(key, text);
            }
            return this;
        }

        public ConfigServerSection bytes(String key, String contentType, byte[] payload) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.putCustomBytes(key, contentType, payload);
            }
            return this;
        }

        public ConfigServerSection remove(String key) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.removeCustom(key);
            }
            return this;
        }

        public ConfigServerSection clear() {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.clearCustom();
            }
            return this;
        }

        public ConfigServerSection diagnostics(
                String key,
                Supplier<Map<String, Object>> summarySupplier,
                IntFunction<Map<String, Object>> snapshotSupplier) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.registerDiagnosticsProvider(key, summarySupplier, snapshotSupplier);
            }
            return this;
        }

        public ConfigServerSection diagnostics(
                String key,
                Supplier<Map<String, Object>> summarySupplier,
                IntFunction<Map<String, Object>> snapshotSupplier,
                Runnable clearAction) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.registerDiagnosticsProvider(key, summarySupplier, snapshotSupplier, clearAction);
            }
            return this;
        }

        public ConfigServerSection removeDiagnostics(String key) {
            AthenaRuntimeServer server = owner.ensureConfigServerReadyForCustomContent();
            if (server != null) {
                server.removeDiagnosticsProvider(key);
            }
            return this;
        }
    }

    private void publishPerformanceMetrics(RobotNetworkTables.Node root) {
        if (root == null) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();

        RobotNetworkTables.Node performance = root.child("Performance");
        RobotNetworkTables.Node status = performance.child("Status");
        status.putBoolean("isReal", RobotBase.isReal());
        status.putBoolean("isSimulation", RobotBase.isSimulation());
        status.putBoolean("isDSAttached", DriverStation.isDSAttached());
        status.putBoolean("isFMSAttached", DriverStation.isFMSAttached());

        RobotNetworkTables.Node loop = performance.child("Loop");
        loop.putDouble("periodSec", getPeriod());
        loop.putDouble("budgetMs", getPeriod() * 1000.0);
        loop.putDouble("schedulerMs", finiteOr(lastLoopSchedulerMs, -1.0));
        loop.putDouble("telemetryMs", finiteOr(lastLoopTelemetryMs, -1.0));
        loop.putDouble("localizationMs", finiteOr(lastLoopLocalizationMs, -1.0));
        loop.putDouble("userMs", finiteOr(lastLoopUserMs, -1.0));
        loop.putDouble("totalMs", finiteOr(lastLoopTotalMs, -1.0));
        loop.putDouble("utilizationPercent", ratioPercent(lastLoopTotalMs, getPeriod() * 1000.0));

        boolean publishSlowMetrics = shouldPublishSlowPerformanceMetrics(nowSeconds);
        boolean publishVerySlowMetrics = shouldPublishVerySlowPerformanceMetrics(nowSeconds);
        if (!publishSlowMetrics && !publishVerySlowMetrics) {
            return;
        }
        if (publishSlowMetrics) {
            lastSlowPerformanceMetricsPublishSeconds = nowSeconds;

            status.putBoolean("isSysActive", RobotController.isSysActive());
            status.putBoolean("isBrownedOut", RobotController.isBrownedOut());
            status.putBoolean("isSystemTimeValid", RobotController.isSystemTimeValid());
            status.putDouble("fpgaTimeMicros", RobotController.getFPGATime());
            status.putDouble("uptimeSec", runtimeMxBean.getUptime() / 1000.0);
            status.putDouble("teamNumber", RobotController.getTeamNumber());

            RobotNetworkTables.Node power = performance.child("Power");
            power.putDouble("batteryVoltage", RobotController.getBatteryVoltage());
            power.putDouble("inputVoltage", RobotController.getInputVoltage());
            power.putDouble("inputCurrentAmps", RobotController.getInputCurrent());
            power.putDouble("cpuTempCelsius", RobotController.getCPUTemp());
            power.putDouble("brownoutVoltage", RobotController.getBrownoutVoltage());
            power.putBoolean("rslState", RobotController.getRSLState());

            RobotNetworkTables.Node rails = performance.child("Rails");
            RobotNetworkTables.Node rail3v3 = rails.child("3V3");
            rail3v3.putDouble("voltage", RobotController.getVoltage3V3());
            rail3v3.putDouble("currentAmps", RobotController.getCurrent3V3());
            rail3v3.putBoolean("enabled", RobotController.getEnabled3V3());
            rail3v3.putDouble("faultCount", RobotController.getFaultCount3V3());
            RobotNetworkTables.Node rail5v = rails.child("5V");
            rail5v.putDouble("voltage", RobotController.getVoltage5V());
            rail5v.putDouble("currentAmps", RobotController.getCurrent5V());
            rail5v.putBoolean("enabled", RobotController.getEnabled5V());
            rail5v.putDouble("faultCount", RobotController.getFaultCount5V());
            RobotNetworkTables.Node rail6v = rails.child("6V");
            rail6v.putDouble("voltage", RobotController.getVoltage6V());
            rail6v.putDouble("currentAmps", RobotController.getCurrent6V());
            rail6v.putBoolean("enabled", RobotController.getEnabled6V());
            rail6v.putDouble("faultCount", RobotController.getFaultCount6V());

            RobotNetworkTables.Node cpu = performance.child("CPU");
            cpu.putDouble("availableProcessors", osMxBean.getAvailableProcessors());
            cpu.putDouble("systemLoadAverage", finiteOr(osMxBean.getSystemLoadAverage(), -1.0));
            if (sunOsMxBean != null) {
                cpu.putDouble("processLoadPercent", finiteOr(sunOsMxBean.getProcessCpuLoad() * 100.0, -1.0));
                cpu.putDouble("systemLoadPercent", finiteOr(sunOsMxBean.getCpuLoad() * 100.0, -1.0));
                cpu.putDouble("processCpuTimeSec", sunOsMxBean.getProcessCpuTime() / 1_000_000_000.0);
            } else {
                cpu.putDouble("processLoadPercent", -1.0);
                cpu.putDouble("systemLoadPercent", -1.0);
                cpu.putDouble("processCpuTimeSec", -1.0);
            }
        }

        if (publishVerySlowMetrics) {
            lastVerySlowPerformanceMetricsPublishSeconds = nowSeconds;

            MemoryUsage heap = memoryMxBean.getHeapMemoryUsage();
            MemoryUsage nonHeap = memoryMxBean.getNonHeapMemoryUsage();
            long heapUsed = Math.max(0L, heap.getUsed());
            long heapCommitted = Math.max(0L, heap.getCommitted());
            long heapMax = Math.max(0L, heap.getMax());
            long nonHeapUsed = Math.max(0L, nonHeap.getUsed());
            long nonHeapCommitted = Math.max(0L, nonHeap.getCommitted());

            RobotNetworkTables.Node memory = performance.child("Memory");
            memory.putDouble("heapUsedBytes", heapUsed);
            memory.putDouble("heapCommittedBytes", heapCommitted);
            memory.putDouble("heapMaxBytes", heapMax);
            memory.putDouble("heapUsedPercent", ratioPercent(heapUsed, heapMax));
            memory.putDouble("nonHeapUsedBytes", nonHeapUsed);
            memory.putDouble("nonHeapCommittedBytes", nonHeapCommitted);
            memory.putDouble("processCommittedVirtualBytes", sunOsMxBean != null ? sunOsMxBean.getCommittedVirtualMemorySize() : -1.0);
            memory.putDouble("ntTopicCacheSize", robotNetworkTables.topicCacheSize());
            memory.putDouble("ntBooleanPublisherCount", robotNetworkTables.booleanPublisherCount());
            memory.putDouble("ntDoublePublisherCount", robotNetworkTables.doublePublisherCount());
            memory.putDouble("ntStringPublisherCount", robotNetworkTables.stringPublisherCount());
            memory.putDouble("ntMechanismToggleCount", robotNetworkTables.mechanismToggleCount());
            memory.putDouble("telemetryEntryCount", telemetry.entryCount());
            memory.putDouble("diagnosticsChannelCount", diagnosticsChannels.size());
            if (sunOsMxBean != null) {
                long totalMemory = Math.max(0L, sunOsMxBean.getTotalMemorySize());
                long freeMemory = Math.max(0L, sunOsMxBean.getFreeMemorySize());
                long usedMemory = Math.max(0L, totalMemory - freeMemory);
                memory.putDouble("systemTotalBytes", totalMemory);
                memory.putDouble("systemFreeBytes", freeMemory);
                memory.putDouble("systemUsedBytes", usedMemory);
                memory.putDouble("systemUsedPercent", ratioPercent(usedMemory, totalMemory));
            } else {
                memory.putDouble("systemTotalBytes", -1.0);
                memory.putDouble("systemFreeBytes", -1.0);
                memory.putDouble("systemUsedBytes", -1.0);
                memory.putDouble("systemUsedPercent", -1.0);
            }

            File storagePath = resolveStorageMetricsPath();
            long storageTotal = Math.max(0L, storagePath.getTotalSpace());
            long storageFree = Math.max(0L, storagePath.getFreeSpace());
            long storageUsable = Math.max(0L, storagePath.getUsableSpace());
            long storageUsed = Math.max(0L, storageTotal - storageUsable);
            memory.putString("storagePath", storagePath.getAbsolutePath());
            if (storageTotal > 0L) {
                memory.putDouble("storageTotalBytes", storageTotal);
                memory.putDouble("storageFreeBytes", storageFree);
                memory.putDouble("storageUsableBytes", storageUsable);
                memory.putDouble("storageUsedBytes", storageUsed);
                memory.putDouble("storageUsedPercent", ratioPercent(storageUsed, storageTotal));
            } else {
                memory.putDouble("storageTotalBytes", -1.0);
                memory.putDouble("storageFreeBytes", -1.0);
                memory.putDouble("storageUsableBytes", -1.0);
                memory.putDouble("storageUsedBytes", -1.0);
                memory.putDouble("storageUsedPercent", -1.0);
            }

            long gcCollections = 0;
            long gcCollectionTimeMs = 0;
            for (GarbageCollectorMXBean gcBean : gcMxBeans) {
                if (gcBean == null) {
                    continue;
                }
                long count = gcBean.getCollectionCount();
                long timeMs = gcBean.getCollectionTime();
                if (count >= 0) {
                    gcCollections += count;
                }
                if (timeMs >= 0) {
                    gcCollectionTimeMs += timeMs;
                }
            }
            RobotNetworkTables.Node jvm = performance.child("JVM");
            jvm.putDouble("gcCollectionCount", gcCollections);
            jvm.putDouble("gcCollectionTimeMs", gcCollectionTimeMs);
        }
    }

    private boolean shouldPublishSlowPerformanceMetrics(double nowSeconds) {
        if (!Double.isFinite(lastSlowPerformanceMetricsPublishSeconds)) {
            return true;
        }
        if (!Double.isFinite(nowSeconds)) {
            return true;
        }
        return nowSeconds - lastSlowPerformanceMetricsPublishSeconds >= PERFORMANCE_SLOW_METRICS_PERIOD_SECONDS;
    }

    private boolean shouldPublishVerySlowPerformanceMetrics(double nowSeconds) {
        if (!Double.isFinite(lastVerySlowPerformanceMetricsPublishSeconds)) {
            return true;
        }
        if (!Double.isFinite(nowSeconds)) {
            return true;
        }
        return nowSeconds - lastVerySlowPerformanceMetricsPublishSeconds >= PERFORMANCE_VERY_SLOW_METRICS_PERIOD_SECONDS;
    }

    private static double ratioPercent(double numerator, double denominator) {
        if (!Double.isFinite(numerator) || !Double.isFinite(denominator) || denominator <= 0.0) {
            return -1.0;
        }
        return (numerator / denominator) * 100.0;
    }

    private static double finiteOr(double value, double fallback) {
        return Double.isFinite(value) ? value : fallback;
    }

    private File resolveStorageMetricsPath() {
        String logDir = DataLogManager.getLogDir();
        if (logDir != null && !logDir.isBlank()) {
            File configured = new File(logDir.trim());
            if (configured.exists()) {
                return configured;
            }
        }
        if (RobotBase.isReal()) {
            return new File("/home/lvuser");
        }
        return Filesystem.getOperatingDirectory();
    }

    /**
     * Publishes all registered mechanisms under {@code Athena/Mechanisms/...}.
     */
    public RobotCore<T> publishNetworkTablesMechanisms() {
        if (!robotNetworkTables.isPublishingEnabled()) {
            return this;
        }
        mechanismsNetworkTablesPublished = true;

        RobotNetworkTables.Node mechanismsNode = mechanismsRootNode;
        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            mech.networkTables(mech.networkTables().resolveNode(mechanismsNode));
        }

        for (Map.Entry<String, SuperstructureMechanism<?, ?>> e : superstructuresByName.entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            e.getValue().networkTables(mechanismsNode.child(e.getKey()));
        }
        return this;
    }

    public RotateToPoint rotateTo(double x, double y) {
        return new RotateToPoint(this, x, y);
    }

    public RotateToAngle rotateTo(double degrees) {
        return new RotateToAngle(this, degrees);
    }

    public RotateToAngle rotateBy(double degrees) {
        return new RotateToAngle(
                this,
                drivetrain().imu().device().getYaw().plus(Rotation2d.fromDegrees(degrees)));
    }

    public RobotSpeeds robotSpeeds() {
        return drivetrain.robotSpeeds();
    }

    public Imu imu() {
        return drivetrain.imu().device();
    }

    public SendableChooser<Command> registerAutoChooser(RobotAuto.AutoKey defaultAuto) {
        // Keep a program chooser alive so selected routine/poses remain available for visualization topics.
        autos.selection().programChooser(defaultAuto);
        return autos.selection().commandChooser(defaultAuto);
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(RobotAuto.AutoKey defaultAuto) {
        return registerAutoProgramChooser(defaultAuto);
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoProgramChooser(RobotAuto.AutoKey defaultProgram) {
        SendableChooser<RobotAuto.AutoRoutine> chooser = autos.selection().programChooser(defaultProgram);
        autoProgramChooserPublisher = publishSendable(
                autoProgramChooserPublisher,
                autoRootNode.child(AUTO_PROGRAM_CHOOSER_KEY),
                chooser);
        return chooser;
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(String defaultAuto) {
        return registerAutoProgramChooser(defaultAuto);
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoProgramChooser(String defaultProgram) {
        return registerAutoProgramChooser(RobotAuto.AutoKey.of(defaultProgram));
    }

    public SendableChooser<Command> registerAutoChooser(String defaultsAuto) {
        return registerAutoChooser(RobotAuto.AutoKey.of(defaultsAuto));
    }

    private void ensureAutoChooserPublished() {
        if (autos.selection().chooser().isPresent() || autos.selection().commandChooser().isPresent()) {
            return;
        }
        autos.routines().all().stream()
                .findFirst()
                .map(RobotAuto.AutoRoutine::key)
                .ifPresent(this::registerAutoProgramChooser);
    }

    public void registerPIDCycles() {
        mechanisms.values().forEach(this::scheduleCustomPidCycle);
    }

    public void resetPIDs() {
        for (Mechanism mech : mechanisms.values()) {
            mech.control().resetPid();
        }
    }

    public ChassisSpeeds createRobotRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    public ChassisSpeeds createFieldRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), localization().pose().getRotation());
    }

    private boolean isAutoInitResetEnabled() {
        if (autoInitResetEntry == null) {
            autoInitResetEntry = autoRootNode.entry("ResetOdometryOnInit");
            initIfAbsent(autoInitResetEntry, autoInitResetEnabled);
        }
        return autoInitResetEntry.getBoolean(autoInitResetEnabled);
    }

    private void publishAutoNetworkTables(RobotNetworkTables.Node autoNode) {
        if (autoNode == null) {
            return;
        }
        autoNode.putBoolean("resetOdometryOnInit", isAutoInitResetEnabled());
        autoNode.putDouble("routineCount", autos.routines().all().size());
        RobotSpeeds speeds = drivetrain != null ? drivetrain.robotSpeeds() : null;
        RobotNetworkTables.Node followerNode = autoNode.child("Follower");
        if (speeds != null) {
            followerNode.putBoolean("enabled", speeds.isSpeedsSourceActive(RobotSpeeds.AUTO_SOURCE));
            ChassisSpeeds input = speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
            ChassisSpeeds output = speeds.getSpeeds(RobotSpeeds.AUTO_SOURCE);
            followerNode.putDouble("inputVxMps", input.vxMetersPerSecond);
            followerNode.putDouble("inputVyMps", input.vyMetersPerSecond);
            followerNode.putDouble("inputOmegaRadPerSec", input.omegaRadiansPerSecond);
            followerNode.putDouble("outputVxMps", output.vxMetersPerSecond);
            followerNode.putDouble("outputVyMps", output.vyMetersPerSecond);
            followerNode.putDouble("outputOmegaRadPerSec", output.omegaRadiansPerSecond);
        }
        RobotLocalization<?> localizationRef = localization;
        if (localizationRef != null) {
            RobotLocalizationConfig cfg = localizationRef.getLocalizationConfig();
            HolonomicPidConstants translation = cfg.translation();
            HolonomicPidConstants rotation = cfg.rotation();
            followerNode.putString("pose", cfg.autoPoseName());
            RobotNetworkTables.Node pidNode = followerNode.child("Pid");
            if (translation != null) {
                pidNode.putDouble("translationKp", translation.kP());
                pidNode.putDouble("translationKi", translation.kI());
                pidNode.putDouble("translationKd", translation.kD());
                pidNode.putDouble("translationIZone", translation.iZone());
            }
            if (rotation != null) {
                pidNode.putDouble("rotationKp", rotation.kP());
                pidNode.putDouble("rotationKi", rotation.kI());
                pidNode.putDouble("rotationKd", rotation.kD());
                pidNode.putDouble("rotationIZone", rotation.iZone());
            }
        }
        RobotNetworkTables.Node selectedNode = autoNode.child("Selected");
        RobotAuto.AutoRoutine selectedRoutine = autos.selection().selected().orElse(null);
        if (selectedRoutine != null) {
            selectedNode.putString("id", selectedRoutine.key().id());
            selectedNode.putString("displayName", selectedRoutine.key().displayName());
            selectedNode.putString("source", selectedRoutine.source().name());
            selectedNode.putString("reference", selectedRoutine.reference());
            selectedNode.putBoolean("hasStartingPose", selectedRoutine.hasStartingPose());
        } else {
            selectedNode.putString("id", "");
            selectedNode.putString("displayName", "");
            selectedNode.putString("source", "");
            selectedNode.putString("reference", "");
            selectedNode.putBoolean("hasStartingPose", false);
        }
        java.util.Optional<java.util.List<Pose2d>> selectedPoses = autos.selection().selectedPoses();
        if (selectedPoses.isEmpty()) {
            selectedPoses = selectedRoutine != null ? autos.routines().poses(selectedRoutine) : java.util.Optional.empty();
        }
        selectedPoses
                .filter(poses -> poses != null && !poses.isEmpty())
                .ifPresentOrElse(
                        poses -> {
                            int trajectoryPointCount = poses.size();
                            autoNode.putBoolean("hasTrajectory", true);
                            autoNode.putDouble("trajectoryPointCount", trajectoryPointCount);
                            selectedNode.putBoolean("hasTrajectory", true);
                            selectedNode.putDouble("trajectoryPointCount", trajectoryPointCount);
                            publishSelectedAutoTrajectoryIfChanged(
                                    buildSelectedAutoTrajectorySignature(selectedRoutine, poses),
                                    trajectoryPointCount,
                                    poses);
                        },
                        () -> {
                            autoNode.putBoolean("hasTrajectory", false);
                            autoNode.putDouble("trajectoryPointCount", 0.0);
                            selectedNode.putBoolean("hasTrajectory", false);
                            selectedNode.putDouble("trajectoryPointCount", 0.0);
                            publishEmptySelectedAutoTrajectoryIfNeeded();
                        });
    }

    private void publishSelectedAutoTrajectoryIfChanged(
            String signature,
            int pointCount,
            List<Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            publishEmptySelectedAutoTrajectoryIfNeeded();
            return;
        }
        String resolvedSignature = signature != null ? signature : EMPTY_AUTO_TRAJECTORY_SIGNATURE;
        if (pointCount == lastPublishedAutoTrajectoryPointCount
                && resolvedSignature.equals(lastPublishedAutoTrajectorySignature)) {
            return;
        }
        selectedAutoTrajectoryPublisher.set(poses.toArray(Pose2d[]::new));
        lastPublishedAutoTrajectorySignature = resolvedSignature;
        lastPublishedAutoTrajectoryPointCount = pointCount;
    }

    private void publishEmptySelectedAutoTrajectoryIfNeeded() {
        if (lastPublishedAutoTrajectoryPointCount == 0
                && EMPTY_AUTO_TRAJECTORY_SIGNATURE.equals(lastPublishedAutoTrajectorySignature)) {
            return;
        }
        selectedAutoTrajectoryPublisher.set(EMPTY_POSE2D_ARRAY);
        lastPublishedAutoTrajectorySignature = EMPTY_AUTO_TRAJECTORY_SIGNATURE;
        lastPublishedAutoTrajectoryPointCount = 0;
    }

    private static String buildSelectedAutoTrajectorySignature(
            RobotAuto.AutoRoutine selectedRoutine,
            List<Pose2d> poses) {
        String routineId = "";
        String routineSource = "";
        String routineReference = "";
        if (selectedRoutine != null) {
            routineId = selectedRoutine.key() != null && selectedRoutine.key().id() != null
                    ? selectedRoutine.key().id()
                    : "";
            routineSource = selectedRoutine.source() != null ? selectedRoutine.source().name() : "";
            routineReference = selectedRoutine.reference() != null ? selectedRoutine.reference() : "";
        }
        int count = poses != null ? poses.size() : 0;
        if (count <= 0) {
            return routineId + "|" + routineSource + "|" + routineReference + "|0";
        }
        Pose2d first = poses.get(0);
        Pose2d last = poses.get(count - 1);
        return routineId + "|" + routineSource + "|" + routineReference + "|" + count + "|"
                + poseSignature(first) + "|" + poseSignature(last);
    }

    private static String poseSignature(Pose2d pose) {
        if (pose == null) {
            return "null";
        }
        return Double.doubleToLongBits(pose.getX()) + ","
                + Double.doubleToLongBits(pose.getY()) + ","
                + Double.doubleToLongBits(pose.getRotation().getRadians());
    }

    private void updateAutoChooserPublishers() {
        updateSendablePublisher(autoProgramChooserPublisher);
    }

    private static NtSendablePublisher publishSendable(
            NtSendablePublisher existing,
            RobotNetworkTables.Node node,
            Sendable sendable) {
        if (existing != null) {
            existing.close();
        }
        return new NtSendablePublisher(node, sendable);
    }

    private static void updateSendablePublisher(NtSendablePublisher publisher) {
        if (publisher != null) {
            publisher.update();
        }
    }

    private static void initIfAbsent(NetworkTableEntry entry, boolean value) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setBoolean(value);
        }
    }

    private static final class NtSendablePublisher {
        private final SendableBuilderImpl builder;

        private NtSendablePublisher(RobotNetworkTables.Node node, Sendable sendable) {
            builder = new SendableBuilderImpl();
            builder.setTable(node.table());
            sendable.initSendable(builder);
            builder.startListeners();
            builder.update();
        }

        private void update() {
            builder.update();
        }

        private void close() {
            builder.close();
        }
    }

    private void resetAutoInitPoseIfConfigured() {
        if (localization == null) {
            return;
        }
        autos.selection().selected().ifPresent(routine -> {
            Boolean override = routine.autoInitResetOverride();
            boolean shouldReset = override != null ? override : isAutoInitResetEnabled();
            if (!shouldReset || !routine.hasStartingPose()) {
                return;
            }
            localization.resetPose(localization.getLocalizationConfig().autoPoseName(), routine.startingPose());
        });
    }
}
