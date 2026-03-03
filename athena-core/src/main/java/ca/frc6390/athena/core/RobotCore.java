package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
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
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.arcp.ArcpRuntime;
import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;
import ca.frc6390.athena.core.arcp.ARCP;
import ca.frc6390.athena.core.arcp.ArcpDeviceWidgets;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.PoseConfig;
import ca.frc6390.athena.core.loop.TimedRunner;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.localization.RobotDrivetrainLocalizationFactory;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.StatefulLike;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanismFactory;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
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
import edu.wpi.first.networktables.NetworkTable;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCore<T extends RobotDrivetrain<T>> extends TimedRobot {
    private static volatile RobotCore<?> activeInstance;
    private static final String AUTO_PROGRAM_CHOOSER_KEY = "ProgramChooser";
    private static final String AUTO_FOLLOWER_TRANSLATION_PID_WIDGET_KEY = "Athena/Auto/Follower/Pid/Translation";
    private static final String AUTO_FOLLOWER_ROTATION_PID_WIDGET_KEY = "Athena/Auto/Follower/Pid/Rotation";
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
            axis.kp(constants.kP())
                    .ki(constants.kI())
                    .kd(constants.kD())
                    .iZone(constants.iZone())
                    .inverted(constants.inverted());
        }
    }

    /**
     * Startup/runtime system behavior for {@link RobotCore}.
     *
     * @param tweaksEnabled startup gate for OS-level tweaks. When {@code true}, Athena applies the
     *        configured sysctl/swap/SystemWebServer settings at boot; when {@code false}, Athena
     *        resets those knobs toward {@link #rioDefaults()} behavior at boot.
     * @param vmOvercommitMode target value for Linux {@code vm.overcommit_memory}. Typical values are:
     *        {@code 0}=heuristic, {@code 1}=always overcommit, {@code 2}=strict commit accounting.
     * @param vmOvercommitRatio target value for Linux {@code vm.overcommit_ratio} (percent).
     *        Primarily meaningful when {@code vmOvercommitMode=2}.
     * @param vmSwappiness target value for Linux {@code vm.swappiness}. Higher values make the kernel
     *        more willing to reclaim anonymous memory to swap.
     * @param loopSwapEnabled whether Athena should enable a loopback swap file at startup.
     * @param loopSwapSizeMiB loopback swap file size in MiB when enabled.
     * @param systemWebServerEnabled whether Athena should try to start/stop NI {@code SystemWebServer}.
     * @param configServerEnabled runtime toggle for Athena's config/diagnostics HTTP server.
     * @param telemetryEnabled runtime toggle for Athena telemetry publishing.
     * @param networkTablesDataLogEnabled runtime toggle for NT datalogging via {@link DataLogManager}.
     */
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

        /**
         * Desktop-oriented defaults that prefer survivability under memory pressure.
         */
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

        /**
         * roboRIO-oriented defaults that avoid Athena-managed OS tweaks.
         */
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

    /**
     * ARCP runtime/layout behavior for {@link RobotCore}.
     *
     * @param enabled master runtime toggle for ARCP.
     * @param layoutProfileName default profile name used for mechanism layout publishing.
     * @param autoMechanismPages when enabled, RobotCore writes one dashboard page per mechanism.
     * @param legacyNt4MirrorEnabled when enabled, keep legacy NT4 auto publishing for streams that
     *        ARCP also publishes. Default is disabled to reduce duplicate publishing overhead.
     */
    public record ArcpConfig(
            boolean enabled,
            String layoutProfileName,
            boolean autoMechanismPages,
            boolean legacyNt4MirrorEnabled) {

        public ArcpConfig {
            layoutProfileName = normalizeArcpProfileName(layoutProfileName);
        }

        public static ArcpConfig defaults() {
            return new ArcpConfig(false, "atheana-generated", true, false);
        }
    }

    public record RobotCoreConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain,
            RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig,
            boolean autoInitResetEnabled, TelemetryRegistry.TelemetryConfig telemetryConfig,
            List<RegisterableMechanism> mechanisms, boolean performanceMode,
            boolean timingDebugEnabled, boolean telemetryEnabled, AutoConfig autoConfig, RobotCoreHooks<T> hooks,
            SystemConfig systemConfig, ArcpConfig arcpConfig) {

        public RobotCoreConfig {
            mechanisms = mechanisms != null ? List.copyOf(mechanisms) : List.of();
            autoConfig = autoConfig != null ? autoConfig : AutoConfig.defaults();
            hooks = hooks != null ? hooks : RobotCoreHooks.<T>empty();
            systemConfig = systemConfig != null ? systemConfig : SystemConfig.defaults();
            arcpConfig = arcpConfig != null ? arcpConfig : ArcpConfig.defaults();
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
                RobotCoreHooks<T> hooks,
                SystemConfig systemConfig) {
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
                    systemConfig,
                    ArcpConfig.defaults());
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
                    SystemConfig.defaults(),
                    ArcpConfig.defaults());
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
        }

        public RobotCoreConfig<T> auto(Consumer<AutoSection> section) {
            if (section == null) {
                return this;
            }
            AutoSection builder = new AutoSection(autoConfig);
            section.accept(builder);
            return auto(builder.config);
        }

        public static final class AutoSection {
            private AutoConfig config;

            private AutoSection(AutoConfig config) {
                this.config = config != null ? config : AutoConfig.defaults();
            }

            public AutoSection config(AutoConfig config) {
                this.config = config != null ? config : AutoConfig.defaults();
                return this;
            }

            public AutoSection pid(
                    double tP,
                    double tI,
                    double tD,
                    double rP,
                    double rI,
                    double rD) {
                config = config.pid(tP, tI, tD, rP, rI, rD);
                return this;
            }

            public AutoSection pid(Consumer<RobotLocalizationConfig.AutoPlannerPidSection> section) {
                config = config.pid(section);
                return this;
            }

            public AutoSection translation(Consumer<RobotLocalizationConfig.PidAxisSection> section) {
                config = config.pid(pid -> pid.translation(section));
                return this;
            }

            public AutoSection rotation(Consumer<RobotLocalizationConfig.PidAxisSection> section) {
                config = config.pid(pid -> pid.rotation(section));
                return this;
            }

            public AutoSection pose(String poseName) {
                config = config.pose(poseName);
                return this;
            }

            public AutoSection registry(Consumer<RobotAuto.RegistrySection> section) {
                config = config.registry(section);
                return this;
            }
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
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
                    systemConfig,
                    arcpConfig);
        }

        public RobotCoreConfig<T> arcp(ArcpConfig arcpConfig) {
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
                    systemConfig,
                    arcpConfig);
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
    private ArcpRuntime arcpRuntime;
    private ARCP arcp;
    private volatile boolean arcpEnabled = false;
    private volatile boolean arcpAutoMechanismPages = true;
    private volatile String arcpLayoutProfileName = "atheana-generated";
    private volatile boolean arcpLegacyNt4MirrorEnabled = false;
    private volatile boolean arcpInitialLayoutPending = false;
    private boolean arcpPublishFailureReported;
    private final Set<String> arcpPublishFailureScopes;
    private long arcpRecordingRequestSequence = 0L;
    private long arcpRecordingLastRequestTimestampMs = 0L;
    private String arcpRecordingLastRequestMode = "";
    private String arcpRecordingLastRequestPhase = "";
    private String arcpRecordingLastRequestReason = "";
    private long arcpRecordingLastClientHeartbeatMs = 0L;
    private long arcpRecordingClientAckSequence = 0L;
    private String arcpRecordingClientAckStatus = "";
    private String arcpAutoSelectedRoutineId = "";
    private String configServerBaseUrl;
    private String cachedConfigExportBaseUrl = "";
    private ConfigExportUrls cachedConfigExportUrls = ConfigExportUrls.empty();
    private final ConfigServerSection configServerSection;
    private final ArcpSection arcpSection;
    private final SystemSection systemSection;
    private final DiagnosticsSection diagnosticsSection;
    private final Map<String, DiagnosticsChannel> diagnosticsChannels;
    private final Map<String, ArcpDashboardLayout.Page> arcpMechanismPages;
    private final HashMap<String, Mechanism> mechanisms;
    private final List<SuperstructureMechanism<?, ?>> registeredSuperstructures;
    private final HashMap<String, SuperstructureMechanism<?, ?>> superstructuresByName;
    private final HashMap<Class<? extends Enum<?>>, List<StateEndpoint>> stateEndpointsByEnum;
    private final HashMap<Enum<?>, StateEndpoint> stateEndpointsByDefaultState;
    private final RobotMechanisms mechanismView;
    private final StateSection stateSection;
    private final Set<Mechanism> scheduledCustomPidMechanisms;
    private Command autonomousCommand;
    private final TelemetryRegistry telemetry;
    private boolean autoInitResetEnabled;
    private NetworkTableEntry autoInitResetEntry;
    private final StructArrayPublisher<Pose2d> selectedAutoTrajectoryPublisher;
    private NtSendablePublisher autoProgramChooserPublisher;
    private PIDController autoFollowerTranslationPidWidget;
    private PIDController autoFollowerRotationPidWidget;
    private final List<Consumer<RobotAuto.RegistrySection>> configuredAutoRegistryBindings;
    private final List<RegisterableMechanism> configuredMechanisms;
    private boolean configuredMechanismsRegistered;
    private final boolean deferNonDriveStartupInit;
    private final boolean deferMechanismRegistration;
    private final double deferredStartupInitBudgetSeconds;
    private final double startupDriveLagGuardSeconds;
    private final Deque<DeferredStartupTask> deferredStartupTasks;
    private boolean deferredStartupInitComplete;
    private boolean deferredStartupInitCompleteLogged;
    private double startupDriveLagGuardUntilSeconds = Double.NaN;
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
    private double lastArcpPublishSeconds = Double.NaN;
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
    private static final double ARCP_NATIVE_PUBLISH_PERIOD_SECONDS = 0.02;
    private static final long ARCP_RECORDING_CLIENT_PRESENCE_WINDOW_MS = 5_000L;
    private static final double COMPETITION_AUTO_PUBLISH_PERIOD_SECONDS = 0.2;
    private static final double STARTUP_LOG_THRESHOLD_SECONDS = 0.05;
    private static final String STARTUP_DEFER_NON_DRIVE_INIT_PROPERTY =
            "athena.startup.deferNonDriveInit";
    private static final String STARTUP_DEFER_MECHANISM_REGISTRATION_PROPERTY =
            "athena.startup.deferMechanismRegistration";
    private static final String STARTUP_DEFERRED_INIT_BUDGET_MS_PROPERTY =
            "athena.startup.deferredInitBudgetMs";
    private static final String STARTUP_DRIVE_LAG_GUARD_SECONDS_PROPERTY =
            "athena.startup.driveLagGuardSeconds";
    private static final double STARTUP_DEFAULT_DEFERRED_INIT_BUDGET_SECONDS = 0.002;
    private static final double STARTUP_DEFAULT_DRIVE_LAG_GUARD_SECONDS = 1.0;
    private static final String MECHANISM_FACTORY_BUILD_PARALLELISM_PROPERTY =
            "athena.mechanism.factoryBuildParallelism";
    private static final int MAX_MECHANISM_FACTORY_BUILD_THREADS = 8;
    private static final AtomicInteger MECHANISM_FACTORY_BUILD_THREAD_COUNTER = new AtomicInteger(1);
    private static final AtomicInteger SYSTEM_TWEAKS_THREAD_COUNTER = new AtomicInteger(1);
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

    private interface StateEndpoint {
        void queue(Enum<?> state);
        void force(Enum<?> state);
        boolean at(Enum<?> state);
        String ownerName();
    }

    private record DeferredStartupTask(String step, Runnable action) {
    }

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
        stateEndpointsByEnum = new HashMap<>();
        stateEndpointsByDefaultState = new HashMap<>();
        mechanismView = new RobotMechanisms(mechanisms, superstructuresByName, registeredSuperstructures);
        stateSection = new StateSection();
        configServerSection = new ConfigServerSection(this);
        arcpSection = new ArcpSection();
        systemSection = new SystemSection();
        diagnosticsChannels = new ConcurrentHashMap<>();
        arcpMechanismPages = new LinkedHashMap<>();
        diagnosticsSection = new DiagnosticsSection(this);
        diagnosticsSection.core().info("lifecycle", "robot core constructed");
        scheduledCustomPidMechanisms = new HashSet<>();
        publishedMechanismsComp = new HashSet<>();
        publishedSuperstructuresComp = new HashSet<>();
        arcpPublishFailureScopes = new HashSet<>();
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
        deferNonDriveStartupInit = parseBooleanProperty(STARTUP_DEFER_NON_DRIVE_INIT_PROPERTY, true);
        deferMechanismRegistration = parseBooleanProperty(STARTUP_DEFER_MECHANISM_REGISTRATION_PROPERTY, false);
        deferredStartupInitBudgetSeconds = parseNonNegativeDoubleProperty(
                STARTUP_DEFERRED_INIT_BUDGET_MS_PROPERTY,
                STARTUP_DEFAULT_DEFERRED_INIT_BUDGET_SECONDS * 1000.0) / 1000.0;
        startupDriveLagGuardSeconds = parseNonNegativeDoubleProperty(
                STARTUP_DRIVE_LAG_GUARD_SECONDS_PROPERTY,
                STARTUP_DEFAULT_DRIVE_LAG_GUARD_SECONDS);
        deferredStartupTasks = new ArrayDeque<>();
        deferredStartupInitComplete = true;
        deferredStartupInitCompleteLogged = false;
        timingDebugEnabled = config.timingDebugEnabled();
        telemetryEnabled = config.telemetryEnabled();
        SystemConfig systemConfig = config.systemConfig() != null ? config.systemConfig() : SystemConfig.defaults();
        ArcpConfig resolvedArcpConfig = config.arcpConfig() != null ? config.arcpConfig() : ArcpConfig.defaults();
        arcpEnabled = resolvedArcpConfig.enabled();
        arcpAutoMechanismPages = resolvedArcpConfig.autoMechanismPages();
        arcpLayoutProfileName = resolvedArcpConfig.layoutProfileName();
        arcpLegacyNt4MirrorEnabled = resolvedArcpConfig.legacyNt4MirrorEnabled();
        applyArcpNtMirrorPolicy();
        if (RobotBase.isReal()) {
            startSystemTweaksInitializationAsync(systemConfig);
            // Runtime-only toggles apply regardless of OS tweak mode.
            systemSection.configServerEnabled(systemConfig.configServerEnabled());
            systemSection.telemetryEnabled(systemConfig.telemetryEnabled());
            systemSection.networkTablesDataLogEnabled(systemConfig.networkTablesDataLogEnabled());
        }
        if (config.performanceMode()) {
            setConfigServerEnabled(false);
            setArcpEnabled(false);
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
            autos.reset().poseResetter(pose -> Commands.runOnce(() -> {
                localization.resetPose(autoPose, pose);
                syncHeadingAxesToFieldPose();
            }));
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
        // constructors run). This can be deferred via startup property when drive-first bring-up is
        // preferred over immediate mechanism availability.
        if (deferMechanismRegistration) {
            diagnosticsSection.core().info(
                    "startup",
                    "deferring mechanism registration from constructor to robotInit/disabled cycles");
        } else {
            timedStartupStep("constructor.registerConfiguredMechanisms", this::registerConfiguredMechanisms);
        }
        logStartupDuration("constructor.total", Timer.getFPGATimestamp() - constructorStart);
    }

    @Override
    public final void robotInit() {
        diagnosticsSection.core().info("lifecycle", "robotInit");
        double robotInitStart = Timer.getFPGATimestamp();
        runRobotInitStep(
                "robotInit.registerConfiguredMechanisms",
                this::registerConfiguredMechanisms,
                deferMechanismRegistration);
        runRobotInitStep("robotInit.startConfigServerIfNeeded", this::startConfigServerIfNeeded, true);
        runRobotInitStep("robotInit.startArcpIfNeeded", this::startArcpIfNeeded, true);
        timedStartupStep("robotInit.configureAutoRegistry", this::configureAutoRegistry);
        timedStartupStep("robotInit.configureAutos", () -> configureAutos(autos));
        timedStartupStep("robotInit.autos.finalizeRegistration", () -> autos.execution().prepare());
        timedStartupStep("robotInit.ensureAutoChooserPublished", this::ensureAutoChooserPublished);
        runRobotInitStep("robotInit.publishConfig", robotNetworkTables::publishConfig, true);
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE)) {
            runRobotInitStep("robotInit.publishNetworkTables", () -> {
                publishNetworkTables();
            }, true);
        }
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS)) {
            runRobotInitStep("robotInit.publishNetworkTablesMechanisms", () -> {
                publishNetworkTablesMechanisms();
            }, true);
        }
        runRobotInitStep("robotInit.runInitHooks", this::runInitHooks, deferMechanismRegistration);
        timedStartupStep("robotInit.runCorePhaseBindings",
                () -> runCorePhaseBindings(RobotCoreHooks.Phase.ROBOT_INIT, coreHooks.initBindings()));
        timedStartupStep("robotInit.onRobotInit", this::onRobotInit);
        runRobotInitStep("robotInit.registerPIDCycles", this::registerPIDCycles, deferMechanismRegistration);
        if (!deferredStartupTasks.isEmpty()) {
            deferredStartupInitComplete = false;
            diagnosticsSection.core().info(
                    "startup",
                    "deferred " + deferredStartupTasks.size()
                            + " non-drive startup steps; processing during disabled loops");
        } else {
            deferredStartupInitComplete = true;
        }
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

    private void startArcpIfNeeded() {
        if (!arcpEnabled) {
            publishAthenaRuntimeNtStatus();
            return;
        }
        if (arcpRuntime != null) {
            publishAthenaRuntimeNtStatus();
            return;
        }
        try {
            ArcpRuntime runtime = ArcpRuntime.create();
            runtime.start();
            arcpRuntime = runtime;
            arcp = new ARCP(runtime);
            lastArcpPublishSeconds = Double.NaN;
            arcpPublishFailureReported = false;
            arcpPublishFailureScopes.clear();
            arcpInitialLayoutPending = arcpAutoMechanismPages;
            diagnosticsSection.core().info(
                    "athena",
                    "runtime started (control="
                            + runtime.controlPort()
                            + ", realtime="
                            + runtime.realtimePort()
                            + ")");
            publishAthenaRuntimeNtStatus();
        } catch (RuntimeException ex) {
            diagnosticsSection.core().error("athena", "failed to start ARCP runtime: " + ex.getMessage());
            arcpRuntime = null;
            arcp = null;
            arcpInitialLayoutPending = false;
            publishAthenaRuntimeNtStatus();
        }
    }

    private void stopArcpIfRunning() {
        ArcpRuntime runtime = arcpRuntime;
        if (runtime == null) {
            return;
        }
        arcp = null;
        try {
            runtime.close();
        } catch (RuntimeException ex) {
            diagnosticsSection.core().warn("athena", "failed to stop ARCP runtime cleanly: " + ex.getMessage());
        } finally {
            arcpRuntime = null;
            lastArcpPublishSeconds = Double.NaN;
            arcpPublishFailureReported = false;
            arcpPublishFailureScopes.clear();
            arcpInitialLayoutPending = false;
            publishAthenaRuntimeNtStatus();
        }
    }

    private void setArcpEnabled(boolean enabled) {
        arcpEnabled = enabled;
        applyArcpNtMirrorPolicy();
        publishAthenaRuntimeNtStatus();
        if (!enabled) {
            stopArcpIfRunning();
            return;
        }
        startArcpIfNeeded();
    }

    private void applyArcpNtMirrorPolicy() {
        if (!arcpEnabled || arcpLegacyNt4MirrorEnabled) {
            return;
        }
        // ARCP already covers core + mechanism streams; disable mirrored NT4 auto publishing
        // by default to avoid duplicate CPU/bandwidth usage.
        robotNetworkTables.disable(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE);
        robotNetworkTables.disable(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS);
    }

    private void publishAthenaRuntimeNtStatus() {
        ArcpRuntime runtime = arcpRuntime;

        boolean runtimeRunning = runtime != null && runtime.isRunning();
        int controlPort = runtimeRunning ? runtime.controlPort() : 0;
        int realtimePort = runtimeRunning ? runtime.realtimePort() : 0;

        NetworkTable runtimeStatus = NetworkTableInstance.getDefault().getTable("Athena").getSubTable("ArcpRuntimeStatus");
        runtimeStatus.getEntry("Enabled").setBoolean(arcpEnabled);
        runtimeStatus.getEntry("Running").setBoolean(runtimeRunning);
        runtimeStatus.getEntry("ControlPort").setDouble(controlPort);
        runtimeStatus.getEntry("RealtimePort").setDouble(realtimePort);
        runtimeStatus.getEntry("Mode").setString(runtimeMode != null ? runtimeMode.name() : "UNKNOWN");
    }

    private void publishArcpSignals(double nowSeconds) {
        ArcpRuntime runtime = arcpRuntime;
        ARCP publisher = arcp;
        if (!arcpEnabled || runtime == null || !runtime.isRunning() || publisher == null) {
            return;
        }

        try {
            // Control writes/invokes must be drained every loop, even when telemetry publish is throttled.
            publisher.dispatchPendingEvents();
            if (Double.isFinite(nowSeconds) && Double.isFinite(lastArcpPublishSeconds)) {
                double elapsed = nowSeconds - lastArcpPublishSeconds;
                if (elapsed >= 0.0 && elapsed < ARCP_NATIVE_PUBLISH_PERIOD_SECONDS) {
                    return;
                }
            }

            publisher.put("Athena/Runtime/Enabled", arcpEnabled);
            publisher.put("Athena/Runtime/Running", true);
            publisher.put("Athena/Runtime/ControlPort", runtime.controlPort());
            publisher.put("Athena/Runtime/RealtimePort", runtime.realtimePort());
            publisher.put("Athena/Runtime/Mode", runtimeMode.name());
            publishArcpAutoSignals(publisher);
            publishArcpLocalizationSignals(publisher);
            publishArcpDrivetrainSignals(publisher);
            publishArcpRecordingSignals(publisher);

            for (Map.Entry<String, Mechanism> entry : mechanisms.entrySet()) {
                if (entry == null || entry.getValue() == null) {
                    continue;
                }
                String mechanismName = normalizeArcpSignalSegment(entry.getKey());
                String root = "Athena/Mechanisms/" + mechanismName;
                Mechanism mechanism = entry.getValue();
                String failureScope = "mechanism:" + mechanismName;
                try {
                    mechanism.publishArcp(publisher, root);
                    arcpPublishFailureScopes.remove(failureScope);
                } catch (RuntimeException ex) {
                    if (arcpPublishFailureScopes.add(failureScope)) {
                        diagnosticsSection.core().warn(
                                "athena",
                                "ARCP signal publish failed for "
                                        + failureScope
                                        + ": "
                                        + ex.getMessage());
                    }
                    arcpPublishFailureReported = true;
                }
            }
            for (Map.Entry<String, SuperstructureMechanism<?, ?>> entry : superstructuresByName.entrySet()) {
                if (entry == null || entry.getValue() == null) {
                    continue;
                }
                String name = normalizeArcpSignalSegment(entry.getKey());
                String root = "Athena/Mechanisms/" + name;
                String failureScope = "superstructure:" + name;
                try {
                    entry.getValue().publishArcp(publisher, root);
                    arcpPublishFailureScopes.remove(failureScope);
                } catch (RuntimeException ex) {
                    if (arcpPublishFailureScopes.add(failureScope)) {
                        diagnosticsSection.core().warn(
                                "athena",
                                "ARCP signal publish failed for "
                                        + failureScope
                                        + ": "
                                        + ex.getMessage());
                    }
                    arcpPublishFailureReported = true;
                }
            }
            if (arcpInitialLayoutPending && arcpAutoMechanismPages) {
                writeArcpMechanismLayoutProfile(arcpLayoutProfileName);
                arcpInitialLayoutPending = false;
            }
            if (Double.isFinite(nowSeconds)) {
                lastArcpPublishSeconds = nowSeconds;
            }
            if (arcpPublishFailureScopes.isEmpty()) {
                arcpPublishFailureReported = false;
            }
        } catch (RuntimeException ex) {
            if (!arcpPublishFailureReported) {
                diagnosticsSection.core().warn("athena", "ARCP signal publish failed: " + ex.getMessage());
                arcpPublishFailureReported = true;
            }
        }
    }

    private void refreshArcpMechanismLayoutIfNeeded() {
        if (!arcpEnabled || !arcpAutoMechanismPages) {
            return;
        }
        if (arcpInitialLayoutPending && !Double.isFinite(lastArcpPublishSeconds)) {
            return;
        }
        writeArcpMechanismLayoutProfile(arcpLayoutProfileName);
    }

    private void writeArcpMechanismLayoutProfile(String profileName) {
        ArcpRuntime runtime = arcpRuntime;
        if (runtime == null || !runtime.isRunning()) {
            return;
        }
        ARCP publisher = arcp;
        String resolvedProfileName = normalizeArcpProfileName(profileName);
        List<ArcpDashboardLayout.Page> pages = new ArrayList<>();
        Set<String> usedPageIds = new HashSet<>();
        appendDefaultArcpCorePages(pages, usedPageIds, publisher);
        synchronized (arcpMechanismPages) {
            for (Map.Entry<String, Mechanism> entry : mechanisms.entrySet()) {
                if (entry == null) {
                    continue;
                }
                String mechanismName = entry.getKey();
                Mechanism mechanism = entry.getValue();
                String mechanismKey = normalizeMechanismPageKey(mechanismName);
                ArcpDashboardLayout.Page page = arcpMechanismPages.get(mechanismKey);
                if (page == null && mechanism != null) {
                    page = arcpMechanismPages.get(normalizeMechanismPageKey(mechanism.getName()));
                }
                if (page == null || page.widgets().isEmpty()) {
                    page = defaultArcpPageForMechanism(mechanismName, mechanism, publisher);
                }
                if (!usedPageIds.add(page.id())) {
                    continue;
                }
                pages.add(page);
            }
        }
        if (pages.isEmpty()) {
            pages.add(defaultArcpPageForMechanism("Dashboard", null, publisher));
        }

        ArcpDashboardLayout.Builder layoutBuilder = ArcpDashboardLayout.builder()
                .activeTabId(pages.get(0).id());
        for (ArcpDashboardLayout.Page page : pages) {
            layoutBuilder.page(page);
        }

        runtime.saveLayout(resolvedProfileName, layoutBuilder.build().toJson());
    }

    private void publishArcpAutoSignals(ARCP publisher) {
        if (publisher == null) {
            return;
        }
        List<RobotAuto.AutoRoutine> routines = new ArrayList<>(autos.routines().all());
        routines.sort((left, right) -> {
            String leftLabel = left != null && left.key() != null && left.key().displayName() != null
                    ? left.key().displayName()
                    : "";
            String rightLabel = right != null && right.key() != null && right.key().displayName() != null
                    ? right.key().displayName()
                    : "";
            return leftLabel.compareToIgnoreCase(rightLabel);
        });
        publisher.put("Athena/Auto/Routines/count", routines.size());

        String[] routineIds = routines.stream()
                .map(routine -> routine != null && routine.key() != null && routine.key().id() != null
                        ? routine.key().id()
                        : "")
                .toArray(String[]::new);
        String[] routineDisplayNames = routines.stream()
                .map(routine -> routine != null && routine.key() != null && routine.key().displayName() != null
                        ? routine.key().displayName()
                        : "")
                .toArray(String[]::new);
        publisher.put("Athena/Auto/Chooser/options", routineIds);
        publisher.put("Athena/Auto/Chooser/displayNames", routineDisplayNames);
        publisher.writableString("Athena/Auto/Chooser/selectedId")
                .onSet(value -> arcpAutoSelectedRoutineId = value != null ? value.trim() : "");

        RobotSpeeds speeds = drivetrain != null ? drivetrain.robotSpeeds() : null;
        if (speeds != null) {
            publisher.put("Athena/Auto/Follower/enabled", speeds.isSpeedsSourceActive(RobotSpeeds.AUTO_SOURCE));
            ChassisSpeeds input = speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
            ChassisSpeeds output = speeds.getSpeeds(RobotSpeeds.AUTO_SOURCE);
            publisher.put("Athena/Auto/Follower/input/vxMps", input.vxMetersPerSecond);
            publisher.put("Athena/Auto/Follower/input/vyMps", input.vyMetersPerSecond);
            publisher.put("Athena/Auto/Follower/input/omegaRadPerSec", input.omegaRadiansPerSecond);
            publisher.put("Athena/Auto/Follower/output/vxMps", output.vxMetersPerSecond);
            publisher.put("Athena/Auto/Follower/output/vyMps", output.vyMetersPerSecond);
            publisher.put("Athena/Auto/Follower/output/omegaRadPerSec", output.omegaRadiansPerSecond);
        }

        RobotAuto.AutoRoutine selectedRoutine = resolveArcpSelectedAutoRoutine(routines);
        publisher.put(
                "Athena/Auto/Chooser/selectedId",
                selectedRoutine != null && selectedRoutine.key() != null && selectedRoutine.key().id() != null
                        ? selectedRoutine.key().id()
                        : "");
        if (selectedRoutine == null) {
            publisher.put("Athena/Auto/Selected/id", "");
            publisher.put("Athena/Auto/Selected/displayName", "");
            publisher.put("Athena/Auto/Selected/source", "");
            publisher.put("Athena/Auto/Selected/reference", "");
            publisher.put("Athena/Auto/Selected/hasStartingPose", false);
            publisher.put("Athena/Auto/Selected/hasTrajectory", false);
            publisher.put("Athena/Auto/Selected/trajectoryPointCount", 0.0);
            publisher.put("Athena/Auto/Selected/trajectory", "[]");
            return;
        }

        publisher.put("Athena/Auto/Selected/id", selectedRoutine.key().id());
        publisher.put("Athena/Auto/Selected/displayName", selectedRoutine.key().displayName());
        publisher.put("Athena/Auto/Selected/source", selectedRoutine.source().name());
        publisher.put("Athena/Auto/Selected/reference", selectedRoutine.reference());
        publisher.put("Athena/Auto/Selected/hasStartingPose", selectedRoutine.hasStartingPose());

        RobotAuto.AutoRoutine chooserSelected = autos.selection().selected().orElse(null);
        boolean usingChooserSelection = chooserSelected != null
                && chooserSelected.key() != null
                && selectedRoutine.key() != null
                && chooserSelected.key().id() != null
                && chooserSelected.key().id().equals(selectedRoutine.key().id());

        java.util.Optional<java.util.List<Pose2d>> selectedPoses =
                usingChooserSelection ? autos.selection().selectedPoses() : java.util.Optional.empty();
        if (selectedPoses.isEmpty()) {
            selectedPoses = autos.routines().poses(selectedRoutine);
        }
        List<Pose2d> poses = selectedPoses.orElse(List.of());
        int trajectoryPointCount = poses.size();
        publisher.put("Athena/Auto/Selected/hasTrajectory", trajectoryPointCount > 0);
        publisher.put("Athena/Auto/Selected/trajectoryPointCount", trajectoryPointCount);
        publisher.put("Athena/Auto/Selected/trajectory", toArcpFieldTrajectoryJson(poses));
    }

    private RobotAuto.AutoRoutine resolveArcpSelectedAutoRoutine(List<RobotAuto.AutoRoutine> routines) {
        if (routines == null || routines.isEmpty()) {
            arcpAutoSelectedRoutineId = "";
            return null;
        }

        String requested = arcpAutoSelectedRoutineId != null ? arcpAutoSelectedRoutineId.trim() : "";
        if (!requested.isBlank()) {
            for (RobotAuto.AutoRoutine routine : routines) {
                if (routine == null || routine.key() == null) {
                    continue;
                }
                String id = routine.key().id() != null ? routine.key().id() : "";
                String display = routine.key().displayName() != null ? routine.key().displayName() : "";
                if (id.equalsIgnoreCase(requested) || display.equalsIgnoreCase(requested)) {
                    arcpAutoSelectedRoutineId = id;
                    return routine;
                }
            }
        }

        RobotAuto.AutoRoutine selected = autos.selection().selected().orElse(null);
        if (selected != null && selected.key() != null && selected.key().id() != null) {
            arcpAutoSelectedRoutineId = selected.key().id();
            return selected;
        }
        return routines.get(0);
    }

    private static String toArcpFieldTrajectoryJson(List<Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            return "[]";
        }
        StringBuilder out = new StringBuilder(poses.size() * 28 + 2);
        out.append('[');
        for (int i = 0; i < poses.size(); i++) {
            Pose2d pose = poses.get(i);
            if (pose == null) {
                continue;
            }
            if (out.length() > 1) {
                out.append(',');
            }
            out.append("{\"x\":")
                    .append(pose.getX())
                    .append(",\"y\":")
                    .append(pose.getY())
                    .append('}');
        }
        out.append(']');
        return out.toString();
    }

    private void publishArcpLocalizationSignals(ARCP publisher) {
        if (publisher == null || localization == null) {
            return;
        }
        String updatesSuppressedPath = "Athena/Localization/Status/updatesSuppressed";
        String visionEnabledPath = "Athena/Localization/Status/visionEnabled";
        String autoPoseNamePath = "Athena/Localization/Status/autoPoseName";

        publisher.writableBoolean(updatesSuppressedPath).onSetBoolean(localization::setSuppressUpdates);
        publisher.writableBoolean(visionEnabledPath).onSetBoolean(value -> localization.enableVisionForLocalization(value));
        publisher.writableString(autoPoseNamePath).onSet(value -> {
            if (value == null) {
                return;
            }
            String trimmed = value.trim();
            if (trimmed.isBlank()) {
                return;
            }
            localization.getLocalizationConfig().poses().autoPoseName(trimmed);
        });

        Pose2d pose = localization.pose();
        publisher.put("Athena/Localization/Pose/xMeters", pose.getX());
        publisher.put("Athena/Localization/Pose/yMeters", pose.getY());
        publisher.put("Athena/Localization/Pose/headingDeg", pose.getRotation().getDegrees());

        Pose3d pose3d = localization.pose3d();
        publisher.put("Athena/Localization/Pose3d/xMeters", pose3d.getX());
        publisher.put("Athena/Localization/Pose3d/yMeters", pose3d.getY());
        publisher.put("Athena/Localization/Pose3d/zMeters", pose3d.getZ());
        publisher.put("Athena/Localization/Pose3d/rollDeg", pose3d.getRotation().getX() * 180.0 / Math.PI);
        publisher.put("Athena/Localization/Pose3d/pitchDeg", pose3d.getRotation().getY() * 180.0 / Math.PI);
        publisher.put("Athena/Localization/Pose3d/yawDeg", pose3d.getRotation().getZ() * 180.0 / Math.PI);

        ChassisSpeeds robotSpeeds = localization.getRobotRelativeSpeeds();
        publisher.put("Athena/Localization/Speeds/robot/vxMps", robotSpeeds.vxMetersPerSecond);
        publisher.put("Athena/Localization/Speeds/robot/vyMps", robotSpeeds.vyMetersPerSecond);
        publisher.put("Athena/Localization/Speeds/robot/omegaRadPerSec", robotSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds fieldSpeeds = localization.getFieldRelativeSpeeds();
        publisher.put("Athena/Localization/Speeds/field/vxMps", fieldSpeeds.vxMetersPerSecond);
        publisher.put("Athena/Localization/Speeds/field/vyMps", fieldSpeeds.vyMetersPerSecond);
        publisher.put("Athena/Localization/Speeds/field/omegaRadPerSec", fieldSpeeds.omegaRadiansPerSecond);

        publisher.put("Athena/Localization/FieldPose/xMeters", pose3d.getX());
        publisher.put("Athena/Localization/FieldPose/zMeters", pose3d.getZ());
        publisher.put("Athena/Localization/FieldPose/thetaDeg", pose.getRotation().getDegrees());

        publisher.put("Athena/Localization/Status/normalizedSpeed", localization.getNormalizedMovementSpeed());
        publisher.put("Athena/Localization/Status/updatesSuppressed", localization.isSuppressUpdates());
        RobotLocalizationConfig localizationConfig = localization.getLocalizationConfig();
        String poseName = localizationConfig != null ? localizationConfig.autoPoseName() : "";
        boolean visionEnabled = localizationConfig != null && localizationConfig.isVisionEnabled();
        String[] poseOptions = localizationConfig != null
                ? localizationConfig.poseConfigs().stream()
                        .filter(cfg -> cfg != null && cfg.name() != null && !cfg.name().isBlank())
                        .map(PoseConfig::name)
                        .toArray(String[]::new)
                : new String[0];
        publisher.put("Athena/Localization/Status/autoPoseOptions", poseOptions);
        publisher.put(autoPoseNamePath, poseName != null ? poseName : "");
        publisher.put("Athena/Localization/Status/visionEnabled", visionEnabled);
    }

    private void publishArcpDrivetrainSignals(ARCP publisher) {
        if (publisher == null || drivetrain == null) {
            return;
        }
        publisher.put("Athena/Drivetrain/Limits/maxVelocityMps", drivetrain.speeds().maxVelocity());
        publisher.put(
                "Athena/Drivetrain/Limits/maxAngularVelocityRadPerSec",
                drivetrain.speeds().maxAngularVelocity());

        RobotSpeeds robotSpeeds = drivetrain.robotSpeeds();
        ChassisSpeeds blendedOutput = robotSpeeds.calculate();
        publisher.put("Athena/Drivetrain/Output/vxMps", blendedOutput.vxMetersPerSecond);
        publisher.put("Athena/Drivetrain/Output/vyMps", blendedOutput.vyMetersPerSecond);
        publisher.put("Athena/Drivetrain/Output/omegaRadPerSec", blendedOutput.omegaRadiansPerSecond);

        Set<String> speedSources = new HashSet<>(drivetrain.speeds().sources());
        speedSources.add(RobotSpeeds.DRIVE_SOURCE);
        speedSources.add(RobotSpeeds.AUTO_SOURCE);
        speedSources.add(RobotSpeeds.FEEDBACK_SOURCE);
        String[] publishedSourceNames = speedSources.stream()
                .filter(name -> name != null && !name.isBlank())
                .sorted()
                .toArray(String[]::new);
        publisher.put("Athena/Drivetrain/Sources/list", publishedSourceNames);

        for (String source : publishedSourceNames) {
            String sourceKey = normalizeArcpSignalSegment(source);
            String sourceRoot = "Athena/Drivetrain/Sources/" + sourceKey;
            publisher.writableBoolean(sourceRoot + "/enabled")
                    .onSetBoolean(value -> drivetrain.speeds().enabled(source, value));
            publisher.put(sourceRoot + "/name", source);
            publisher.put(sourceRoot + "/enabled", robotSpeeds.isSpeedsSourceActive(source));

            ChassisSpeeds input = robotSpeeds.getInputSpeeds(source);
            publisher.put(sourceRoot + "/input/vxMps", input.vxMetersPerSecond);
            publisher.put(sourceRoot + "/input/vyMps", input.vyMetersPerSecond);
            publisher.put(sourceRoot + "/input/omegaRadPerSec", input.omegaRadiansPerSecond);

            ChassisSpeeds output = robotSpeeds.getSpeeds(source);
            publisher.put(sourceRoot + "/output/vxMps", output.vxMetersPerSecond);
            publisher.put(sourceRoot + "/output/vyMps", output.vyMetersPerSecond);
            publisher.put(sourceRoot + "/output/omegaRadPerSec", output.omegaRadiansPerSecond);
            publisher.put(sourceRoot + "/output/magnitudeMps", Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond));
        }

        Imu imuDevice = drivetrain.imu().device();
        if (imuDevice != null) {
            imuDevice.publishArcp(publisher, "Athena/Drivetrain/Imu");
        }

        if (drivetrain instanceof SwerveDrivetrain swerve) {
            publishArcpSwerveDrivetrainSignals(publisher, swerve);
            return;
        }
        if (drivetrain instanceof DifferentialDrivetrain differential) {
            publishArcpDifferentialDrivetrainSignals(publisher, differential);
        }
    }

    private void publishArcpSwerveDrivetrainSignals(ARCP publisher, SwerveDrivetrain drivetrain) {
        if (publisher == null || drivetrain == null) {
            return;
        }
        SwerveDrivetrain.ModulesRuntimeSection modulesSection = (SwerveDrivetrain.ModulesRuntimeSection) drivetrain.modules();
        var modules = modulesSection.all();
        if (modules == null || modules.length == 0) {
            return;
        }

        String[] labels = {"FL", "FR", "BL", "BR"};
        String[] keys = new String[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            if (module == null) {
                continue;
            }
            String label = i < labels.length ? labels[i] : ("M" + (i + 1));
            keys[i] = label;
            String moduleRoot = "Athena/Drivetrain/Swerve/Modules/" + label;
            var state = module.getState();
            double angleDeg = state.angle != null ? state.angle.getDegrees() : 0.0;
            double speedMps = state.speedMetersPerSecond;
            publisher.put(moduleRoot + "/name", label);
            publisher.put(moduleRoot + "/angleDeg", angleDeg);
            publisher.put(moduleRoot + "/speedMps", speedMps);
            publisher.put(moduleRoot + "/driveCommandPercent", module.getDriveCommandPercent());
            publisher.put(moduleRoot + "/steerCommandPercent", module.getSteerCommandPercent());

            publisher.writableDouble(moduleRoot + "/command/angleDeg")
                    .onSetDouble(value -> module.setSteerAngle(Rotation2d.fromDegrees(value)));
            publisher.writableDouble(moduleRoot + "/command/speedMps")
                    .onSetDouble(value -> module.setDesiredState(
                            new edu.wpi.first.math.kinematics.SwerveModuleState(
                                    value,
                                    Rotation2d.fromDegrees(module.getState().angle.getDegrees()))));
            publisher.writableBoolean(moduleRoot + "/command/inverted")
                    .onSetBoolean(value -> module.getDriveMotorController().setInverted(value));
            publisher.writableDouble(moduleRoot + "/command/offset")
                    .onSetDouble(module::setOffset);

            publisher.put(moduleRoot + "/command/inverted", module.getDriveMotorController().isInverted());
            if (module.getSteerEncoder() != null) {
                publisher.put(moduleRoot + "/command/offset", module.getSteerEncoder().getOffset());
                module.getSteerEncoder().publishArcp(publisher, moduleRoot + "/Encoder");
            }
            if (module.getDriveMotorController() != null) {
                module.getDriveMotorController().publishArcp(publisher, moduleRoot + "/Motors/Drive");
            }
            if (module.getSteerMotorController() != null) {
                module.getSteerMotorController().publishArcp(publisher, moduleRoot + "/Motors/Steer");
            }
        }
        publisher.put("Athena/Drivetrain/Swerve/modules", keys);
    }

    private void publishArcpDifferentialDrivetrainSignals(ARCP publisher, DifferentialDrivetrain drivetrain) {
        if (publisher == null || drivetrain == null) {
            return;
        }
        DifferentialDrivetrain.ModulesRuntimeSection modulesSection =
                (DifferentialDrivetrain.ModulesRuntimeSection) drivetrain.modules();
        MotorControllerGroup left = modulesSection.leftMotors();
        MotorControllerGroup right = modulesSection.rightMotors();
        if (left == null || right == null) {
            return;
        }
        String root = "Athena/Drivetrain/Differential";
        left.publishArcp(publisher, root + "/Left");
        right.publishArcp(publisher, root + "/Right");
        double leftSpeed = left.getEncoderGroup() != null ? left.getEncoderGroup().getVelocity() : 0.0;
        double rightSpeed = right.getEncoderGroup() != null ? right.getEncoderGroup().getVelocity() : 0.0;
        publisher.put(root + "/leftSpeedMps", leftSpeed);
        publisher.put(root + "/rightSpeedMps", rightSpeed);
        Imu imu = drivetrain.imu().device();
        if (imu != null && imu.getYaw() != null) {
            publisher.put(root + "/headingDeg", imu.getYaw().getDegrees());
        }
        publisher.writableDouble(root + "/command/leftPercent").onSetDouble(left::setSpeed);
        publisher.writableDouble(root + "/command/rightPercent").onSetDouble(right::setSpeed);
        publisher.writableDouble(root + "/command/leftVoltage").onSetDouble(left::setVoltage);
        publisher.writableDouble(root + "/command/rightVoltage").onSetDouble(right::setVoltage);
        publisher.command(root + "/command/stop").onInvoke(() -> {
            left.stopMotors();
            right.stopMotors();
        });
    }

    private void publishArcpRecordingSignals(ARCP publisher) {
        if (publisher == null) {
            return;
        }
        publisher.writableI64(
                "Athena/Recording/Client/heartbeatMs").onSetI64(value -> arcpRecordingLastClientHeartbeatMs = value);
        publisher.writableI64(
                "Athena/Recording/Client/ackSequence").onSetI64(value -> arcpRecordingClientAckSequence = value);
        publisher.writableString(
                "Athena/Recording/Client/ackStatus").onSet(value -> arcpRecordingClientAckStatus = value != null ? value : "");

        long nowMs = System.currentTimeMillis();
        long heartbeatAgeMs = arcpRecordingLastClientHeartbeatMs > 0L
                ? Math.max(0L, nowMs - arcpRecordingLastClientHeartbeatMs)
                : -1L;
        boolean clientPresent = heartbeatAgeMs >= 0L && heartbeatAgeMs <= ARCP_RECORDING_CLIENT_PRESENCE_WINDOW_MS;

        publisher.put("Athena/Recording/Client/present", clientPresent);
        publisher.put("Athena/Recording/Client/heartbeatAgeMs", heartbeatAgeMs);
        publisher.put("Athena/Recording/Client/lastAckSequence", arcpRecordingClientAckSequence);
        publisher.put("Athena/Recording/Client/lastAckStatus", arcpRecordingClientAckStatus);

        publisher.put("Athena/Recording/Request/sequence", arcpRecordingRequestSequence);
        publisher.put("Athena/Recording/Request/timestampMs", arcpRecordingLastRequestTimestampMs);
        publisher.put("Athena/Recording/Request/mode", arcpRecordingLastRequestMode);
        publisher.put("Athena/Recording/Request/phase", arcpRecordingLastRequestPhase);
        publisher.put("Athena/Recording/Request/reason", arcpRecordingLastRequestReason);
    }

    private void triggerArcpRecordingRequest(String phase) {
        String normalizedPhase = phase != null ? phase.trim().toLowerCase() : "";
        if (normalizedPhase.isEmpty()) {
            normalizedPhase = "enabled";
        }
        arcpRecordingRequestSequence += 1L;
        arcpRecordingLastRequestTimestampMs = System.currentTimeMillis();
        arcpRecordingLastRequestMode = DriverStation.isFMSAttached() ? "match" : "practice";
        arcpRecordingLastRequestPhase = normalizedPhase;
        arcpRecordingLastRequestReason = arcpRecordingLastRequestMode + "_start";
    }

    private void appendDefaultArcpCorePages(
            List<ArcpDashboardLayout.Page> pages,
            Set<String> usedPageIds,
            ARCP publisher) {
        appendPageIfAbsent(pages, usedPageIds, defaultArcpPageForAuto(publisher));
        appendPageIfAbsent(pages, usedPageIds, defaultArcpPageForLocalization(publisher));
        appendPageIfAbsent(pages, usedPageIds, defaultArcpPageForDrivetrain(publisher));
    }

    private static void appendPageIfAbsent(
            List<ArcpDashboardLayout.Page> pages,
            Set<String> usedPageIds,
            ArcpDashboardLayout.Page page) {
        if (page == null || pages == null || usedPageIds == null) {
            return;
        }
        if (usedPageIds.add(page.id())) {
            pages.add(page);
        }
    }

    private static int firstExistingSignalId(ARCP publisher, String... paths) {
        if (publisher == null || paths == null || paths.length == 0) {
            return 0;
        }
        for (String path : paths) {
            if (path == null || path.isBlank()) {
                continue;
            }
            int id = publisher.existingSignalId(path);
            if (id > 0) {
                return id;
            }
        }
        return 0;
    }

    private static Map<String, Object> dropdownOption(String label, String value) {
        Map<String, Object> option = new LinkedHashMap<>();
        option.put("label", label != null ? label : "");
        option.put("value", value != null ? value : "");
        return option;
    }

    private List<RobotAuto.AutoRoutine> sortedAutoRoutines() {
        List<RobotAuto.AutoRoutine> routines = new ArrayList<>(autos.routines().all());
        routines.sort((left, right) -> {
            String leftLabel = left != null && left.key() != null && left.key().displayName() != null
                    ? left.key().displayName()
                    : "";
            String rightLabel = right != null && right.key() != null && right.key().displayName() != null
                    ? right.key().displayName()
                    : "";
            return leftLabel.compareToIgnoreCase(rightLabel);
        });
        return routines;
    }

    private List<String> sortedLocalizationPoseNames() {
        if (localization == null) {
            return List.of();
        }
        RobotLocalizationConfig cfg = localization.getLocalizationConfig();
        if (cfg == null || cfg.poseConfigs() == null || cfg.poseConfigs().isEmpty()) {
            return List.of();
        }
        List<String> names = new ArrayList<>();
        for (PoseConfig poseConfig : cfg.poseConfigs()) {
            if (poseConfig == null || poseConfig.name() == null) {
                continue;
            }
            String trimmed = poseConfig.name().trim();
            if (!trimmed.isBlank()) {
                names.add(trimmed);
            }
        }
        names.sort(String::compareToIgnoreCase);
        return names;
    }

    private List<String> sortedDrivetrainSourceNames() {
        Set<String> sourceNames = new HashSet<>();
        sourceNames.add(RobotSpeeds.DRIVE_SOURCE);
        sourceNames.add(RobotSpeeds.AUTO_SOURCE);
        sourceNames.add(RobotSpeeds.FEEDBACK_SOURCE);
        if (drivetrain != null) {
            sourceNames.addAll(drivetrain.speeds().sources());
        }
        List<String> sorted = new ArrayList<>();
        for (String source : sourceNames) {
            if (source == null || source.isBlank()) {
                continue;
            }
            sorted.add(source.trim());
        }
        sorted.sort(String::compareToIgnoreCase);
        return sorted;
    }

    private ArcpDashboardLayout.Page defaultArcpPageForAuto(ARCP publisher) {
        ArcpDashboardLayout.Page.Builder page = ArcpDashboardLayout.Page.builder()
                .id("tab-auto")
                .name("Auto");
        if (publisher == null) {
            return page.build();
        }

        String chooserPath = "Athena/Auto/Chooser/selectedId";
        String selectedNamePath = "Athena/Auto/Selected/displayName";
        int chooserSignalId = publisher.existingSignalId(chooserPath);

        List<Map<String, Object>> chooserOptions = new ArrayList<>();
        for (RobotAuto.AutoRoutine routine : sortedAutoRoutines()) {
            if (routine == null || routine.key() == null) {
                continue;
            }
            String id = routine.key().id() != null ? routine.key().id() : "";
            if (id.isBlank()) {
                continue;
            }
            String display = routine.key().displayName() != null && !routine.key().displayName().isBlank()
                    ? routine.key().displayName()
                    : id;
            chooserOptions.add(dropdownOption(display, id));
        }

        int trajectorySignalId = publisher.existingSignalId("Athena/Auto/Selected/trajectory");
        int fieldXSignalId = publisher.existingSignalId("Athena/Localization/Pose/xMeters");
        int fieldYSignalId = publisher.existingSignalId("Athena/Localization/Pose/yMeters");
        int fieldHeadingSignalId = publisher.existingSignalId("Athena/Localization/Pose/headingDeg");

        int outputVxId = publisher.existingSignalId("Athena/Auto/Follower/output/vxMps");
        int outputVyId = publisher.existingSignalId("Athena/Auto/Follower/output/vyMps");
        int outputOmegaId = publisher.existingSignalId("Athena/Auto/Follower/output/omegaRadPerSec");
        int inputVxId = publisher.existingSignalId("Athena/Auto/Follower/input/vxMps");
        int inputVyId = publisher.existingSignalId("Athena/Auto/Follower/input/vyMps");
        int inputOmegaId = publisher.existingSignalId("Athena/Auto/Follower/input/omegaRadPerSec");

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-auto-chooser")
                .kind("dropdown")
                .title("Auto Chooser")
                .signalId(firstExistingSignalId(publisher, chooserPath, selectedNamePath))
                .layout(0, 0, 10, 1)
                .config("signalPath", chooserPath)
                .config("options", chooserOptions)
                .config("commit", "button")
                .config("buttonLabel", "Set")
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-auto-field-preview")
                .kind("field")
                .title("Auto Field Preview")
                .signalId(firstExistingSignalId(publisher, "Athena/Auto/Selected/trajectory", chooserPath))
                .layout(0, 1, 14, 8)
                .config("trajectorySignalId", trajectorySignalId)
                .config("xSignalId", fieldXSignalId)
                .config("ySignalId", fieldYSignalId)
                .config("headingSignalId", fieldHeadingSignalId)
                .config("allowPoseSet", false)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-auto-follower-output")
                .kind("graph")
                .title("Follower Output")
                .signalId(firstExistingSignalId(publisher, "Athena/Auto/Follower/output/vxMps", chooserPath))
                .layout(14, 1, 10, 4)
                .config("series", List.of(
                        graphSeries(outputVxId, "vx", "#f87171", "line", "metric"),
                        graphSeries(outputVyId, "vy", "#22d3ee", "line", "metric"),
                        graphSeries(outputOmegaId, "omega", "#a78bfa", "line", "metric")))
                .config("yMin", -10.0)
                .config("yMax", 10.0)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-auto-follower-input")
                .kind("graph")
                .title("Follower Input")
                .signalId(firstExistingSignalId(publisher, "Athena/Auto/Follower/input/vxMps", chooserPath))
                .layout(14, 5, 10, 4)
                .config("series", List.of(
                        graphSeries(inputVxId, "vx", "#fb7185", "line", "metric"),
                        graphSeries(inputVyId, "vy", "#67e8f9", "line", "metric"),
                        graphSeries(inputOmegaId, "omega", "#c4b5fd", "line", "metric")))
                .config("yMin", -10.0)
                .config("yMax", 10.0)
                .build());
        return page.build();
    }

    private ArcpDashboardLayout.Page defaultArcpPageForLocalization(ARCP publisher) {
        ArcpDashboardLayout.Page.Builder page = ArcpDashboardLayout.Page.builder()
                .id("tab-localization")
                .name("Localization");
        if (publisher == null) {
            return page.build();
        }

        String headingPath = "Athena/Localization/Pose/headingDeg";
        String normalizedSpeedPath = "Athena/Localization/Status/normalizedSpeed";
        String visionEnabledPath = "Athena/Localization/Status/visionEnabled";
        String updatesSuppressedPath = "Athena/Localization/Status/updatesSuppressed";
        String autoPoseNamePath = "Athena/Localization/Status/autoPoseName";

        int headingId = publisher.existingSignalId(headingPath);
        int normalizedSpeedId = publisher.existingSignalId(normalizedSpeedPath);
        int visionEnabledId = publisher.existingSignalId(visionEnabledPath);
        int updatesSuppressedId = publisher.existingSignalId(updatesSuppressedPath);
        int autoPoseNameId = publisher.existingSignalId(autoPoseNamePath);

        int robotVxId = publisher.existingSignalId("Athena/Localization/Speeds/robot/vxMps");
        int robotVyId = publisher.existingSignalId("Athena/Localization/Speeds/robot/vyMps");
        int robotOmegaId = publisher.existingSignalId("Athena/Localization/Speeds/robot/omegaRadPerSec");
        int fieldVxId = publisher.existingSignalId("Athena/Localization/Speeds/field/vxMps");
        int fieldVyId = publisher.existingSignalId("Athena/Localization/Speeds/field/vyMps");
        int fieldOmegaId = publisher.existingSignalId("Athena/Localization/Speeds/field/omegaRadPerSec");

        int fieldXId = publisher.existingSignalId("Athena/Localization/FieldPose/xMeters");
        int fieldZId = publisher.existingSignalId("Athena/Localization/FieldPose/zMeters");
        int fieldThetaId = publisher.existingSignalId("Athena/Localization/FieldPose/thetaDeg");

        List<Map<String, Object>> autoPoseOptions = new ArrayList<>();
        for (String poseName : sortedLocalizationPoseNames()) {
            autoPoseOptions.add(dropdownOption(poseName, poseName));
        }

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-heading-compass")
                .kind("compass")
                .title("Heading")
                .signalId(firstExistingSignalId(publisher, headingPath, normalizedSpeedPath))
                .layout(0, 0, 5, 4)
                .config("signalPath", headingPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-robot-speeds")
                .kind("graph")
                .title("Robot Relative Speeds")
                .signalId(firstExistingSignalId(publisher, "Athena/Localization/Speeds/robot/vxMps", headingPath))
                .layout(5, 0, 9, 4)
                .config("series", List.of(
                        graphSeries(robotVxId, "vx", "#f87171", "line", "metric"),
                        graphSeries(robotVyId, "vy", "#22d3ee", "line", "metric"),
                        graphSeries(robotOmegaId, "omega", "#a78bfa", "line", "metric")))
                .config("yMin", -10.0)
                .config("yMax", 10.0)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-field-speeds")
                .kind("graph")
                .title("Field Relative Speeds")
                .signalId(firstExistingSignalId(publisher, "Athena/Localization/Speeds/field/vxMps", headingPath))
                .layout(14, 0, 10, 4)
                .config("series", List.of(
                        graphSeries(fieldVxId, "vx", "#fb7185", "line", "metric"),
                        graphSeries(fieldVyId, "vy", "#67e8f9", "line", "metric"),
                        graphSeries(fieldOmegaId, "omega", "#c4b5fd", "line", "metric")))
                .config("yMin", -10.0)
                .config("yMax", 10.0)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-vision")
                .kind("toggle")
                .title("Vision")
                .signalId(visionEnabledId)
                .layout(0, 4, 4, 1)
                .config("signalPath", visionEnabledPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-updates")
                .kind("toggle")
                .title("Suppress Updates")
                .signalId(updatesSuppressedId)
                .layout(4, 4, 5, 1)
                .config("signalPath", updatesSuppressedPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-auto-pose")
                .kind("dropdown")
                .title("Auto Pose Name")
                .signalId(autoPoseNameId)
                .layout(9, 4, 11, 1)
                .config("signalPath", autoPoseNamePath)
                .config("options", autoPoseOptions)
                .config("commit", "button")
                .config("buttonLabel", "Set")
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-speed")
                .kind("metric")
                .title("Norm Speed")
                .signalId(normalizedSpeedId)
                .layout(20, 4, 4, 1)
                .config("signalPath", normalizedSpeedPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-localization-field-view")
                .kind("field")
                .title("Field Pose (X/Z/Theta)")
                .signalId(firstExistingSignalId(publisher, "Athena/Localization/FieldPose/xMeters", headingPath))
                .layout(0, 5, 24, 8)
                .config("xSignalId", fieldXId)
                .config("ySignalId", fieldZId)
                .config("headingSignalId", fieldThetaId)
                .config("allowPoseSet", false)
                .config("fieldLength", 16.54)
                .config("fieldWidth", 4.0)
                .build());
        return page.build();
    }

    private ArcpDashboardLayout.Page defaultArcpPageForDrivetrain(ARCP publisher) {
        ArcpDashboardLayout.Page.Builder page = ArcpDashboardLayout.Page.builder()
                .id("tab-drivetrain")
                .name("Drivetrain");
        if (publisher == null) {
            return page.build();
        }

        String maxVelocityPath = "Athena/Drivetrain/Limits/maxVelocityMps";
        String maxOmegaPath = "Athena/Drivetrain/Limits/maxAngularVelocityRadPerSec";
        String outputVxPath = "Athena/Drivetrain/Output/vxMps";
        String outputVyPath = "Athena/Drivetrain/Output/vyMps";
        String outputOmegaPath = "Athena/Drivetrain/Output/omegaRadPerSec";

        int maxVelocityId = publisher.existingSignalId(maxVelocityPath);
        int maxOmegaId = publisher.existingSignalId(maxOmegaPath);
        int outputVxId = publisher.existingSignalId(outputVxPath);
        int outputVyId = publisher.existingSignalId(outputVyPath);
        int outputOmegaId = publisher.existingSignalId(outputOmegaPath);
        int imuYawId = publisher.existingSignalId("Athena/Drivetrain/Imu/yawDeg");
        int imuPitchId = publisher.existingSignalId("Athena/Drivetrain/Imu/pitchDeg");
        int imuRollId = publisher.existingSignalId("Athena/Drivetrain/Imu/rollDeg");

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-drivetrain-max-velocity")
                .kind("metric")
                .title("Max Velocity (m/s)")
                .signalId(maxVelocityId)
                .layout(0, 0, 3, 1)
                .config("signalPath", maxVelocityPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-drivetrain-max-omega")
                .kind("metric")
                .title("Max Omega (rad/s)")
                .signalId(maxOmegaId)
                .layout(3, 0, 3, 1)
                .config("signalPath", maxOmegaPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-drivetrain-blended-output")
                .kind("graph")
                .title("Blended Output")
                .signalId(firstExistingSignalId(publisher, outputVxPath, "Athena/Drivetrain/Output/vyMps"))
                .layout(0, 1, 10, 4)
                .config("series", List.of(
                        graphSeries(outputVxId, "vx", "#f87171", "line", "metric"),
                        graphSeries(outputVyId, "vy", "#22d3ee", "line", "metric"),
                        graphSeries(outputOmegaId, "omega", "#a78bfa", "line", "metric")))
                .config("yMin", -10.0)
                .config("yMax", 10.0)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-drivetrain-imu-3d")
                .kind("imu_3d")
                .title("IMU 3D")
                .signalId(firstExistingSignalId(publisher, "Athena/Drivetrain/Imu/yawDeg", outputVxPath))
                .layout(10, 0, 6, 5)
                .config("yawSignalId", imuYawId)
                .config("pitchSignalId", imuPitchId)
                .config("rollSignalId", imuRollId)
                .build());

        List<String> sourceNames = sortedDrivetrainSourceNames();
        List<Map<String, Object>> sourceMatrixItems = new ArrayList<>();
        for (String sourceName : sourceNames) {
            String key = normalizeArcpSignalSegment(sourceName);
            int enabledId = publisher.existingSignalId("Athena/Drivetrain/Sources/" + key + "/enabled");
            int magnitudeId = publisher.existingSignalId("Athena/Drivetrain/Sources/" + key + "/output/magnitudeMps");
            if (enabledId <= 0) {
                continue;
            }
            Map<String, Object> item = new LinkedHashMap<>();
            item.put("signalId", enabledId);
            item.put("label", sourceName);
            item.put("group", "Sources");
            item.put("healthyWhen", "true");
            item.put("toggleSignalId", enabledId);
            item.put("valueSignalId", magnitudeId);
            sourceMatrixItems.add(item);
        }

        int sourceMatrixSignalId = firstExistingSignalId(
                publisher,
                "Athena/Drivetrain/Sources/" + normalizeArcpSignalSegment(RobotSpeeds.DRIVE_SOURCE) + "/enabled",
                "Athena/Drivetrain/Sources/" + normalizeArcpSignalSegment(RobotSpeeds.AUTO_SOURCE) + "/enabled",
                "Athena/Drivetrain/Sources/" + normalizeArcpSignalSegment(RobotSpeeds.FEEDBACK_SOURCE) + "/enabled");
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-drivetrain-source-matrix")
                .kind("status_matrix")
                .title("Speed Source Status")
                .signalId(sourceMatrixSignalId)
                .layout(16, 0, 8, 5)
                .config("items", sourceMatrixItems)
                .config("columns", Math.max(1, Math.min(4, sourceMatrixItems.size())))
                .config("showSummary", true)
                .config("mode", "toggle_value")
                .build());

        String sourceListId = "w-drivetrain-source-list";
        int sourceListRows = Math.max(4, sourceNames.size() + 2);
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(sourceListId)
                .kind("layout_list")
                .title("Speed Sources")
                .layout(0, 5, 10, sourceListRows)
                .build());
        int sourceRow = 0;
        for (String sourceName : sourceNames) {
            String key = normalizeArcpSignalSegment(sourceName);
            int vxId = publisher.existingSignalId("Athena/Drivetrain/Sources/" + key + "/output/vxMps");
            int vyId = publisher.existingSignalId("Athena/Drivetrain/Sources/" + key + "/output/vyMps");
            int omegaId = publisher.existingSignalId("Athena/Drivetrain/Sources/" + key + "/output/omegaRadPerSec");
            if (vxId <= 0 && vyId <= 0 && omegaId <= 0) {
                continue;
            }
            String accordionId = "w-drivetrain-source-accordion-" + key;
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id(accordionId)
                    .kind("layout_accordion")
                    .title(sourceName)
                    .signalId(firstExistingSignalId(
                            publisher,
                            "Athena/Drivetrain/Sources/" + key + "/enabled",
                            "Athena/Drivetrain/Sources/" + key + "/output/vxMps"))
                    .layout(0, sourceRow, 12, 1)
                    .parentLayoutId(sourceListId)
                    .config("collapsed", true)
                    .config("expandedRows", 3)
                    .build());
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-drivetrain-source-graph-" + key)
                    .kind("graph")
                    .title(sourceName + " Output")
                    .signalId(firstExistingSignalId(
                            publisher,
                            "Athena/Drivetrain/Sources/" + key + "/output/vxMps",
                            "Athena/Drivetrain/Sources/" + key + "/output/vyMps",
                            "Athena/Drivetrain/Sources/" + key + "/output/omegaRadPerSec"))
                    .layout(0, 0, 12, 3)
                    .parentLayoutId(accordionId)
                    .config("series", List.of(
                            graphSeries(vxId, "vx", "#fb7185", "line", "metric"),
                            graphSeries(vyId, "vy", "#67e8f9", "line", "metric"),
                            graphSeries(omegaId, "omega", "#c4b5fd", "line", "metric")))
                    .config("yMin", -10.0)
                    .config("yMax", 10.0)
                    .build());
            sourceRow += 1;
        }

        if (drivetrain instanceof SwerveDrivetrain) {
            List<Map<String, Object>> swerveBindings = new ArrayList<>();
            String[] moduleKeys = {"fl", "fr", "bl", "br"};
            String[] moduleLabels = {"FL", "FR", "BL", "BR"};
            for (int i = 0; i < moduleKeys.length; i++) {
                String moduleRoot = "Athena/Drivetrain/Swerve/Modules/" + moduleLabels[i];
                int angleId = publisher.existingSignalId(moduleRoot + "/angleDeg");
                int speedId = publisher.existingSignalId(moduleRoot + "/speedMps");
                Map<String, Object> binding = new LinkedHashMap<>();
                binding.put("key", moduleKeys[i]);
                binding.put("label", moduleLabels[i]);
                binding.put("angleSignalId", angleId);
                binding.put("speedSignalId", speedId);
                swerveBindings.add(binding);

                int moduleSignalId = firstExistingSignalId(
                        publisher,
                        moduleRoot + "/angleDeg",
                        moduleRoot + "/speedMps");
                int moduleX = 10 + (i % 2) * 7;
                int moduleY = 5 + (i / 2) * 3;
                page.widget(ArcpDashboardLayout.Widget.builder()
                        .id("w-drivetrain-swerve-module-" + moduleKeys[i])
                        .kind("swerve_module")
                        .title("Module " + moduleLabels[i])
                        .signalId(moduleSignalId)
                        .layout(moduleX, moduleY, 7, 3)
                        .config("label", moduleLabels[i])
                        .config("angleSignalId", angleId)
                        .config("speedSignalId", speedId)
                        .config("commandAngleSignalId", publisher.existingSignalId(moduleRoot + "/command/angleDeg"))
                        .config("commandSpeedSignalId", publisher.existingSignalId(moduleRoot + "/command/speedMps"))
                        .config("commandInvertedSignalId", publisher.existingSignalId(moduleRoot + "/command/inverted"))
                        .config("commandOffsetSignalId", publisher.existingSignalId(moduleRoot + "/command/offset"))
                        .build());
            }

            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-drivetrain-swerve-drive")
                    .kind("swerve_drive")
                    .title("Swerve Drive")
                    .signalId(firstExistingSignalId(
                            publisher,
                            "Athena/Drivetrain/Swerve/Modules/FL/angleDeg",
                            "Athena/Drivetrain/Swerve/Modules/FR/angleDeg"))
                    .layout(10, 11, 14, 6)
                    .config("headingSignalId", imuYawId)
                    .config("maxSpeed", drivetrain.speeds().maxVelocity())
                    .config("modules", swerveBindings)
                    .build());

            int moduleHardwareStartY = 17;
            for (int i = 0; i < moduleLabels.length; i++) {
                String moduleLabel = moduleLabels[i];
                String moduleRoot = "Athena/Drivetrain/Swerve/Modules/" + moduleLabel;
                int col = (i % 2) * 12;
                int row = moduleHardwareStartY + (i / 2) * 8;
                page.widget(ArcpDeviceWidgets.motor(moduleRoot + "/Motors/Drive", widget -> widget
                        .id("w-drivetrain-module-drive-" + moduleLabel.toLowerCase())
                        .title(moduleLabel + " Drive Motor")
                        .layout(col, row, 6, 5)
                        .showOtherFields(true)));
                page.widget(ArcpDeviceWidgets.encoder(moduleRoot + "/Encoder", widget -> widget
                        .id("w-drivetrain-module-encoder-" + moduleLabel.toLowerCase())
                        .title(moduleLabel + " Encoder")
                        .layout(col + 6, row, 6, 7)));
            }
        } else if (drivetrain instanceof DifferentialDrivetrain) {
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-drivetrain-differential-drive")
                    .kind("differential_drive")
                    .title("Differential Drive")
                    .signalId(firstExistingSignalId(
                            publisher,
                            "Athena/Drivetrain/Differential/leftSpeedMps",
                            "Athena/Drivetrain/Differential/rightSpeedMps"))
                    .layout(10, 5, 14, 5)
                    .config("leftSpeedSignalId", publisher.existingSignalId("Athena/Drivetrain/Differential/leftSpeedMps"))
                    .config("rightSpeedSignalId", publisher.existingSignalId("Athena/Drivetrain/Differential/rightSpeedMps"))
                    .config("headingSignalId", publisher.existingSignalId("Athena/Drivetrain/Differential/headingDeg"))
                    .config("maxSpeed", drivetrain.speeds().maxVelocity())
                    .build());
            page.widget(ArcpDeviceWidgets.motor("Athena/Drivetrain/Differential/Left", widget -> widget
                    .id("w-drivetrain-diff-left")
                    .title("Left Motors")
                    .layout(10, 10, 7, 5)
                    .showOtherFields(true)));
            page.widget(ArcpDeviceWidgets.motor("Athena/Drivetrain/Differential/Right", widget -> widget
                    .id("w-drivetrain-diff-right")
                    .title("Right Motors")
                    .layout(17, 10, 7, 5)
                    .showOtherFields(true)));
        }
        return page.build();
    }

    private static ArcpDashboardLayout.Page defaultArcpPageForMechanism(String mechanismName) {
        return defaultArcpPageForMechanism(mechanismName, null, null);
    }

    private static ArcpDashboardLayout.Page defaultArcpPageForMechanism(
            String mechanismName,
            Mechanism mechanism,
            ARCP publisher) {
        String displayName =
                (mechanismName == null || mechanismName.isBlank()) ? "Mechanism" : mechanismName.trim();
        String slug = normalizeMechanismPageKey(displayName);
        ArcpDashboardLayout.Page.Builder page = ArcpDashboardLayout.Page.builder()
                .id("tab-" + slug)
                .name(displayName);
        if (publisher == null) {
            return page.build();
        }

        String root = "Athena/Mechanisms/" + normalizeArcpSignalSegment(displayName);
        String positionPath = root + "/Control/Status/position";
        String velocityPath = root + "/Control/Status/velocity";
        String setpointPath = root + "/Control/Setpoint/value";
        String outputPath = root + "/Control/Output/value";
        String outputTypePath = root + "/Control/Status/outputType";
        String emergencyPath = root + "/Control/Status/emergencyStopped";
        String faultReasonPath = root + "/Control/Status/faultReason";
        String diagnosticsUrlPath = root + "/Meta/diagnosticsUrl";

        int positionId = publisher.existingSignalId(positionPath);
        int velocityId = publisher.existingSignalId(velocityPath);
        int setpointId = publisher.existingSignalId(setpointPath);
        int outputId = publisher.existingSignalId(outputPath);
        int outputTypeId = publisher.existingSignalId(outputTypePath);
        int emergencyId = publisher.existingSignalId(emergencyPath);
        int faultReasonId = publisher.existingSignalId(faultReasonPath);
        int diagnosticsUrlId = publisher.existingSignalId(diagnosticsUrlPath);

        OutputType mechanismOutputType =
                mechanism != null && mechanism.outputType() != null ? mechanism.outputType() : OutputType.PERCENT;
        String outputOverlayPath;
        String outputOverlayLabel;
        int outputOverlayId;
        if (mechanismOutputType == OutputType.POSITION) {
            outputOverlayPath = positionPath;
            outputOverlayLabel = "position";
            outputOverlayId = positionId;
        } else if (mechanismOutputType == OutputType.VELOCITY) {
            outputOverlayPath = velocityPath;
            outputOverlayLabel = "velocity";
            outputOverlayId = velocityId;
        } else {
            outputOverlayPath = outputPath;
            outputOverlayLabel = mechanismOutputType == OutputType.VOLTAGE ? "voltage" : "output";
            outputOverlayId = outputId;
        }

        List<Map<String, Object>> setpointSeries = new ArrayList<>();
        if (setpointId > 0) {
            setpointSeries.add(graphSeries(setpointId, "setpoint", "#fbbf24", "step", "state"));
        }
        if (outputOverlayId > 0) {
            setpointSeries.add(graphSeries(outputOverlayId, outputOverlayLabel, "#22d3ee", "line", "metric"));
        }

        List<Map<String, Object>> statusItems = new ArrayList<>();
        if (emergencyId > 0) {
            Map<String, Object> emergencyItem = new LinkedHashMap<>();
            emergencyItem.put("signalId", emergencyId);
            emergencyItem.put("label", "E-Stop");
            emergencyItem.put("group", displayName);
            emergencyItem.put("healthyWhen", "false");
            emergencyItem.put("toggleSignalId", emergencyId);
            emergencyItem.put("valueSignalId", null);
            statusItems.add(emergencyItem);
        }

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-status")
                .kind("status_matrix")
                .title("Mechanism Status")
                .signalId(firstExistingSignalId(publisher, emergencyPath, faultReasonPath))
                .layout(0, 0, 8, 2)
                .config("items", statusItems)
                .config("columns", 1)
                .config("showSummary", false)
                .config("mode", "toggle")
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-fault")
                .kind("text")
                .title("Fault Reason")
                .signalId(faultReasonId)
                .layout(8, 0, 8, 2)
                .config("signalPath", faultReasonPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-diagnostics")
                .kind("text")
                .title("Diagnostics URL")
                .signalId(diagnosticsUrlId)
                .layout(16, 0, 8, 1)
                .config("signalPath", diagnosticsUrlPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-output-type")
                .kind("text")
                .title("Output Type")
                .signalId(outputTypeId)
                .layout(16, 1, 8, 1)
                .config("signalPath", outputTypePath)
                .build());

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-position-graph")
                .kind("graph")
                .title("Position")
                .signalId(firstExistingSignalId(publisher, positionPath, velocityPath))
                .layout(0, 2, 8, 3)
                .config("signalPath", positionPath)
                .config(
                        "series",
                        positionId > 0
                                ? List.of(graphSeries(positionId, "position", "#f87171", "line", "metric"))
                                : List.of())
                .config("showLegend", false)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-velocity-graph")
                .kind("graph")
                .title("Velocity")
                .signalId(firstExistingSignalId(publisher, velocityPath, positionPath))
                .layout(8, 2, 8, 3)
                .config("signalPath", velocityPath)
                .config(
                        "series",
                        velocityId > 0
                                ? List.of(graphSeries(velocityId, "velocity", "#22d3ee", "line", "metric"))
                                : List.of())
                .config("showLegend", false)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-setpoint-graph")
                .kind("graph")
                .title("Setpoint vs " + outputOverlayLabel)
                .signalId(firstExistingSignalId(publisher, setpointPath, outputOverlayPath))
                .layout(16, 2, 8, 3)
                .config("signalPath", setpointPath)
                .config("series", setpointSeries)
                .config("showLegend", true)
                .build());

        int leftY = 5;
        int rightY = 5;

        if (mechanism != null && mechanism.motors().device() != null) {
            String[] motorKeys = mechanism.motors().device().getControllerTopicKeys();
            if (motorKeys.length > 0) {
                String motorsListId = "w-" + slug + "-motors-list";
                int motorListHeight = Math.max(8, 5 + motorKeys.length + 2);
                page.widget(ArcpDashboardLayout.Widget.builder()
                        .id(motorsListId)
                        .kind("layout_list")
                        .title("Motors")
                        .layout(0, leftY, 12, motorListHeight)
                        .build());
                appendDefaultArcpMotorWidgets(page, slug, root, mechanism, publisher, motorsListId, 0);
                leftY += motorListHeight;
            }
        }

        if (mechanism != null && mechanism.encoder().device() != null) {
            int encoderY = leftY;
            page.widget(ArcpDeviceWidgets.encoder(root + "/Encoder", widget -> widget
                    .id("w-" + slug + "-encoder-standalone")
                    .title("Encoder")
                    .layout(0, encoderY, 12, 7)));
            leftY += 7;
        }

        if (mechanism != null) {
            GenericLimitSwitch[] switches = mechanism.limitSwitches();
            int switchCount = 0;
            if (switches != null) {
                for (GenericLimitSwitch sw : switches) {
                    if (sw != null) {
                        switchCount += 1;
                    }
                }
            }
            if (switchCount > 0) {
                String dioListId = "w-" + slug + "-dio-list";
                int dioListHeight = Math.max(4, switchCount + 2);
                page.widget(ArcpDashboardLayout.Widget.builder()
                        .id(dioListId)
                        .kind("layout_list")
                        .title("Limit Switches")
                        .layout(0, leftY, 12, dioListHeight)
                        .build());
                appendDefaultArcpLimitSwitchWidgets(page, slug, root, mechanism, publisher, dioListId, 0);
                leftY += dioListHeight;
            }
        }

        if (mechanism instanceof StatefulLike<?>) {
            int currentStateId = publisher.existingSignalId(root + "/Control/StateMachine/currentState");
            int goalStateId = publisher.existingSignalId(root + "/Control/StateMachine/goalState");
            int nextStateId = publisher.existingSignalId(root + "/Control/StateMachine/nextState");
            int queueId = publisher.existingSignalId(root + "/Control/StateMachine/queue");
            int atGoalId = publisher.existingSignalId(root + "/Control/StateMachine/atGoalState");
            int availableStatesId = publisher.existingSignalId(root + "/Control/StateMachine/availableStates");
            int clearQueueId = publisher.existingSignalId(root + "/Control/StateMachine/command/clearQueue");

            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-" + slug + "-state-machine")
                    .kind("state_machine")
                    .title("State Machine")
                    .signalId(firstExistingSignalId(publisher, root + "/Control/StateMachine/goalState"))
                    .layout(12, rightY, 12, 4)
                    .config("topicPath", root + "/Control/StateMachine")
                    .config("currentStateSignalId", currentStateId)
                    .config("goalStateSignalId", goalStateId)
                    .config("nextStateSignalId", nextStateId)
                    .config("queueSignalId", queueId)
                    .config("atGoalSignalId", atGoalId)
                    .config("availableStatesSignalId", availableStatesId)
                    .config("clearQueueCommandSignalId", clearQueueId)
                    .build());
            rightY += 4;
        }

        if (mechanism != null && !mechanism.loops().names().isEmpty()) {
            List<String> loopNames = new ArrayList<>();
            for (String loopName : mechanism.loops().names()) {
                if (loopName == null || loopName.isBlank()) {
                    continue;
                }
                loopNames.add(loopName);
            }
            if (!loopNames.isEmpty()) {
                String controllerListId = "w-" + slug + "-controller-list";
                int controllerListHeight = Math.max(8, loopNames.size() + 4);
                page.widget(ArcpDashboardLayout.Widget.builder()
                        .id(controllerListId)
                        .kind("layout_list")
                        .title("Controllers")
                        .layout(12, rightY, 12, controllerListHeight)
                        .build());

                List<Map<String, Object>> controllerMatrixItems = new ArrayList<>();
                for (String loopName : loopNames) {
                    String loopKey = normalizeArcpSignalSegment(loopName);
                    int enabledId = firstExistingSignalId(
                            publisher,
                            root + "/Control/Loops/" + loopKey + "/enabled",
                            root + "/Control/Output/Loops/" + loopKey + "/enabled");
                    int valueId = firstExistingSignalId(
                            publisher,
                            root + "/Control/Output/Loops/" + loopKey + "/value",
                            root + "/Control/Loops/" + loopKey + "/output/value");
                    if (enabledId <= 0) {
                        continue;
                    }
                    Map<String, Object> item = new LinkedHashMap<>();
                    item.put("signalId", enabledId);
                    item.put("label", loopName);
                    item.put("group", "Control Loops");
                    item.put("healthyWhen", "true");
                    item.put("toggleSignalId", enabledId);
                    item.put("valueSignalId", valueId);
                    controllerMatrixItems.add(item);
                }
                int controllerMatrixAnchorId = controllerMatrixItems.isEmpty()
                        ? firstExistingSignalId(
                                publisher,
                                root + "/Control/Output/value",
                                root + "/Control/Setpoint/value")
                        : ((Number) controllerMatrixItems.get(0).get("signalId")).intValue();

                page.widget(ArcpDashboardLayout.Widget.builder()
                        .id("w-" + slug + "-controller-status")
                        .kind("status_matrix")
                        .title("Controller Status")
                        .signalId(controllerMatrixAnchorId)
                        .layout(0, 0, 12, 2)
                        .parentLayoutId(controllerListId)
                        .config("items", controllerMatrixItems)
                        .config("columns", Math.max(1, Math.min(3, controllerMatrixItems.size())))
                        .config("showSummary", true)
                        .config("mode", "toggle_value")
                        .build());
                appendDefaultArcpControllerWidgets(page, slug, root, mechanism, publisher, controllerListId, 2);
                rightY += controllerListHeight;
            }
        }

        int bottomY = Math.max(leftY, rightY);

        String sysidListId = "w-" + slug + "-sysid-list";
        int sysidListHeight = 8;
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(sysidListId)
                .kind("layout_list")
                .title("SysId")
                .layout(0, bottomY, 12, sysidListHeight)
                .build());

        String sysidRoot = root + "/SysId";
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-active")
                .kind("state")
                .title("Active")
                .signalId(publisher.existingSignalId(sysidRoot + "/active"))
                .layout(0, 0, 4, 1)
                .parentLayoutId(sysidListId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-request-test")
                .kind("action")
                .title("Request Test Mode")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/requestTestMode"))
                .layout(4, 0, 8, 1)
                .parentLayoutId(sysidListId)
                .build());

        String quasiAccordionId = "w-" + slug + "-sysid-quasi-accordion";
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(quasiAccordionId)
                .kind("layout_accordion")
                .title("Quasistatic")
                .signalId(firstExistingSignalId(publisher, sysidRoot + "/active"))
                .layout(0, 1, 12, 1)
                .parentLayoutId(sysidListId)
                .config("collapsed", true)
                .config("expandedRows", 2)
                .build());
        String quasiGridId = "w-" + slug + "-sysid-quasi-grid";
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(quasiGridId)
                .kind("layout_grid")
                .title("Quasistatic Actions")
                .signalId(firstExistingSignalId(publisher, sysidRoot + "/active"))
                .layout(0, 0, 12, 2)
                .parentLayoutId(quasiAccordionId)
                .config("autoSize", false)
                .config("columns", 12)
                .config("rows", 1)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-quasi-forward")
                .kind("action")
                .title("Forward")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/quasistaticForward"))
                .layout(0, 0, 6, 1)
                .parentLayoutId(quasiGridId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-quasi-reverse")
                .kind("action")
                .title("Reverse")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/quasistaticReverse"))
                .layout(6, 0, 6, 1)
                .parentLayoutId(quasiGridId)
                .build());

        String dynamicAccordionId = "w-" + slug + "-sysid-dynamic-accordion";
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(dynamicAccordionId)
                .kind("layout_accordion")
                .title("Dynamic")
                .signalId(firstExistingSignalId(publisher, sysidRoot + "/active"))
                .layout(0, 2, 12, 1)
                .parentLayoutId(sysidListId)
                .config("collapsed", true)
                .config("expandedRows", 2)
                .build());
        String dynamicGridId = "w-" + slug + "-sysid-dynamic-grid";
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(dynamicGridId)
                .kind("layout_grid")
                .title("Dynamic Actions")
                .signalId(firstExistingSignalId(publisher, sysidRoot + "/active"))
                .layout(0, 0, 12, 2)
                .parentLayoutId(dynamicAccordionId)
                .config("autoSize", false)
                .config("columns", 12)
                .config("rows", 1)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-dynamic-forward")
                .kind("action")
                .title("Forward")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/dynamicForward"))
                .layout(0, 0, 6, 1)
                .parentLayoutId(dynamicGridId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-dynamic-reverse")
                .kind("action")
                .title("Reverse")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/dynamicReverse"))
                .layout(6, 0, 6, 1)
                .parentLayoutId(dynamicGridId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-sysid-cancel")
                .kind("action")
                .title("Cancel SysId")
                .signalId(publisher.existingSignalId(sysidRoot + "/Commands/cancel"))
                .layout(0, 3, 12, 1)
                .parentLayoutId(sysidListId)
                .build());

        String commandListId = "w-" + slug + "-commands-list";
        int commandListHeight = 6;
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(commandListId)
                .kind("layout_list")
                .title("Mechanism Commands")
                .layout(12, bottomY, 12, commandListHeight)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-estop-toggle")
                .kind("toggle")
                .title("Emergency Stop")
                .signalId(emergencyId)
                .layout(0, 0, 12, 1)
                .parentLayoutId(commandListId)
                .config("signalPath", emergencyPath)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-force-estop")
                .kind("action")
                .title("Force E-Stop")
                .signalId(publisher.existingSignalId(root + "/Control/Commands/forceEStop"))
                .layout(0, 1, 6, 1)
                .parentLayoutId(commandListId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-release-estop")
                .kind("action")
                .title("Release E-Stop")
                .signalId(publisher.existingSignalId(root + "/Control/Commands/releaseEStop"))
                .layout(6, 1, 6, 1)
                .parentLayoutId(commandListId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-stop-output")
                .kind("action")
                .title("Stop Output")
                .signalId(publisher.existingSignalId(root + "/Control/Commands/stopOutput"))
                .layout(0, 2, 6, 1)
                .parentLayoutId(commandListId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-neutral-brake")
                .kind("action")
                .title("Neutral Brake")
                .signalId(publisher.existingSignalId(root + "/Control/Commands/setNeutralBrake"))
                .layout(6, 2, 6, 1)
                .parentLayoutId(commandListId)
                .build());
        page.widget(ArcpDashboardLayout.Widget.builder()
                .id("w-" + slug + "-cmd-neutral-coast")
                .kind("action")
                .title("Neutral Coast")
                .signalId(publisher.existingSignalId(root + "/Control/Commands/setNeutralCoast"))
                .layout(0, 3, 6, 1)
                .parentLayoutId(commandListId)
                .build());

        if (mechanism instanceof StatefulLike<?>) {
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-" + slug + "-cmd-clear-queue")
                    .kind("action")
                    .title("Clear State Queue")
                    .signalId(publisher.existingSignalId(root + "/Control/StateMachine/command/clearQueue"))
                    .layout(6, 3, 6, 1)
                    .parentLayoutId(commandListId)
                    .build());
        }

        return page.build();
    }

    private static int appendDefaultArcpMotorWidgets(
            ArcpDashboardLayout.Page.Builder page,
            String slug,
            String root,
            Mechanism mechanism,
            ARCP publisher,
            String parentLayoutId,
            int startRow) {
        MotorControllerGroup motorGroup = mechanism.motors().device();
        if (motorGroup == null) {
            return startRow;
        }
        String[] keys = motorGroup.getControllerTopicKeys();
        if (keys.length == 0) {
            return startRow;
        }

        int nextRow = startRow;
        String motorGroupRoot = root + "/Motors";
        int groupAnchorId = publisher.existingSignalId(motorGroupRoot + "/Summary/avgTempC");
        if (groupAnchorId <= 0) {
            groupAnchorId = publisher.existingSignalId(root + "/Control/Status/output");
        }
        final int groupRow = nextRow;

        page.widget(ArcpDeviceWidgets.motor(groupAnchorId, widget -> widget
                .id("w-" + slug + "-motor-group")
                .title("Motor Group")
                .layout(0, groupRow, 12, 5)
                .parentLayoutId(parentLayoutId)
                .topicPath(root)
                .showOtherFields(false)
                .typeSignalPath(root + "/Control/Status/outputType")
                .connectedSignalPath(motorGroupRoot + "/Summary/allConnected")
                .outputSignalPath(root + "/Control/Output/value")
                .velocitySignalPath(motorGroupRoot + "/Encoders/Summary/avgVelocity")
                .positionSignalPath(motorGroupRoot + "/Encoders/Summary/avgPosition")
                .temperatureSignalPath(motorGroupRoot + "/Summary/avgTempC")
                .commandSignalPath(root + "/Control/Output/value")));
        nextRow += 5;

        for (int i = 0; i < keys.length; i++) {
            final int index = i;
            String displayName = keys[index] != null && !keys[index].isBlank() ? keys[index] : ("Motor " + (index + 1));
            // Motor topic keys are pre-sanitized by MotorControllerGroup to ARCP-safe segments.
            String key = keys[index] != null && !keys[index].isBlank() ? keys[index] : ("motor-" + index);
            String motorRoot = root + "/Motors/Motors/" + key;
            int anchorId = publisher.existingSignalId(motorRoot + "/tempC");
            String accordionId = "w-" + slug + "-motor-accordion-" + index;

            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id(accordionId)
                    .kind("layout_accordion")
                    .title(displayName)
                    .signalId(anchorId)
                    .layout(0, nextRow, 12, 1)
                    .parentLayoutId(parentLayoutId)
                    .config("collapsed", true)
                    .config("expandedRows", 5)
                    .build());
            page.widget(ArcpDeviceWidgets.motor(anchorId, widget -> widget
                    .id("w-" + slug + "-motor-" + index)
                    .title(displayName)
                    .layout(0, 0, 12, 5)
                    .parentLayoutId(accordionId)
                    .topicPath(motorRoot)
                    .showOtherFields(true)
                    .canIdSignalPath(motorRoot + "/canId")
                    .canbusSignalPath(motorRoot + "/canbus")
                    .typeSignalPath(motorRoot + "/type")
                    .connectedSignalPath(motorRoot + "/connected")
                    .stalledSignalPath(motorRoot + "/stalled")
                    .neutralModeSignalPath(motorRoot + "/neutralMode")
                    .currentLimitSignalPath(motorRoot + "/currentLimitA")
                    .invertedSignalPath(motorRoot + "/inverted")
                    .brakeModeSignalPath(motorRoot + "/brakeMode")
                    .outputSignalPath(root + "/Control/Output/value")
                    .velocitySignalPath(motorRoot + "/Encoder/velocity")
                    .positionSignalPath(motorRoot + "/Encoder/position")
                    .currentSignalPath(motorRoot + "/currentA")
                    .temperatureSignalPath(motorRoot + "/tempC")
                    .voltageSignalPath(motorRoot + "/voltage")
                    .commandSignalPath(motorRoot + "/outputCommandPercent")));
            nextRow += 1;
        }
        return nextRow;
    }

    private static int appendDefaultArcpEncoderWidget(
            ArcpDashboardLayout.Page.Builder page,
            String slug,
            String root,
            Mechanism mechanism,
            ARCP publisher,
            String parentLayoutId,
            int startRow) {
        if (mechanism.encoder().device() == null) {
            return startRow;
        }
        String encoderRoot = root + "/Encoder";
        int anchorId = publisher.existingSignalId(encoderRoot + "/position");
        String accordionId = "w-" + slug + "-encoder-accordion";

        page.widget(ArcpDashboardLayout.Widget.builder()
                .id(accordionId)
                .kind("layout_accordion")
                .title("Encoder")
                .signalId(anchorId)
                .layout(0, startRow, 12, 1)
                .parentLayoutId(parentLayoutId)
                .config("collapsed", true)
                .config("expandedRows", 7)
                .build());
        page.widget(ArcpDeviceWidgets.encoder(anchorId, widget -> widget
                .id("w-" + slug + "-encoder")
                .title("Encoder")
                .layout(0, 0, 12, 7)
                .parentLayoutId(accordionId)
                .topicPath(encoderRoot)
                .positionSignalPath(encoderRoot + "/position")
                .velocitySignalPath(encoderRoot + "/velocity")
                .rateSignalPath(encoderRoot + "/rate")
                .absoluteSignalPath(encoderRoot + "/absolutePosition")
                .connectedSignalPath(encoderRoot + "/connected")
                .conversionSignalPath(encoderRoot + "/conversion")
                .conversionOffsetSignalPath(encoderRoot + "/conversionOffset")
                .discontinuityPointSignalPath(encoderRoot + "/discontinuityPoint")
                .discontinuityRangeSignalPath(encoderRoot + "/discontinuityRange")
                .ratioSignalPath(encoderRoot + "/gearRatio")
                .offsetSignalPath(encoderRoot + "/offset")
                .canIdSignalPath(encoderRoot + "/canId")
                .canbusSignalPath(encoderRoot + "/canbus")
                .typeSignalPath(encoderRoot + "/type")
                .invertedSignalPath(encoderRoot + "/inverted")
                .supportsSimulationSignalPath(encoderRoot + "/supportsSimulation")
                .rawAbsoluteSignalPath(encoderRoot + "/rawAbsolute")));
        return startRow + 1;
    }

    private static int appendDefaultArcpLimitSwitchWidgets(
            ArcpDashboardLayout.Page.Builder page,
            String slug,
            String root,
            Mechanism mechanism,
            ARCP publisher,
            String parentLayoutId,
            int startRow) {
        GenericLimitSwitch[] switches = mechanism.limitSwitches();
        if (switches == null || switches.length == 0) {
            return startRow;
        }

        int nextRow = startRow;
        for (int i = 0; i < switches.length; i++) {
            GenericLimitSwitch sw = switches[i];
            if (sw == null) {
                continue;
            }
            String switchRoot = root + "/Sensors/LimitSwitches/dio-" + sw.getPort();
            int anchorId = publisher.signalId(switchRoot + "/active");
            String accordionId = "w-" + slug + "-dio-accordion-" + sw.getPort();
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id(accordionId)
                    .kind("layout_accordion")
                    .title("DIO " + sw.getPort())
                    .signalId(anchorId)
                    .layout(0, nextRow, 12, 1)
                    .parentLayoutId(parentLayoutId)
                    .config("collapsed", true)
                    .config("expandedRows", 2)
                    .build());
            page.widget(ArcpDeviceWidgets.dio(anchorId, widget -> widget
                    .id("w-" + slug + "-dio-" + sw.getPort())
                    .title("DIO " + sw.getPort())
                    .layout(0, 0, 12, 2)
                    .parentLayoutId(accordionId)
                    .topicPath(switchRoot)
                    .showOtherFields(true)
                    .valueSignal(publisher.signalId(switchRoot + "/active"))
                    .outputSignal(publisher.signalId(switchRoot + "/hardstop"))
                    .invertedSignal(publisher.signalId(switchRoot + "/inverted"))
                    .channelSignal(publisher.signalId(switchRoot + "/port"))
                    .portSignal(publisher.signalId(switchRoot + "/port"))
                    .modeSignal(publisher.signalId(switchRoot + "/blockDirection"))
                    .nameSignal(publisher.signalId(switchRoot + "/name"))
                    .typeSignal(publisher.signalId(switchRoot + "/type"))));
            nextRow += 1;
        }
        return nextRow;
    }

    private static int appendDefaultArcpControllerWidgets(
            ArcpDashboardLayout.Page.Builder page,
            String slug,
            String root,
            Mechanism mechanism,
            ARCP publisher,
            String parentLayoutId,
            int startRow) {
        if (mechanism == null || mechanism.loops().names().isEmpty()) {
            return startRow;
        }
        int nextRow = startRow;
        for (String loopName : mechanism.loops().names()) {
            if (loopName == null || loopName.isBlank()) {
                continue;
            }
            String loopKey = normalizeArcpSignalSegment(loopName);
            String loopRoot = root + "/Control/Loops/" + loopKey;
            int anchorId = publisher.signalId(loopRoot + "/output/value");
            List<Map<String, Object>> params = new ArrayList<>();

            MechanismConfig.PidProfile pidProfile = mechanism.getControlLoopPidProfile(loopName);
            if (pidProfile != null) {
                addControllerParam(params, "kp", "kP", publisher.writableDouble(loopRoot + "/pid/kp").signalId());
                addControllerParam(params, "ki", "kI", publisher.writableDouble(loopRoot + "/pid/ki").signalId());
                addControllerParam(params, "kd", "kD", publisher.writableDouble(loopRoot + "/pid/kd").signalId());
                addControllerParam(params, "izone", "I Zone", publisher.writableDouble(loopRoot + "/pid/izone").signalId());
                addControllerParam(
                        params,
                        "setpoint",
                        "Setpoint",
                        publisher.writableDouble(root + "/Control/Setpoint/value").signalId());
            }

            MechanismConfig.FeedforwardProfile ffProfile = mechanism.getControlLoopFeedforwardProfile(loopName);
            if (ffProfile != null) {
                addControllerParam(params, "ks", "kS", publisher.writableDouble(loopRoot + "/ff/ks").signalId());
                addControllerParam(params, "kg", "kG", publisher.writableDouble(loopRoot + "/ff/kg").signalId());
                addControllerParam(params, "kv", "kV", publisher.writableDouble(loopRoot + "/ff/kv").signalId());
                addControllerParam(params, "ka", "kA", publisher.writableDouble(loopRoot + "/ff/ka").signalId());
            }

            MechanismConfig.BangBangProfile bangBangProfile = mechanism.getControlLoopBangBangProfile(loopName);
            if (bangBangProfile != null) {
                addControllerParam(
                        params,
                        "high_output",
                        "High Out",
                        publisher.writableDouble(loopRoot + "/bangBang/highOutput").signalId());
                addControllerParam(
                        params,
                        "low_output",
                        "Low Out",
                        publisher.writableDouble(loopRoot + "/bangBang/lowOutput").signalId());
                addControllerParam(
                        params,
                        "tolerance",
                        "Tolerance",
                        publisher.writableDouble(loopRoot + "/bangBang/tolerance").signalId());
            }

            if (params.isEmpty()) {
                continue;
            }
            String accordionId = "w-" + slug + "-controller-accordion-" + loopKey;

            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id(accordionId)
                    .kind("layout_accordion")
                    .title(loopName + " Controller")
                    .signalId(anchorId)
                    .layout(0, nextRow, 12, 1)
                    .parentLayoutId(parentLayoutId)
                    .config("collapsed", true)
                    .config("expandedRows", 3)
                    .build());
            page.widget(ArcpDashboardLayout.Widget.builder()
                    .id("w-" + slug + "-controller-" + loopKey)
                    .kind("controller")
                    .title(loopName + " Controller")
                    .signalId(anchorId)
                    .layout(0, 0, 12, 3)
                    .parentLayoutId(accordionId)
                    .config("topicPath", loopRoot)
                    .config("params", params)
                    .build());
            nextRow += 1;
        }
        return nextRow;
    }

    private static void addControllerParam(List<Map<String, Object>> params, String key, String label, int signalId) {
        Map<String, Object> param = new LinkedHashMap<>();
        param.put("key", key);
        param.put("label", label);
        param.put("signalId", signalId);
        params.add(param);
    }

    private static Map<String, Object> graphSeries(
            int signalId,
            String label,
            String color,
            String style,
            String role) {
        Map<String, Object> series = new LinkedHashMap<>();
        series.put("signalId", signalId);
        series.put("label", label);
        series.put("color", color);
        series.put("style", style);
        series.put("role", role);
        return series;
    }

    private static String normalizeArcpProfileName(String profileName) {
        if (profileName == null || profileName.isBlank()) {
            return "atheana-generated";
        }
        return profileName.trim();
    }

    private static String normalizeArcpSignalSegment(String raw) {
        if (raw == null || raw.isBlank()) {
            return "mechanism";
        }
        String input = raw.trim();
        StringBuilder out = new StringBuilder(input.length());
        for (int i = 0; i < input.length(); i++) {
            char ch = input.charAt(i);
            boolean valid = (ch >= 'a' && ch <= 'z')
                    || (ch >= 'A' && ch <= 'Z')
                    || (ch >= '0' && ch <= '9')
                    || ch == '-'
                    || ch == '_';
            out.append(valid ? ch : '_');
        }
        String normalized = out.toString();
        return normalized.isEmpty() ? "mechanism" : normalized;
    }

    private static String normalizeMechanismPageKey(String mechanismName) {
        if (mechanismName == null || mechanismName.isBlank()) {
            return "mechanism";
        }
        String lower = mechanismName.trim().toLowerCase();
        StringBuilder out = new StringBuilder(lower.length());
        boolean lastDash = false;
        for (int i = 0; i < lower.length(); i++) {
            char ch = lower.charAt(i);
            boolean valid = (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9');
            if (valid) {
                out.append(ch);
                lastDash = false;
                continue;
            }
            if (!lastDash) {
                out.append('-');
                lastDash = true;
            }
        }
        String normalized = out.toString();
        while (normalized.startsWith("-")) {
            normalized = normalized.substring(1);
        }
        while (normalized.endsWith("-")) {
            normalized = normalized.substring(0, normalized.length() - 1);
        }
        return normalized.isEmpty() ? "mechanism" : normalized;
    }

    /**
     * Runtime API for publishing user-defined content via the Athena config server.
     */
    public ConfigServerSection configServer() {
        return configServerSection;
    }

    /**
     * Runtime ARCP controls.
     */
    public ArcpSection arcp() {
        return arcpSection;
    }

    public RobotCore<T> arcp(Consumer<ArcpSection> section) {
        if (section != null) {
            section.accept(arcpSection);
        }
        return this;
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
        prelightSelectedAutoIfDisabled();

        double cycleStartSeconds = nowSeconds();
        boolean startupDriveLagGuardActive = isStartupDriveLagGuardActive(cycleStartSeconds);

        long t0Ns = System.nanoTime();
        CommandScheduler.getInstance().run();
        long t1Ns = System.nanoTime();
        if (!startupDriveLagGuardActive) {
            telemetry.tick();
            AthenaNT.tick();
        }
        long t2Ns = System.nanoTime();
        if (!startupDriveLagGuardActive && localization != null) {
            localization.updateAutoVisualization(autos);
        }
        long t3Ns = System.nanoTime();
        if (runtimeMode != RuntimeMode.TEST) {
            runCoreControlLoops(cycleStartSeconds);
            runRegisteredPhaseHooks(RobotCoreHooks.Phase.ROBOT_PERIODIC);
            runCorePeriodicBindings(
                    RobotCoreHooks.Phase.ROBOT_PERIODIC,
                    coreHooks.periodicBindings(),
                    periodicHookRunners);
        }
        onRobotPeriodic();
        if (!startupDriveLagGuardActive) {
            robotNetworkTables.refresh();
            robotNetworkTables.beginPublishCycle();
            updateAutoChooserPublishers();

            // Keep control-loop work ahead of NT publishing to reduce input lag under I/O stalls.
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
            publishArcpSignals(cycleStartSeconds);
        }
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
            // Mechanism control/status values are dynamic and should stream continuously.
            // Keep publication round-robin/batched to cap per-cycle NT overhead.
            boolean needsRefresh = remainingMechanismRefreshCount > 0;
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
            // Superstructure state (state machine/inputs) is dynamic and must be published continuously.
            // Keep this round-robin batched to avoid bursting NT bandwidth.
            boolean needsRefresh = remainingSuperstructureRefreshCount > 0;
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
        triggerArcpRecordingRequest("autonomous");
        armStartupDriveLagGuardWindow();
        prepareDrivetrainForModeTransition(false, true);
        boolean autoPoseReset = resetAutoInitPoseIfConfigured();
        if (autoPoseReset) {
            syncHeadingAxesToFieldPose();
        }
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
        resetDriverAxesToAllianceForward();
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
        triggerArcpRecordingRequest("teleop");
        armStartupDriveLagGuardWindow();
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        resetDriverAxesToAllianceForward();
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
        startupDriveLagGuardUntilSeconds = Double.NaN;
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        resetDriverAxesToAllianceForward();
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
        processDeferredStartupSteps();
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
        startupDriveLagGuardUntilSeconds = Double.NaN;
        CommandScheduler.getInstance().cancelAll();
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(false, false);
        resetPeriodicRunners(testPeriodicHookRunners);
        onTestInit();
    }

    @Override
    public final void testExit() {
        diagnosticsSection.core().info("mode", "testExit");
        onTestExit();
    }

    @Override
    public final void testPeriodic() {
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

    private void runRobotInitStep(String step, Runnable action, boolean allowDeferred) {
        if (action == null) {
            return;
        }
        if (!deferNonDriveStartupInit || !allowDeferred) {
            timedStartupStep(step, action);
            return;
        }
        deferredStartupTasks.addLast(new DeferredStartupTask(step, action));
        deferredStartupInitComplete = false;
        deferredStartupInitCompleteLogged = false;
    }

    private void processDeferredStartupSteps() {
        if (!deferNonDriveStartupInit || deferredStartupTasks.isEmpty()) {
            if (deferredStartupTasks.isEmpty()) {
                deferredStartupInitComplete = true;
            }
            return;
        }
        double cycleStart = nowSeconds();
        boolean hasCycleStart = Double.isFinite(cycleStart);
        while (!deferredStartupTasks.isEmpty()) {
            DeferredStartupTask task = deferredStartupTasks.pollFirst();
            if (task == null || task.action() == null) {
                continue;
            }
            timedStartupStep("deferred." + task.step(), task.action());
            if (deferredStartupInitBudgetSeconds <= 0.0 || !hasCycleStart) {
                continue;
            }
            double now = nowSeconds();
            if (Double.isFinite(now) && (now - cycleStart) >= deferredStartupInitBudgetSeconds) {
                break;
            }
        }
        if (deferredStartupTasks.isEmpty()) {
            deferredStartupInitComplete = true;
            if (!deferredStartupInitCompleteLogged) {
                deferredStartupInitCompleteLogged = true;
                diagnosticsSection.core().info("startup", "deferred startup initialization complete");
            }
        }
    }

    private void armStartupDriveLagGuardWindow() {
        if (!Double.isFinite(startupDriveLagGuardSeconds) || startupDriveLagGuardSeconds <= 0.0) {
            startupDriveLagGuardUntilSeconds = Double.NaN;
            return;
        }
        double now = nowSeconds();
        if (!Double.isFinite(now)) {
            startupDriveLagGuardUntilSeconds = Double.NaN;
            return;
        }
        startupDriveLagGuardUntilSeconds = now + startupDriveLagGuardSeconds;
    }

    private boolean isStartupDriveLagGuardActive(double nowSeconds) {
        if (!Double.isFinite(startupDriveLagGuardUntilSeconds) || !Double.isFinite(nowSeconds)) {
            return false;
        }
        return nowSeconds < startupDriveLagGuardUntilSeconds;
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
        return autos.execution().prelightSelectedCommand()
                .or(autos.execution()::selectedCommand)
                .orElse(null);
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
        axis.kp(constants.kP())
                .ki(constants.kI())
                .kd(constants.kD())
                .iZone(constants.iZone())
                .inverted(constants.inverted());
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

    private boolean applySystemTweaks(SystemConfig config) {
        boolean ok = true;
        ok = systemSection.overcommitMode(config.vmOvercommitMode()) && ok;
        ok = systemSection.overcommitRatio(config.vmOvercommitRatio()) && ok;
        ok = systemSection.swappiness(config.vmSwappiness()) && ok;
        ok = systemSection.setLoopSwapEnabled(config.loopSwapEnabled(), config.loopSwapSizeMiB()) && ok;
        ok = systemSection.setSystemWebServerEnabled(config.systemWebServerEnabled()) && ok;
        return ok;
    }

    private boolean resetSystemTweaksToRioDefaults() {
        SystemConfig defaults = SystemConfig.rioDefaults();
        boolean ok = true;
        ok = systemSection.overcommitMode(defaults.vmOvercommitMode()) && ok;
        ok = systemSection.overcommitRatio(defaults.vmOvercommitRatio()) && ok;
        ok = systemSection.swappiness(defaults.vmSwappiness()) && ok;
        ok = systemSection.setLoopSwapEnabled(false, defaults.loopSwapSizeMiB()) && ok;
        ok = systemSection.setSystemWebServerEnabled(defaults.systemWebServerEnabled()) && ok;
        return ok;
    }

    private void startSystemTweaksInitializationAsync(SystemConfig config) {
        if (!RobotBase.isReal()) {
            return;
        }
        final SystemConfig resolvedConfig = config != null ? config : SystemConfig.defaults();
        final boolean tweaksEnabled = resolvedConfig.tweaksEnabled();
        diagnosticsSection.core().info(
                "startup",
                tweaksEnabled
                        ? "scheduling system tweaks in background"
                        : "scheduling system tweak reset in background");
        Thread thread = new Thread(
                () -> {
                    double start = Timer.getFPGATimestamp();
                    boolean tweaksOk;
                    try {
                        tweaksOk = tweaksEnabled
                                ? applySystemTweaks(resolvedConfig)
                                : resetSystemTweaksToRioDefaults();
                    } catch (Throwable ex) {
                        String message = "system tweak startup task crashed: "
                                + ex.getClass().getSimpleName()
                                + (ex.getMessage() != null && !ex.getMessage().isBlank()
                                        ? " - " + ex.getMessage()
                                        : "");
                        DriverStation.reportWarning("[Athena][System] " + message, false);
                        diagnosticsSection.core().warn("system", message);
                        return;
                    }
                    logStartupDuration("background.systemTweaks", Timer.getFPGATimestamp() - start);
                    if (!tweaksOk) {
                        diagnosticsSection.core().warn(
                                "system",
                                tweaksEnabled
                                        ? "one or more system tweaks failed to apply"
                                        : "one or more system tweaks failed to reset");
                    }
                },
                "athena-system-tweaks-" + SYSTEM_TWEAKS_THREAD_COUNTER.getAndIncrement());
        thread.setDaemon(true);
        thread.start();
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
        boolean updated = runSudoCommandAndReport(
                "sysctl " + key + "=" + value,
                SYSTEM_WEBSERVER_CONTROL_TIMEOUT_SECONDS,
                "/sbin/sysctl",
                "-w",
                key + "=" + value);
        if (!updated) {
            return false;
        }
        Integer applied = readIntFromFile(procPath);
        if (applied != null && applied.intValue() == value) {
            return true;
        }
        String message = "sysctl verify failed for " + key + " (expected " + value + ")";
        DriverStation.reportWarning("[Athena][System] " + message, false);
        diagnosticsSection.core().warn("system", message);
        return false;
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
                killProcessesByName(SYSTEM_WEBCONTAINER_PROCESS_NAME);
                killProcessesByName(SYSTEM_WEBSERVER_PROCESS_NAME);
                yield !isSystemWebServerRunning();
            }
            case "start" -> {
                if (isSystemWebServerRunning()) {
                    yield true;
                }
                runSudoCommandAndReport(
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

    private List<String> findLoopDevicesForSwapFile() {
        SystemCommandResult result = runPrivilegedCommand(
                SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                "/sbin/losetup",
                "-j",
                SYSTEM_LOOP_SWAP_FILE_PATH);
        if (result.timedOut()) {
            return List.of();
        }
        if (result.exitCode() != 0) {
            return List.of();
        }
        if (result.output().isBlank()) {
            return List.of();
        }
        List<String> devices = new ArrayList<>();
        for (String rawLine : result.output().lines().toList()) {
            String line = rawLine != null ? rawLine.trim() : "";
            if (line.isBlank()) {
                continue;
            }
            int colon = line.indexOf(':');
            if (colon <= 0) {
                continue;
            }
            String device = line.substring(0, colon).trim();
            if (device.isBlank()) {
                continue;
            }
            if (!devices.contains(device)) {
                devices.add(device);
            }
        }
        return devices;
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

    private boolean isSwapPathActive(String path) {
        if (path == null || path.isBlank()) {
            return false;
        }
        String normalizedPath = path.trim();
        try {
            List<String> lines = java.nio.file.Files.readAllLines(java.nio.file.Path.of("/proc/swaps"));
            for (int i = 1; i < lines.size(); i++) {
                String line = lines.get(i) != null ? lines.get(i).trim() : "";
                if (line.isBlank()) {
                    continue;
                }
                String[] columns = line.split("\\s+");
                if (columns.length > 0 && normalizedPath.equals(columns[0])) {
                    return true;
                }
            }
        } catch (IOException ignored) {
            return false;
        }
        return false;
    }

    private boolean isSwapDeviceActive(String loopDevice) {
        return isSwapPathActive(loopDevice);
    }

    private boolean deleteLoopSwapFileIfPresent() {
        java.nio.file.Path path = java.nio.file.Path.of(SYSTEM_LOOP_SWAP_FILE_PATH);
        try {
            java.nio.file.Files.deleteIfExists(path);
            return true;
        } catch (IOException ignored) {
            boolean removed = runSudoCommandAndReport(
                    "loop swap file delete",
                    SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                    "/bin/rm",
                    "-f",
                    SYSTEM_LOOP_SWAP_FILE_PATH);
            if (removed) {
                return true;
            }
            String message = "loop swap file cleanup failed: " + SYSTEM_LOOP_SWAP_FILE_PATH;
            DriverStation.reportWarning("[Athena][System] " + message, false);
            diagnosticsSection.core().warn("system", message);
            return false;
        }
    }

    private boolean disableLoopSwap(boolean deleteFile) {
        boolean ok = true;
        if (isSwapPathActive(SYSTEM_LOOP_SWAP_FILE_PATH)) {
            ok = runSudoCommandAndReport(
                            "loop swapoff (file)",
                            SYSTEM_LOOP_SWAP_CONTROL_TIMEOUT_SECONDS,
                            "/sbin/swapoff",
                            SYSTEM_LOOP_SWAP_FILE_PATH)
                    && ok;
        }
        for (String loopDevice : findLoopDevicesForSwapFile()) {
            if (isSwapDeviceActive(loopDevice)) {
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
        }
        if (deleteFile) {
            ok = deleteLoopSwapFileIfPresent() && ok;
        }
        return ok && !isLoopSwapRunning();
    }

    private boolean configureLoopSwapEnabled(boolean enabled, int sizeMiB) {
        if (!RobotBase.isReal()) {
            return true;
        }
        int resolvedSizeMiB = clampLoopSwapSizeMiB(sizeMiB);
        long expectedBytes = (long) resolvedSizeMiB * 1024L * 1024L;
        long existingBytes = loopSwapFileSizeBytes();
        List<String> loopDevices = findLoopDevicesForSwapFile();
        boolean active = loopDevices.stream().anyMatch(this::isSwapDeviceActive);
        boolean directFileSwapActive = isSwapPathActive(SYSTEM_LOOP_SWAP_FILE_PATH);

        if (!enabled) {
            return disableLoopSwap(true);
        }

        if (!directFileSwapActive
                && loopDevices.size() == 1
                && active
                && existingBytes == expectedBytes) {
            return true;
        }

        if (!disableLoopSwap(false)) {
            return false;
        }

        if (!ensureLoopSwapFileSized(resolvedSizeMiB)) {
            return false;
        }
        String loopDevice = findFreeLoopDevice();
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
        if (isSwapPathActive(SYSTEM_LOOP_SWAP_FILE_PATH)) {
            return true;
        }
        for (String loopDevice : findLoopDevicesForSwapFile()) {
            if (isSwapDeviceActive(loopDevice)) {
                return true;
            }
        }
        return false;
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
        drivetrain().speeds().stop(RobotSpeeds.AUTO_SOURCE);
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
            indexMechanismStateEndpoint(mech);
        }
        refreshArcpMechanismLayoutIfNeeded();
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
            indexSuperstructureStateEndpoint(s);
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
        int factoryCount = 0;
        for (RegisterableMechanism entry : entries) {
            if (entry instanceof RegisterableMechanismFactory) {
                factoryCount++;
            }
        }
        @SuppressWarnings("unchecked")
        Future<RegisterableMechanism>[] builtFactories = factoryCount > 1
                ? new Future[entries.length]
                : null;
        ExecutorService factoryBuildExecutor = null;
        if (builtFactories != null) {
            int parallelism = resolveMechanismFactoryBuildParallelism(factoryCount);
            if (parallelism > 1) {
                factoryBuildExecutor = Executors.newFixedThreadPool(parallelism, mechanismFactoryBuildThreadFactory());
                for (int i = 0; i < entries.length; i++) {
                    RegisterableMechanism entry = entries[i];
                    if (!(entry instanceof RegisterableMechanismFactory factory)) {
                        continue;
                    }
                    builtFactories[i] = factoryBuildExecutor.submit(() -> buildRegisterableMechanism(factory));
                }
            } else {
                builtFactories = null;
            }
        }
        try {
            for (int i = 0; i < entries.length; i++) {
                RegisterableMechanism entry = entries[i];
                if (entry == null) {
                    continue;
                }
                if (entry instanceof RegisterableMechanismFactory factory) {
                    RegisterableMechanism built = builtFactories != null && builtFactories[i] != null
                            ? awaitRegisterableMechanismBuild(builtFactories[i], factory)
                            : buildRegisterableMechanism(factory);
                    if (built == null) {
                        continue;
                    }
                    registerMechanism(built);
                    continue;
                }
                if (entry instanceof SuperstructureMechanism<?, ?> superstructure) {
                    registerSuperstructureInternal(superstructure);
                    RobotNetworkTables.Node superNode = mechanismsRootNode.child(
                            superstructure.getName() != null ? superstructure.getName() : "Superstructure");
                    robotNetworkTables.superstructureConfig(superNode);
                    superstructure.setRobotCore(this);
                    superstructure.diagnostics().info("lifecycle", "superstructure registered");
                    registerSuperstructureDiagnosticsProviderIfReady(superstructure);
                    indexSuperstructureStateEndpoint(superstructure);
                    superstructure.networkTables().ownerPath(superstructure.getName());
                }
                registerMechanism(entry.flattenForRegistration().toArray(Mechanism[]::new));
            }
        } finally {
            if (factoryBuildExecutor != null) {
                factoryBuildExecutor.shutdown();
            }
        }
        return this;
    }

    private static RegisterableMechanism buildRegisterableMechanism(RegisterableMechanismFactory factory) {
        RegisterableMechanism built = factory.build();
        if (built == factory) {
            throw new IllegalStateException("RegisterableMechanismFactory returned itself.");
        }
        return built;
    }

    private static RegisterableMechanism awaitRegisterableMechanismBuild(
            Future<RegisterableMechanism> future,
            RegisterableMechanismFactory factory) {
        try {
            RegisterableMechanism built = future.get();
            if (built == factory) {
                throw new IllegalStateException("RegisterableMechanismFactory returned itself.");
            }
            return built;
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("Interrupted while building registerable mechanism factory", ex);
        } catch (ExecutionException ex) {
            Throwable cause = ex.getCause();
            if (cause instanceof RuntimeException runtime) {
                throw runtime;
            }
            if (cause instanceof Error error) {
                throw error;
            }
            throw new IllegalStateException("Failed to build registerable mechanism factory", cause);
        }
    }

    private static int resolveMechanismFactoryBuildParallelism(int factoryCount) {
        Integer configured = parsePositiveIntProperty(MECHANISM_FACTORY_BUILD_PARALLELISM_PROPERTY);
        if (configured != null) {
            return Math.max(1, Math.min(factoryCount, configured));
        }
        int cpus = Math.max(1, Runtime.getRuntime().availableProcessors());
        int defaultThreads = Math.max(2, cpus * 2);
        int capped = Math.max(1, Math.min(MAX_MECHANISM_FACTORY_BUILD_THREADS, defaultThreads));
        return Math.max(1, Math.min(factoryCount, capped));
    }

    private static Integer parsePositiveIntProperty(String propertyName) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.isBlank()) {
            return null;
        }
        try {
            int parsed = Integer.parseInt(raw.trim());
            return parsed > 0 ? parsed : null;
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    private static boolean parseBooleanProperty(String propertyName, boolean fallback) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        String normalized = raw.trim().toLowerCase(java.util.Locale.ROOT);
        if ("true".equals(normalized) || "1".equals(normalized) || "yes".equals(normalized)) {
            return true;
        }
        if ("false".equals(normalized) || "0".equals(normalized) || "no".equals(normalized)) {
            return false;
        }
        return fallback;
    }

    private static double parseNonNegativeDoubleProperty(String propertyName, double fallback) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        try {
            double parsed = Double.parseDouble(raw.trim());
            if (!Double.isFinite(parsed) || parsed < 0.0) {
                return fallback;
            }
            return parsed;
        } catch (NumberFormatException ignored) {
            return fallback;
        }
    }

    private static ThreadFactory mechanismFactoryBuildThreadFactory() {
        return runnable -> {
            Thread thread = new Thread(
                    runnable,
                    "athena-mechanism-factory-build-" + MECHANISM_FACTORY_BUILD_THREAD_COUNTER.getAndIncrement());
            thread.setDaemon(true);
            return thread;
        };
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

    public StateSection state() {
        return stateSection;
    }

    public boolean hasStateOwner(Class<? extends Enum<?>> enumType) {
        return state().hasOwner(enumType);
    }

    public String stateOwnerName(Class<? extends Enum<?>> enumType) {
        return state().ownerName(enumType);
    }

    public final class StateSection {
        private StateSection() {}

        public <E extends Enum<E>> boolean queue(E state) {
            if (state == null) {
                return false;
            }
            StateEndpoint endpoint = resolveStateEndpoint(state);
            if (endpoint == null) {
                return false;
            }
            endpoint.queue(state);
            return true;
        }

        public <E extends Enum<E>> boolean force(E state) {
            if (state == null) {
                return false;
            }
            StateEndpoint endpoint = resolveStateEndpoint(state);
            if (endpoint == null) {
                return false;
            }
            endpoint.force(state);
            return true;
        }

        public <E extends Enum<E>> boolean at(E state) {
            if (state == null) {
                return false;
            }
            StateEndpoint endpoint = resolveStateEndpoint(state);
            return endpoint != null && endpoint.at(state);
        }

        public boolean hasOwner(Class<? extends Enum<?>> enumType) {
            if (enumType == null) {
                return false;
            }
            return resolveStateEndpoint(enumType) != null;
        }

        public String ownerName(Class<? extends Enum<?>> enumType) {
            if (enumType == null) {
                return "";
            }
            StateEndpoint endpoint = resolveStateEndpoint(enumType);
            return endpoint != null ? endpoint.ownerName() : "";
        }
    }

    private StateEndpoint resolveStateEndpoint(Enum<?> state) {
        if (state == null) {
            return null;
        }
        StateEndpoint endpoint = stateEndpointsByDefaultState.get(state);
        if (endpoint != null) {
            return endpoint;
        }
        rebuildStateEndpoints();
        endpoint = stateEndpointsByDefaultState.get(state);
        if (endpoint != null) {
            return endpoint;
        }
        List<StateEndpoint> enumEndpoints = stateEndpointsByEnum.get(state.getDeclaringClass());
        if (enumEndpoints == null || enumEndpoints.isEmpty()) {
            return null;
        }
        return enumEndpoints.size() == 1 ? enumEndpoints.get(0) : null;
    }

    private StateEndpoint resolveStateEndpoint(Class<? extends Enum<?>> enumType) {
        if (enumType == null) {
            return null;
        }
        List<StateEndpoint> enumEndpoints = stateEndpointsByEnum.get(enumType);
        if (enumEndpoints == null || enumEndpoints.isEmpty()) {
            return null;
        }
        return enumEndpoints.get(0);
    }

    private void rebuildStateEndpoints() {
        stateEndpointsByEnum.clear();
        stateEndpointsByDefaultState.clear();
        for (Mechanism mechanism : mechanisms.values()) {
            indexMechanismStateEndpoint(mechanism);
        }
        for (SuperstructureMechanism<?, ?> superstructure : superstructuresByName.values()) {
            indexSuperstructureStateEndpoint(superstructure);
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private void indexMechanismStateEndpoint(Mechanism mechanism) {
        if (!(mechanism instanceof StatefulLike<?> stateful)) {
            return;
        }
        Enum<?> goal = stateful.stateMachine().goal();
        if (goal == null) {
            return;
        }
        String ownerName = mechanism.getName() != null ? mechanism.getName() : mechanism.getClass().getSimpleName();
        StatefulLike raw = (StatefulLike) stateful;
        registerStateEndpoint(goal, ownerName, new StateEndpoint() {
            @Override
            public void queue(Enum<?> state) {
                raw.stateMachine().queue(state);
            }

            @Override
            public void force(Enum<?> state) {
                raw.stateMachine().force(state);
            }

            @Override
            public boolean at(Enum<?> state) {
                return raw.stateMachine().at(state);
            }

            @Override
            public String ownerName() {
                return ownerName;
            }
        });
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private void indexSuperstructureStateEndpoint(SuperstructureMechanism<?, ?> superstructure) {
        if (superstructure == null || superstructure.stateMachine() == null) {
            return;
        }
        Enum<?> goal = superstructure.stateMachine().goal();
        if (goal == null) {
            return;
        }
        String ownerName = superstructure.getName() != null
                ? superstructure.getName()
                : superstructure.getClass().getSimpleName();
        SuperstructureMechanism.StateMachineSection raw = superstructure.stateMachine();
        registerStateEndpoint(goal, ownerName, new StateEndpoint() {
            @Override
            public void queue(Enum<?> state) {
                raw.machine().queue(state);
            }

            @Override
            public void force(Enum<?> state) {
                raw.machine().force(state);
            }

            @Override
            public boolean at(Enum<?> state) {
                return raw.machine().at(state);
            }

            @Override
            public String ownerName() {
                return ownerName;
            }
        });
    }

    private void registerStateEndpoint(
            Enum<?> defaultState,
            String ownerName,
            StateEndpoint endpoint) {
        if (endpoint == null || defaultState == null) {
            return;
        }
        Class<? extends Enum<?>> enumType = defaultState.getDeclaringClass();
        List<StateEndpoint> existing = stateEndpointsByEnum.get(enumType);
        if (existing == null) {
            existing = new ArrayList<>();
            stateEndpointsByEnum.put(enumType, existing);
        }
        StateEndpoint previousDefault = stateEndpointsByDefaultState.put(defaultState, endpoint);
        if (previousDefault != null && previousDefault != endpoint) {
            throw new IllegalStateException(
                    "State enum default state " + enumType.getName() + "." + defaultState.name()
                            + " is owned by multiple registrations: '" + previousDefault.ownerName() + "' and '"
                            + ownerName + "'.");
        }
        existing.add(endpoint);
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
     * Convenience helper to disable only Athena-managed auto NetworkTables publishing while preserving
     * manually authored NetworkTables writes from your own logic.
     */
    public RobotCore<T> disableAutoNetworkTablesPublishing() {
        robotNetworkTables.disableAutoPublishing();
        return this;
    }

    /**
     * Disable all Athena-defined NetworkTables flags (core/mechanisms/optional widgets), while preserving
     * manual user-owned NetworkTables writes.
     */
    public RobotCore<T> disableAllAthenaAutoNetworkTablesPublishing() {
        robotNetworkTables.disableAthenaPublishingFlags();
        return this;
    }

    /**
     * Convenience helper to re-enable Athena-managed auto NetworkTables publishing.
     */
    public RobotCore<T> enableAutoNetworkTablesPublishing() {
        robotNetworkTables.enableAutoPublishing();
        return this;
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

    public final class ArcpSection {
        private ArcpSection() {
        }

        public ArcpSection enabled(boolean enabled) {
            setArcpEnabled(enabled);
            return this;
        }

        public ArcpSection start() {
            setArcpEnabled(true);
            return this;
        }

        public ArcpSection stop() {
            setArcpEnabled(false);
            return this;
        }

        public ArcpSection restart() {
            setArcpEnabled(false);
            setArcpEnabled(true);
            return this;
        }

        public boolean enabled() {
            return arcpEnabled;
        }

        public ArcpSection autoMechanismPages(boolean enabled) {
            arcpAutoMechanismPages = enabled;
            if (enabled) {
                ArcpRuntime runtime = arcpRuntime;
                if (runtime != null && runtime.isRunning() && !Double.isFinite(lastArcpPublishSeconds)) {
                    arcpInitialLayoutPending = true;
                }
                refreshArcpMechanismLayoutIfNeeded();
            } else {
                arcpInitialLayoutPending = false;
            }
            return this;
        }

        public boolean autoMechanismPages() {
            return arcpAutoMechanismPages;
        }

        public ArcpSection legacyNt4MirrorEnabled(boolean enabled) {
            arcpLegacyNt4MirrorEnabled = enabled;
            if (enabled) {
                robotNetworkTables.enable(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE);
                robotNetworkTables.enable(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS);
            } else {
                applyArcpNtMirrorPolicy();
            }
            return this;
        }

        public boolean legacyNt4MirrorEnabled() {
            return arcpLegacyNt4MirrorEnabled;
        }

        public ArcpSection layoutProfileName(String profileName) {
            arcpLayoutProfileName = normalizeArcpProfileName(profileName);
            if (arcpAutoMechanismPages) {
                refreshArcpMechanismLayoutIfNeeded();
            }
            return this;
        }

        public String layoutProfileName() {
            return arcpLayoutProfileName;
        }

        public boolean running() {
            ArcpRuntime runtime = arcpRuntime;
            return runtime != null && runtime.isRunning();
        }

        public int controlPort() {
            ArcpRuntime runtime = arcpRuntime;
            return runtime != null ? runtime.controlPort() : 0;
        }

        public int realtimePort() {
            ArcpRuntime runtime = arcpRuntime;
            return runtime != null ? runtime.realtimePort() : 0;
        }

        public ArcpRuntime runtime() {
            return arcpRuntime;
        }

        public ArcpSection page(String mechanismName, Consumer<ArcpDashboardLayout.Page.Builder> section) {
            if (section == null) {
                return this;
            }
            ArcpDashboardLayout.Page.Builder builder = ArcpDashboardLayout.Page.builder();
            section.accept(builder);
            return page(mechanismName, builder.build());
        }

        public ArcpSection page(String mechanismName, ArcpDashboardLayout.Page page) {
            if (page == null) {
                return this;
            }
            String keySource = mechanismName;
            if (keySource == null || keySource.isBlank()) {
                keySource = page.name();
            }
            String key = normalizeMechanismPageKey(keySource);
            synchronized (arcpMechanismPages) {
                arcpMechanismPages.put(key, page);
            }
            refreshArcpMechanismLayoutIfNeeded();
            return this;
        }

        public ArcpSection clearPages() {
            synchronized (arcpMechanismPages) {
                arcpMechanismPages.clear();
            }
            refreshArcpMechanismLayoutIfNeeded();
            return this;
        }

        public ArcpSection publishMechanismPages() {
            writeArcpMechanismLayoutProfile(arcpLayoutProfileName);
            return this;
        }

        public ArcpSection saveLayoutProfile(String profileName, String layoutJson) {
            ArcpRuntime runtime = arcpRuntime;
            if (runtime == null || !runtime.isRunning()) {
                throw new IllegalStateException("ARCP runtime is not running");
            }
            runtime.saveLayout(normalizeArcpProfileName(profileName), layoutJson);
            return this;
        }

        public String loadLayoutProfile(String profileName) {
            ArcpRuntime runtime = arcpRuntime;
            if (runtime == null || !runtime.isRunning()) {
                throw new IllegalStateException("ARCP runtime is not running");
            }
            return runtime.loadLayout(normalizeArcpProfileName(profileName));
        }

        public String[] listLayoutProfiles() {
            ArcpRuntime runtime = arcpRuntime;
            if (runtime == null || !runtime.isRunning()) {
                return new String[0];
            }
            return runtime.listLayouts();
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
         * Resets OS-level tweaks to {@link SystemConfig#rioDefaults()} values.
         */
        public boolean resetTweaksToRioDefaults() {
            return resetSystemTweaksToRioDefaults();
        }

        /**
         * Conservative defaults for low-RAM targets without changing NT-controlled publish flags/period.
         */
        public SystemSection applyLowMemoryDefaults() {
            setConfigServerEnabled(false);
            setArcpEnabled(false);
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
            setLoopSwapEnabled(enabled, sizeMiB);
            return this;
        }

        public SystemSection loopSwapEnabled(boolean enabled) {
            setLoopSwapEnabled(enabled, SYSTEM_LOOP_SWAP_DEFAULT_SIZE_MIB);
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
                pidNode.putBoolean("translationInverted", translation.inverted());
                autoFollowerTranslationPidWidget = syncAutoFollowerPidWidget(
                        autoFollowerTranslationPidWidget,
                        translation,
                        false,
                        AUTO_FOLLOWER_TRANSLATION_PID_WIDGET_KEY);
            }
            if (rotation != null) {
                pidNode.putDouble("rotationKp", rotation.kP());
                pidNode.putDouble("rotationKi", rotation.kI());
                pidNode.putDouble("rotationKd", rotation.kD());
                pidNode.putDouble("rotationIZone", rotation.iZone());
                pidNode.putBoolean("rotationInverted", rotation.inverted());
                autoFollowerRotationPidWidget = syncAutoFollowerPidWidget(
                        autoFollowerRotationPidWidget,
                        rotation,
                        true,
                        AUTO_FOLLOWER_ROTATION_PID_WIDGET_KEY);
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
        long hash = 0xcbf29ce484222325L;
        for (int i = 0; i < poses.size(); i++) {
            Pose2d pose = poses.get(i);
            if (pose == null) {
                hash = mixTrajectoryHash(hash, i + 1L);
                continue;
            }
            hash = mixTrajectoryHash(hash, Double.doubleToLongBits(pose.getX()));
            hash = mixTrajectoryHash(hash, Double.doubleToLongBits(pose.getY()));
            hash = mixTrajectoryHash(hash, Double.doubleToLongBits(pose.getRotation().getRadians()));
        }
        return routineId + "|" + routineSource + "|" + routineReference + "|" + count + "|"
                + Long.toUnsignedString(hash, 16);
    }

    private static long mixTrajectoryHash(long hash, long value) {
        hash ^= value;
        return hash * 0x100000001b3L;
    }

    private static PIDController syncAutoFollowerPidWidget(
            PIDController existing,
            HolonomicPidConstants constants,
            boolean continuous,
            String dashboardKey) {
        if (constants == null) {
            return existing;
        }
        PIDController controller = existing;
        if (controller == null) {
            controller = new PIDController(constants.kP(), constants.kI(), constants.kD());
            SmartDashboard.putData(dashboardKey, controller);
        }
        controller.setPID(constants.kP(), constants.kI(), constants.kD());
        controller.setIZone(constants.iZone());
        if (continuous) {
            controller.enableContinuousInput(-Math.PI, Math.PI);
        } else {
            controller.disableContinuousInput();
        }
        return controller;
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

    private boolean resetAutoInitPoseIfConfigured() {
        if (localization == null) {
            return false;
        }
        RobotAuto.AutoRoutine routine = autos.selection().selected().orElse(null);
        if (routine == null) {
            return false;
        }
        Boolean override = routine.autoInitResetOverride();
        boolean shouldReset = override != null ? override : isAutoInitResetEnabled();
        if (!shouldReset) {
            return false;
        }

        Pose2d resetPose = null;
        if (routine.hasStartingPose()) {
            resetPose = routine.startingPose();
        } else {
            resetPose = autos.selection().selectedPoses()
                    .filter(poses -> poses != null && !poses.isEmpty())
                    .map(poses -> poses.get(0))
                    .orElse(null);
        }
        if (resetPose == null) {
            return false;
        }

        localization.resetPose(localization.getLocalizationConfig().autoPoseName(), resetPose);
        return true;
    }

    private void syncHeadingAxesToFieldPose() {
        RobotLocalization<?> localizationRef = localization;
        RobotDrivetrain<?> drivetrainRef = drivetrain;
        if (localizationRef == null || drivetrainRef == null) {
            return;
        }
        Imu imu = drivetrainRef.imu().device();
        if (imu == null) {
            return;
        }
        Rotation2d newFieldHeading = localizationRef.getFieldPose().getRotation();
        Rotation2d previousField = imu.getVirtualAxis("field");
        Rotation2d previousDriver = imu.getVirtualAxis("driver");
        Rotation2d previousDrift = imu.getVirtualAxis("drift");
        Rotation2d driverOffsetFromField = previousDriver.minus(previousField);
        Rotation2d driftOffsetFromField = previousDrift.minus(previousField);

        imu.setVirtualAxis("field", newFieldHeading);
        imu.setVirtualAxis("driver", newFieldHeading.plus(driverOffsetFromField));
        imu.setVirtualAxis("drift", newFieldHeading.plus(driftOffsetFromField));
    }

    private void resetDriverAxesToAllianceForward() {
        RobotDrivetrain<?> drivetrainRef = drivetrain;
        if (drivetrainRef == null) {
            return;
        }
        Imu imu = drivetrainRef.imu().device();
        if (imu == null) {
            return;
        }
        Rotation2d driverHeading = DriverStation.getAlliance()
                .filter(alliance -> alliance == DriverStation.Alliance.Red)
                .map(alliance -> Rotation2d.fromDegrees(180.0))
                .orElse(Rotation2d.kZero);
        imu.setVirtualAxis("driver", driverHeading);
        imu.setVirtualAxis("drift", driverHeading);
    }

    private void prelightSelectedAutoIfDisabled() {
        if (runtimeMode != RuntimeMode.DISABLED) {
            return;
        }
        autos.execution().prelightSelectedCommand();
    }
}
