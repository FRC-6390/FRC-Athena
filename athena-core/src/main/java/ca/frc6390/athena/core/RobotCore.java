package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.lang.management.RuntimeMXBean;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.loop.TimedControlLoopRunner;
import ca.frc6390.athena.core.loop.TimedPeriodicHookRunner;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCore<T extends RobotDrivetrain<T>> extends TimedRobot {
    private static volatile RobotCore<?> activeInstance;

    public record RobotCoreConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain,
            RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig,
            boolean autoInitResetEnabled, TelemetryRegistry.TelemetryConfig telemetryConfig,
            List<RegisterableMechanism> mechanisms, boolean performanceMode,
            boolean timingDebugEnabled, boolean telemetryEnabled, RobotCoreHooks<T> hooks) {

        public RobotCoreConfig {
            mechanisms = mechanisms != null ? List.copyOf(mechanisms) : List.of();
            hooks = hooks != null ? hooks : RobotCoreHooks.<T>empty();
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
                    RobotCoreHooks.empty());
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
                    RobotCoreHooks.empty());
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks);
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
                    hooks.hooks(section));
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
                    hooks.inputs(section));
        }

        public RobotCore<T> create() {
            return new RobotCore<>(this);
        }
    }

    private final RobotDrivetrain<T> drivetrain;
    private final RobotLocalization<?> localization;
    private final RobotVision vision;
    private final RobotNetworkTables robotNetworkTables = new RobotNetworkTables();
    private RobotVisionSim visionSim;
    private Notifier visionSimNotifier;
    private final RobotAuto autos;
    private final RobotCopilot copilot;
    private AthenaConfigServer configServer;
    private String configServerBaseUrl;
    private final HashMap<String, Mechanism> mechanisms;
    private final List<SuperstructureMechanism<?, ?>> registeredSuperstructures;
    private final HashMap<String, SuperstructureMechanism<?, ?>> superstructuresByName;
    private final RobotMechanisms mechanismView;
    private final Set<Mechanism> scheduledCustomPidMechanisms;
    private Command autonomousCommand;
    private final TelemetryRegistry telemetry;
    private boolean autoInitResetEnabled;
    private NetworkTableEntry autoInitResetEntry;
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
    private int mechanismPublishCursor = 0;
    private int superstructurePublishCursor = 0;
    private int remainingMechanismRefreshCount = 0;
    private int remainingSuperstructureRefreshCount = 0;
    private long lastMechanismsConfigRevision = -1;
    private double lastMechanismAutoPublishSeconds = Double.NaN;
    private long lastCoreNetworkTablesConfigRevision = -1;
    private double lastCoreNetworkTablesPublishSeconds = Double.NaN;
    private final RuntimeMXBean runtimeMxBean;
    private final MemoryMXBean memoryMxBean;
    private final java.lang.management.OperatingSystemMXBean osMxBean;
    private final com.sun.management.OperatingSystemMXBean sunOsMxBean;
    private final List<GarbageCollectorMXBean> gcMxBeans;
    private double lastLoopSchedulerMs = Double.NaN;
    private double lastLoopTelemetryMs = Double.NaN;
    private double lastLoopLocalizationMs = Double.NaN;
    private double lastLoopUserMs = Double.NaN;
    private double lastLoopTotalMs = Double.NaN;
    private final RobotCoreHooks<T> coreHooks;
    private final RobotCoreContextImpl coreHookContext;
    private final TypedInputResolver hookInputResolver;
    private final List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> periodicHookRunners;
    private final List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> disabledPeriodicHookRunners;
    private final List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> teleopPeriodicHookRunners;
    private final List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> autonomousPeriodicHookRunners;
    private final List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> testPeriodicHookRunners;
    private final List<TimedControlLoopRunner<RobotCoreHooks.ControlLoop<T>>> coreControlLoopRunners;
    private final Map<String, PIDController> coreControlLoopPids;
    private final Map<String, OutputType> coreControlLoopPidOutputTypes;
    private final Map<String, SimpleMotorFeedforward> coreControlLoopFeedforwards;
    private final Map<String, OutputType> coreControlLoopFeedforwardOutputTypes;
    private static final double MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS = 0.05;
    private static final int MECHANISM_AUTO_PUBLISH_BATCH_SIZE = 2;
    private static final double STARTUP_LOG_THRESHOLD_SECONDS = 0.05;

    public RobotCore(RobotCoreConfig<T> config) {
        double constructorStart = Timer.getFPGATimestamp();
        activeInstance = this;
        drivetrain = timedStartupStep("constructor.driveTrain.build", () -> config.driveTrain.build());
        localization = timedStartupStep("constructor.localization.create",
                () -> drivetrain.localization(config.localizationConfig()));
        vision = timedStartupStep("constructor.vision.create", () -> RobotVision.fromConfig(config.visionConfig));
        autos = new RobotAuto().attachRobotCore(this);
        copilot = new RobotCopilot(drivetrain.get(), localization, RobotCopilot.inferDriveStyle(drivetrain.get()));
        mechanisms = new HashMap<>();
        registeredSuperstructures = new ArrayList<>();
        superstructuresByName = new HashMap<>();
        mechanismView = new RobotMechanisms(mechanisms, superstructuresByName, registeredSuperstructures);
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
        configuredMechanisms = config.mechanisms() != null ? List.copyOf(config.mechanisms()) : List.of();
        configuredMechanismsRegistered = false;
        timingDebugEnabled = config.timingDebugEnabled();
        telemetryEnabled = config.telemetryEnabled();
        if (config.performanceMode()) {
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

        if (localization != null) {
            localization.attachRobotNetworkTables(robotNetworkTables);
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
            String autoPose = config.localizationConfig().autoPoseName();
            autos.setAutoPlanResetter(pose -> Commands.runOnce(() -> localization.resetPose(autoPose, pose)));
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
        double robotInitStart = Timer.getFPGATimestamp();
        timedStartupStep("robotInit.registerConfiguredMechanisms", this::registerConfiguredMechanisms);
        timedStartupStep("robotInit.startConfigServerIfNeeded", this::startConfigServerIfNeeded);
        timedStartupStep("robotInit.configureAutos", () -> configureAutos(autos));
        timedStartupStep("robotInit.autos.finalizeRegistration", autos::finalizeRegistration);
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
            for (SuperstructureMechanism<?, ?> superstructure : top.flattenSuperstructures()) {
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
            for (SuperstructureMechanism<?, ?> superstructure : top.flattenSuperstructures()) {
                if (superstructure == null || !visited.add(superstructure)) {
                    continue;
                }
                superstructure.runLifecycleHooks(phase);
            }
        }
    }

    private void startConfigServerIfNeeded() {
        if (configServer != null) {
            return;
        }
        try {
            configServer = AthenaConfigServer.start(this, 5806);
            configServerBaseUrl = configServer.baseUrl();
        } catch (Exception e) {
            // Config export is a convenience feature; do not fail robot init if it cannot bind.
            System.out.println("[Athena][ConfigServer] Failed to start: " + e.getMessage());
            configServer = null;
            configServerBaseUrl = null;
        }
    }

    /**
     * Returns the config server base URL (if available) used to build per-mechanism download links.
     */
    public String getConfigServerBaseUrl() {
        return configServerBaseUrl;
    }

    @Override
    public final void robotPeriodic() {
        LoopTiming.beginCycle();
        robotNetworkTables.refresh();

        // Auto publish when enabled at runtime from Athena/NetworkTableConfig.
        if (robotNetworkTables.isPublishingEnabled()) {
            if (robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE)) {
                double now = Timer.getFPGATimestamp();
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
                autoPublishMechanismsIncremental();
            }
        }

        double t0 = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double t1 = Timer.getFPGATimestamp();
        telemetry.tick();
        double t2 = Timer.getFPGATimestamp();
        if (localization != null && !DriverStation.isDisabled()) {
            localization.updateAutoVisualization(autos);
        }
        double t3 = Timer.getFPGATimestamp();
        runCoreControlLoops();
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.ROBOT_PERIODIC);
        runCorePeriodicBindings(
                RobotCoreHooks.Phase.ROBOT_PERIODIC,
                coreHooks.periodicBindings(),
                periodicHookRunners);
        onRobotPeriodic();
        double t4 = Timer.getFPGATimestamp();
        lastLoopSchedulerMs = (t1 - t0) * 1000.0;
        lastLoopTelemetryMs = (t2 - t1) * 1000.0;
        lastLoopLocalizationMs = (t3 - t2) * 1000.0;
        lastLoopUserMs = (t4 - t3) * 1000.0;
        lastLoopTotalMs = (t4 - t0) * 1000.0;
        LoopTiming.endCycle(t0, t1, t2, t3, t4);
    }

    private void autoPublishMechanismsIncremental() {
        double now = RobotTime.nowSeconds();
        if (!Double.isFinite(now)) {
            now = Timer.getFPGATimestamp();
        }
        if (Double.isFinite(lastMechanismAutoPublishSeconds)
                && (now - lastMechanismAutoPublishSeconds) < MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS) {
            return;
        }
        lastMechanismAutoPublishSeconds = now;

        mechanismsNetworkTablesPublished = true;
        RobotNetworkTables.Node mechanismsNode = robotNetworkTables.root().child("Mechanisms");
        RobotNetworkTables.Node supersNode = robotNetworkTables.root().child("Superstructures");

        long revision = robotNetworkTables.revision();
        if (revision != lastMechanismsConfigRevision) {
            lastMechanismsConfigRevision = revision;
            remainingMechanismRefreshCount = Math.max(remainingMechanismRefreshCount, mechanisms.size());
            remainingSuperstructureRefreshCount =
                    Math.max(remainingSuperstructureRefreshCount, superstructuresByName.size());
        }

        publishMechanismBatch(mechanismsNode);
        publishSuperstructureBatch(supersNode);
    }

    private void publishMechanismBatch(RobotNetworkTables.Node mechanismsNode) {
        mechanismPublishKeysBuffer.clear();
        mechanismPublishKeysBuffer.addAll(mechanisms.keySet());
        if (mechanismPublishKeysBuffer.isEmpty()) {
            publishedMechanismsComp.clear();
            mechanismPublishCursor = 0;
            remainingMechanismRefreshCount = 0;
            return;
        }
        publishedMechanismsComp.retainAll(mechanismPublishKeysBuffer);

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
            mech.networkTables(mech.resolveDefaultMechanismNode(mechanismsNode));
            publishedMechanismsComp.add(name);
            if (needsRefresh) {
                remainingMechanismRefreshCount--;
            }
        }
        if (remainingMechanismRefreshCount < 0) {
            remainingMechanismRefreshCount = 0;
        }
    }

    private void publishSuperstructureBatch(RobotNetworkTables.Node supersNode) {
        superstructurePublishKeysBuffer.clear();
        superstructurePublishKeysBuffer.addAll(superstructuresByName.keySet());
        if (superstructurePublishKeysBuffer.isEmpty()) {
            publishedSuperstructuresComp.clear();
            superstructurePublishCursor = 0;
            remainingSuperstructureRefreshCount = 0;
            return;
        }
        publishedSuperstructuresComp.retainAll(superstructurePublishKeysBuffer);

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
            superstructure.networkTables(supersNode.child(key));
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
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.TELEOP_INIT);
        resetPeriodicRunners(teleopPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.TELEOP_INIT, coreHooks.teleopInitBindings());
        onTeleopInit();
    }

    @Override
    public final void teleopExit() {
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
        cancelAutonomousCommand();
        prepareDrivetrainForModeTransition(true, false);
        runRegisteredPhaseHooks(RobotCoreHooks.Phase.DISABLED_INIT);
        resetPeriodicRunners(disabledPeriodicHookRunners);
        runCorePhaseBindings(RobotCoreHooks.Phase.DISABLED_INIT, coreHooks.disabledInitBindings());
        onDisabledInit();
    }

    @Override
    public final void disabledExit() {
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

    private List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> createPeriodicHookRunners(
            List<RobotCoreHooks.PeriodicHookBinding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return new ArrayList<>();
        }
        List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> runners = new ArrayList<>(bindings.size());
        for (RobotCoreHooks.PeriodicHookBinding<T> binding : bindings) {
            if (binding == null || binding.hook() == null) {
                continue;
            }
            runners.add(new TimedPeriodicHookRunner<>(binding.hook(), binding.periodMs()));
        }
        return runners;
    }

    private List<TimedControlLoopRunner<RobotCoreHooks.ControlLoop<T>>> createCoreControlLoopRunners(
            List<RobotCoreHooks.ControlLoopBinding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return new ArrayList<>();
        }
        List<TimedControlLoopRunner<RobotCoreHooks.ControlLoop<T>>> runners = new ArrayList<>(bindings.size());
        for (RobotCoreHooks.ControlLoopBinding<T> binding : bindings) {
            if (binding == null || binding.loop() == null || binding.name() == null || binding.name().isBlank()) {
                continue;
            }
            runners.add(new TimedControlLoopRunner<>(binding.name(), binding.periodSeconds(), binding.loop()));
        }
        return runners;
    }

    private void runCoreControlLoops() {
        if (coreControlLoopRunners == null || coreControlLoopRunners.isEmpty()) {
            return;
        }
        double nowSeconds = RobotTime.nowSeconds();
        if (!Double.isFinite(nowSeconds)) {
            nowSeconds = Timer.getFPGATimestamp();
        }
        coreHookContext.setPhase(RobotCoreHooks.Phase.ROBOT_PERIODIC);
        for (TimedControlLoopRunner<RobotCoreHooks.ControlLoop<T>> runner : coreControlLoopRunners) {
            if (runner == null || !runner.shouldRun(nowSeconds)) {
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
            runner.setLastOutput(runner.loop().calculate(coreHookContext));
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
            List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> runners) {
        coreHookContext.setPhase(phase);
        runCoreBindings(bindings);
        if (runners == null || runners.isEmpty()) {
            return;
        }
        double nowMs = hookNowMs();
        for (TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>> runner : runners) {
            if (runner == null || !runner.shouldRun(nowMs)) {
                continue;
            }
            runner.run(hook -> hook.apply(coreHookContext), nowMs);
        }
    }

    private void runCoreBindings(List<RobotCoreHooks.Binding<T>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return;
        }
        for (RobotCoreHooks.Binding<T> binding : bindings) {
            if (binding == null) {
                continue;
            }
            binding.apply(coreHookContext);
        }
    }

    private void resetPeriodicRunners(List<TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>>> runners) {
        if (runners == null || runners.isEmpty()) {
            return;
        }
        for (TimedPeriodicHookRunner<RobotCoreHooks.Binding<T>> runner : runners) {
            if (runner != null) {
                runner.reset();
            }
        }
    }

    private static double hookNowMs() {
        double nowSeconds = RobotTime.nowSeconds();
        if (!Double.isFinite(nowSeconds)) {
            nowSeconds = Timer.getFPGATimestamp();
        }
        return nowSeconds * 1000.0;
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

    public static RobotCore<?> getActiveInstance() {
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

    protected Command createAutonomousCommand() {
        return autos.buildSelectedCommand().orElse(null);
    }

    private void prepareDrivetrainForModeTransition(boolean driveEnabled, boolean autoEnabled) {
        drivetrain.resetDriveState();
        RobotSpeeds speeds = drivetrain.getRobotSpeeds();
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
        getDrivetrain().getRobotSpeeds().stopSpeeds("auto");
    }

    private void scheduleCustomPidCycle(Mechanism mechanism) {
        if (!mechanism.isCustomPIDCycle()) {
            return;
        }
        if (!scheduledCustomPidMechanisms.add(mechanism)) {
            return;
        }
        double period = mechanism.getPidPeriod();
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
                    mech.resolveDefaultMechanismNode(robotNetworkTables.root().child("Mechanisms"));
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
            // Ensure per-superstructure NetworkTableConfig entries exist under Athena/Superstructures.
            RobotNetworkTables.Node superNode =
                    robotNetworkTables.root().child("Superstructures").child(s.getName() != null ? s.getName() : "Superstructure");
            robotNetworkTables.mechanismConfig(superNode);
            s.setRobotCore(this);
            // Publish mechanisms under the owning superstructure name to avoid duplicates and
            // keep dashboards navigable at scale.
            s.assignDashboardOwners(s.getName());
            registerMechanism(s.getMechanisms().all().toArray(Mechanism[]::new));
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
                        robotNetworkTables.root().child("Superstructures").child(superstructure.getName() != null ? superstructure.getName() : "Superstructure");
                robotNetworkTables.mechanismConfig(superNode);
                superstructure.setRobotCore(this);
                superstructure.assignDashboardOwners(superstructure.getName());
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

    /**
     * Returns the registered mechanism with the given name, or {@code null} if not found.
     */
    public Mechanism getMechanism(String name) {
        return mechanisms.get(name);
    }

    /**
     * Returns the first registered mechanism assignable to the requested type, or {@code null}.
     */
    public <M extends Mechanism> M getMechanism(Class<M> type) {
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
     * Returns an unmodifiable view of all registered mechanisms keyed by name.
     */
    public RobotMechanisms getMechanisms() {
        return mechanismView;
    }

    /**
     * Convenience alias for {@link #getMechanisms()} to enable fluent access patterns like
     * {@code robot.getMechanism().turret("Turret")}.
     */
    public RobotMechanisms getMechanism() {
        return mechanismView;
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
        return candidate;
    }

    public RobotLocalization<?> getLocalization() {
        return localization;
    }

    public RobotCopilot getCopilot() {
        return copilot;
    }

    public T getDrivetrain() {
        return drivetrain.get();
    }

    public RobotVision getVision() {
        return vision;
    }

    /**
     * IntelliSense-friendly facade over Athena NetworkTables publishing controls (global flags).
     *
     * <p>Per-mechanism controls live on each mechanism under
     * {@code Athena/Mechanisms/.../<MechName>/NetworkTableConfig/...} and can be accessed via
     * {@code mech.networkTablesConfig()}.</p>
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
        publishPerformanceMetrics(root);

        coreNetworkTablesPublished = true;
        return this;
    }

    private void publishConfigExportUrls(RobotNetworkTables.Node root) {
        if (root == null) {
            return;
        }
        RobotNetworkTables.Node cfg = root.child("Config");
        String base = configServerBaseUrl;
        cfg.putString("baseUrl", base != null ? base : "");
        root.putString("configBaseUrl", base != null ? base : "");
        if (base == null || base.isBlank()) {
            cfg.putString("indexUrlJson", "");
            cfg.putString("indexUrlToml", "");
            cfg.putString("allZipUrl", "");
            cfg.putString("mechanismsBaseUrl", "");
            cfg.putString("autoLogUrl", "");
            root.putString("configIndexUrlJson", "");
            root.putString("configIndexUrlToml", "");
            root.putString("configAllZipUrl", "");
            root.putString("configMechanismsBaseUrl", "");
            root.putString("configAutoLogUrl", "");
            return;
        }
        cfg.putString("indexUrlJson", base + "/Athena/config/index.json");
        cfg.putString("indexUrlToml", base + "/Athena/config/index.toml");
        cfg.putString("allZipUrl", base + "/Athena/config/all.zip");
        cfg.putString("mechanismsBaseUrl", base + "/Athena/config/mechanisms/");
        cfg.putString("autoLogUrl", base + "/Athena/auto/log");
        root.putString("configIndexUrlJson", base + "/Athena/config/index.json");
        root.putString("configIndexUrlToml", base + "/Athena/config/index.toml");
        root.putString("configAllZipUrl", base + "/Athena/config/all.zip");
        root.putString("configMechanismsBaseUrl", base + "/Athena/config/mechanisms/");
        root.putString("configAutoLogUrl", base + "/Athena/auto/log");
    }

    private void publishPerformanceMetrics(RobotNetworkTables.Node root) {
        if (root == null) {
            return;
        }

        RobotNetworkTables.Node performance = root.child("Performance");
        RobotNetworkTables.Node status = performance.child("Status");
        status.putBoolean("isReal", RobotBase.isReal());
        status.putBoolean("isSimulation", RobotBase.isSimulation());
        status.putBoolean("isDSAttached", DriverStation.isDSAttached());
        status.putBoolean("isFMSAttached", DriverStation.isFMSAttached());
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

        RobotNetworkTables.Node loop = performance.child("Loop");
        loop.putDouble("periodSec", getPeriod());
        loop.putDouble("budgetMs", getPeriod() * 1000.0);
        loop.putDouble("schedulerMs", finiteOr(lastLoopSchedulerMs, -1.0));
        loop.putDouble("telemetryMs", finiteOr(lastLoopTelemetryMs, -1.0));
        loop.putDouble("localizationMs", finiteOr(lastLoopLocalizationMs, -1.0));
        loop.putDouble("userMs", finiteOr(lastLoopUserMs, -1.0));
        loop.putDouble("totalMs", finiteOr(lastLoopTotalMs, -1.0));
        loop.putDouble("utilizationPercent", ratioPercent(lastLoopTotalMs, getPeriod() * 1000.0));
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

    /**
     * Publishes all registered mechanisms under {@code Athena/Mechanisms/...}.
     */
    public RobotCore<T> publishNetworkTablesMechanisms() {
        if (!robotNetworkTables.isPublishingEnabled()) {
            return this;
        }
        mechanismsNetworkTablesPublished = true;

        RobotNetworkTables.Node mechanismsNode = robotNetworkTables.root().child("Mechanisms");
        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            mech.networkTables(mech.resolveDefaultMechanismNode(mechanismsNode));
        }

        RobotNetworkTables.Node supersNode = robotNetworkTables.root().child("Superstructures");
        for (Map.Entry<String, SuperstructureMechanism<?, ?>> e : superstructuresByName.entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            e.getValue().networkTables(supersNode.child(e.getKey()));
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
                getDrivetrain().getIMU().getYaw().plus(Rotation2d.fromDegrees(degrees)));
    }

    public RobotSpeeds getRobotSpeeds() {
        return drivetrain.getRobotSpeeds();
    }

    public Imu getIMU() {
        return drivetrain.getIMU();
    }

    public SendableChooser<Command> registerAutoChooser(RobotAuto.AutoKey defaultAuto) {
        SendableChooser<Command> chooser = autos.createCommandChooser(defaultAuto);
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(RobotAuto.AutoKey defaultAuto) {
        SendableChooser<RobotAuto.AutoRoutine> chooser = autos.createChooser(defaultAuto);
        SmartDashboard.putData("Auto Routine Chooser", chooser);
        return chooser;
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(String defaultAuto) {
        return registerAutoRoutineChooser(RobotAuto.AutoKey.of(defaultAuto));
    }

    public SendableChooser<Command> registerAutoChooser(String defaultsAuto) {
        return registerAutoChooser(RobotAuto.AutoKey.of(defaultsAuto));
    }

    public RobotAuto getAutos() {
        return autos;
    }

    private void ensureAutoChooserPublished() {
        if (autos.getAutoChooser() != null || autos.getCommandChooser() != null) {
            return;
        }
        autos.getAutos().stream()
                .findFirst()
                .map(RobotAuto.AutoRoutine::key)
                .ifPresent(this::registerAutoChooser);
    }

    public void registerPIDCycles() {
        mechanisms.values().forEach(this::scheduleCustomPidCycle);
    }

    public void resetPIDs() {
        for (Mechanism mech : mechanisms.values()) {
            mech.resetPID();
        }
    }

    public ChassisSpeeds createRobotRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getFieldPose().getRotation());
    }

    public ChassisSpeeds createFieldRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getFieldPose().getRotation());
    }

    private boolean isAutoInitResetEnabled() {
        if (autoInitResetEntry == null) {
            autoInitResetEntry = SmartDashboard.getEntry("Auto/Reset Odometry On Init");
            autoInitResetEntry.setBoolean(autoInitResetEnabled);
        }
        return autoInitResetEntry.getBoolean(autoInitResetEnabled);
    }

    private void resetAutoInitPoseIfConfigured() {
        if (localization == null) {
            return;
        }
        autos.getSelectedAuto().ifPresent(routine -> {
            Boolean override = routine.autoInitResetOverride();
            boolean shouldReset = override != null ? override : isAutoInitResetEnabled();
            if (!shouldReset || !routine.hasStartingPose()) {
                return;
            }
            localization.resetPose(localization.getLocalizationConfig().autoPoseName(), routine.startingPose());
        });
    }
}
