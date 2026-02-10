package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.localization.RobotLocalization;
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
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
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
            boolean timingDebugEnabled, boolean telemetryEnabled) {

        public static RobotCoreConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config) {
            return new RobotCoreConfig<>(
                    config,
                    RobotLocalizationConfig.defualt(),
                    RobotVisionConfig.defualt(),
                    true,
                    TelemetryRegistry.TelemetryConfig.defualt(),
                    List.of(),
                    false,
                    false,
                    true);
        }

        public static RobotCoreConfig<DifferentialDrivetrain> differential(DifferentialDrivetrainConfig config) {
            return new RobotCoreConfig<>(
                    config,
                    RobotLocalizationConfig.defualt(),
                    RobotVisionConfig.defualt(),
                    true,
                    TelemetryRegistry.TelemetryConfig.defualt(),
                    List.of(),
                    false,
                    false,
                    true);
        }

        public RobotCoreConfig<T> setLocalization(RobotLocalizationConfig localizationConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setVision(RobotVisionConfig visionConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setVision(ConfigurableCamera... cameras) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    RobotVisionConfig.defualt().addCameras(cameras),
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setAutoInitResetEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    enabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setTelemetry(TelemetryRegistry.TelemetryConfig telemetryConfig) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled);
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
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setPerformanceMode(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    enabled,
                    timingDebugEnabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setTimingDebugEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    enabled,
                    telemetryEnabled);
        }

        public RobotCoreConfig<T> setTelemetryEnabled(boolean enabled) {
            return new RobotCoreConfig<>(
                    driveTrain,
                    localizationConfig,
                    visionConfig,
                    autoInitResetEnabled,
                    telemetryConfig,
                    mechanisms,
                    performanceMode,
                    timingDebugEnabled,
                    enabled);
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
    private long lastMechanismsConfigRevision = -1;
    private double lastMechanismAutoPublishSeconds = Double.NaN;
    private long lastCoreNetworkTablesConfigRevision = -1;
    private double lastCoreNetworkTablesPublishSeconds = Double.NaN;
    private static final double MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS = 0.5;

    public RobotCore(RobotCoreConfig<T> config) {
        activeInstance = this;
        drivetrain = config.driveTrain.build();
        localization = drivetrain.localization(config.localizationConfig());
        vision = RobotVision.fromConfig(config.visionConfig);
        autos = new RobotAuto();
        copilot = new RobotCopilot(drivetrain.get(), localization, RobotCopilot.inferDriveStyle(drivetrain.get()));
        mechanisms = new HashMap<>();
        registeredSuperstructures = new ArrayList<>();
        superstructuresByName = new HashMap<>();
        mechanismView = new RobotMechanisms(mechanisms, superstructuresByName, registeredSuperstructures);
        scheduledCustomPidMechanisms = new HashSet<>();
        publishedMechanismsComp = new HashSet<>();
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

        if (localization != null) {
            localization.attachRobotNetworkTables(robotNetworkTables);
            if (vision != null) {
                localization.setRobotVision(vision);
            }
        }
        // Choreo auto configuration should not depend on vision. Only attempt configuration when the
        // backend exists to avoid warning spam for projects that do not include Athena Choreo.
        if (localization != null && AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).isPresent()) {
            localization.configureChoreo(drivetrain);
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
        registerConfiguredMechanisms();
    }

    @Override
    public final void robotInit() {
        registerConfiguredMechanisms();
        startConfigServerIfNeeded();
        configureAutos(autos);
        autos.finalizeRegistration();
        ensureAutoChooserPublished();
        robotNetworkTables.publishConfig();
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_CORE)) {
            publishNetworkTables();
        }
        if (robotNetworkTables.isPublishingEnabled() && robotNetworkTables.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS)) {
            publishNetworkTablesMechanisms();
        }
        onRobotInit();
        registerPIDCycles();
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
        onRobotPeriodic();
        double t4 = Timer.getFPGATimestamp();
        LoopTiming.endCycle(t0, t1, t2, t3, t4);
    }

    private void autoPublishMechanismsIncremental() {
        double now = Timer.getFPGATimestamp();
        if (Double.isFinite(lastMechanismAutoPublishSeconds)
                && (now - lastMechanismAutoPublishSeconds) < MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS) {
            return;
        }
        lastMechanismAutoPublishSeconds = now;

        mechanismsNetworkTablesPublished = true;
        RobotNetworkTables.Node mechanismsNode = robotNetworkTables.root().child("Mechanisms");
        RobotNetworkTables.Node supersNode = robotNetworkTables.root().child("Superstructures");

        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            String name = mech.getName();
            if (publishedMechanismsComp.contains(name)) {
                continue;
            }
            mech.networkTables(mech.resolveDefaultMechanismNode(mechanismsNode));
            publishedMechanismsComp.add(name);
        }

        for (Map.Entry<String, SuperstructureMechanism<?, ?>> e : superstructuresByName.entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            e.getValue().networkTables(supersNode.child(e.getKey()));
        }

        long revision = robotNetworkTables.revision();
        if (revision != lastMechanismsConfigRevision) {
            lastMechanismsConfigRevision = revision;
            for (String published : publishedMechanismsComp) {
                Mechanism mech = mechanisms.get(published);
                if (mech == null) {
                    continue;
                }
                mech.networkTables(mech.resolveDefaultMechanismNode(mechanismsNode));
            }
            for (Map.Entry<String, SuperstructureMechanism<?, ?>> e : superstructuresByName.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                e.getValue().networkTables(supersNode.child(e.getKey()));
            }
        }
    }

    @Override
    public final void autonomousInit() {
        drivetrain.getRobotSpeeds().setSpeedSourceState("drive", false);
        drivetrain.getRobotSpeeds().stopSpeeds("drive");
        drivetrain.getRobotSpeeds().setSpeedSourceState("auto", true);
        drivetrain.getRobotSpeeds().stopSpeeds("auto");
        resetAutoInitPoseIfConfigured();
        scheduleAutonomousCommand();
        onAutonomousInit();
    }

    @Override
    public final void autonomousExit() {
        onAutonomousExit();
    }

    @Override
    public final void autonomousPeriodic() {
        if (autonomousCommand != null
                && !CommandScheduler.getInstance().isScheduled(autonomousCommand)) {
            drivetrain.getRobotSpeeds().stopSpeeds("auto");
        }
        onAutonomousPeriodic();
    }

    @Override
    public final void teleopInit() {
        cancelAutonomousCommand();
        drivetrain.getRobotSpeeds().setSpeedSourceState("drive", true);
        drivetrain.getRobotSpeeds().setSpeedSourceState("auto", false);
        drivetrain.getRobotSpeeds().stopSpeeds("auto");
        onTeleopInit();
    }

    @Override
    public final void teleopExit() {
        onTeleopExit();
    }

    @Override
    public final void teleopPeriodic() {
        onTeleopPeriodic();
    }

    @Override
    public final void disabledInit() {
        cancelAutonomousCommand();
        drivetrain.getRobotSpeeds().setSpeedSourceState("drive", true);
        drivetrain.getRobotSpeeds().setSpeedSourceState("auto", false);
        drivetrain.getRobotSpeeds().stopSpeeds("auto");
        onDisabledInit();
    }

    @Override
    public final void disabledExit() {
        onDisabledExit();
    }

    @Override
    public final void disabledPeriodic() {
        onDisabledPeriodic();
    }

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
        cancelAutonomousCommand();
        onTestInit();
    }

    @Override
    public final void testExit() {
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

    protected void onRobotInit() {}

    protected void onRobotPeriodic() {}

    public RobotCore<T> setAutoInitResetEnabled(boolean enabled) {
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
            root.putString("configIndexUrlJson", "");
            root.putString("configIndexUrlToml", "");
            root.putString("configAllZipUrl", "");
            root.putString("configMechanismsBaseUrl", "");
            return;
        }
        cfg.putString("indexUrlJson", base + "/Athena/config/index.json");
        cfg.putString("indexUrlToml", base + "/Athena/config/index.toml");
        cfg.putString("allZipUrl", base + "/Athena/config/all.zip");
        cfg.putString("mechanismsBaseUrl", base + "/Athena/config/mechanisms/");
        root.putString("configIndexUrlJson", base + "/Athena/config/index.json");
        root.putString("configIndexUrlToml", base + "/Athena/config/index.toml");
        root.putString("configAllZipUrl", base + "/Athena/config/all.zip");
        root.putString("configMechanismsBaseUrl", base + "/Athena/config/mechanisms/");
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

    public SendableChooser<Command> registerAutoChooser(String defualtAuto) {
        return registerAutoChooser(RobotAuto.AutoKey.of(defualtAuto));
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
