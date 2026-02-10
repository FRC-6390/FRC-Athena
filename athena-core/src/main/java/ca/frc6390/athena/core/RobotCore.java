package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
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
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.dashboard.ShuffleboardControls;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private RobotVisionSim visionSim;
    private Notifier visionSimNotifier;
    private final RobotAuto autos;
    private final RobotCopilot copilot;
    private final HashMap<String, Mechanism> mechanisms;
    private final Set<Mechanism> scheduledCustomPidMechanisms;
    private Command autonomousCommand;
    private final TelemetryRegistry telemetry;
    private boolean autoInitResetEnabled;
    private NetworkTableEntry autoInitResetEntry;
    private final List<RegisterableMechanism> configuredMechanisms;
    private final boolean timingDebugEnabled;
    private final boolean telemetryEnabled;
    private boolean mechanismsShuffleboardPublished;
    private boolean coreShuffleboardPublished;
    private boolean debugShuffleboardPublished;
    private boolean debugMechanismsShuffleboardPublished;
    private final Set<String> publishedMechanismsComp;
    private final Set<String> publishedMechanismsDebug;
    private double lastMechanismAutoPublishSeconds = Double.NaN;
    private static final double MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS = 0.5;

    public RobotCore(RobotCoreConfig<T> config) {
        activeInstance = this;
        drivetrain = config.driveTrain.build();
        localization = drivetrain.localization(config.localizationConfig());
        vision = RobotVision.fromConfig(config.visionConfig);
        autos = new RobotAuto();
        copilot = new RobotCopilot(drivetrain.get(), localization, RobotCopilot.inferDriveStyle(drivetrain.get()));
        mechanisms = new HashMap<>();
        scheduledCustomPidMechanisms = new HashSet<>();
        publishedMechanismsComp = new HashSet<>();
        publishedMechanismsDebug = new HashSet<>();
        autonomousCommand = null;
        telemetry = TelemetryRegistry.create(config.telemetryConfig());
        autoInitResetEnabled = config.autoInitResetEnabled();
        configuredMechanisms = config.mechanisms() != null ? List.copyOf(config.mechanisms()) : List.of();
        timingDebugEnabled = config.timingDebugEnabled();
        telemetryEnabled = config.telemetryEnabled();
        if (config.performanceMode()) {
            RobotSendableSystem.setShuffleboardEnabled(false);
            telemetry.setEnabled(false);
        }
        if (!telemetryEnabled) {
            telemetry.setEnabled(false);
        }
        LoopTiming.setDebugAlways(timingDebugEnabled);
        mechanismsShuffleboardPublished = false;
        coreShuffleboardPublished = false;
        debugShuffleboardPublished = false;
        debugMechanismsShuffleboardPublished = false;

        if (localization != null && vision != null) {
            localization.setRobotVision(vision);
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
    }

    @Override
    public final void robotInit() {
        registerConfiguredMechanisms();
        configureAutos(autos);
        ensureAutoChooserPublished();
        ShuffleboardControls.publishConfigTab();
        if (RobotSendableSystem.isShuffleboardEnabled() && ShuffleboardControls.enabled(ca.frc6390.athena.dashboard.ShuffleboardControls.Flag.AUTO_PUBLISH_CORE)) {
            shuffleboard(SendableLevel.COMP);
        }
        if (RobotSendableSystem.isShuffleboardEnabled() && ShuffleboardControls.enabled(ca.frc6390.athena.dashboard.ShuffleboardControls.Flag.AUTO_PUBLISH_MECHANISMS)) {
            shuffleboardMechanisms(SendableLevel.COMP);
        }
        onRobotInit();
        registerPIDCycles();
    }

    @Override
    public final void robotPeriodic() {
        LoopTiming.beginCycle();
        ShuffleboardControls.refresh();

        // Auto publish when enabled at runtime from Athena/Config.
        if (RobotSendableSystem.isShuffleboardEnabled()) {
            if (!coreShuffleboardPublished
                    && ShuffleboardControls.enabled(ca.frc6390.athena.dashboard.ShuffleboardControls.Flag.AUTO_PUBLISH_CORE)) {
                shuffleboard(SendableLevel.COMP);
            }
            if (ShuffleboardControls.enabled(ca.frc6390.athena.dashboard.ShuffleboardControls.Flag.AUTO_PUBLISH_MECHANISMS)) {
                autoPublishMechanismsIncremental(SendableLevel.COMP);
            }
        }

        // DEBUG gate is runtime-controlled from Athena/Config, but we intentionally do not auto-publish
        // DEBUG dashboards here. Auto-publishing DEBUG after COMP can easily double-add titles to the
        // same Shuffleboard containers (WPILib throws), and it also spikes CPU on large robots.

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

    private void autoPublishMechanismsIncremental(SendableLevel level) {
        double now = Timer.getFPGATimestamp();
        if (Double.isFinite(lastMechanismAutoPublishSeconds)
                && (now - lastMechanismAutoPublishSeconds) < MECHANISM_AUTO_PUBLISH_PERIOD_SECONDS) {
            return;
        }
        lastMechanismAutoPublishSeconds = now;

        // Mark as "published" so DEBUG enabling later can opt into mechanism publishing paths.
        mechanismsShuffleboardPublished = true;
        if (level == SendableLevel.DEBUG) {
            debugMechanismsShuffleboardPublished = true;
        }

        ShuffleboardTab tab = Shuffleboard.getTab(RobotSendableSystem.SHUFFLEBOARD_ROOT + "/Mechanisms");
        ShuffleboardLayout list = tab.getLayout("All", BuiltInLayouts.kList);

        Set<String> published = (level == SendableLevel.DEBUG) ? publishedMechanismsDebug : publishedMechanismsComp;
        for (Mechanism mech : mechanisms.values()) {
            if (mech == null) {
                continue;
            }
            String name = mech.getName();
            if (published.contains(name)) {
                continue;
            }
            if (!ShuffleboardControls.mechanismAllowed(name, level)) {
                continue;
            }
            mech.shuffleboard(list.getLayout(name, BuiltInLayouts.kList), level);
            published.add(name);
            if (level == SendableLevel.DEBUG) {
                // DEBUG publish implies COMP container was also created.
                publishedMechanismsComp.add(name);
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
            mechanisms.put(mech.getName(), mech);
            ShuffleboardControls.registerMechanism(mech.getName());
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
            s.setRobotCore(this);
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
            if (entry instanceof SuperstructureMechanism<?, ?> superstructure) {
                superstructure.setRobotCore(this);
            }
            registerMechanism(entry.flattenForRegistration().toArray(Mechanism[]::new));
        }
        return this;
    }

    private void registerConfiguredMechanisms() {
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
    public java.util.Map<String, Mechanism> getMechanisms() {
        return Collections.unmodifiableMap(mechanisms);
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

    public RobotCore<T> shuffleboard() {
        return shuffleboard("Drivetrain");
    }

    public RobotCore<T> shuffleboard(String drive) {
        return shuffleboard(drive, "Localization");
    }

    public RobotCore<T> shuffleboard(SendableLevel level) {
        return shuffleboard("Drivetrain", level);
    }

    public RobotCore<T> shuffleboard(String drive, SendableLevel level) {
        return shuffleboard(drive, "Localization", level);
    }

    public RobotCore<T> shuffleboard(String drive, String local) {
        return shuffleboard(drive, local, SendableLevel.COMP);
    }

    public RobotCore<T> shuffleboard(String drive, String local, SendableLevel level) {
        drivetrain.shuffleboard(drive, level);

        if (localization != null) {
            localization.shuffleboard(local, level);
        }

        if (vision != null) {
            vision.shuffleboard("Robot Vision", level);
        }

        if (level == SendableLevel.DEBUG) {
            debugShuffleboardPublished = true;
        }
        coreShuffleboardPublished = true;

        return this;
    }

    /**
     * Publishes all registered mechanisms to Shuffleboard under a single tab.
     *
     * <p>Note: {@link #shuffleboard()} only publishes drivetrain/localization/vision. Mechanisms must
     * be published explicitly (either via this helper or by calling {@code mechanism.shuffleboard(...)}).</p>
     */
    public RobotCore<T> shuffleboardMechanisms() {
        return shuffleboardMechanisms("Mechanisms", SendableLevel.COMP);
    }

    public RobotCore<T> shuffleboardMechanisms(SendableLevel level) {
        return shuffleboardMechanisms("Mechanisms", level);
    }

    public RobotCore<T> shuffleboardMechanisms(String tabName, SendableLevel level) {
        if (!RobotSendableSystem.isShuffleboardEnabled()) {
            return this;
        }
        mechanismsShuffleboardPublished = true;
        if (level == SendableLevel.DEBUG) {
            debugMechanismsShuffleboardPublished = true;
        }
        String resolved = tabName == null || tabName.isBlank() ? "Mechanisms" : tabName;
        ShuffleboardTab tab = Shuffleboard.getTab(RobotSendableSystem.SHUFFLEBOARD_ROOT + "/" + resolved);
        ShuffleboardLayout list = tab.getLayout("All", BuiltInLayouts.kList);
        for (Mechanism mech : mechanisms.values()) {
            String name = mech != null ? mech.getName() : "unknown";
            if (mech == null) {
                continue;
            }
            if (!ShuffleboardControls.mechanismAllowed(name, level)) {
                continue;
            }
            mech.shuffleboard(list.getLayout(name, BuiltInLayouts.kList), level);
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
