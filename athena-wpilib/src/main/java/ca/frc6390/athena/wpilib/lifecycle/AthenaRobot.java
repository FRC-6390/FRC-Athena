package ca.frc6390.athena.wpilib.lifecycle;

import static edu.wpi.first.units.Units.Seconds;
import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ActionRequests;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.wpilib.arcp.ArcpBridge;
import ca.frc6390.athena.wpilib.simulation.WpilibSimPhysicsEngine;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.mechanism.persist.Locks;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import ca.frc6390.athena.mechanism.persist.TuningLocks;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import ca.frc6390.athena.wpilib.telemetry.MechanismTracePublisher;
import ca.frc6390.athena.wpilib.telemetry.MechanismTelemetryPublisher;
import ca.frc6390.athena.wpilib.telemetry.AutoPreviewPublisher;
import ca.frc6390.athena.wpilib.telemetry.LoopTimingPublisher;
import ca.frc6390.athena.wpilib.telemetry.PowerTelemetryPublisher;
import ca.frc6390.athena.wpilib.telemetry.SystemTelemetryPublisher;
import ca.frc6390.athena.wpilib.system.ClassPreloader;
import ca.frc6390.athena.wpilib.system.MemoryPressure;
import ca.frc6390.athena.wpilib.system.SystemRuntime;
import ca.frc6390.athena.wpilib.system.SystemStatus;
import ca.frc6390.athena.wpilib.system.SystemTuning;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Subclassable WPILib robot host backed by Athena mechanisms.
 */
public abstract class AthenaRobot extends TimedRobot implements Mechanism {
    private RobotRuntime runtime;
    private double lastTimestampSeconds;
    private double lastSimulationTimestampSeconds;
    private MechanismTracePublisher tracePublisher;
    private MechanismTelemetryPublisher mechanismTelemetryPublisher;
    private TelemetrySchema mechanismTelemetrySchema;
    private AutoPreviewPublisher autoPreviewPublisher;
    private SystemRuntime systemRuntime;
    /** Dashboard entries the mechanism telemetry creates per publish; the whole schema is bound in a few cycles. */
    private static final int TELEMETRY_BINDINGS_PER_PUBLISH = 48;
    /** A count alone does not bound initialization work on a slower roboRIO. */
    private static final double TELEMETRY_BINDING_BUDGET_SECONDS = 0.001;
    /**
     * Real-time priority of the robot thread while a cycle runs. Above every normal thread (JIT compilers,
     * garbage collector, NetworkTables, Athena's workers) and below the roboRIO's system real-time threads.
     */
    private static final int DEFAULT_LOOP_PRIORITY = 15;
    private int loopPriority = DEFAULT_LOOP_PRIORITY;
    private boolean preloadClasses = true;
    private ClassPreloader classPreloader;
    private SystemTelemetryPublisher systemTelemetryPublisher;
    private PowerTelemetryPublisher powerTelemetryPublisher;
    private Supplier<PowerSnapshot> powerTelemetrySource;
    private LoopTimingPublisher loopTimingPublisher;
    private final List<SendableChooser<String>> dashboardAutoChoosers = new ArrayList<>();
    private final Map<Integer, DigitalInput> digitalInputs = new ConcurrentHashMap<>();
    private MechanismTracePublisher.Profile traceProfile = MechanismTracePublisher.Profile.OFF;
    private SystemTuning systemTuning = SystemTuning.automatic();
    private VisionSimulationField simulationVisionField = VisionSimulationField.EMPTY;
    private int reportedSystemFailures;
    private MemoryPressure reportedPressure = MemoryPressure.NORMAL;
    /** Pressure the telemetry read-only period was last set for; null until first set. */
    private MemoryPressure telemetryPeriodPressure;
    private boolean reportedJvmConfiguration;
    private ReplayBridge replay;
    /** Start of the current enabled period (autonomous, teleop or test), or NaN while disabled. */
    private double enabledSinceSeconds = Double.NaN;
    private ArcpBridge arcp;
    private PersistentStore persistentStore;
    private boolean closed;
    private final TuningLocks tuningLocks = new TuningLocks()
            .define(Locks.InMatch.class, DriverStation::isFMSAttached);
    private Path persistenceDirectory;

    /**
     * Returns the owned Athena mechanism runtime.
     *
     * @return runtime
     */
    public final RobotRuntime athena() {
        if (runtime == null) {
            throw new IllegalStateException("Athena has not been created yet.");
        }
        return runtime;
    }

    public final void set(Action Action) {
        athena().request(Objects.requireNonNull(Action, "Action"));
    }

    /** Selects the live mechanism telemetry profile. Safe to call from the robot constructor. */
    public final void traceTelemetry(MechanismTracePublisher.Profile profile) {
        traceProfile = profile == null ? MechanismTracePublisher.Profile.OFF : profile;
        if (tracePublisher != null) {
            tracePublisher.profile(traceProfile);
        }
    }

    /**
     * Defines when a lock group is locked. Writes from the dashboard to values in the group are rejected
     * while {@code locked} returns true. A group without a definition behaves as {@code Locks.InMatch},
     * which is locked while FMS is attached. Safe to call from the robot constructor.
     *
     * @param group lock group class, such as {@code Locks.InMatch.class} or a robot-defined class
     * @param locked condition that is true while writes are rejected
     */
    public final void tuningLock(Class<?> group, BooleanSupplier locked) {
        tuningLocks.define(group, locked);
    }

    /**
     * Selects the directory of the persistent telemetry store. The default is
     * {@code /home/lvuser/athena/store} on the roboRIO and a temporary directory in simulation. Safe to
     * call from the robot constructor.
     */
    public final void persistenceDirectory(Path directory) {
        if (runtime != null) {
            throw new IllegalStateException("The persistence directory must be selected before robotInit.");
        }
        persistenceDirectory = java.util.Objects.requireNonNull(directory, "directory");
    }

    /** Selects roboRIO memory monitoring and tuning. Safe to call from the robot constructor. */
    public final void systemTuning(SystemTuning tuning) {
        if (runtime != null) {
            throw new IllegalStateException("System tuning must be selected before robotInit.");
        }
        systemTuning = Objects.requireNonNull(tuning, "tuning");
    }

    /**
     * Selects the AprilTag layout that simulated cameras see (simulation only). Without it, simulated cameras see a
     * single default tag. Safe to call from the robot constructor.
     */
    public final void simulationVisionField(VisionSimulationField field) {
        if (runtime != null) {
            throw new IllegalStateException("The simulation vision field must be selected before robotInit.");
        }
        simulationVisionField = Objects.requireNonNull(field, "field");
    }

    /**
     * Sets the real-time priority the robot thread runs each cycle at (1 to 99), or 0 to leave it a normal
     * thread. The default is 15. Safe to call from the robot constructor.
     *
     * <p>On Linux a normal Java thread priority has no effect, so the JIT compilers, the garbage collector and
     * every background thread compete with the robot loop on equal terms; on the roboRIO's two cores the JIT
     * alone keeps a core busy for the first minutes, and cycles stretch unpredictably. With a real-time priority
     * the cycle runs first and the other threads use the time between cycles. The priority is dropped while the
     * robot waits for its next cycle, so those threads still run.</p>
     */
    public final void realtimeLoopPriority(int priority) {
        if (priority < 0 || priority > 99) {
            throw new IllegalArgumentException("Real-time loop priority must be 0 (off) or 1 to 99.");
        }
        loopPriority = priority;
    }

    /**
     * Whether Athena's classes are loaded on a background thread at boot (default true), so the first enabled
     * cycle does not load them on the robot thread. Safe to call from the robot constructor.
     */
    public final void preloadClasses(boolean enabled) {
        if (runtime != null) {
            throw new IllegalStateException("Class preloading must be selected before robotInit.");
        }
        preloadClasses = enabled;
    }

    /** Returns the boot-time class preload, or null when it is off or robotInit has not run. */
    public final ClassPreloader classPreload() {
        return classPreloader;
    }

    @Override
    protected void loopFunc() {
        // Threads.setCurrentThreadPriority returns false where real-time priority is unavailable (simulation).
        boolean boosted = loopPriority > 0 && Threads.setCurrentThreadPriority(true, loopPriority);
        try {
            super.loopFunc();
        } finally {
            if (boosted) {
                Threads.setCurrentThreadPriority(false, 0);
            }
        }
    }

    /** Returns the latest JVM and roboRIO system-health snapshot. */
    public final SystemStatus systemStatus() {
        if (systemRuntime == null) {
            throw new IllegalStateException("System monitoring has not started yet.");
        }
        return systemRuntime.status();
    }

    /** Returns an Action that sets this robot's Action once when it runs. */
    public final Action Action(Action Action) {
        return Actions.run(() -> set(Action));
    }

    @Override
    public final void robotInit() {
        if (preloadClasses) {
            boolean real = RobotBase.isReal();
            classPreloader = ClassPreloader.start("ca.frc6390.athena.",
                    name -> !real || !(name.contains(".sim.") || name.contains(".simulation.")
                            || name.contains(".benchmarks.")),
                    AthenaRobot.class.getClassLoader());
        }
        lastTimestampSeconds = timestampSeconds();
        lastSimulationTimestampSeconds = lastTimestampSeconds;
        runtime = createRuntime(RobotBase.isSimulation(), digitalInputs, simulationVisionField);
        runtime.powerBusVoltage(edu.wpi.first.wpilibj.RobotController::getBatteryVoltage);
        runtime.failureReporter(ca.frc6390.athena.robot.RuntimeFailureReporter.async(
                new ca.frc6390.athena.robot.RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) {
                DriverStation.reportError(message, cause == null ? new StackTraceElement[0] : cause.getStackTrace());
            }

            @Override public void warning(String message, Throwable cause) {
                DriverStation.reportWarning(message, cause == null ? new StackTraceElement[0] : cause.getStackTrace());
            }
        }));
        ActionRequests.bind(runtime::request);
        runtime.tuningLocks(tuningLocks);
        persistentStore = openPersistentStore();
        runtime.persistentStore(persistentStore, DriverStation::getMatchTime);
        runtime.register(this);
        replay = ReplayBridge.attach(runtime, ReplayBridge.discoverPolicy(this), RobotBase.isSimulation());
        mechanismTelemetrySchema = runtime.mechanismTelemetrySchema();
        arcp = ArcpBridge.attach(this, runtime, replay.recorder(), tuningLocks::isLocked, persistentStore,
                RobotBase.isSimulation());
        configureAutoChoosers();
        tracePublisher = new MechanismTracePublisher().profile(
                RobotBase.isSimulation() && traceProfile == MechanismTracePublisher.Profile.SUMMARY
                        ? MechanismTracePublisher.Profile.CAPTURE
                        : traceProfile);
        // Creating dashboard entries is slow on the roboRIO: bind the schema a few entries per publish, so the
        // robot's first cycles do not each create hundreds of entries at once.
        mechanismTelemetryPublisher = new MechanismTelemetryPublisher().bindBudget(TELEMETRY_BINDINGS_PER_PUBLISH)
                .bindTimeBudget(Seconds.of(TELEMETRY_BINDING_BUDGET_SECONDS));
        autoPreviewPublisher = new AutoPreviewPublisher();
        systemRuntime = SystemRuntime.create(systemTuning, RobotBase.isReal());
        systemTelemetryPublisher = new SystemTelemetryPublisher();
        powerTelemetryPublisher = new PowerTelemetryPublisher().bindBudget(2)
                .bindTimeBudget(Seconds.of(TELEMETRY_BINDING_BUDGET_SECONDS));
        powerTelemetrySource = runtime::powerSnapshot;
        loopTimingPublisher = new LoopTimingPublisher();
        systemRuntime.start();
        publishSystemTelemetry();
        publishMechanismTelemetry();
        run(LifecycleMode.ROBOT, LifecyclePhase.INIT, true, simulationActive());
        runtime.preparePathsForStartup();
        replay.boot();
        publishAutoPreview();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        replay.periodic();
        arcp.periodic(timestampSeconds());
        if (loopTimingPublisher != null) loopTimingPublisher.cycleFinished();
    }

    @Override
    public final void disabledInit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.INIT, false, simulationActive());
        if (!Double.isNaN(enabledSinceSeconds)) {
            runtime.recordDisable(Seconds.of(timestampSeconds() - enabledSinceSeconds));
            enabledSinceSeconds = Double.NaN;
        }
        replay.initialized(LifecycleMode.DISABLED);
    }

    @Override
    public final void disabledPeriodic() {
        run(LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, simulationActive());
        publishTraces();
    }

    @Override
    public final void disabledExit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.EXIT, false, simulationActive());
    }

    @Override
    public final void autonomousInit() {
        enabledSinceSeconds = timestampSeconds();
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, simulationActive());
        replay.initialized(LifecycleMode.AUTONOMOUS);
    }

    @Override
    public final void autonomousPeriodic() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void autonomousExit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, simulationActive());
        replay.exited(LifecycleMode.AUTONOMOUS);
    }

    @Override
    public final void teleopInit() {
        enabledSinceSeconds = timestampSeconds();
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TELEOP, LifecyclePhase.INIT, true, simulationActive());
        replay.initialized(LifecycleMode.TELEOP);
    }

    @Override
    public final void teleopPeriodic() {
        run(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void teleopExit() {
        run(LifecycleMode.TELEOP, LifecyclePhase.EXIT, true, simulationActive());
        if (DriverStation.isFMSAttached()) {
            runtime.recordMatchEnd();
        }
        replay.exited(LifecycleMode.TELEOP);
    }

    @Override
    public final void testInit() {
        enabledSinceSeconds = timestampSeconds();
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TEST, LifecyclePhase.INIT, true, simulationActive());
        replay.initialized(LifecycleMode.TEST);
    }

    @Override
    public final void testPeriodic() {
        run(LifecycleMode.TEST, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void testExit() {
        run(LifecycleMode.TEST, LifecyclePhase.EXIT, true, simulationActive());
    }

    @Override
    public final void simulationInit() {
        run(LifecycleMode.SIMULATION, LifecyclePhase.INIT, true, true);
    }

    @Override
    public final void simulationPeriodic() {
        double timestamp = timestampSeconds();
        athena().simulationPeriodic(timestamp, simulationElapsed(timestamp));
    }

    private void run(LifecycleMode mode, LifecyclePhase phase, boolean enabled, boolean simulation) {
        // TimedRobot runs the mode periodic callback first and robotPeriodic last in every cycle.
        if (phase == LifecyclePhase.PERIODIC && loopTimingPublisher != null) loopTimingPublisher.cycleStarted();
        double timestamp = timestampSeconds();
        double dtSeconds = elapsed(timestamp);
        if (tracePublisher != null) {
            tracePublisher.profile(effectiveTraceProfile());
            athena().mechanismTraceLevel(tracePublisher.traceLevel());
            athena().mechanismTracePeriod(Seconds.of(tracePublisher.runtimePeriodSeconds()));
        }
        EventContext eventContext = new EventContext(timestamp, dtSeconds, mode, phase, enabled, simulation);
        athena().periodic(
                new MechanismContext(timestamp, 0.0, dtSeconds, enabled, mode == LifecycleMode.AUTONOMOUS, simulation),
                eventContext);
    }

    private void publishPowerTelemetry() {
        if (runtime != null && powerTelemetryPublisher != null) {
            powerTelemetryPublisher.publish(powerTelemetrySource);
        }
    }

    private void publishTraces() {
        publishSystemTelemetry();
        publishPowerTelemetry();
        if (tracePublisher != null && pressure() == MemoryPressure.NORMAL) {
            tracePublisher.publish(athena().mechanismTraces());
        }
        publishMechanismTelemetry();
        publishAutoPreview();
    }

    private void publishMechanismTelemetry() {
        if (mechanismTelemetryPublisher != null) {
            mechanismTelemetryPublisher.publish(mechanismTelemetrySchema);
        }
    }

    private void publishAutoPreview() {
        if (autoPreviewPublisher != null && pressure() != MemoryPressure.CRITICAL) {
            autoPreviewPublisher.publish(athena().selectedAutoPreviews(), athena().runningAutoName());
        }
    }

    private void publishSystemTelemetry() {
        if (systemRuntime == null || systemTelemetryPublisher == null) return;
        systemRuntime.update();
        SystemStatus status = systemRuntime.status();
        systemTelemetryPublisher.publish(status);
        if (status.pressure() != telemetryPeriodPressure) {
            // Only on a pressure change: the period is a unit-converted measure, rebuilt on every call.
            var diagnosticPeriod = switch (status.pressure()) {
                case NORMAL -> Seconds.of(0.10);
                case WARNING -> Seconds.of(0.50);
                case CRITICAL -> Seconds.of(1.00);
            };
            mechanismTelemetryPublisher.readOnlyPeriod(diagnosticPeriod);
            powerTelemetryPublisher.readOnlyPeriod(diagnosticPeriod);
            telemetryPeriodPressure = status.pressure();
        }
        while (reportedSystemFailures < status.failures().size()) {
            DriverStation.reportWarning(
                    "Athena system tuning: " + status.failures().get(reportedSystemFailures++), false);
        }
        if (status.pressure() != reportedPressure) {
            DriverStation.reportWarning(
                    "Athena memory pressure is " + status.pressure() + ": " + status.pressureReason(), false);
            reportedPressure = status.pressure();
        }
        if (!reportedJvmConfiguration && !status.jvmConfigurationHealthy()
                && !status.recommendedJvmArguments().isEmpty()) {
            reportedJvmConfiguration = true;
            DriverStation.reportWarning(
                    "Athena recommends roboRIO JVM arguments: "
                            + String.join(" ", status.recommendedJvmArguments()), false);
        }
    }

    private MemoryPressure pressure() {
        return systemRuntime == null ? MemoryPressure.NORMAL : systemRuntime.status().pressure();
    }

    private MechanismTracePublisher.Profile effectiveTraceProfile() {
        if (pressure() != MemoryPressure.NORMAL) return MechanismTracePublisher.Profile.OFF;
        return RobotBase.isSimulation() && traceProfile == MechanismTracePublisher.Profile.SUMMARY
                ? MechanismTracePublisher.Profile.CAPTURE
                : traceProfile;
    }

    private void configureAutoChoosers() {
        dashboardAutoChoosers.clear();
        for (AutoChooser model : athena().autoChoosers()) {
            SendableChooser<String> dashboard = new SendableChooser<>();
            dashboard.setDefaultOption(model.defaultName(), model.defaultName());
            model.optionNames().stream()
                    .filter(name -> !name.equals(model.defaultName()))
                    .forEach(name -> dashboard.addOption(name, name));
            dashboard.onChange(model::selectIfPresent);
            SmartDashboard.putData(model.dashboardName(), dashboard);
            dashboardAutoChoosers.add(dashboard);
        }
    }

    private boolean simulationActive() {
        return runtime != null && runtime.simulationSession() != null;
    }

    private double elapsed(double timestampSeconds) {
        double dtSeconds = timestampSeconds - lastTimestampSeconds;
        lastTimestampSeconds = timestampSeconds;
        return Double.isFinite(dtSeconds) && dtSeconds >= 0.0 ? dtSeconds : 0.0;
    }

    private double simulationElapsed(double timestampSeconds) {
        double dtSeconds = timestampSeconds - lastSimulationTimestampSeconds;
        lastSimulationTimestampSeconds = timestampSeconds;
        return Double.isFinite(dtSeconds) && dtSeconds >= 0.0 ? dtSeconds : 0.0;
    }

    private static double timestampSeconds() {
        return Timer.getFPGATimestamp();
    }

    static RobotRuntime createRuntime(boolean simulation) {
        return createRuntime(simulation, new ConcurrentHashMap<>(), VisionSimulationField.EMPTY);
    }

    private PersistentStore openPersistentStore() {
        Path directory = persistenceDirectory != null ? persistenceDirectory
                : ArcpBridge.discoverPolicy(this).storeDirectory().orElse(RobotBase.isReal()
                ? Path.of("/home/lvuser/athena/store")
                : Path.of(System.getProperty("java.io.tmpdir"), "athena-sim-store"));
        try {
            PersistentStore store = PersistentStore.open(directory);
            ArcpBridge.installDeployedTuning(store, ArcpBridge.discoverPolicy(this), RobotBase.isSimulation());
            return store;
        } catch (RuntimeException exception) {
            DriverStation.reportWarning("Athena persistent telemetry is disabled: " + exception.getMessage(), false);
            return null;
        }
    }

    static RobotRuntime createRuntime(
            boolean simulation,
            Map<Integer, DigitalInput> inputs,
            VisionSimulationField visionField) {
        RobotRuntime runtime = simulation
                ? RobotRuntime.simulated(SimulationSession.create()
                        .physicsEngine(new WpilibSimPhysicsEngine())
                        .visionField(visionField))
                : RobotRuntime.create();
        return runtime.digitalInputs(device -> {
            DigitalInput input = inputs.computeIfAbsent(device.channel(), DigitalInput::new);
            return input::get;
        });
    }

    @Override
    public synchronized void close() {
        if (closed) return;
        closed = true;
        RobotRuntime closingRuntime = runtime;
        runtime = null;
        RuntimeException failure = null;
        failure = closeResource(failure, arcp, "ARCP bridge");
        arcp = null;
        try {
            closeRuntime(closingRuntime);
        } catch (RuntimeException exception) {
            failure = exception;
        }
        failure = closeResource(failure, tracePublisher, "mechanism trace publisher");
        failure = closeResource(failure, mechanismTelemetryPublisher, "mechanism telemetry publisher");
        failure = closeResource(failure, autoPreviewPublisher, "auto preview publisher");
        failure = closeResource(failure, systemTelemetryPublisher, "system telemetry publisher");
        failure = closeResource(failure, powerTelemetryPublisher, "power telemetry publisher");
        powerTelemetrySource = null;
        failure = closeResource(failure, loopTimingPublisher, "loop timing publisher");
        failure = closeResource(failure, systemRuntime, "system runtime");
        failure = closeResource(failure, persistentStore, "persistent store");
        persistentStore = null;
        for (DigitalInput input : digitalInputs.values()) {
            failure = closeResource(failure, input, "digital input");
        }
        digitalInputs.clear();
        try {
            super.close();
        } catch (RuntimeException exception) {
            failure = appendFailure(failure, exception);
        }
        if (failure != null) throw failure;
    }

    static void closeRuntime(RobotRuntime runtime) {
        if (runtime == null) return;
        ActionRequests.clear();
        runtime.close();
    }

    private static RuntimeException closeResource(
            RuntimeException failure,
            AutoCloseable resource,
            String description) {
        if (resource == null) return failure;
        try {
            resource.close();
            return failure;
        } catch (Exception exception) {
            RuntimeException wrapped = exception instanceof RuntimeException runtimeException
                    ? runtimeException
                    : new IllegalStateException("Failed to close " + description + ".", exception);
            return appendFailure(failure, wrapped);
        }
    }

    private static RuntimeException appendFailure(RuntimeException failure, RuntimeException additional) {
        if (failure == null) return additional;
        failure.addSuppressed(additional);
        return failure;
    }

}
