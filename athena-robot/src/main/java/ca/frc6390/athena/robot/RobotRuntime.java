package ca.frc6390.athena.robot;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.AutoPlan;
import ca.frc6390.athena.auto.AutoPreview;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.auto.PathPreview;
import ca.frc6390.athena.commands.CommandGraph;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementSnapshot;
import ca.frc6390.athena.runtime.RuntimeDependencyChecks;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.CameraAdapters;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulations;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.function.BooleanSupplier;

/**
 * Root Athena runtime that composes hardware, mechanisms, drivetrain, vision, localization, autos, commands, and sim.
 */
public final class RobotRuntime implements AutoCloseable {
    private final HardwareGraph hardwareGraph;
    private final SimulationSession simulationSession;
    private final MechanismScheduler mechanisms;
    private final CommandGraph commands = new CommandGraph();
    private final List<AutoChooser> autoChoosers = new ArrayList<>();
    private final List<PathProvider> pathProviders = new ArrayList<>();
    private List<AutoPreview> cachedAutoPreviews = List.of();
    private long cachedAutoPreviewRevision = Long.MIN_VALUE;
    private final List<VisionGraph> visionGraphs = new ArrayList<>();
    private final List<Localization> localizations = new ArrayList<>();
    private final List<MeasurementSnapshot> localizationSnapshots = new ArrayList<>();
    private final Set<Localization> registeredLocalizations =
            java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
    private final Set<CameraDevice> registeredCameras =
            java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
    private final Set<SimModel> registeredSimulationModels = new LinkedHashSet<>();
    private final Set<ControlBinding> registeredSimulationControls =
            java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
    private final Set<MotorDevice> modeledSimulationMotors = new LinkedHashSet<>();
    private RuntimeWorkers workers = RuntimeWorkers.none();
    private Function<DigitalInputDevice, BooleanSupplier> digitalInputResolver;
    private double localizationMaxAgeSeconds = Double.POSITIVE_INFINITY;
    private boolean localizationRefreshWhileDisabled = true;
    private RuntimeFailureReporter failureReporter = RuntimeFailureReporter.stderr();
    private final Set<String> reportedFailures = new LinkedHashSet<>();
    private final Set<Object> activeRecoverableFailures = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<Object, Integer> recoverySamples = new IdentityHashMap<>();
    private long cycleSequence;
    private volatile RuntimeCycleSnapshot cycleSnapshot;
    private volatile boolean closed;

    private RobotRuntime(HardwareGraph hardwareGraph, SimulationSession simulationSession) {
        this.hardwareGraph = Objects.requireNonNull(hardwareGraph, "hardwareGraph");
        this.simulationSession = simulationSession;
        mechanisms = MechanismScheduler.create(hardwareGraph);
        cycleSnapshot = new RuntimeCycleSnapshot(
                0, 0.0, 0.0, LifecycleMode.ROBOT, LifecyclePhase.PERIODIC,
                false, false, simulationSession != null, HardwareCycleSnapshot.empty(),
                List.of(), List.of(), List.of());
    }

    /**
     * Creates a robot runtime using discovered hardware backends.
     *
     * @return robot runtime
     */
    public static RobotRuntime create() {
        return using(HardwareGraph.discovered());
    }

    /**
     * Creates a robot runtime using an explicit hardware graph.
     *
     * @param hardwareGraph hardware graph
     * @return robot runtime
     */
    public static RobotRuntime using(HardwareGraph hardwareGraph) {
        return new RobotRuntime(hardwareGraph, null);
    }

    /**
     * Creates a normal robot runtime backed by a simulation session.
     *
     * @param simulationSession simulation session
     * @return robot runtime
     */
    public static RobotRuntime simulated(SimulationSession simulationSession) {
        SimulationSession safeSession = simulationSession == null ? SimulationSession.create() : simulationSession;
        return new RobotRuntime(safeSession.hardwareGraph(), safeSession);
    }

    /**
     * Returns the hardware graph.
     *
     * @return hardware graph
     */
    public HardwareGraph hardwareGraph() {
        ensureOpen();
        return hardwareGraph;
    }

    /** Selects where recoverable runtime failures are reported. */
    public RobotRuntime failureReporter(RuntimeFailureReporter reporter) {
        ensureOpen();
        failureReporter = Objects.requireNonNull(reporter, "reporter");
        return this;
    }

    /**
     * Returns hardware refresh failures recorded during the latest runtime cycle.
     *
     * @return latest hardware refresh failures
     */
    public List<HardwareGraph.RefreshFailure> hardwareRefreshFailures() {
        return hardwareGraph.refreshFailures();
    }

    /**
     * Returns the simulation session, if this root is simulation-backed.
     *
     * @return simulation session or null
     */
    public SimulationSession simulationSession() {
        ensureOpen();
        return simulationSession;
    }

    /**
     * Configures optional runtime workers. No workers run unless configured.
     *
     * @param workers worker group
     * @return this runtime
     */
    public RobotRuntime workers(RuntimeWorkers workers) {
        ensureOpen();
        this.workers.close();
        this.workers = workers == null ? RuntimeWorkers.none() : workers;
        return this;
    }

    /**
     * Starts configured async workers.
     *
     * @return this runtime
     */
    public RobotRuntime startWorkers() {
        ensureOpen();
        workers.start();
        return this;
    }

    /**
     * Stops configured async workers.
     *
     * @return this runtime
     */
    public RobotRuntime stopWorkers() {
        ensureOpen();
        workers.close();
        return this;
    }

    /**
     * Configures the robot host responsible for reading declared digital inputs.
     * Existing and subsequently registered declarations are bound automatically.
     *
     * @param resolver host reader factory
     * @return this runtime
     */
    public RobotRuntime digitalInputs(Function<DigitalInputDevice, BooleanSupplier> resolver) {
        ensureOpen();
        digitalInputResolver = Objects.requireNonNull(resolver, "resolver");
        bindDigitalInputs();
        return this;
    }

    /**
     * Samples declared signal inputs from the main runtime loop on the requested period.
     *
     * @param periodSeconds period in seconds
     * @return this runtime
     */
    public RobotRuntime fastSignalSampling(double periodSeconds) {
        return workers(RuntimeWorkers.inline(RuntimeWorker.every(
                "signals",
                periodSeconds,
                mechanisms::sampleSignals)));
    }

    /**
     * Samples declared signal inputs from a caller-owned scheduler.
     *
     * @param periodSeconds period in seconds
     * @param executor scheduler
     * @return this runtime
     */
    public RobotRuntime fastSignalSampling(double periodSeconds, ScheduledExecutorService executor) {
        return workers(RuntimeWorkers.async(
                executor,
                RuntimeWorker.every("signals", periodSeconds, mechanisms::sampleSignals))).startWorkers();
    }

    /**
     * Returns the command graph.
     *
     * @return command graph
     */
    public CommandGraph commands() {
        ensureOpen();
        return commands;
    }

    /**
     * Registers a mechanism.
     *
     * @param mechanism mechanism
     * @return this runtime
     */
    public RobotRuntime register(Mechanism mechanism) {
        ensureOpen();
        mechanisms.register(mechanism);
        RuntimeGraphDiscovery.Result services = RuntimeGraphDiscovery.inspect(mechanism);
        services.cameraOwners().forEach(mechanisms::declarationOwner);
        registerDiscoveredCameras(services.cameras());
        localization(services.localizations().toArray(Localization[]::new));
        services.pathProviders().forEach(this::registerPathProvider);
        services.autoChoosers().forEach(this::registerAutoChooser);
        mechanisms.motorDevices().stream().filter(motor -> !motor.isDisabled()).forEach(hardwareGraph::motor);
        mechanisms.encoderDevices().forEach(hardwareGraph::encoder);
        mechanisms.imuDevices().forEach(hardwareGraph::imu);
        processHardwareBindingFailures();
        if (simulationSession != null) {
            registerSimulationModels();
            mechanisms.bindInMemoryRuntime();
        }
        bindDigitalInputs();
        return this;
    }

    private void registerDiscoveredCameras(List<CameraDevice> cameras) {
        cameras(cameras.toArray(CameraDevice[]::new));
    }

    private void bindDigitalInputs() {
        if (digitalInputResolver == null) {
            return;
        }
        mechanisms.digitalInputDevices().forEach(input -> {
            BooleanSupplier reader = Objects.requireNonNull(
                    digitalInputResolver.apply(input),
                    "Digital input resolver returned null for " + input.defaultName());
            mechanisms.digitalInput(input, reader);
        });
    }

    private void registerSimulationModels() {
        for (SimModel model : mechanisms.simulationModels()) {
            if (registeredSimulationModels.add(model)) {
                modeledSimulationMotors.addAll(model.motors());
                simulationSession.model("mechanism-" + registeredSimulationModels.size(), model);
            }
        }
        for (ControlBinding control : mechanisms.controlBindings()) {
            if (registeredSimulationControls.contains(control)
                    || control.motors().isEmpty()
                    || control.motors().stream().anyMatch(modeledSimulationMotors::contains)) {
                continue;
            }
            SimModel model = SimModel.motor(control.motors().toArray(MotorDevice[]::new));
            if (control.feedback() != null) {
                for (var encoder : control.feedback().encoders()) {
                    model = model.encoder(encoder);
                }
            }
            registeredSimulationControls.add(control);
            registeredSimulationModels.add(model);
            modeledSimulationMotors.addAll(model.motors());
            simulationSession.model("automatic-control-" + registeredSimulationControls.size(), model);
        }
    }

    /**
     * Adds a vision graph from camera declarations.
     *
     * @param cameras cameras
     * @return this runtime
     */
    public RobotRuntime cameras(CameraDevice... cameras) {
        ensureOpen();
        CameraDevice[] additions = cameras == null ? new CameraDevice[0] : Arrays.stream(cameras)
                .filter(Objects::nonNull)
                .filter(registeredCameras::add)
                .toArray(CameraDevice[]::new);
        if (additions.length == 0) {
            return this;
        }
        VisionSimulation simulation = discoverVisionSimulation(additions);
        VisionGraph graph = VisionGraph.of(bindCameras(simulation, additions));
        visionGraphs.add(graph);
        if (simulation != null) {
            simulationSession.vision(simulation);
        }
        return this;
    }

    /**
     * Adds a vision graph.
     *
     * @param visionGraph vision graph
     * @return this runtime
     */
    public RobotRuntime vision(VisionGraph visionGraph) {
        ensureOpen();
        VisionGraph safeGraph = bindVisionGraphForSimulation(Objects.requireNonNull(visionGraph, "visionGraph"));
        visionGraphs.add(safeGraph);
        return this;
    }

    private VisionGraph bindVisionGraphForSimulation(VisionGraph visionGraph) {
        CameraDevice[] cameras = visionGraph.cameras().stream()
                .map(VisionGraph.CameraRuntime::camera)
                .toArray(CameraDevice[]::new);
        VisionSimulation simulation = discoverVisionSimulation(cameras);
        if (simulation != null) {
            simulationSession.vision(simulation);
        }
        for (CameraDevice camera : cameras) {
            if (!camera.hasBoundSignals()) {
                if (simulation != null) {
                    simulation.bind(camera);
                } else {
                    try {
                        CameraAdapters.bindDiscovered(camera);
                    } catch (RuntimeException exception) {
                        handleFailure("camera", camera, camera.failurePolicy(), exception);
                    }
                }
            }
        }
        return visionGraph;
    }

    private VisionSimulation discoverVisionSimulation(CameraDevice... cameras) {
        if (simulationSession == null) {
            return null;
        }
        List<CameraDevice> unboundCameras = cameras == null ? List.of() : Arrays.stream(cameras)
                .filter(Objects::nonNull)
                .filter(camera -> !camera.hasBoundSignals())
                .toList();
        if (unboundCameras.isEmpty()) {
            return null;
        }
        return VisionSimulations.createDiscovered(unboundCameras, simulationSession.visionField()).orElse(null);
    }

    private CameraDevice[] bindCameras(VisionSimulation simulation, CameraDevice... cameras) {
        if (cameras == null || cameras.length == 0) {
            return cameras;
        }
        CameraDevice[] bound = new CameraDevice[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            CameraDevice camera = cameras[i];
            if (camera == null || camera.hasBoundSignals()) {
                bound[i] = camera;
            } else if (simulation != null) {
                bound[i] = simulation.bind(camera);
            } else {
                try {
                    bound[i] = CameraAdapters.bindDiscovered(camera);
                    mechanisms.declarationAlias(bound[i], camera);
                    if (!bound[i].hasBoundSignals() && RuntimeDependencyChecks.enabled()) {
                        handleFailure(
                                "camera",
                                camera,
                                camera.failurePolicy(),
                                new IllegalStateException(CameraAdapters.missingAdapterMessage(camera)));
                    }
                } catch (RuntimeException exception) {
                    bound[i] = camera;
                    handleFailure("camera", camera, camera.failurePolicy(), exception);
                }
            }
        }
        return bound;
    }

    /**
     * Adds localization pipelines evaluated during robot periodic.
     *
     * @param pipelines localization pipelines
     * @return this runtime
     */
    public RobotRuntime localization(Localization... pipelines) {
        ensureOpen();
        if (pipelines != null) {
            for (Localization pipeline : pipelines) {
                if (pipeline != null) {
                    if (registeredLocalizations.add(pipeline)) {
                        localizations.add(pipeline);
                        localizationSnapshots.add(new MeasurementSnapshot(pipeline));
                    }
                }
            }
        }
        return this;
    }

    /**
     * Sets the max age applied to root-owned localization refresh snapshots.
     *
     * @param seconds max age in seconds
     * @return this runtime
     */
    public RobotRuntime localizationMaxAge(double seconds) {
        ensureOpen();
        localizationMaxAgeSeconds = Double.isFinite(seconds) && seconds >= 0.0 ? seconds : Double.POSITIVE_INFINITY;
        return this;
    }

    /**
     * Controls whether localization refreshes while the robot is disabled. Enabled by default.
     *
     * @param enabled true to refresh while disabled
     * @return this runtime
     */
    public RobotRuntime localizationRefreshWhileDisabled(boolean enabled) {
        ensureOpen();
        localizationRefreshWhileDisabled = enabled;
        return this;
    }

    /**
     * Returns the latest root-owned localization measurements.
     *
     * @return cached localization measurements
     */
    public List<Measurement> localizationMeasurements() {
        List<Measurement> values = new ArrayList<>();
        localizationSnapshots.forEach(snapshot -> values.addAll(snapshot.measurements()));
        return List.copyOf(values);
    }

    /** Returns auto-discovered chooser declarations for dashboard adapters. */
    public List<AutoChooser> autoChoosers() {
        return List.copyOf(autoChoosers);
    }

    /** Running auto name for telemetry, or an empty string outside autonomous. */
    public String runningAutoName() {
        return autoChoosers.stream()
                .map(AutoChooser::runningName)
                .flatMap(java.util.Optional::stream)
                .findFirst()
                .orElse("");
    }

    private void registerPathProvider(PathProvider provider) {
        if (pathProviders.stream().noneMatch(value -> value == provider)) {
            pathProviders.add(Objects.requireNonNull(provider, "provider"));
            cachedAutoPreviewRevision = Long.MIN_VALUE;
        }
    }

    private void registerAutoChooser(AutoChooser chooser) {
        if (autoChoosers.stream().anyMatch(value -> value == chooser)) return;
        AutoChooser safeChooser = Objects.requireNonNull(chooser, "chooser");
        safeChooser.options().values().forEach(action -> mechanisms.paths(action, this::pathRuntime));
        autoChoosers.add(safeChooser);
        cachedAutoPreviewRevision = Long.MIN_VALUE;
    }

    private PathRuntime pathRuntime(PathAction path) {
        List<PathProvider> owners = pathProviders.stream().filter(provider -> provider.owns(path)).toList();
        if (owners.isEmpty()) {
            throw new IllegalStateException("No path provider owns '" + path.key() + "'. "
                    + "Declare its PathProvider as a Robot field before the AutoChooser.");
        }
        if (owners.size() > 1) {
            throw new IllegalStateException("Multiple path providers own '" + path.key() + "'.");
        }
        PathProvider provider = owners.get(0);
        PathRuntime runtime = provider.runtime();
        return simulationSession == null ? runtime : simulationPathRuntime(provider, runtime);
    }

    private PathRuntime simulationPathRuntime(PathProvider provider, PathRuntime delegate) {
        return new PathRuntime() {
            @Override
            public void initialize(PathAction path, MechanismContext context) {
                if (path.resetsOdometry()) {
                    provider.preview(path).flatMap(preview -> preview.poses().stream().findFirst())
                            .ifPresent(pose -> simulationSession.resetPose(new PoseSnapshot(
                                    pose.xMeters(), pose.yMeters(), pose.headingRadians())));
                }
                delegate.initialize(path, context);
            }

            @Override
            public void execute(PathAction path, MechanismContext context) {
                delegate.execute(path, context);
            }

            @Override
            public Action output(PathAction path, MechanismContext context) {
                return delegate.output(path, context);
            }

            @Override
            public List<?> ownership(PathAction path) {
                return delegate.ownership(path);
            }

            @Override
            public Map<String, Action> activeMarkers(PathAction path, MechanismContext context) {
                return delegate.activeMarkers(path, context);
            }

            @Override
            public boolean isFinished(PathAction path, MechanismContext context) {
                return delegate.isFinished(path, context);
            }

            @Override
            public void end(PathAction path, MechanismContext context, boolean interrupted) {
                delegate.end(path, context, interrupted);
            }
        };
    }

    /** Builds dashboard-safe previews of the currently selected autonomous Actions. */
    public List<AutoPreview> selectedAutoPreviews() {
        long revision = 1L;
        for (AutoChooser chooser : autoChoosers) revision = 31L * revision + chooser.revision();
        for (PathProvider provider : pathProviders) revision = 31L * revision + provider.previewRevision();
        if (revision == cachedAutoPreviewRevision) return cachedAutoPreviews;

        List<AutoPreview> previews = new ArrayList<>();
        for (AutoChooser chooser : autoChoosers) {
            AutoPlan plan = AutoPlan.inspect(chooser.selectedAction());
            List<String> steps = new ArrayList<>(plan.steps());
            List<PathPreview> paths = new ArrayList<>();
            for (var path : plan.paths()) {
                boolean found = false;
                for (PathProvider provider : pathProviders) {
                    try {
                        var preview = provider.preview(path);
                        if (preview.isPresent()) {
                            paths.add(preview.get());
                            found = true;
                            break;
                        }
                    } catch (RuntimeException exception) {
                        steps.add("PREVIEW ERROR " + path.key() + ": " + exception.getMessage());
                        found = true;
                        break;
                    }
                }
                if (!found) steps.add("NO GEOMETRY " + path.key());
            }
            previews.add(new AutoPreview(chooser.selectedName(), steps, paths));
        }
        cachedAutoPreviews = List.copyOf(previews);
        cachedAutoPreviewRevision = revision;
        return cachedAutoPreviews;
    }

    /**
     * Schedules a command Action.
     *
     * @param Action command Action
     * @return this runtime
     */
    public RobotRuntime schedule(CommandAction Action) {
        ensureOpen();
        commands.schedule(Action);
        return this;
    }

    /**
     * Requests an Action owned by a registered mechanism.
     *
     * @param Action Action
     * @return this runtime
     */
    public RobotRuntime request(Action Action) {
        ensureOpen();
        mechanisms.request(Action);
        return this;
    }

    /** Cancels a requested mechanism action. */
    public RobotRuntime cancel(Action action) {
        ensureOpen();
        mechanisms.cancel(action);
        return this;
    }

    public boolean isActionRunning(Action action) { return mechanisms.isRunning(action); }
    public boolean isActionComplete(Action action) { return mechanisms.isComplete(action); }

    /**
     * Runs robot periodic.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return mechanism outputs
     */
    public List<ResolvedOutput> robotPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(nowSeconds, dtSeconds, LifecycleMode.ROBOT, true, false, false);
    }

    /**
     * Returns traces captured by the latest mechanism cycle. This method performs no hardware I/O.
     *
     * @return latest mechanism traces
     */
    public List<MechanismTraceSnapshot> mechanismTraces() {
        return mechanisms.traceSnapshots();
    }

    /** Returns the completed immutable snapshot for the latest runtime cycle. */
    public RuntimeCycleSnapshot cycleSnapshot() {
        return cycleSnapshot;
    }

    /** Returns query helpers for explaining the latest runtime cycle. */
    public RuntimeInspection inspect() {
        return new RuntimeInspection(cycleSnapshot);
    }

    /** Controls core trace materialization independently of a telemetry transport. */
    public RobotRuntime mechanismTraceLevel(
            ca.frc6390.athena.mechanism.core.MechanismTraceLevel level) {
        ensureOpen();
        mechanisms.traceLevel(level);
        return this;
    }

    /** Sets the minimum interval between materialized mechanism trace snapshots. */
    public RobotRuntime mechanismTracePeriodSeconds(double periodSeconds) {
        ensureOpen();
        mechanisms.tracePeriodSeconds(periodSeconds);
        return this;
    }

    /** Returns the hierarchical mechanism-owned telemetry and live-tuning contract. */
    public ca.frc6390.athena.mechanism.core.TelemetrySchema mechanismTelemetrySchema() {
        return mechanisms.telemetrySchema();
    }

    /**
     * Runs teleop periodic.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return mechanism outputs
     */
    public List<ResolvedOutput> teleopPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(nowSeconds, dtSeconds, LifecycleMode.TELEOP, true, false, false);
    }

    /**
     * Runs autonomous periodic.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return mechanism outputs
     */
    public List<ResolvedOutput> autoPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(
                        nowSeconds,
                        dtSeconds,
                        LifecycleMode.AUTONOMOUS,
                        LifecyclePhase.PERIODIC,
                        true,
                        false));
    }

    /**
     * Runs disabled periodic.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return mechanism outputs
     */
    public List<ResolvedOutput> disabledPeriodic(double nowSeconds, double dtSeconds) {
        endAutos();
        commands.cancelAll();
        return periodic(nowSeconds, dtSeconds, LifecycleMode.DISABLED, false, false, false);
    }

    /**
     * Steps the simulation session without running robot mechanisms again.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return no mechanism outputs
     */
    public List<ResolvedOutput> simulationPeriodic(double nowSeconds, double dtSeconds) {
        ensureOpen();
        publishSimulationStep(nowSeconds, dtSeconds, true, false);
        updateCycleSnapshot(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION,
                        LifecyclePhase.PERIODIC, true, true));
        return List.of();
    }

    /**
     * Runs one runtime cycle from a prebuilt lifecycle context.
     *
     * @param mechanismContext mechanism context
     * @param eventContext event context
     * @return mechanism outputs
     */
    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        ensureOpen();
        MechanismContext safeMechanismContext = mechanismContext == null
                ? MechanismContext.empty()
                : mechanismContext;
        EventContext safeEventContext = eventContext == null ? EventContext.empty() : eventContext;
        workers.runDue(safeMechanismContext.nowSeconds());
        hardwareGraph.refreshInputs();
        refreshFailures(safeMechanismContext.nowSeconds());
        refreshLocalizations(safeMechanismContext);
        boolean autonomous = safeEventContext.mode() == LifecycleMode.AUTONOMOUS && safeEventContext.enabled();
        // Autonomous-init hooks select the routine. Start it on the first periodic cycle,
        // after those hooks have run, so the default routine is never briefly scheduled.
        if (autonomous && safeEventContext.phase() == LifecyclePhase.PERIODIC) startAutos();
        else if (!safeEventContext.enabled()
                || safeEventContext.phase() == LifecyclePhase.EXIT
                || (safeEventContext.mode() != LifecycleMode.AUTONOMOUS
                        && safeEventContext.phase() == LifecyclePhase.INIT)) endAutos();
        if (!safeEventContext.enabled()) {
            commands.cancelAll();
        } else {
            commands.periodic();
        }
        List<ResolvedOutput> outputs = runMechanisms(safeMechanismContext, safeEventContext);
        if (safeMechanismContext.simulation()
                && simulationSession != null
                && safeEventContext.mode() == LifecycleMode.SIMULATION) {
            publishSimulationStep(
                    safeMechanismContext.nowSeconds(),
                    safeMechanismContext.dtSeconds(),
                    safeMechanismContext.enabled(),
                    safeMechanismContext.autonomous());
        }
        updateCycleSnapshot(safeMechanismContext, safeEventContext);
        return outputs;
    }

    private List<ResolvedOutput> periodic(
            double nowSeconds,
            double dtSeconds,
            LifecycleMode mode,
            boolean enabled,
            boolean autonomous,
            boolean simulation) {
        ensureOpen();
        MechanismContext mechanismContext = new MechanismContext(nowSeconds, 0.0, dtSeconds, enabled, autonomous, simulation);
        workers.runDue(nowSeconds);
        hardwareGraph.refreshInputs();
        refreshFailures(nowSeconds);
        refreshLocalizations(mechanismContext);
        commands.periodic();
        EventContext eventContext = new EventContext(
                nowSeconds,
                dtSeconds,
                mode,
                LifecyclePhase.PERIODIC,
                enabled,
                simulation);
        List<ResolvedOutput> outputs = runMechanisms(mechanismContext, eventContext);
        updateCycleSnapshot(mechanismContext, eventContext);
        return outputs;
    }

    private void publishSimulationStep(double nowSeconds, double dtSeconds, boolean enabled, boolean autonomous) {
        if (simulationSession == null) {
            return;
        }
        simulationSession.step(dtSeconds);
        hardwareGraph.refreshInputs();
        refreshFailures(nowSeconds + dtSeconds);
        refreshLocalizations(new MechanismContext(
                nowSeconds + dtSeconds,
                0.0,
                dtSeconds,
                enabled,
                autonomous,
                true));
    }

    private List<ResolvedOutput> runMechanisms(MechanismContext mechanismContext, EventContext eventContext) {
        if (simulationSession != null) {
            return simulationSession.withDigitalInputs(() -> mechanisms.periodic(mechanismContext, eventContext));
        }
        return mechanisms.periodic(mechanismContext, eventContext);
    }

    private void updateCycleSnapshot(MechanismContext context, EventContext eventContext) {
        List<RuntimeCycleSnapshot.DigitalInput> digitalInputs = mechanisms.digitalInputDevices().stream()
                .map(input -> new RuntimeCycleSnapshot.DigitalInput(
                        input.defaultName(), input.channel(), safeDigitalRead(input, true), safeDigitalRead(input, false)))
                .toList();
        List<RuntimeCycleSnapshot.LocalizationInput> localizationInputs = new ArrayList<>(localizations.size());
        for (int index = 0; index < localizations.size(); index++) {
            Localization localization = localizations.get(index);
            localizationInputs.add(new RuntimeCycleSnapshot.LocalizationInput(
                    index,
                    localization.getClass().getSimpleName(),
                    localizationSnapshots.get(index).measurements()));
        }
        cycleSnapshot = new RuntimeCycleSnapshot(
                ++cycleSequence,
                context.nowSeconds(),
                context.dtSeconds(),
                eventContext.mode(),
                eventContext.phase(),
                context.enabled(),
                context.autonomous(),
                context.simulation(),
                hardwareGraph.cycleSnapshot(),
                digitalInputs,
                mechanisms.traceSnapshots(),
                localizationInputs);
    }

    private static boolean safeDigitalRead(DigitalInputDevice input, boolean raw) {
        try {
            return raw ? input.raw() : input.active();
        } catch (RuntimeException exception) {
            return false;
        }
    }

    private void processHardwareBindingFailures() {
        for (HardwareGraph.BindingFailure failure : hardwareGraph.bindingFailures()) {
            handleHardwareFailure("binding", failure.declaration(), failure.exception());
        }
    }

    private void refreshFailures(double nowSeconds) {
        Set<Object> failedThisCycle = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (HardwareGraph.BindingFailure failure : hardwareGraph.drainOperationFailures()) {
            failedThisCycle.add(failure.declaration());
            handleHardwareFailure("operation", failure.declaration(), failure.exception());
        }
        for (HardwareGraph.RefreshFailure failure : hardwareGraph.refreshFailures()) {
            failedThisCycle.add(failure.declaration());
            handleHardwareFailure("refresh", failure.declaration(), failure.exception());
        }
        for (VisionGraph graph : visionGraphs) {
            graph.refresh(nowSeconds);
            for (VisionGraph.RefreshFailure failure : graph.refreshFailures()) {
                failedThisCycle.add(failure.camera());
                handleFailure("camera", failure.camera(), failure.camera().failurePolicy(), failure.exception());
            }
        }
        recoverHealthyDeclarations(failedThisCycle);
    }

    private void handleHardwareFailure(String phase, Object declaration, RuntimeException exception) {
        FailurePolicy policy;
        String type;
        if (declaration instanceof MotorDevice motor) {
            policy = motor.failurePolicy();
            type = "motor";
        } else if (declaration instanceof EncoderDevice encoder) {
            policy = encoder.failurePolicy();
            type = "encoder";
        } else if (declaration instanceof ImuDevice imu) {
            policy = imu.failurePolicy();
            type = "IMU";
        } else {
            policy = FailurePolicy.DISABLE_MECHANISM;
            type = "hardware";
        }
        handleFailure(type + " " + phase, declaration, policy, exception);
    }

    private void handleFailure(
            String type,
            Object declaration,
            FailurePolicy policy,
            RuntimeException exception) {
        if (policy == FailurePolicy.PANIC) {
            throw exception;
        }
        if (policy == FailurePolicy.DISABLE_DEVICE) {
            mechanisms.disableDeclaration(declaration);
        } else if (policy == FailurePolicy.DISABLE_MECHANISM
                && !mechanisms.disableOwner(declaration)) {
            mechanisms.disableDeclaration(declaration);
        }
        if (policy == FailurePolicy.DISABLE_DEVICE || policy == FailurePolicy.DISABLE_MECHANISM) {
            activeRecoverableFailures.add(declaration);
            recoverySamples.remove(declaration);
        }
        String identity = declarationIdentity(declaration);
        String message = "Athena " + type + " failure for " + identity
                + "; policy=" + policy + ". " + exception.getMessage();
        String key = type + ":" + identity + ":" + exception.getClass().getName();
        if (!reportedFailures.add(key)) {
            return;
        }
        if (policy == FailurePolicy.WARN) {
            failureReporter.warning(message, exception);
        } else {
            failureReporter.error(message, exception);
        }
    }

    private void recoverHealthyDeclarations(Set<Object> failedThisCycle) {
        for (Object declaration : List.copyOf(activeRecoverableFailures)) {
            if (failedThisCycle.contains(declaration)) {
                recoverySamples.remove(declaration);
                continue;
            }
            int healthySamples = recoverySamples.merge(declaration, 1, Integer::sum);
            int requiredSamples = declaration instanceof CameraDevice camera
                    ? camera.recoveryPolicy().healthySamples()
                    : 3;
            if (healthySamples < requiredSamples) continue;
            mechanisms.recoverFailure(declaration);
            activeRecoverableFailures.remove(declaration);
            recoverySamples.remove(declaration);
            String identity = declarationIdentity(declaration);
            reportedFailures.removeIf(key -> key.contains(":" + identity + ":"));
        }
    }

    private static String declarationIdentity(Object declaration) {
        if (declaration instanceof MotorDevice motor) return motor.defaultName();
        if (declaration instanceof EncoderDevice encoder) return encoder.defaultName();
        if (declaration instanceof ImuDevice imu) return imu.defaultName();
        if (declaration instanceof CameraDevice camera) return camera.kind().key() + ":" + camera.name();
        return declaration.getClass().getSimpleName();
    }

    private void startAutos() {
        for (AutoChooser chooser : autoChoosers) {
            if (chooser.runningAction().isEmpty()) mechanisms.request(chooser.start());
        }
    }

    private void endAutos() {
        for (AutoChooser chooser : autoChoosers) chooser.stop().ifPresent(mechanisms::cancel);
    }

    private void refreshLocalizations(MechanismContext context) {
        if (!context.enabled() && !localizationRefreshWhileDisabled) {
            return;
        }
        for (int i = 0; i < localizations.size(); i++) {
            Localization localization = localizations.get(i);
            MeasurementSignal signal = localization.refresh(
                    hardwareGraph,
                    context.nowSeconds(),
                    context.dtSeconds());
            if (localizationMaxAgeSeconds != Double.POSITIVE_INFINITY) {
                signal = signal.maxAge(context.nowSeconds(), localizationMaxAgeSeconds);
            }
            localizationSnapshots.get(i).refresh(signal);
        }
    }

    /** Stops runtime activity and releases owned hardware bindings and handles. */
    @Override
    public synchronized void close() {
        if (closed) return;
        closed = true;
        RuntimeException failure = null;
        try {
            workers.close();
        } catch (RuntimeException exception) {
            failure = exception;
        }
        try {
            commands.cancelAll();
        } catch (RuntimeException exception) {
            failure = appendFailure(failure, exception);
        }
        try {
            hardwareGraph.close();
        } catch (RuntimeException exception) {
            failure = appendFailure(failure, exception);
        }
        if (failure != null) throw failure;
    }

    private void ensureOpen() {
        if (closed) {
            throw new IllegalStateException("Robot runtime is closed.");
        }
    }

    private static RuntimeException appendFailure(RuntimeException failure, RuntimeException additional) {
        if (failure == null) return additional;
        failure.addSuppressed(additional);
        return failure;
    }
}
