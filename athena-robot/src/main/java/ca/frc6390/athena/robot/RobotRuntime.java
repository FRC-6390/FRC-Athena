package ca.frc6390.athena.robot;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.commands.CommandGraph;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
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
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementSnapshot;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.CameraAdapters;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulations;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.function.BooleanSupplier;

/**
 * Root Athena runtime that composes hardware, mechanisms, drivetrain, vision, localization, autos, commands, and sim.
 */
public final class RobotRuntime {
    private final HardwareGraph hardwareGraph;
    private final SimulationSession simulationSession;
    private final MechanismScheduler mechanisms;
    private final CommandGraph commands = new CommandGraph();
    private final List<AutoRuntime> autos = new ArrayList<>();
    private final List<PathGraph> pathGraphs = new ArrayList<>();
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
    private boolean localizationRefreshWhileDisabled;

    private RobotRuntime(HardwareGraph hardwareGraph, SimulationSession simulationSession) {
        this.hardwareGraph = Objects.requireNonNull(hardwareGraph, "hardwareGraph");
        this.simulationSession = simulationSession;
        mechanisms = MechanismScheduler.create(hardwareGraph);
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
        return hardwareGraph;
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
        return simulationSession;
    }

    /**
     * Configures optional runtime workers. No workers run unless configured.
     *
     * @param workers worker group
     * @return this runtime
     */
    public RobotRuntime workers(RuntimeWorkers workers) {
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
        workers.start();
        return this;
    }

    /**
     * Stops configured async workers.
     *
     * @return this runtime
     */
    public RobotRuntime stopWorkers() {
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
        return commands;
    }

    /**
     * Registers a mechanism.
     *
     * @param mechanism mechanism
     * @return this runtime
     */
    public RobotRuntime register(Mechanism mechanism) {
        mechanisms.register(mechanism);
        RuntimeGraphDiscovery.Result services = RuntimeGraphDiscovery.inspect(mechanism);
        registerDiscoveredCameras(services.cameras());
        localization(services.localizations().toArray(Localization[]::new));
        mechanisms.followerMotors().forEach(hardwareGraph::motor);
        mechanisms.encoderDevices().forEach(hardwareGraph::encoder);
        mechanisms.imuDevices().forEach(hardwareGraph::imu);
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
                    CameraAdapters.bindDiscovered(camera);
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

    private static CameraDevice[] bindCameras(VisionSimulation simulation, CameraDevice... cameras) {
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
                bound[i] = CameraAdapters.bindDiscovered(camera);
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
        localizationMaxAgeSeconds = Double.isFinite(seconds) && seconds >= 0.0 ? seconds : Double.POSITIVE_INFINITY;
        return this;
    }

    /**
     * Controls whether localization refreshes while the robot is disabled.
     *
     * @param enabled true to refresh while disabled
     * @return this runtime
     */
    public RobotRuntime localizationRefreshWhileDisabled(boolean enabled) {
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

    /**
     * Adds an autonomous runtime and its marker graph.
     *
     * @param autoRuntime autonomous runtime
     * @param pathGraph path marker graph
     * @return this runtime
     */
    public RobotRuntime auto(AutoRuntime autoRuntime, PathGraph pathGraph) {
        autos.add(Objects.requireNonNull(autoRuntime, "autoRuntime"));
        if (pathGraph != null) {
            pathGraphs.add(pathGraph);
        }
        return this;
    }

    /**
     * Schedules a command Action.
     *
     * @param Action command Action
     * @return this runtime
     */
    public RobotRuntime schedule(CommandAction Action) {
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
        mechanisms.request(Action);
        return this;
    }

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
        autos.forEach(auto -> auto.end(true));
        pathGraphs.forEach(graph -> graph.endAll(true));
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
        publishSimulationStep(nowSeconds, dtSeconds, true, false);
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
        MechanismContext safeMechanismContext = mechanismContext == null
                ? MechanismContext.empty()
                : mechanismContext;
        EventContext safeEventContext = eventContext == null ? EventContext.empty() : eventContext;
        workers.runDue(safeMechanismContext.nowSeconds());
        hardwareGraph.refreshInputs();
        visionGraphs.forEach(VisionGraph::refresh);
        refreshLocalizations(safeMechanismContext);
        if (safeEventContext.mode() == LifecycleMode.AUTONOMOUS
                && safeEventContext.phase() == LifecyclePhase.PERIODIC) {
            autos.forEach(AutoRuntime::periodic);
        }
        if (!safeEventContext.enabled()) {
            autos.forEach(auto -> auto.end(true));
            pathGraphs.forEach(graph -> graph.endAll(true));
            commands.cancelAll();
        } else {
            commands.periodic();
        }
        List<ResolvedOutput> outputs = runMechanisms(safeMechanismContext, safeEventContext);
        if (safeMechanismContext.simulation() && simulationSession != null) {
            publishSimulationStep(
                    safeMechanismContext.nowSeconds(),
                    safeMechanismContext.dtSeconds(),
                    safeMechanismContext.enabled(),
                    safeMechanismContext.autonomous());
        }
        return outputs;
    }

    private List<ResolvedOutput> periodic(
            double nowSeconds,
            double dtSeconds,
            LifecycleMode mode,
            boolean enabled,
            boolean autonomous,
            boolean simulation) {
        MechanismContext mechanismContext = new MechanismContext(nowSeconds, 0.0, dtSeconds, enabled, autonomous, simulation);
        workers.runDue(nowSeconds);
        hardwareGraph.refreshInputs();
        visionGraphs.forEach(VisionGraph::refresh);
        refreshLocalizations(mechanismContext);
        commands.periodic();
        EventContext eventContext = new EventContext(
                nowSeconds,
                dtSeconds,
                mode,
                LifecyclePhase.PERIODIC,
                enabled,
                simulation);
        return runMechanisms(mechanismContext, eventContext);
    }

    private void publishSimulationStep(double nowSeconds, double dtSeconds, boolean enabled, boolean autonomous) {
        if (simulationSession == null) {
            return;
        }
        simulationSession.step(dtSeconds);
        hardwareGraph.refreshInputs();
        visionGraphs.forEach(VisionGraph::refresh);
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
}
