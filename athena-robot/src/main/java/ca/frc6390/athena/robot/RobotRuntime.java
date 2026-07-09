package ca.frc6390.athena.robot;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.commands.CommandGraph;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.localization.ref.LocalizationPipeline;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementSnapshot;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.ref.CameraDevice;
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

/**
 * Root Athena runtime that composes hardware, mechanisms, drivetrain, vision, localization, autos, commands, and sim.
 */
public final class RobotRuntime {
    private final HardwareGraph hardwareGraph;
    private final SimulationSession simulationSession;
    private final ca.frc6390.athena.mechanism.core.RobotRuntime mechanisms;
    private final CommandGraph commands = new CommandGraph();
    private final List<AutoRuntime> autos = new ArrayList<>();
    private final List<PathGraph> pathGraphs = new ArrayList<>();
    private final List<VisionGraph> visionGraphs = new ArrayList<>();
    private final List<LocalizationPipeline> localizations = new ArrayList<>();
    private final List<MeasurementSnapshot> localizationSnapshots = new ArrayList<>();
    private final Set<SimModel> registeredSimulationModels = new LinkedHashSet<>();
    private RuntimeWorkers workers = RuntimeWorkers.none();
    private double localizationMaxAgeSeconds = Double.POSITIVE_INFINITY;
    private boolean localizationRefreshWhileDisabled;

    private RobotRuntime(HardwareGraph hardwareGraph, SimulationSession simulationSession) {
        this.hardwareGraph = Objects.requireNonNull(hardwareGraph, "hardwareGraph");
        this.simulationSession = simulationSession;
        mechanisms = ca.frc6390.athena.mechanism.core.RobotRuntime.create(hardwareGraph);
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
        if (simulationSession != null) {
            registerSimulationModels();
            mechanisms.bindInMemoryRuntime();
        }
        return this;
    }

    private void registerSimulationModels() {
        for (SimModel model : mechanisms.simulationModels()) {
            if (registeredSimulationModels.add(model)) {
                simulationSession.model("mechanism-" + registeredSimulationModels.size(), model);
            }
        }
    }

    /**
     * Adds a vision graph from camera declarations.
     *
     * @param cameras cameras
     * @return this runtime
     */
    public RobotRuntime cameras(CameraDevice... cameras) {
        VisionSimulation simulation = discoverVisionSimulation(cameras);
        VisionGraph graph = VisionGraph.of(bindSimulatedCameras(simulation, cameras));
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
        VisionSimulation simulation = discoverVisionSimulation(visionGraph.cameras().stream()
                .map(VisionGraph.CameraRuntime::camera)
                .toArray(CameraDevice[]::new));
        if (simulation != null) {
            simulationSession.vision(simulation);
            return visionGraph.bind(camera -> camera.hasBoundSignals() ? camera : simulation.bind(camera));
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

    private static CameraDevice[] bindSimulatedCameras(VisionSimulation simulation, CameraDevice... cameras) {
        if (cameras == null || cameras.length == 0 || simulation == null) {
            return cameras;
        }
        CameraDevice[] bound = new CameraDevice[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            CameraDevice camera = cameras[i];
            bound[i] = camera == null || camera.hasBoundSignals() ? camera : simulation.bind(camera);
        }
        return bound;
    }

    /**
     * Adds localization pipelines evaluated during robot periodic.
     *
     * @param pipelines localization pipelines
     * @return this runtime
     */
    public RobotRuntime localization(LocalizationPipeline... pipelines) {
        if (pipelines != null) {
            for (LocalizationPipeline pipeline : pipelines) {
                if (pipeline != null) {
                    localizations.add(pipeline);
                    localizationSnapshots.add(new MeasurementSnapshot(pipeline));
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
     * Sets a mechanism Action.
     *
     * @param mechanism mechanism
     * @param Action Action
     * @return this runtime
     */
    public RobotRuntime set(Mechanism mechanism, Action Action) {
        mechanisms.set(mechanism, Action);
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
     * Runs simulation periodic.
     *
     * @param nowSeconds timestamp
     * @param dtSeconds timestep
     * @return mechanism outputs
     */
    public List<ResolvedOutput> simulationPeriodic(double nowSeconds, double dtSeconds) {
        List<ResolvedOutput> outputs = periodic(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, true, false, true);
        publishSimulationStep(nowSeconds, dtSeconds, true, false);
        return outputs;
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
            MeasurementSignal signal = localizations.get(i);
            if (localizationMaxAgeSeconds != Double.POSITIVE_INFINITY) {
                signal = signal.maxAge(context.nowSeconds(), localizationMaxAgeSeconds);
            }
            localizationSnapshots.get(i).refresh(signal);
        }
    }
}
