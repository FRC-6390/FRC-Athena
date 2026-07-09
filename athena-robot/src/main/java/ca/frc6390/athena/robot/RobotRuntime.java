package ca.frc6390.athena.robot;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.commands.CommandGraph;
import ca.frc6390.athena.commands.CommandState;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.localization.ref.LocalizationPipeline;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.mechanism.core.State;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementSnapshot;
import ca.frc6390.athena.sim.runtime.SimRuntime;
import ca.frc6390.athena.vision.ref.CameraDevice;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Root Athena runtime that composes hardware, mechanisms, drivetrain, vision, localization, autos, commands, and sim.
 */
public final class RobotRuntime {
    private final HardwareGraph hardwareGraph;
    private final SimRuntime simRuntime;
    private final ca.frc6390.athena.mechanism.core.RobotRuntime mechanisms;
    private final CommandGraph commands = new CommandGraph();
    private final List<AutoRuntime> autos = new ArrayList<>();
    private final List<PathGraph> pathGraphs = new ArrayList<>();
    private final List<VisionGraph> visionGraphs = new ArrayList<>();
    private final List<LocalizationPipeline> localizations = new ArrayList<>();
    private final List<MeasurementSnapshot> localizationSnapshots = new ArrayList<>();
    private double localizationMaxAgeSeconds = Double.POSITIVE_INFINITY;
    private boolean localizationRefreshWhileDisabled;

    private RobotRuntime(HardwareGraph hardwareGraph, SimRuntime simRuntime) {
        this.hardwareGraph = Objects.requireNonNull(hardwareGraph, "hardwareGraph");
        this.simRuntime = simRuntime;
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
     * Creates a robot runtime backed by simulation handles.
     *
     * @param simRuntime simulation runtime
     * @return robot runtime
     */
    public static RobotRuntime simulated(SimRuntime simRuntime) {
        SimRuntime safeRuntime = simRuntime == null ? new SimRuntime() : simRuntime;
        return new RobotRuntime(safeRuntime.hardwareGraph(), safeRuntime);
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
     * Returns the simulation runtime, if this root is simulation-backed.
     *
     * @return simulation runtime or null
     */
    public SimRuntime simRuntime() {
        return simRuntime;
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
        if (simRuntime != null) {
            mechanisms.bindInMemoryRuntime();
        }
        return this;
    }

    /**
     * Adds a vision graph from camera declarations.
     *
     * @param cameras cameras
     * @return this runtime
     */
    public RobotRuntime cameras(CameraDevice... cameras) {
        visionGraphs.add(VisionGraph.of(cameras));
        return this;
    }

    /**
     * Adds a vision graph.
     *
     * @param visionGraph vision graph
     * @return this runtime
     */
    public RobotRuntime vision(VisionGraph visionGraph) {
        visionGraphs.add(Objects.requireNonNull(visionGraph, "visionGraph"));
        return this;
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
     * Schedules a command state.
     *
     * @param state command state
     * @return this runtime
     */
    public RobotRuntime schedule(CommandState state) {
        commands.schedule(state);
        return this;
    }

    /**
     * Sets a mechanism state.
     *
     * @param mechanism mechanism
     * @param state state
     * @return this runtime
     */
    public RobotRuntime set(Mechanism mechanism, State state) {
        mechanisms.set(mechanism, state);
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
        autos.forEach(AutoRuntime::periodic);
        return periodic(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, true, true, false);
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
        if (simRuntime != null) {
            simRuntime.step(dtSeconds);
        }
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
        List<ResolvedOutput> outputs = mechanisms.periodic(safeMechanismContext, safeEventContext);
        if (safeMechanismContext.simulation() && simRuntime != null) {
            simRuntime.step(safeMechanismContext.dtSeconds());
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
        visionGraphs.forEach(VisionGraph::refresh);
        hardwareGraph.refreshInputs();
        refreshLocalizations(mechanismContext);
        commands.periodic();
        EventContext eventContext = new EventContext(
                nowSeconds,
                dtSeconds,
                mode,
                LifecyclePhase.PERIODIC,
                enabled,
                simulation);
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
