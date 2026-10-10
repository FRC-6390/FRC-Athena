package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.model.DriftMonitor.DriftReport;
import ca.frc6390.athena.mechanism.model.ExcitationGate.ExcitationCriteria;
import ca.frc6390.athena.mechanism.model.Identification.FeedforwardFit;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;
import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import ca.frc6390.athena.hardware.units.UnitConversions;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Characterization declaration for one control binding: runnable test Actions, identified model,
 * continuous learning, passive payload learning, and model history.
 *
 * <p>Declare it as a field. The introspector expands it into Actions named
 * {@code <field>/QuasistaticForward}, {@code QuasistaticReverse}, {@code DynamicForward},
 * {@code DynamicReverse}, {@code FrictionSweep}, {@code Backlash}, {@code Latency}, {@code FitModel},
 * {@code AdoptRunningEstimate} and {@code Rebaseline}.</p>
 *
 * <p>Settings methods return a new declaration. The collected data and the active model are shared
 * between the declarations that come from one {@code ControlBinding.characterization()} call.</p>
 */
public final class Characterization implements TelemetrySource {
    /** Largest voltage the tests apply unless {@link #maxVoltage(double)} says otherwise. */
    public static final double DEFAULT_MAX_VOLTAGE = 4.0;
    /** Quasistatic ramp rate in volts per second. */
    public static final double DEFAULT_RAMP_RATE = 0.5;
    /** Seconds of enabled time after which a disable snapshots the model. */
    public static final double HISTORY_MIN_ENABLED_SECONDS = 30.0;
    /** Minimum samples in the test data before a fit is attempted. */
    public static final int MIN_RUN_SAMPLES = 30;
    /** Minimum holding samples before a payload estimate is considered. */
    public static final int MIN_PAYLOAD_SAMPLES = 100;
    /** Breakaway speed (base units per second) used to find static friction. */
    public static final double STATIC_BREAKAWAY_SPEED = 0.01;
    /** Speed (base units per second) that ends a latency measurement. */
    public static final double LATENCY_SPEED = 0.05;
    /** Friction table bins. */
    public static final int FRICTION_BINS = 8;
    /** Forgetting factor of the continuous-learning estimator. */
    public static final double FORGETTING_FACTOR = 0.995;
    /** Initial covariance of the continuous-learning estimator. */
    public static final double INITIAL_COVARIANCE = 100.0;
    private static final int RUN_CAPACITY = 5000;
    private static final int PAYLOAD_CAPACITY = 500;
    private static final int HISTORY_CAPACITY = 32;
    private static final double MIN_COS_SPREAD = 0.1;

    /**
     * Payload estimate with its confidence.
     *
     * @param deltaKG change of the gravity amplitude, in volts (zero until confident)
     * @param standardError standard error of the fitted amplitude
     * @param samples holding samples used
     * @param confident whether the estimate passes the sample, t-ratio, and spread checks
     */
    public record Payload(double deltaKG, double standardError, int samples, boolean confident) {
    }

    /** Mutable data shared by the declarations derived from one binding. */
    private static final class State {
        MechanismModel active;
        MechanismModel baseline;
        ModelStorage storage = new InMemoryModelStorage();
        DoubleSupplier clock = MonotonicClock.SYSTEM;
        final Deque<CharacterizationSample> runs = new ArrayDeque<>();
        final Deque<CharacterizationSample> holds = new ArrayDeque<>();
        CharacterizationSample previousRun;
        ExcitationGate gate = new ExcitationGate(ExcitationCriteria.DEFAULT);
        RecursiveLeastSquares running;
        final ModelHistory history = new ModelHistory(HISTORY_CAPACITY);
        String excitation = "not learning";
        boolean observing;
    }

    private final ControlBinding<?> control;
    private final double maxVoltage;
    private final double rampRate;
    private final boolean learnContinuously;
    private final double driftTolerance;
    private final double gearRatio;
    private final BooleanSupplier payloadGate;
    private final State state;
    private final ControlSysId routine;
    private final Map<String, Action> actions;

    private Characterization(
            ControlBinding<?> control,
            double maxVoltage,
            double rampRate,
            boolean learnContinuously,
            double driftTolerance,
            double gearRatio,
            BooleanSupplier payloadGate,
            State state) {
        this.control = control;
        this.maxVoltage = maxVoltage;
        this.rampRate = rampRate;
        this.learnContinuously = learnContinuously;
        this.driftTolerance = driftTolerance;
        this.gearRatio = gearRatio;
        this.payloadGate = payloadGate;
        this.state = state;
        this.routine = control.sysId()
                .rampRate(rampRate)
                .maxVoltage(maxVoltage)
                .stepVoltage(maxVoltage)
                .logger(new RunLog());
        this.actions = buildActions();
    }

    /** Creates the declaration for a binding. Used by {@code ControlBinding.characterization()}. */
    public static Characterization of(ControlBinding<?> control) {
        Objects.requireNonNull(control, "control");
        return new Characterization(control, DEFAULT_MAX_VOLTAGE, DEFAULT_RAMP_RATE, false,
                DriftMonitor.DEFAULT_TOLERANCE, 1.0, null,
                newState(control.motionKind()));
    }

    private static State newState(MotionKind kind) {
        State state = new State();
        state.active = MechanismModel.empty(kind);
        state.baseline = null;
        state.running = newRunning(kind);
        return state;
    }

    private static RecursiveLeastSquares newRunning(MotionKind kind) {
        return new RecursiveLeastSquares(Identification.columns(kind), FORGETTING_FACTOR, INITIAL_COVARIANCE);
    }

    private Map<String, Action> buildActions() {
        Action quasistaticForward = routine.quasistaticForward();
        Action quasistaticReverse = routine.quasistaticReverse();
        Map<String, Action> map = new LinkedHashMap<>();
        map.put("QuasistaticForward", quasistaticForward);
        map.put("QuasistaticReverse", quasistaticReverse);
        map.put("DynamicForward", routine.dynamicForward());
        map.put("DynamicReverse", routine.dynamicReverse());
        map.put("FrictionSweep", Actions.then(quasistaticForward, quasistaticReverse));
        map.put("Backlash", Actions.then(quasistaticForward, quasistaticReverse));
        map.put("Latency", routine.dynamicForward());
        map.put("FitModel", Actions.run(this::doFitModel));
        map.put("AdoptRunningEstimate", Actions.run(this::doAdoptRunningEstimate));
        map.put("Rebaseline", Actions.run(this::doRebaseline));
        return map;
    }

    // ----- settings -----

    /** Largest voltage any test applies. Must be positive. */
    public Characterization maxVoltage(double volts) {
        requirePositive(volts, "maxVoltage");
        return copy(volts, rampRate, learnContinuously, driftTolerance, gearRatio, payloadGate);
    }

    /** Largest voltage any test applies, as a WPILib voltage measure (for example {@code Volts.of(4)}). */
    public Characterization maxVoltage(Measure<?> volts) {
        return maxVoltage(UnitConversions.value(volts, Units.Volts));
    }

    /** Quasistatic ramp rate in volts per second. */
    public Characterization rampRate(double voltsPerSecond) {
        requirePositive(voltsPerSecond, "rampRate");
        return copy(maxVoltage, voltsPerSecond, learnContinuously, driftTolerance, gearRatio, payloadGate);
    }

    /** Enables recursive least-squares learning from normal operation. Never replaces the active model. */
    public Characterization learnContinuously(boolean enabled) {
        return copy(maxVoltage, rampRate, enabled, driftTolerance, gearRatio, payloadGate);
    }

    /** Relative band (for example 0.25 for 25%) beyond which a parameter counts as drifted. */
    public Characterization driftTolerance(double fraction) {
        requirePositive(fraction, "driftTolerance");
        return copy(maxVoltage, rampRate, learnContinuously, fraction, gearRatio, payloadGate);
    }

    /**
     * Motor turns per mechanism turn (the reduction). Used to convert the identified inertia and
     * to report external load in motor-side terms. Defaults to 1.
     */
    public Characterization gearRatio(double motorPerMechanism) {
        requirePositive(motorPerMechanism, "gearRatio");
        return copy(maxVoltage, rampRate, learnContinuously, driftTolerance, motorPerMechanism, payloadGate);
    }

    /** Collects payload samples only while the supplier is true (for example while holding a game piece). */
    public Characterization payloadWhen(BooleanSupplier loaded) {
        return copy(maxVoltage, rampRate, learnContinuously, driftTolerance, gearRatio,
                Objects.requireNonNull(loaded, "loaded"));
    }

    /** Overrides the excitation thresholds used by continuous learning. */
    public Characterization excitation(ExcitationCriteria criteria) {
        state.gate = new ExcitationGate(Objects.requireNonNull(criteria, "criteria"));
        return this;
    }

    /** Sets where {@link #save(String)} writes models. The default is in memory. */
    public Characterization storage(ModelStorage storage) {
        state.storage = Objects.requireNonNull(storage, "storage");
        return this;
    }

    /** Sets the monotonic clock used for sample times and debounce. Tests inject their own. */
    public Characterization clock(DoubleSupplier clock) {
        state.clock = Objects.requireNonNull(clock, "clock");
        return this;
    }

    /** Uses code defaults as the starting active model and baseline. */
    public Characterization defaults(MechanismModel model) {
        Objects.requireNonNull(model, "model");
        state.active = model;
        state.baseline = model;
        reseedRunning(model);
        return this;
    }

    private Characterization copy(double maxVoltage, double rampRate, boolean learn, double drift,
            double gear, BooleanSupplier payload) {
        return new Characterization(control, maxVoltage, rampRate, learn, drift, gear, payload, state);
    }

    // ----- actions -----

    /** Forward quasistatic ramp. Hold to run; releasing ends the test. */
    public Action quasistaticForward() {
        return actions.get("QuasistaticForward");
    }

    /** Reverse quasistatic ramp. */
    public Action quasistaticReverse() {
        return actions.get("QuasistaticReverse");
    }

    /** Forward step to {@code maxVoltage}. Also the latency test. */
    public Action dynamicForward() {
        return actions.get("DynamicForward");
    }

    /** Reverse step to {@code -maxVoltage}. */
    public Action dynamicReverse() {
        return actions.get("DynamicReverse");
    }

    /** Forward then reverse quasistatic ramps. Gives the friction table and the breakaway points. */
    public Action frictionSweep() {
        return actions.get("FrictionSweep");
    }

    /** Forward then reverse ramps. Gives backlash from the reversal. */
    public Action backlash() {
        return actions.get("Backlash");
    }

    /** Dynamic step response. Gives latency from the step's onset to the first motion. */
    public Action latency() {
        return actions.get("Latency");
    }

    /** Fits the model from the collected test data and makes it active and the baseline. */
    public Action fitModel() {
        return actions.get("FitModel");
    }

    /** Makes the running (continuous-learning) estimate the active model. */
    public Action adoptRunningEstimate() {
        return actions.get("AdoptRunningEstimate");
    }

    /** Makes the active model the drift baseline. */
    public Action rebaseline() {
        return actions.get("Rebaseline");
    }

    /** Exposed Actions by short name, for introspection. Identity is stable for one declaration. */
    public Map<String, Action> actions() {
        return Map.copyOf(actions);
    }

    /** Returns the characterized control binding. */
    public ControlBinding<?> control() {
        return control;
    }

    // ----- models -----

    /** Returns the active model. */
    public MechanismModel model() {
        return state.active;
    }

    /** Returns the drift baseline, or null before the first fit or defaults. */
    public MechanismModel baseline() {
        return state.baseline;
    }

    /** Returns the collected test samples, oldest first. Storage is bounded; the oldest samples drop first. */
    public List<CharacterizationSample> samples() {
        return List.copyOf(new ArrayList<>(state.runs));
    }

    /**
     * Fits every parameter from the collected test data. Does not change the active model. Throws
     * when there is not enough data.
     */
    public MechanismModel fitRuns() {
        List<CharacterizationSample> runs = List.copyOf(state.runs);
        if (runs.size() < MIN_RUN_SAMPLES) {
            throw new IllegalStateException("Run the characterization tests before fitting (have "
                    + runs.size() + " samples, need " + MIN_RUN_SAMPLES + ").");
        }
        MotionKind kind = control.motionKind();
        double deadband = Identification.DEFAULT_VELOCITY_DEADBAND;
        FeedforwardFit fit = Identification.fitFeedforward(runs, kind, deadband);
        double staticVolts = Identification.staticFriction(runs, kind, fit, STATIC_BREAKAWAY_SPEED)
                .orElse(fit.kineticStaticVolts());
        FrictionTable table = Identification.frictionTable(runs, kind, fit, deadband, FRICTION_BINS);
        double backlash = Identification.backlash(runs, deadband).orElse(Double.NaN);
        double latency = Identification.latency(runs, LATENCY_SPEED).orElse(Double.NaN);
        Payload payload = payload();
        double payloadDelta = payload.confident() ? payload.deltaKG() : 0.0;

        double torque = 0.0;
        double resistance = 0.0;
        List<String> kinds = new ArrayList<>();
        for (MotorDevice motor : control.motors()) {
            kinds.add(motorKey(motor));
        }
        MotorDevice output = control.output();
        if (output != null) {
            MotorConstants constants = MotorConstants.forKind(output.kind()).orElse(null);
            if (constants != null) {
                torque = constants.torqueConstant();
                resistance = constants.resistanceOhms();
            }
        }
        return new MechanismModel(kind, staticVolts, fit.kineticStaticVolts(), fit.kV(), fit.kA(), fit.kG(),
                fit.gravityOffsetRadians(), table, backlash, latency, payloadDelta, torque, resistance,
                kinds.size(), kinds, gearRatio, fit.standardErrors(), fit.rSquared(), fit.samples());
    }

    /** Estimate of the external payload from holding samples, with confidence. */
    public Payload payload() {
        List<CharacterizationSample> holds = List.copyOf(state.holds);
        MotionKind kind = control.motionKind();
        if (holds.size() < MIN_PAYLOAD_SAMPLES) {
            return new Payload(0.0, Double.NaN, holds.size(), false);
        }
        MechanismModel base = state.active;
        FeedforwardFit baseline = new FeedforwardFit(base.kineticFrictionVolts(), base.kV(), base.kA(),
                base.kG(), base.gravityOffsetRadians(), Map.of(), 0.0, 0.0, 0);
        Identification.PayloadEstimate estimate = Identification.payload(holds, kind, baseline,
                base.staticFrictionVolts());
        boolean spread = kind != MotionKind.ROTARY || cosineSpread(holds) >= MIN_COS_SPREAD;
        boolean significant = Double.isFinite(estimate.standardError()) && estimate.standardError() > 0.0
                && Math.abs(estimate.deltaKG()) > 2.0 * estimate.standardError();
        boolean confident = spread && significant;
        return new Payload(confident ? estimate.deltaKG() : 0.0, estimate.standardError(),
                estimate.samples(), confident);
    }

    /** Returns the running (continuous-learning) estimate, or the active model before any update. */
    public MechanismModel runningEstimate() {
        if (state.running.updates() == 0) {
            return state.active;
        }
        MechanismModel active = state.active;
        double[] c = state.running.coefficients();
        double kG;
        double offset;
        if (control.motionKind() == MotionKind.ROTARY) {
            kG = Math.hypot(c[3], c[4]);
            offset = Math.atan2(-c[4], c[3]);
        } else {
            kG = c[3];
            offset = 0.0;
        }
        return new MechanismModel(active.motionKind(), active.staticFrictionVolts(), c[0], c[1], c[2], kG,
                offset, active.friction(), active.backlash(), active.latencySeconds(),
                active.payloadDelta(), active.motorTorqueConstant(), active.motorResistance(),
                active.motorCount(), active.motorKinds(), active.gearRatio(), active.standardErrors(),
                active.rSquared(), (int) Math.min(Integer.MAX_VALUE, state.running.updates()));
    }

    /** Compares the running estimate with the baseline. */
    public DriftReport drift() {
        return DriftMonitor.evaluate(state.baseline, runningEstimate(), driftTolerance);
    }

    /** Returns whether continuous learning is enabled. */
    public boolean learnContinuously() {
        return learnContinuously;
    }

    /** Returns the history of snapshots. */
    public ModelHistory history() {
        return state.history;
    }

    /** Snapshots the running estimate when the robot was enabled for more than 30 s. Returns true if taken. */
    public boolean recordDisable(double enabledSeconds) {
        if (!(enabledSeconds > HISTORY_MIN_ENABLED_SECONDS)) {
            return false;
        }
        snapshot("disable");
        return true;
    }

    /** Snapshots the running estimate at the end of a match. */
    public void recordMatchEnd() {
        snapshot("match");
    }

    /** Saves the active model to the configured storage. */
    public void save(String key) {
        state.storage.save(Objects.requireNonNull(key, "key"), state.active);
    }

    /** Clock used for sample and debounce times. */
    public DoubleSupplier clock() {
        return state.clock;
    }

    /**
     * Feeds one cycle of normal operation to the learners. Continuous learning updates the running
     * estimate only when the excitation gate accepts the sample. The payload buffer collects holds
     * only while the payload supplier is true.
     *
     * @param appliedVolts voltage applied to the output motor
     * @param position base-unit position
     * @param velocity base-unit velocity
     * @param acceleration base-unit acceleration
     * @param directionHint sign of the commanded motion
     */
    public synchronized void observeNormalOperation(double appliedVolts, double position, double velocity,
            double acceleration, double directionHint) {
        if (!Double.isFinite(appliedVolts) || !Double.isFinite(position)
                || !Double.isFinite(velocity) || !Double.isFinite(acceleration)) {
            return;
        }
        MotionKind kind = control.motionKind();
        double deadband = Identification.DEFAULT_VELOCITY_DEADBAND;
        if (learnContinuously) {
            double[] regressor = Identification.features(kind, position, velocity, acceleration, deadband);
            ExcitationGate.Decision decision = state.gate.offer(regressor, velocity, acceleration);
            state.excitation = decision.reason();
            if (decision.accepted()) {
                state.running.update(regressor, appliedVolts);
            }
        }
        if (payloadGate != null && payloadGate.getAsBoolean() && Math.abs(velocity) <= deadband) {
            SysIdState hold = directionHint < 0.0 ? SysIdState.QUASISTATIC_REVERSE : SysIdState.QUASISTATIC_FORWARD;
            addBounded(state.holds, new CharacterizationSample(hold, state.clock.getAsDouble(),
                    appliedVolts, position, velocity, acceleration, Double.NaN), PAYLOAD_CAPACITY);
        }
    }

    /** Publishes drift, excitation, and sample counts. */
    @Override
    public Map<String, TelemetryValue> telemetry() {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        values.put("driftAlert", TelemetryValue.bool(() -> drift().alert()));
        values.put("driftMessage", TelemetryValue.string(() -> drift().message()));
        values.put("runningSamples", TelemetryValue.number(() -> state.running.updates()));
        values.put("excitation", TelemetryValue.string(() -> state.excitation));
        values.put("testSamples", TelemetryValue.number(() -> state.runs.size()));
        return values;
    }

    // ----- internals -----

    private void snapshot(String reason) {
        state.history.add(new ModelHistory.Snapshot(state.clock.getAsDouble(), reason, runningEstimate()));
    }

    private void doFitModel() {
        MechanismModel fitted = fitRuns();
        state.active = fitted;
        state.baseline = fitted;
        reseedRunning(fitted);
    }

    private void doAdoptRunningEstimate() {
        state.active = runningEstimate();
        reseedRunning(state.active);
    }

    private void doRebaseline() {
        state.baseline = state.active;
        snapshot("rebaseline");
    }

    private void reseedRunning(MechanismModel model) {
        MotionKind kind = control.motionKind();
        state.running = newRunning(kind);
        double[] seed;
        if (kind == MotionKind.ROTARY) {
            seed = new double[] {
                model.kineticFrictionVolts(), model.kV(), model.kA(),
                model.kG() * Math.cos(model.gravityOffsetRadians()),
                -model.kG() * Math.sin(model.gravityOffsetRadians())};
        } else {
            seed = new double[] {model.kineticFrictionVolts(), model.kV(), model.kA(), model.kG()};
        }
        state.running.seed(seed);
        state.gate.reset();
    }

    private static String motorKey(MotorDevice motor) {
        MotorKind kind = motor.kind();
        return kind == null ? "unknown" : kind.key();
    }

    private static double cosineSpread(List<CharacterizationSample> holds) {
        double mean = 0.0;
        double[] values = new double[holds.size()];
        for (int i = 0; i < values.length; i++) {
            values[i] = Math.cos(2.0 * Math.PI * holds.get(i).position());
            mean += values[i];
        }
        mean /= values.length;
        double sum = 0.0;
        for (double value : values) {
            sum += (value - mean) * (value - mean);
        }
        return Math.sqrt(sum / values.length);
    }

    private static <T> void addBounded(Deque<T> deque, T value, int capacity) {
        deque.addLast(value);
        while (deque.size() > capacity) {
            deque.removeFirst();
        }
    }

    private static void requirePositive(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException("Characterization " + name + " must be positive and finite.");
        }
    }

    /** Receives the SysId routine's samples and converts them, with acceleration, into run data. */
    private final class RunLog implements SysIdLog {
        @Override
        public synchronized void record(SysIdSample sample) {
            double now = state.clock.getAsDouble();
            double acceleration = 0.0;
            CharacterizationSample previous = state.previousRun;
            if (previous != null && previous.state() == sample.state()) {
                double dt = now - previous.timeSeconds();
                if (dt > 1.0e-6) {
                    acceleration = (sample.velocity() - previous.velocity()) / dt;
                }
            }
            CharacterizationSample converted = new CharacterizationSample(sample.state(), now,
                    sample.appliedVoltage(), sample.position(), sample.velocity(), acceleration,
                    sample.currentAmps());
            state.previousRun = converted;
            addBounded(state.runs, converted, RUN_CAPACITY);
        }

        @Override
        public synchronized void end() {
            state.previousRun = null;
        }
    }
}
