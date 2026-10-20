package ca.frc6390.athena.mechanism.model;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.hardware.units.UnitConversions;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.CharacterizationRoutine;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlMode;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.model.DriftMonitor.DriftReport;
import ca.frc6390.athena.mechanism.model.ExcitationGate.ExcitationCriteria;
import ca.frc6390.athena.mechanism.model.Identification.FeedforwardFit;
import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdLogProvider;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

/**
 * The one characterization API for a control binding: the four SysId tests, a whole-routine action, safety limits,
 * the identified model, continuous learning, passive payload learning, model history, and the log export.
 *
 * <p>Declare it as a field, next to the binding:</p>
 * <pre>{@code
 * public final Characterization shooter = velocity.characterization()
 *         .maxVoltage(Volts.of(6.0))
 *         .rampRate(Volts.of(1.0).per(Seconds).of(1.0))  // see rampRate(...) for the accepted form
 *         .timeout(Seconds.of(8.0))
 *         .gravity(Gravity.NONE);
 * }</pre>
 *
 * <p>The introspector expands the field into Actions named {@code <field>/QuasistaticForward},
 * {@code QuasistaticReverse}, {@code DynamicForward}, {@code DynamicReverse}, {@code Routine},
 * {@code FrictionSweep}, {@code Backlash}, {@code Latency}, {@code FitModel}, {@code AdoptRunningEstimate} and
 * {@code Rebaseline}.</p>
 *
 * <p>Settings methods return a new declaration. The collected data, the exported log and the active model are shared
 * between the declarations that come from one {@code ControlBinding.characterization()} call.</p>
 */
public final class Characterization implements TelemetrySource {
    /** Largest voltage the tests apply unless {@link #maxVoltage(Voltage)} says otherwise. */
    public static final double DEFAULT_MAX_VOLTAGE = 4.0;
    /** Quasistatic ramp rate in volts per second. */
    public static final double DEFAULT_RAMP_RATE = 0.5;
    /** Per-test safety timeout in seconds. */
    public static final double DEFAULT_TIMEOUT = 10.0;
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

    /** Declared settings. Immutable; each settings call builds a new declaration from a copy. */
    private record Settings(
            double maxVolts,
            double rampVoltsPerSecond,
            double timeoutSeconds,
            double minNormalized,
            double maxNormalized,
            boolean learnContinuously,
            double driftTolerance,
            double gearRatio,
            BooleanSupplier payloadGate) {
    }

    /** Mutable data shared by the declarations derived from one binding. */
    private static final class State {
        Gravity gravity;
        MechanismModel active;
        MechanismModel baseline;
        ModelStorage storage = new InMemoryModelStorage();
        String storageKey;
        String logName;
        DoubleSupplier clock = MonotonicClock.SYSTEM;
        final Deque<CharacterizationSample> runs = new ArrayDeque<>();
        final Deque<CharacterizationSample> holds = new ArrayDeque<>();
        CharacterizationSample previousRun;
        ExcitationGate gate = new ExcitationGate(ExcitationCriteria.DEFAULT);
        RecursiveLeastSquares running;
        final ModelHistory history = new ModelHistory(HISTORY_CAPACITY);
        String excitation = "not learning";
        SysIdLog export;
        boolean exportResolved;
    }

    private final ControlBinding<?> control;
    private final Settings settings;
    private final State state;
    private final RunLog runLog;
    private final CharacterizationRoutine engine;
    private final Map<String, Action> actions;

    private Characterization(ControlBinding<?> control, Settings settings, State state, RunLog runLog) {
        this.control = control;
        this.settings = settings;
        this.state = state;
        this.runLog = runLog;
        this.engine = CharacterizationRoutine.forControl(control, runLog)
                .withRampRate(settings.rampVoltsPerSecond())
                .withMaxVoltage(settings.maxVolts())
                .withTimeout(settings.timeoutSeconds())
                .withRange(settings.minNormalized(), settings.maxNormalized());
        this.actions = buildActions();
    }

    /** Creates the declaration for a binding. Used by {@code ControlBinding.characterization()}. */
    public static Characterization of(ControlBinding<?> control) {
        Objects.requireNonNull(control, "control");
        State state = new State();
        state.gravity = defaultGravity(control);
        state.active = MechanismModel.empty(control.motionKind());
        state.baseline = null;
        state.running = newRunning(state.gravity);
        Settings settings = new Settings(DEFAULT_MAX_VOLTAGE, DEFAULT_RAMP_RATE, DEFAULT_TIMEOUT,
                Double.NaN, Double.NaN, false, DriftMonitor.DEFAULT_TOLERANCE, gearFromFeedback(control),
                null);
        return new Characterization(control, settings, state, new RunLog(control, state));
    }

    /**
     * Gravity model used when the declaration does not set one. Velocity bindings have no gravity term. Position
     * bindings use the cosine model for rotary mechanisms and a constant for linear ones.
     */
    static Gravity defaultGravity(ControlBinding<?> control) {
        if (control.mode() == ControlMode.VELOCITY) {
            return Gravity.NONE;
        }
        return Gravity.of(control.motionKind());
    }

    /**
     * Motor turns per mechanism turn, read from the binding's feedback encoder. The encoder position is
     * {@code raw * gearRatio * conversion} in its measured unit, so one raw turn is
     * {@code gearRatio * conversion / unitsPerRevolution} mechanism turns. Without an encoder, or when the encoder
     * measures a distance, the default of 1 is used; {@link #gearRatio(double)} overrides it either way.
     */
    static double gearFromFeedback(ControlBinding<?> control) {
        if (control.feedback() == null
                || !(control.feedback().position() instanceof EncoderDevice.PositionFeedback feedback)) {
            return 1.0;
        }
        EncoderDevice encoder = feedback.encoder();
        if (!(encoder.measuresUnit() instanceof AngleUnit angle)) {
            return 1.0;
        }
        double unitsPerRevolution = angle.fromBaseUnits(2.0 * Math.PI);
        double mechanismTurnsPerRawTurn = encoder.gearRatio() * encoder.conversion() / unitsPerRevolution;
        if (!(mechanismTurnsPerRawTurn > 0.0) || !Double.isFinite(mechanismTurnsPerRawTurn)) {
            return 1.0;
        }
        return 1.0 / mechanismTurnsPerRawTurn;
    }

    private static RecursiveLeastSquares newRunning(Gravity gravity) {
        return new RecursiveLeastSquares(Identification.columns(gravity), FORGETTING_FACTOR, INITIAL_COVARIANCE);
    }

    private Map<String, Action> buildActions() {
        Action quasistaticForward = engine.quasistaticForward();
        Action quasistaticReverse = engine.quasistaticReverse();
        Action dynamicForward = engine.dynamicForward();
        Action dynamicReverse = engine.dynamicReverse();
        Map<String, Action> map = new LinkedHashMap<>();
        map.put("QuasistaticForward", quasistaticForward);
        map.put("QuasistaticReverse", quasistaticReverse);
        map.put("DynamicForward", dynamicForward);
        map.put("DynamicReverse", dynamicReverse);
        map.put("Routine", Actions.then(Actions.then(Actions.then(quasistaticForward, quasistaticReverse),
                dynamicForward), dynamicReverse));
        map.put("FrictionSweep", Actions.then(quasistaticForward, quasistaticReverse));
        map.put("Backlash", Actions.then(quasistaticForward, quasistaticReverse));
        map.put("Latency", dynamicForward);
        map.put("FitModel", Actions.run(this::doFitModel));
        map.put("AdoptRunningEstimate", Actions.run(this::doAdoptRunningEstimate));
        map.put("Rebaseline", Actions.run(this::doRebaseline));
        return map;
    }

    // ----- settings: safety limits (typed) -----

    /** Largest voltage any test applies. Must be positive and finite. Ramps stop at it; steps hold it. */
    public Characterization maxVoltage(Voltage volts) {
        return copy(withSettings(s -> new Settings(positive(volts.in(Units.Volts), "maxVoltage"),
                s.rampVoltsPerSecond(), s.timeoutSeconds(), s.minNormalized(), s.maxNormalized(),
                s.learnContinuously(), s.driftTolerance(), s.gearRatio(), s.payloadGate())));
    }

    /**
     * Quasistatic ramp rate, as a voltage per time. Built with {@code Volts.of(1).per(Seconds).of(0.5)} style
     * measures or any measure of volts per second; the dimension is checked when the setting is applied.
     */
    public Characterization rampRate(edu.wpi.first.units.measure.Velocity<edu.wpi.first.units.VoltageUnit> rate) {
        return copy(withSettings(s -> new Settings(s.maxVolts(),
                positive(rate.baseUnitMagnitude() / UnitConversions.baseScale(Units.Volts), "rampRate"),
                s.timeoutSeconds(), s.minNormalized(), s.maxNormalized(), s.learnContinuously(),
                s.driftTolerance(), s.gearRatio(), s.payloadGate())));
    }

    /** Per-test safety timeout. Must be positive. A held test ends at the timeout and does not restart until released. */
    public Characterization timeout(Time time) {
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(),
                positive(time.in(Units.Seconds), "timeout"),
                s.minNormalized(), s.maxNormalized(), s.learnContinuously(), s.driftTolerance(), s.gearRatio(),
                s.payloadGate())));
    }

    /**
     * Travel range for a rotary mechanism. A test ends one cycle after the position leaves [min, max]. Both bounds are
     * required; the range is measured from the feedback position, so it must match the mechanism's rotation.
     */
    public Characterization range(Angle min, Angle max) {
        return withRange(min, max, MotionKind.ROTARY, Units.Rotations);
    }

    /** Travel range for a linear mechanism. A test ends one cycle after the position leaves [min, max]. */
    public Characterization range(Distance min, Distance max) {
        return withRange(min, max, MotionKind.LINEAR, Units.Meters);
    }

    private Characterization withRange(edu.wpi.first.units.Measure<?> min,
            edu.wpi.first.units.Measure<?> max, MotionKind expected, edu.wpi.first.units.Unit base) {
        Objects.requireNonNull(min, "min");
        Objects.requireNonNull(max, "max");
        if (engine.unit().angular() != (expected == MotionKind.ROTARY)) {
            throw new IllegalArgumentException("Characterization range must be a " + expected
                    + " measure for this binding's feedback.");
        }
        double low = base.fromBaseUnits(min.baseUnitMagnitude());
        double high = base.fromBaseUnits(max.baseUnitMagnitude());
        if (!(low < high)) {
            throw new IllegalArgumentException("Characterization range minimum must be below its maximum.");
        }
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(), s.timeoutSeconds(),
                low, high, s.learnContinuously(), s.driftTolerance(), s.gearRatio(), s.payloadGate())));
    }

    // ----- settings: model and logging -----

    /** Chooses the gravity model fitted by the tests. The default follows the binding; see {@link Gravity}. */
    public Characterization gravity(Gravity gravity) {
        Objects.requireNonNull(gravity, "gravity");
        state.gravity = gravity;
        state.running = newRunning(gravity);
        state.gate.reset();
        return this;
    }

    /** Name of the routine in the exported log. Defaults to the output motor's name. */
    public Characterization logName(String name) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("Characterization log name must not be blank.");
        }
        state.logName = name;
        return this;
    }

    /**
     * Sends every test sample to {@code log} (for example a custom logger or a test recorder) instead of the default
     * WPILib log. Set this before the first test runs.
     */
    public Characterization exportTo(SysIdLog log) {
        state.export = Objects.requireNonNull(log, "log");
        state.exportResolved = true;
        return this;
    }

    /** Enables recursive least-squares learning from normal operation. Never replaces the active model. */
    public Characterization learnContinuously(boolean enabled) {
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(), s.timeoutSeconds(),
                s.minNormalized(), s.maxNormalized(), enabled, s.driftTolerance(), s.gearRatio(),
                s.payloadGate())));
    }

    /** Relative band (for example 0.25 for 25%) beyond which a parameter counts as drifted. */
    public Characterization driftTolerance(double fraction) {
        requirePositive(fraction, "driftTolerance");
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(), s.timeoutSeconds(),
                s.minNormalized(), s.maxNormalized(), s.learnContinuously(), fraction, s.gearRatio(),
                s.payloadGate())));
    }

    /**
     * Motor turns per mechanism turn (the reduction). Used to convert the identified inertia and
     * to report external load in motor-side terms. Defaults to the feedback encoder's reduction, else 1.
     */
    public Characterization gearRatio(double motorPerMechanism) {
        requirePositive(motorPerMechanism, "gearRatio");
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(), s.timeoutSeconds(),
                s.minNormalized(), s.maxNormalized(), s.learnContinuously(), s.driftTolerance(),
                motorPerMechanism, s.payloadGate())));
    }

    /** Collects payload samples only while the supplier is true (for example while holding a game piece). */
    public Characterization payloadWhen(BooleanSupplier loaded) {
        Objects.requireNonNull(loaded, "loaded");
        return copy(withSettings(s -> new Settings(s.maxVolts(), s.rampVoltsPerSecond(), s.timeoutSeconds(),
                s.minNormalized(), s.maxNormalized(), s.learnContinuously(), s.driftTolerance(), s.gearRatio(),
                loaded)));
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

    /**
     * Keeps the active model, the continuous-learning estimate and the snapshot history in {@code storage} under
     * {@code key} (the characterization's field path). Restores whatever is stored there now: the active model
     * becomes the active model and drift baseline, the stored estimate seeds the running estimate, and the history is
     * reloaded. From then on fits, adopts, rebaselines and history snapshots are written through.
     *
     * @param storage destination, such as a persistent store
     * @param key stable key, for example {@code "turret/angle"}
     * @return this declaration
     */
    public Characterization persistAs(ModelStorage storage, String key) {
        Objects.requireNonNull(storage, "storage");
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("Characterization storage key must not be blank.");
        }
        state.storage = storage;
        state.storageKey = key;
        storage.load(key).ifPresent(this::defaults);
        storage.loadRunning(key).ifPresent(this::reseedRunning);
        storage.loadHistory(key).forEach(state.history::add);
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

    private static UnaryOperator<Settings> withSettings(UnaryOperator<Settings> edit) {
        return edit;
    }

    private Characterization copy(UnaryOperator<Settings> edit) {
        return new Characterization(control, edit.apply(settings), state, runLog);
    }

    private static double positive(double value, String name) {
        requirePositive(value, name);
        return value;
    }

    private static void requirePositive(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException("Characterization " + name + " must be positive and finite.");
        }
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

    /**
     * The whole routine in one action: forward quasistatic, reverse quasistatic, forward dynamic, reverse dynamic.
     * Hold it to run all four; each test uses the limits above. Fit afterwards with {@link #fitModel()}.
     */
    public Action routine() {
        return actions.get("Routine");
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

    // ----- results -----

    /** Returns the active model. */
    public MechanismModel model() {
        return state.active;
    }

    /** Returns the drift baseline, or null before the first fit or defaults. */
    public MechanismModel baseline() {
        return state.baseline;
    }

    /** Static friction kS of the active model, as a voltage. */
    public Voltage kS() {
        return Units.Volts.of(state.active.kineticFrictionVolts());
    }

    /** Gravity amplitude kG of the active model, as a voltage (zero for {@link Gravity#NONE}). */
    public Voltage kG() {
        return Units.Volts.of(state.active.kG());
    }

    /**
     * Velocity feedforward kV of the active model, in volts per (base unit per second). Base units are rotations
     * for rotary bindings and meters for linear ones.
     */
    public double kV() {
        return state.active.kV();
    }

    /** Acceleration feedforward kA of the active model, in volts per (base unit per second squared). */
    public double kA() {
        return state.active.kA();
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
        Gravity gravity = state.gravity;
        double deadband = Identification.DEFAULT_VELOCITY_DEADBAND;
        FeedforwardFit fit = Identification.fitFeedforward(runs, gravity, deadband);
        double staticVolts = Identification.staticFriction(runs, gravity, fit, STATIC_BREAKAWAY_SPEED)
                .orElse(fit.kineticStaticVolts());
        FrictionTable table = Identification.frictionTable(runs, gravity, fit, deadband, FRICTION_BINS);
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
                torque = constants.raw().torqueConstantNmPerAmp();
                resistance = constants.resistance().in(Units.Ohms);
            }
        }
        return new MechanismModel(kind, staticVolts, fit.kineticStaticVolts(), fit.kV(), fit.kA(), fit.kG(),
                fit.gravityOffsetRadians(), table, backlash, latency, payloadDelta, torque, resistance,
                kinds.size(), kinds, settings.gearRatio(), fit.standardErrors(), fit.rSquared(), fit.samples());
    }

    /** Estimate of the external payload from holding samples, with confidence. */
    public Payload payload() {
        List<CharacterizationSample> holds = List.copyOf(state.holds);
        if (holds.size() < MIN_PAYLOAD_SAMPLES || state.gravity == Gravity.NONE) {
            return new Payload(0.0, Double.NaN, holds.size(), false);
        }
        MechanismModel base = state.active;
        FeedforwardFit baseline = new FeedforwardFit(base.kineticFrictionVolts(), base.kV(), base.kA(),
                base.kG(), base.gravityOffsetRadians(), Map.of(), 0.0, 0.0, 0);
        Identification.PayloadEstimate estimate = Identification.payload(holds, state.gravity, baseline,
                base.staticFrictionVolts());
        boolean spread = state.gravity != Gravity.COSINE || cosineSpread(holds) >= MIN_COS_SPREAD;
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
        switch (state.gravity) {
            case NONE -> {
                kG = 0.0;
                offset = 0.0;
            }
            case CONSTANT -> {
                kG = c[3];
                offset = 0.0;
            }
            case COSINE -> {
                kG = Math.hypot(c[3], c[4]);
                offset = Math.atan2(-c[4], c[3]);
            }
            default -> throw new IllegalStateException("Unhandled gravity " + state.gravity);
        }
        return new MechanismModel(active.motionKind(), active.staticFrictionVolts(), c[0], c[1], c[2], kG,
                offset, active.friction(), active.backlash(), active.latencySeconds(),
                active.payloadDelta(), active.motorTorqueConstant(), active.motorResistance(),
                active.motorCount(), active.motorKinds(), active.gearRatio(), active.standardErrors(),
                active.rSquared(), (int) Math.min(Integer.MAX_VALUE, state.running.updates()));
    }

    /** Compares the running estimate with the baseline. */
    public DriftReport drift() {
        return DriftMonitor.evaluate(state.baseline, runningEstimate(), settings.driftTolerance());
    }

    /** Returns whether continuous learning is enabled. */
    public boolean learnContinuously() {
        return settings.learnContinuously();
    }

    /** Returns the gravity model in effect. */
    public Gravity gravity() {
        return state.gravity;
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

    /** Motor turns per mechanism turn in effect (encoder-derived unless overridden). */
    double gearRatio() {
        return settings.gearRatio();
    }

    /** Returns the store key set by {@link #persistAs}, or null when the characterization is not persisted. */
    public String storageKey() {
        return state.storageKey;
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
        double deadband = Identification.DEFAULT_VELOCITY_DEADBAND;
        if (settings.learnContinuously()) {
            double[] regressor = Identification.features(state.gravity, position, velocity, acceleration, deadband);
            ExcitationGate.Decision decision = state.gate.offer(regressor, velocity, acceleration);
            state.excitation = decision.reason();
            if (decision.accepted()) {
                state.running.update(regressor, appliedVolts);
            }
        }
        BooleanSupplier payloadGate = settings.payloadGate();
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
        persist();
    }

    /** Writes the active model, running estimate and history through when a key is set. */
    private void persist() {
        if (state.storageKey == null) {
            return;
        }
        state.storage.save(state.storageKey, state.active);
        state.storage.saveRunning(state.storageKey, runningEstimate());
        state.storage.saveHistory(state.storageKey, state.history.snapshots());
    }

    private void doFitModel() {
        MechanismModel fitted = fitRuns();
        state.active = fitted;
        state.baseline = fitted;
        reseedRunning(fitted);
        persist();
    }

    private void doAdoptRunningEstimate() {
        state.active = runningEstimate();
        reseedRunning(state.active);
        persist();
    }

    private void doRebaseline() {
        state.baseline = state.active;
        snapshot("rebaseline");
    }

    private void reseedRunning(MechanismModel model) {
        Gravity gravity = state.gravity;
        state.running = newRunning(gravity);
        double[] seed = switch (gravity) {
            case NONE -> new double[] {model.kineticFrictionVolts(), model.kV(), model.kA()};
            case CONSTANT -> new double[] {model.kineticFrictionVolts(), model.kV(), model.kA(), model.kG()};
            case COSINE -> new double[] {
                model.kineticFrictionVolts(), model.kV(), model.kA(),
                model.kG() * Math.cos(model.gravityOffsetRadians()),
                -model.kG() * Math.sin(model.gravityOffsetRadians())};
        };
        state.running.seed(seed);
        state.gate.reset();
    }

    private static SysIdLog discoverExport(String routine, String motor) {
        Iterator<SysIdLogProvider> providers = ServiceLoader.load(SysIdLogProvider.class).iterator();
        while (true) {
            try {
                if (!providers.hasNext()) {
                    return null;
                }
                SysIdLog log = providers.next().open(routine, motor);
                if (log != null) {
                    return log;
                }
            } catch (ServiceConfigurationError error) {
                throw new IllegalStateException("Unable to load the Athena SysId log provider.", error);
            }
        }
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

    /**
     * Receives the routine's samples: converts them into run data and forwards each raw sample to the export log.
     * A sample's state is measured after the previous cycle's command has acted, so the acceleration between two
     * samples is produced by the earlier command. Each run row therefore pairs that command with the acceleration of
     * its interval, and with the interval's mean position and velocity. The first sample of a run has no interval and
     * is not stored.
     */
    private static final class RunLog implements SysIdLog {
        private final ControlBinding<?> control;
        private final State owner;

        RunLog(ControlBinding<?> control, State owner) {
            this.control = control;
            this.owner = owner;
        }

        /** Resolves the export log on first use: the explicit one, else the WPILib log provider when one is installed. */
        private SysIdLog exportLog() {
            if (!owner.exportResolved) {
                owner.exportResolved = true;
                String routine = owner.logName != null ? owner.logName : control.output().defaultName();
                owner.export = discoverExport(routine, control.output().defaultName());
            }
            return owner.export;
        }

        @Override
        public synchronized void record(SysIdSample sample) {
            SysIdLog export = exportLog();
            if (export != null) {
                export.record(sample);
            }
            double now = owner.clock.getAsDouble();
            CharacterizationSample raw = new CharacterizationSample(sample.state(), now,
                    sample.appliedVoltage(), sample.position(), sample.velocity(), 0.0, sample.currentAmps());
            CharacterizationSample previous = owner.previousRun;
            owner.previousRun = raw;
            if (previous == null || previous.state() != sample.state()) {
                return;
            }
            double dt = now - previous.timeSeconds();
            if (dt <= 1.0e-6) {
                return;
            }
            double acceleration = (raw.velocity() - previous.velocity()) / dt;
            CharacterizationSample converted = new CharacterizationSample(sample.state(), now,
                    previous.voltage(),
                    0.5 * (previous.position() + raw.position()),
                    0.5 * (previous.velocity() + raw.velocity()),
                    acceleration,
                    previous.currentAmps());
            addBounded(owner.runs, converted, RUN_CAPACITY);
        }

        @Override
        public synchronized void end() {
            owner.previousRun = null;
            if (owner.export != null) {
                owner.export.end();
            }
        }
    }
}
