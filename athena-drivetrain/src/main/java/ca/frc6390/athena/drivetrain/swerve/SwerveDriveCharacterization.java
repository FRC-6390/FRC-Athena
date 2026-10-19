package ca.frc6390.athena.drivetrain.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.LifecycleAction;
import ca.frc6390.athena.mechanism.model.CharacterizationSample;
import ca.frc6390.athena.mechanism.model.Gravity;
import ca.frc6390.athena.mechanism.model.Identification;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.VoltageUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Whole-drivetrain drive characterization for a swerve layout.
 *
 * <p>Every test points all modules straight (steer held at zero), applies the same voltage to every drive motor,
 * and records the average measured drive velocity in meters per second. The four SysId phases are exposed as
 * actions; holding one runs it and releasing (or the timeout) ends it. Drive and steer are owned for the duration
 * of each test and released afterwards.</p>
 *
 * <pre>{@code
 * public final SwerveDriveCharacterization driveTest = kinematics.characterization()
 *         .maxVoltage(Volts.of(6.0))
 *         .timeout(Seconds.of(8.0));
 * }</pre>
 *
 * <p>Settings are read when a test starts. Call them before the actions are first used. {@link #kS()},
 * {@link #kV()} and {@link #kA()} fit the recorded samples with no gravity term.</p>
 */
public final class SwerveDriveCharacterization {
    private static final double DEFAULT_MAX_VOLTS = 4.0;
    private static final double DEFAULT_RAMP_VOLTS_PER_SECOND = 0.5;
    private static final double DEFAULT_TIMEOUT_SECONDS = 10.0;

    private final List<SwerveModule> modules;
    private final List<Object> ownership;
    private double maxVolts = DEFAULT_MAX_VOLTS;
    private double rampVoltsPerSecond = DEFAULT_RAMP_VOLTS_PER_SECOND;
    private double timeoutSeconds = DEFAULT_TIMEOUT_SECONDS;
    private final List<CharacterizationSample> samples = new ArrayList<>();
    private Identification.FeedforwardFit fit;
    private Action quasistaticForward;
    private Action quasistaticReverse;
    private Action dynamicForward;
    private Action dynamicReverse;

    SwerveDriveCharacterization(List<SwerveKinematics.Module> positioned) {
        modules = positioned.stream().map(SwerveKinematics.Module::module).toList();
        ownership = modules.stream()
                .flatMap(module -> List.<Object>of(module.drive.get(), module.steer.get(), module.angle.get()).stream())
                .toList();
    }

    /** Largest voltage applied to every drive motor. Dynamic tests hold it; quasistatic ramps stop at it. */
    public SwerveDriveCharacterization maxVoltage(Voltage volts) {
        maxVolts = positive(Objects.requireNonNull(volts, "volts").in(Volts), "max voltage");
        return this;
    }

    /** Quasistatic ramp rate, for example {@code Volts.of(1).per(Seconds).of(0.5)} style units. */
    public SwerveDriveCharacterization rampRate(Velocity<VoltageUnit> rate) {
        rampVoltsPerSecond = positive(Objects.requireNonNull(rate, "rate").in(Volts.per(Seconds)), "ramp rate");
        return this;
    }

    /** Safety timeout for each test. */
    public SwerveDriveCharacterization timeout(Time timeout) {
        timeoutSeconds = positive(Objects.requireNonNull(timeout, "timeout").in(Seconds), "timeout");
        return this;
    }

    /** Forward quasistatic ramp. Hold to run. */
    public Action quasistaticForward() {
        return quasistaticForward = lazy(quasistaticForward, SysIdState.QUASISTATIC_FORWARD);
    }

    /** Reverse quasistatic ramp. Hold to run. */
    public Action quasistaticReverse() {
        return quasistaticReverse = lazy(quasistaticReverse, SysIdState.QUASISTATIC_REVERSE);
    }

    /** Forward dynamic step held at the voltage limit. */
    public Action dynamicForward() {
        return dynamicForward = lazy(dynamicForward, SysIdState.DYNAMIC_FORWARD);
    }

    /** Reverse dynamic step held at the negated voltage limit. */
    public Action dynamicReverse() {
        return dynamicReverse = lazy(dynamicReverse, SysIdState.DYNAMIC_REVERSE);
    }

    /** Forward quasistatic, reverse quasistatic, forward dynamic, reverse dynamic, in that order. */
    public Action routine() {
        return Actions.then(Actions.then(Actions.then(quasistaticForward(), quasistaticReverse()),
                dynamicForward()), dynamicReverse());
    }

    /** Samples recorded by every test since the last {@link #clear()}. */
    public synchronized List<CharacterizationSample> samples() {
        return List.copyOf(samples);
    }

    /** Drops the recorded samples and the fit. */
    public synchronized void clear() {
        samples.clear();
        fit = null;
    }

    /** Static friction voltage fitted from the samples (gravity NONE). */
    public Voltage kS() {
        return Volts.of(fit().kineticStaticVolts());
    }

    /** Volts per meter per second, fitted from the samples (gravity NONE). */
    public double kV() {
        return fit().kV();
    }

    /** Volts per meter per second squared, fitted from the samples (gravity NONE). */
    public double kA() {
        return fit().kA();
    }

    /** Full fit with standard errors and fit quality. */
    public synchronized Identification.FeedforwardFit fit() {
        if (fit == null) {
            fit = Identification.fitFeedforward(samples, Gravity.NONE, Identification.DEFAULT_VELOCITY_DEADBAND);
        }
        return fit;
    }

    private Action lazy(Action current, SysIdState state) {
        return current != null ? current : phase(state);
    }

    private Action phase(SysIdState state) {
        Run run = new Run(state);
        Action drive = Actions.compute(context -> run.cycle(context.dtSeconds()), ownership);
        Action lifecycle = LifecycleAction.builder("SwerveDriveCharacterization/" + state)
                .onStart(run::start)
                .until(run::finished)
                .build();
        return Actions.race(lifecycle, drive);
    }

    private double voltage(SysIdState state, double elapsedSeconds) {
        return switch (state) {
            case QUASISTATIC_FORWARD -> Math.min(maxVolts, Math.max(0.0, elapsedSeconds) * rampVoltsPerSecond);
            case QUASISTATIC_REVERSE -> -Math.min(maxVolts, Math.max(0.0, elapsedSeconds) * rampVoltsPerSecond);
            case DYNAMIC_FORWARD -> maxVolts;
            case DYNAMIC_REVERSE -> -maxVolts;
            case NONE -> 0.0;
        };
    }

    private static double positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException("Drive characterization " + name + " must be positive.");
        }
        return value;
    }

    /** One test phase. Per-test state; reset on every start. */
    private final class Run {
        private final SysIdState state;
        private double elapsedSeconds;
        private double previousVelocity;
        private boolean started;

        private Run(SysIdState state) {
            this.state = state;
        }

        void start() {
            elapsedSeconds = 0.0;
            previousVelocity = 0.0;
            started = false;
        }

        boolean finished() {
            return elapsedSeconds >= timeoutSeconds;
        }

        /** Advances the test by one cycle and returns the hardware action that applies its voltage. */
        Action cycle(double dtSeconds) {
            double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
            return Actions.computeHardware(ownership, context -> {
                double position = 0.0;
                double velocity = 0.0;
                for (SwerveModule module : modules) {
                    position += RuntimeHardwareAccess.call(context, module.driveVelocity::position);
                    velocity += RuntimeHardwareAccess.call(context, module.driveVelocity::velocity);
                }
                position /= modules.size();
                velocity /= modules.size();
                boolean first = !started;
                started = true;
                if (!first) {
                    elapsedSeconds += dt;
                }
                double acceleration = first ? 0.0 : (velocity - previousVelocity) / dt;
                previousVelocity = velocity;
                double volts = voltage(state, elapsedSeconds);
                record(new CharacterizationSample(state, elapsedSeconds, volts, position, velocity,
                        acceleration, Double.NaN));
                List<Action> actions = new ArrayList<>(modules.size());
                for (SwerveModule module : modules) {
                    actions.add(module.characterizeDrive(volts));
                }
                return Actions.parallel(actions.toArray(Action[]::new));
            });
        }
    }

    private synchronized void record(CharacterizationSample sample) {
        samples.add(sample);
        fit = null;
    }
}
