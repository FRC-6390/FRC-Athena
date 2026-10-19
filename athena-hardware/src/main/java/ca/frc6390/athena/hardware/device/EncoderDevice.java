package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.DeviceActionFactories;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Reusable encoder declaration for robot constants.
 *
 * <p>{@link #measuresUnit()} is the WPILib unit the encoder's {@link #position()} and
 * {@link #velocity()} are expressed in. It is always set: a declaration starts as rotations
 * ({@link #of(EncoderKind, int)}) and changes with {@link #measures(AngleUnit)} or
 * {@link #measures(DistanceUnit)}. Typed bindings check it at declaration time.</p>
 */
public record EncoderDevice(
        EncoderSource source,
        boolean isInverted,
        double gearRatio,
        double conversion,
        double offset,
        FailurePolicy failurePolicy,
        boolean baselineConfigApplied,
        Unit measuresUnit) implements PositionSignal, VelocitySignal {
    private static final RuntimeBindings<EncoderDevice, EncoderHandle> RUNTIMES = new RuntimeBindings<>();

    public static EncoderDevice of(EncoderKind kind, int id) {
        return connected(kind, "rio", new HardwareAddress.Can(id));
    }

    static EncoderDevice connected(EncoderKind kind, String bus, HardwareAddress connection) {
        return new EncoderDevice(
                new EncoderSource.Standalone(kind, bus, connection),
                false,
                1.0,
                1.0,
                0.0,
                FailurePolicy.DISABLE_MECHANISM,
                false,
                Units.Rotations);
    }

    static EncoderDevice integratedMotor(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.IntegratedMotor(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                motor.failurePolicy(),
                false,
                Units.Rotations);
    }

    static EncoderDevice motorAbsolute(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.MotorAbsolute(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                motor.failurePolicy(),
                false,
                Units.Rotations);
    }

    public EncoderDevice {
        Objects.requireNonNull(source, "source");
        if (!Double.isFinite(gearRatio) || gearRatio <= 0.0) {
            throw new IllegalArgumentException("Encoder gear ratio must be positive.");
        }
        if (!Double.isFinite(conversion) || conversion <= 0.0) {
            throw new IllegalArgumentException("Encoder conversion must be positive.");
        }
        if (!Double.isFinite(offset)) {
            throw new IllegalArgumentException("Encoder offset must be finite.");
        }
        failurePolicy = failurePolicy == null ? FailurePolicy.DISABLE_MECHANISM : failurePolicy;
        if (measuresUnit == null) {
            throw new IllegalArgumentException("Encoder must declare its unit with measures(...).");
        }
        if (!(measuresUnit instanceof AngleUnit) && !(measuresUnit instanceof DistanceUnit)) {
            throw new IllegalArgumentException("Encoder measures must be an angle or distance unit.");
        }
    }

    public EncoderKind kind() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.kind();
        }
        return EncoderKinds.INTEGRATED_MOTOR;
    }

    public int id() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return requireCan(standalone).id();
        }
        return motorSource().id();
    }

    public HardwareAddress connection() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.connection();
        }
        throw new IllegalStateException(
                "Motor-integrated encoder " + defaultName() + " does not have a standalone connection.");
    }

    public String bus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.bus();
        }
        return motorSource().canbus();
    }

    public String canbus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            requireCan(standalone);
            return standalone.bus();
        }
        return motorSource().canbus();
    }

    public EncoderDevice canbus(String canbus) {
        if (source instanceof EncoderSource.Standalone standalone) {
            requireCan(standalone);
            return new EncoderDevice(
                    new EncoderSource.Standalone(standalone.kind(), canbus, standalone.connection()),
                    isInverted,
                    gearRatio,
                    conversion,
                    offset,
                    failurePolicy,
                    baselineConfigApplied,
                    measuresUnit);
        }
        return this;
    }

    public EncoderDevice inverted() {
        return inverted(true);
    }

    public EncoderDevice inverted(boolean inverted) {
        return new EncoderDevice(source, inverted, gearRatio, conversion, offset, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    /**
     * Declares the reduction: a value of 10 means ten sensor rotations per mechanism rotation (a 10:1
     * reduction). Stores {@code 1.0 / reduction}, the same declaration as
     * {@code gearing(GearRatio.reduction(10.0, 1.0))}.
     *
     * @param reduction sensor rotations per mechanism rotation, finite and non-zero
     * @return updated declaration
     */
    public EncoderDevice reduction(double reduction) {
        if (!Double.isFinite(reduction) || reduction == 0.0) {
            throw new IllegalArgumentException("Encoder reduction must be finite and non-zero.");
        }
        return withGearRatio(1.0 / reduction);
    }

    /**
     * Declares the sensor-to-mechanism gearing from a {@link GearRatio}.
     *
     * @param gearing sensor-to-mechanism gearing
     * @return updated declaration
     */
    public EncoderDevice gearing(GearRatio gearing) {
        Objects.requireNonNull(gearing, "gearing");
        return withGearRatio(gearing.ratio());
    }

    private EncoderDevice withGearRatio(double gearRatio) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    /**
     * Declares that {@link #position()} and {@link #velocity()} are expressed in {@code unit} and
     * sets one full revolution of the encoder in that unit (360 for {@link Units#Degrees}, 1 for
     * {@link Units#Rotations}).
     *
     * @param unit angle unit reported by this encoder
     * @return updated declaration
     */
    public EncoderDevice measures(AngleUnit unit) {
        Objects.requireNonNull(unit, "unit");
        double oneRevolution = unit.fromBaseUnits(2.0 * Math.PI);
        return new EncoderDevice(source, isInverted, gearRatio, oneRevolution, offset,
                failurePolicy, baselineConfigApplied, unit);
    }

    /**
     * Declares that {@link #position()} and {@link #velocity()} are expressed in {@code unit}, with
     * one encoder revolution equal to one {@code unit} until {@link #travelPerRevolution(Distance)}
     * sets the real travel (for example a wheel circumference).
     *
     * @param unit distance unit reported by this encoder
     * @return updated declaration
     */
    public EncoderDevice measures(DistanceUnit unit) {
        Objects.requireNonNull(unit, "unit");
        return new EncoderDevice(source, isInverted, gearRatio, 1.0, offset, failurePolicy, baselineConfigApplied,
                unit);
    }

    /**
     * Sets the linear travel of one encoder revolution after the reduction, for example a wheel
     * circumference. Requires {@link #measures(DistanceUnit)} first.
     *
     * @param travel travel per revolution
     * @return updated declaration
     */
    public EncoderDevice travelPerRevolution(Distance travel) {
        Objects.requireNonNull(travel, "travel");
        if (!(measuresUnit instanceof DistanceUnit distance)) {
            throw new IllegalStateException("travelPerRevolution requires measures(DistanceUnit) first.");
        }
        return withConversion(UnitConversions.value(travel, distance));
    }

    private EncoderDevice withConversion(double conversion) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    public EncoderDevice offset(double offset) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    /** Selects how Athena responds if this encoder cannot be created or stops responding. */
    public EncoderDevice failurePolicy(FailurePolicy policy) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, Objects.requireNonNull(policy, "policy"), baselineConfigApplied, measuresUnit);
    }

    /**
     * Lets the backend overwrite sensor configuration stored on the device with the raw baseline
     * Athena expects (for a CANcoder: magnet offset 0, counter-clockwise positive, discontinuity
     * 0.5) when the encoder is activated.
     *
     * <p>Off by default: offsets in robot code are usually measured against whatever is stored on
     * the device, so overwriting it would shift them. Without this, the backend only warns when the
     * stored configuration differs from the baseline. Re-measure {@link #offset(double)} after
     * enabling it.</p>
     *
     * @return updated declaration
     */
    public EncoderDevice applyBaselineConfig() {
        return applyBaselineConfig(true);
    }

    /**
     * Enables or disables writing the backend's baseline sensor configuration at activation.
     *
     * @param apply true to overwrite stored device configuration
     * @return updated declaration
     */
    public EncoderDevice applyBaselineConfig(boolean apply) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, failurePolicy, apply,
                measuresUnit);
    }

    /**
     * Creates an action that sets this encoder's relative position.
     *
     * @param position relative position in this declaration's configured units
     * @return position-setting action
     */
    public DeviceAction setPosition(double position) {
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        return DeviceActionFactories.get().encoderSetPosition(this, position);
    }

    /**
     * Reads configured mechanism position from the runtime's latest cached snapshot.
     *
     * @return configured mechanism position
     */
    @Override
    public double position() {
        return positionFromRotations(runtimeHandle().positionRotations());
    }

    /**
     * Returns the current absolute position in configured mechanism units.
     *
     * @return absolute position
     */
    public double absolutePosition() {
        return positionFromRotations(runtimeHandle().absolutePositionRotations());
    }

    /** Returns a feedback signal backed by this encoder's absolute-position channel. */
    public PositionSignal absoluteSignal() {
        return new AbsolutePositionSignal(this);
    }

    /**
     * Reads configured mechanism velocity from the runtime's latest cached snapshot.
     *
     * @return configured mechanism velocity
     */
    @Override
    public double velocity() {
        return velocityFromRotationsPerSecond(runtimeHandle().velocityRotationsPerSecond());
    }

    /** Binds this declaration to one runtime-owned cached encoder handle. */
    public AutoCloseable bindRuntime(RuntimeScope scope, EncoderHandle handle) {
        return RUNTIMES.bind(this, scope, handle);
    }

    @Override
    public java.util.List<?> dependencies() {
        return java.util.List.of(this);
    }

    /**
     * Converts raw sensor rotations into configured mechanism position.
     *
     * @param rotations raw sensor rotations
     * @return configured position
     */
    public double positionFromRotations(double rotations) {
        double directed = isInverted ? -rotations : rotations;
        return (directed - offset) * gearRatio * conversion;
    }

    /**
     * Converts raw sensor velocity into configured mechanism velocity.
     *
     * @param rotationsPerSecond raw sensor rotations per second
     * @return configured velocity
     */
    public double velocityFromRotationsPerSecond(double rotationsPerSecond) {
        double directed = isInverted ? -rotationsPerSecond : rotationsPerSecond;
        return directed * gearRatio * conversion;
    }

    /**
     * Wraps a value in this encoder's configured unit ({@link #measuresUnit()}) as a WPILib measure, so it can be
     * passed to the typed conversions. Use it for values read from the configured position, such as a stored
     * {@link #setPosition(double)} target.
     *
     * @param value value in {@link #measuresUnit()}
     * @return the same value as a measure
     */
    @SuppressWarnings({"unchecked", "rawtypes"})
    public Measure<?> measure(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        return measuresUnit.of(value);
    }

    /**
     * Converts a configured mechanism position into raw sensor rotations. The measure must be in this encoder's
     * dimension (an angle for {@link #measures(AngleUnit)}, a distance for {@link #measures(DistanceUnit)}); it is
     * converted to {@link #measuresUnit()} before the inverse.
     *
     * @param position configured position, for example {@code Degrees.of(90)}
     * @return raw sensor rotations
     * @throws IllegalArgumentException when the measure is not in this encoder's dimension or is not finite
     */
    public double rotationsFromPosition(Measure<?> position) {
        Objects.requireNonNull(position, "position");
        double value = UnitConversions.value(position, measuresUnit);
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        double directed = value / (gearRatio * conversion) + offset;
        return isInverted ? -directed : directed;
    }

    public String defaultName() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return sanitize(kind().key()) + "_" + sanitize(standalone.connection().identity()) + "_"
                    + standalone.connection().primaryAddress();
        }
        return sanitize(kind().key()) + "_" + motorSource().id();
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    public sealed interface EncoderSource
            permits EncoderSource.Standalone, EncoderSource.IntegratedMotor, EncoderSource.MotorAbsolute {
        record Standalone(EncoderKind kind, String bus, HardwareAddress connection) implements EncoderSource {
            public Standalone {
                Objects.requireNonNull(kind, "kind");
                bus = bus == null || bus.isBlank() ? "rio" : bus;
                Objects.requireNonNull(connection, "connection");
            }
        }

        record IntegratedMotor(MotorDevice motor) implements EncoderSource {
            public IntegratedMotor {
                Objects.requireNonNull(motor, "motor");
            }
        }

        record MotorAbsolute(MotorDevice motor) implements EncoderSource {
            public MotorAbsolute {
                Objects.requireNonNull(motor, "motor");
            }
        }
    }

    private record AbsolutePositionSignal(EncoderDevice encoder) implements PositionSignal {
        @Override
        public double position() {
            return encoder.absolutePosition();
        }

        @Override
        public java.util.List<?> dependencies() {
            return java.util.List.of(encoder);
        }
    }

    private EncoderHandle runtimeHandle() {
        ActionContext current = RuntimeHardwareAccess.current();
        if (current != null) {
            return current.encoder(this);
        }
        EncoderHandle bound = RUNTIMES.find(this);
        if (bound != null) {
            return bound;
        }
        return RUNTIMES.get(this, "Encoder " + defaultName());
    }

    private MotorDevice motorSource() {
        if (source instanceof EncoderSource.IntegratedMotor integrated) {
            return integrated.motor();
        }
        return ((EncoderSource.MotorAbsolute) source).motor();
    }

    private static HardwareAddress.Can requireCan(EncoderSource.Standalone standalone) {
        if (standalone.connection() instanceof HardwareAddress.Can can) {
            return can;
        }
        throw new IllegalStateException("Encoder " + standalone.kind().key() + " is not connected over CAN.");
    }
}
