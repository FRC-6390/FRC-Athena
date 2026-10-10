package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Reusable encoder declaration for robot constants.
 *
 * <p>{@link #measuresUnit()} is the WPILib unit the encoder's {@link #position()} and
 * {@link #velocity()} are expressed in, set by {@link #measures(AngleUnit)} or
 * {@link #measures(DistanceUnit)}. It is null for legacy declarations, which keep the
 * {@link EncoderUnit} / conversion contract. Typed bindings check it at declaration time.</p>
 */
public record EncoderDevice(
        EncoderSource source,
        boolean isInverted,
        double gearRatio,
        double conversion,
        double offset,
        EncoderUnit units,
        FailurePolicy failurePolicy,
        boolean baselineConfigApplied,
        Unit measuresUnit) implements PositionSignal, VelocitySignal {
    private static final RuntimeBindings<EncoderDevice, EncoderHandle> RUNTIMES = new RuntimeBindings<>();

    public EncoderDevice(
            EncoderSource source,
            boolean isInverted,
            double gearRatio,
            double conversion,
            double offset,
            EncoderUnit units,
            FailurePolicy failurePolicy,
            boolean baselineConfigApplied) {
        this(source, isInverted, gearRatio, conversion, offset, units, failurePolicy, baselineConfigApplied, null);
    }

    public EncoderDevice(
            EncoderSource source,
            boolean isInverted,
            double gearRatio,
            double conversion,
            double offset,
            EncoderUnit units,
            FailurePolicy failurePolicy) {
        this(source, isInverted, gearRatio, conversion, offset, units, failurePolicy, false);
    }

    public EncoderDevice(
            EncoderSource source,
            boolean isInverted,
            double gearRatio,
            double conversion,
            double offset,
            EncoderUnit units) {
        this(source, isInverted, gearRatio, conversion, offset, units, FailurePolicy.DISABLE_MECHANISM);
    }

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
                EncoderUnit.RAW,
                FailurePolicy.DISABLE_MECHANISM);
    }

    static EncoderDevice integratedMotor(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.IntegratedMotor(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                EncoderUnit.RAW,
                motor.failurePolicy());
    }

    static EncoderDevice motorAbsolute(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.MotorAbsolute(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                EncoderUnit.RAW,
                motor.failurePolicy());
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
        units = units == null ? EncoderUnit.RAW : units;
        failurePolicy = failurePolicy == null ? FailurePolicy.DISABLE_MECHANISM : failurePolicy;
        if (measuresUnit != null && !(measuresUnit instanceof AngleUnit) && !(measuresUnit instanceof DistanceUnit)) {
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
                    units,
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
        return new EncoderDevice(source, inverted, gearRatio, conversion, offset, units, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    public EncoderDevice gearRatio(double gearRatio) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    public EncoderDevice gearRatio(GearRatio gearRatio) {
        Objects.requireNonNull(gearRatio, "gearRatio");
        return gearRatio(gearRatio.ratio());
    }

    /**
     * Typed alternative to {@link #gearRatio(double)} that reads like the reduction on the robot: a
     * value of 10 means ten sensor rotations per mechanism rotation (a 10:1 reduction). Equivalent to
     * {@code gearRatio(1.0 / reduction)}, the same as {@code GearRatio.reduction(10.0, 1.0)}.
     *
     * @param reduction sensor rotations per mechanism rotation, finite and non-zero
     * @return updated declaration
     */
    public EncoderDevice reduction(double reduction) {
        if (!Double.isFinite(reduction) || reduction == 0.0) {
            throw new IllegalArgumentException("Encoder reduction must be finite and non-zero.");
        }
        return gearRatio(1.0 / reduction);
    }

    /**
     * Typed alternative to {@link #gearRatio(GearRatio)}.
     *
     * @param gearing sensor-to-mechanism gearing
     * @return updated declaration
     */
    public EncoderDevice gearing(GearRatio gearing) {
        return gearRatio(gearing);
    }

    /**
     * Declares that {@link #position()} and {@link #velocity()} are expressed in {@code unit} and
     * sets the conversion to one full revolution of the encoder in that unit (360 for
     * {@link Units#Degrees}, 1 for {@link Units#Rotations}). Call {@link #conversion(double)} after
     * this to override it.
     *
     * @param unit angle unit reported by this encoder
     * @return updated declaration
     */
    public EncoderDevice measures(AngleUnit unit) {
        Objects.requireNonNull(unit, "unit");
        EncoderUnit encoderUnit = units;
        if (unit.equals(Units.Degrees)) {
            encoderUnit = EncoderUnit.DEGREES;
        } else if (unit.equals(Units.Radians)) {
            encoderUnit = EncoderUnit.RADIANS;
        } else if (unit.equals(Units.Rotations)) {
            encoderUnit = EncoderUnit.ROTATIONS;
        }
        double oneRevolution = unit.fromBaseUnits(2.0 * Math.PI);
        return new EncoderDevice(source, isInverted, gearRatio, oneRevolution, offset, encoderUnit,
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
        EncoderUnit encoderUnit = unit.equals(Units.Meters) ? EncoderUnit.METERS : units;
        return new EncoderDevice(source, isInverted, gearRatio, 1.0, offset, encoderUnit,
                failurePolicy, baselineConfigApplied, unit);
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
        return conversion(UnitConversions.value(travel, distance));
    }

    public EncoderDevice conversion(double conversion) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    public EncoderDevice wheelDiameterMeters(double diameterMeters) {
        if (!Double.isFinite(diameterMeters) || diameterMeters <= 0.0) {
            throw new IllegalArgumentException("Wheel diameter must be positive.");
        }
        return conversion(Math.PI * diameterMeters);
    }

    public EncoderDevice wheelDiameterInches(double diameterInches) {
        return wheelDiameterMeters(diameterInches * 0.0254);
    }

    public EncoderDevice units(EncoderUnit units) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    public EncoderDevice offset(double offset) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units, failurePolicy,
                baselineConfigApplied, measuresUnit);
    }

    /** Selects how Athena responds if this encoder cannot be created or stops responding. */
    public EncoderDevice failurePolicy(FailurePolicy policy) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units,
                Objects.requireNonNull(policy, "policy"), baselineConfigApplied, measuresUnit);
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
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units, failurePolicy, apply);
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
        return action("setPosition", new Class<?>[] {EncoderDevice.class, double.class}, this, position);
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
     * Converts configured mechanism position into raw sensor rotations.
     *
     * @param position configured position
     * @return raw sensor rotations
     */
    public double rotationsFromPosition(double position) {
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        double directed = position / (gearRatio * conversion) + offset;
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

    @SuppressWarnings("unchecked")
    private static DeviceAction action(String method, Class<?>[] parameterTypes, Object... args) {
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod(method, parameterTypes);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (DeviceAction) factory.invoke(null, args);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("Encoder action factories require athena-mechanisms on the classpath.", exception);
        }
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
