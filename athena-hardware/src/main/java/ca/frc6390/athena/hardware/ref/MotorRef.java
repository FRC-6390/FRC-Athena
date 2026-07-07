package ca.frc6390.athena.hardware.ref;

import java.util.Objects;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.api.hardware.MotorId;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.spec.NeutralMode;
import ca.frc6390.athena.hardware.spec.VendorOptions;
import ca.frc6390.athena.mechanism.core.MechanismState;
import ca.frc6390.athena.mechanism.core.Output;

/**
 * Reusable motor declaration for robot constants.
 */
public record MotorRef(
        MotorKind kind,
        int id,
        String canbus,
        boolean isInverted,
        NeutralMode neutralMode,
        int currentLimitAmps,
        VendorOptions vendorOptions,
        MotorFollowerRef follower) {
    /**
     * Creates a motor ref on the default roboRIO CAN bus.
     *
     * @param kind motor kind
     * @param id device id
     * @return motor ref
     */
    public static MotorRef of(MotorKind kind, int id) {
        return new MotorRef(kind, id, "rio", false, NeutralMode.COAST, 40, VendorOptions.empty(), null);
    }

    /**
     * Creates a motor ref from a bare identity.
     *
     * @param id motor identity
     * @return motor ref
     */
    public static MotorRef of(MotorId id) {
        Objects.requireNonNull(id, "id");
        return of(id.kind(), id.id()).canbus(id.canbus());
    }

    public MotorRef {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        neutralMode = neutralMode == null ? NeutralMode.COAST : neutralMode;
        vendorOptions = vendorOptions == null ? VendorOptions.empty() : vendorOptions;
    }

    /**
     * Returns this motor's bare identity.
     *
     * @return motor id
     */
    public MotorId idRef() {
        return new MotorId(kind, id, canbus);
    }

    /**
     * Returns this motor's integrated encoder.
     *
     * @return integrated encoder ref
     */
    public EncoderRef encoder() {
        return EncoderRef.integratedMotor(this);
    }

    /**
     * Creates a percent-output state targeting this motor.
     *
     * @param percent percent output
     * @return motor state
     */
    public MechanismState percent(double percent) {
        return new PercentState(this, percent);
    }

    /**
     * Creates a dynamic percent-output state targeting this motor.
     *
     * @param percent percent supplier
     * @return motor state
     */
    public MechanismState percent(DoubleSupplier percent) {
        return new DynamicPercentState(this, percent);
    }

    /**
     * Alias for {@link #percent(double)}.
     *
     * @param percentage percent output
     * @return motor state
     */
    public MechanismState percentage(double percentage) {
        return percent(percentage);
    }

    /**
     * Alias for {@link #percent(DoubleSupplier)}.
     *
     * @param percentage percent supplier
     * @return motor state
     */
    public MechanismState percentage(DoubleSupplier percentage) {
        return percent(percentage);
    }

    /**
     * Creates a voltage state targeting this motor.
     *
     * @param volts voltage output
     * @return motor state
     */
    public MechanismState voltage(double volts) {
        return new VoltageState(this, volts);
    }

    /**
     * Creates a dynamic voltage state targeting this motor.
     *
     * @param volts voltage supplier
     * @return motor state
     */
    public MechanismState voltage(DoubleSupplier volts) {
        return new DynamicVoltageState(this, volts);
    }

    /**
     * Moves this motor to another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated ref
     */
    public MotorRef canbus(String canbus) {
        return new MotorRef(
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true if inverted
     * @return updated ref
     */
    public MotorRef inverted(boolean inverted) {
        return new MotorRef(
                kind, id, canbus, inverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Marks this motor as inverted.
     *
     * @return updated ref
     */
    public MotorRef inverted() {
        return inverted(true);
    }

    /**
     * Uses brake neutral mode.
     *
     * @return updated ref
     */
    public MotorRef brake() {
        return neutralMode(NeutralMode.BRAKE);
    }

    /**
     * Uses coast neutral mode.
     *
     * @return updated ref
     */
    public MotorRef coast() {
        return neutralMode(NeutralMode.COAST);
    }

    /**
     * Sets neutral mode.
     *
     * @param neutralMode neutral mode
     * @return updated ref
     */
    public MotorRef neutralMode(NeutralMode neutralMode) {
        return new MotorRef(
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Sets current limit.
     *
     * @param amps current limit in amps
     * @return updated ref
     */
    public MotorRef currentLimit(int amps) {
        return new MotorRef(kind, id, canbus, isInverted, neutralMode, amps, vendorOptions, follower);
    }

    /**
     * Makes this motor follow a leader motor.
     *
     * @param leader leader motor
     * @return updated ref
     */
    public MotorRef follow(MotorRef leader) {
        return new MotorRef(
                kind,
                id,
                canbus,
                isInverted,
                neutralMode,
                currentLimitAmps,
                vendorOptions,
                new MotorFollowerRef(leader));
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + id;
    }

    /**
     * Adds typed vendor-specific options.
     *
     * @param optionType option class
     * @param configure option configuration callback
     * @param <T> option type
     * @return updated ref
     */
    public <T> MotorRef vendor(Class<T> optionType, Consumer<T> configure) {
        Objects.requireNonNull(optionType, "optionType");
        Objects.requireNonNull(configure, "configure");
        try {
            T options = optionType.getDeclaredConstructor().newInstance();
            configure.accept(options);
            return new MotorRef(
                    kind,
                      id,
                      canbus,
                      isInverted,
                      neutralMode,
                      currentLimitAmps,
                      vendorOptions.with(optionType, options),
                      follower);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalArgumentException(
                    "Vendor option type must expose a no-argument constructor: " + optionType.getName(),
                    exception);
        }
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    /**
     * State that directly targets a motor ref.
     */
    public interface MotorState extends MechanismState {
        /**
         * Returns the target motor.
         *
         * @return motor
         */
        MotorRef motor();
    }

    /**
     * Direct motor percent-output state.
     *
     * @param motor target motor
     * @param percent percent output
     */
    public record PercentState(MotorRef motor, double percent) implements MotorState, Output.Percent {
        public PercentState {
            Objects.requireNonNull(motor, "motor");
        }
    }

    /**
     * Dynamic direct motor percent-output state.
     *
     * @param motor target motor
     * @param percentSupplier percent supplier
     */
    public record DynamicPercentState(MotorRef motor, DoubleSupplier percentSupplier)
            implements MotorState, Output.Percent {
        public DynamicPercentState {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    /**
     * Direct motor voltage state.
     *
     * @param motor target motor
     * @param volts voltage output
     */
    public record VoltageState(MotorRef motor, double volts) implements MotorState, Output.Voltage {
        public VoltageState {
            Objects.requireNonNull(motor, "motor");
        }
    }

    /**
     * Dynamic direct motor voltage state.
     *
     * @param motor target motor
     * @param voltsSupplier voltage supplier
     */
    public record DynamicVoltageState(MotorRef motor, DoubleSupplier voltsSupplier)
            implements MotorState, Output.Voltage {
        public DynamicVoltageState {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }
}
