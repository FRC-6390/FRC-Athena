package ca.frc6390.athena.hardware.device;

import java.util.Objects;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorControllerKind;
import ca.frc6390.athena.hardware.vendor.VendorOptions;
import ca.frc6390.athena.hardware.runtime.DeviceAction;

/**
 * Reusable motor declaration for robot constants.
 */
public record MotorDevice(
        MotorKind kind,
        int id,
        String canbus,
        boolean isInverted,
        MotorNeutralMode neutralMode,
        int currentLimitAmps,
        VendorOptions vendorOptions,
        MotorFollowerBinding follower) {
    /**
     * Creates a motor ref on the default roboRIO CAN bus.
     *
     * @param kind motor kind
     * @param id device id
     * @return motor ref
     */
    public static MotorDevice of(MotorKind kind, int id) {
        return new MotorDevice(kind, id, "rio", false, MotorNeutralMode.COAST, 40, VendorOptions.empty(), null);
    }

    /** Creates a motor with an explicit controller and physical motor pairing. */
    public static MotorDevice of(MotorControllerKind controller, MotorKind motor, int id) {
        Objects.requireNonNull(motor, "motor");
        return of(motor.controlledBy(controller), id);
    }

    public MotorDevice {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        neutralMode = neutralMode == null ? MotorNeutralMode.COAST : neutralMode;
        vendorOptions = vendorOptions == null ? VendorOptions.empty() : vendorOptions;
    }

    /**
     * Returns this motor's integrated encoder.
     *
     * @return integrated encoder ref
     */
    public EncoderDevice encoder() {
        return EncoderDevice.integratedMotor(this);
    }

    /**
     * Returns a motor-controller attached absolute encoder.
     *
     * @return absolute encoder declaration
     */
    public EncoderDevice absoluteEncoder() {
        return EncoderDevice.motorAbsolute(this);
    }

    public <T extends DeviceAction> T percent(double percent) {
        return action("percent", new Class<?>[] {MotorDevice.class, double.class}, this, percent);
    }

    public <T extends DeviceAction> T percent(DoubleSupplier percent) {
        return action("percent", new Class<?>[] {MotorDevice.class, DoubleSupplier.class}, this, percent);
    }

    public <T extends DeviceAction> T voltage(double volts) {
        return action("voltage", new Class<?>[] {MotorDevice.class, double.class}, this, volts);
    }

    public <T extends DeviceAction> T voltage(DoubleSupplier volts) {
        return action("voltage", new Class<?>[] {MotorDevice.class, DoubleSupplier.class}, this, volts);
    }

    /** Stops commanding this motor and applies its configured neutral mode. */
    public <T extends DeviceAction> T neutral() {
        return action("neutral", new Class<?>[] {MotorDevice.class}, this);
    }

    /**
     * Moves this motor to another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated ref
     */
    public MotorDevice canbus(String canbus) {
        return new MotorDevice(
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true if inverted
     * @return updated ref
     */
    public MotorDevice inverted(boolean inverted) {
        return new MotorDevice(
                kind, id, canbus, inverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Marks this motor as inverted.
     *
     * @return updated ref
     */
    public MotorDevice inverted() {
        return inverted(true);
    }

    /**
     * Uses brake neutral mode.
     *
     * @return updated ref
     */
    public MotorDevice brake() {
        return neutralMode(MotorNeutralMode.BRAKE);
    }

    /**
     * Uses coast neutral mode.
     *
     * @return updated ref
     */
    public MotorDevice coast() {
        return neutralMode(MotorNeutralMode.COAST);
    }

    /**
     * Sets neutral mode.
     *
     * @param neutralMode neutral mode
     * @return updated ref
     */
    public MotorDevice neutralMode(MotorNeutralMode neutralMode) {
        return new MotorDevice(
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps, vendorOptions, follower);
    }

    /**
     * Sets current limit.
     *
     * @param amps current limit in amps
     * @return updated ref
     */
    public MotorDevice currentLimit(int amps) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, amps, vendorOptions, follower);
    }

    /**
     * Makes this motor follow a leader motor.
     *
     * @param leader leader motor
     * @return updated ref
     */
    public MotorDevice follow(MotorDevice leader) {
        Objects.requireNonNull(leader, "leader");
        if (id == leader.id() && canbus.equals(leader.canbus())) {
            throw new IllegalArgumentException("A motor cannot follow itself.");
        }
        if (!canbus.equals(leader.canbus())) {
            throw new IllegalArgumentException("A motor and its leader must use the same CAN bus.");
        }
        return new MotorDevice(
                kind,
                id,
                canbus,
                isInverted,
                neutralMode,
                currentLimitAmps,
                vendorOptions,
                new MotorFollowerBinding(leader));
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
    public <T> MotorDevice vendor(Class<T> optionType, Consumer<T> configure) {
        Objects.requireNonNull(optionType, "optionType");
        Objects.requireNonNull(configure, "configure");
        try {
            T options = optionType.getDeclaredConstructor().newInstance();
            configure.accept(options);
            return new MotorDevice(
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

    @SuppressWarnings("unchecked")
    private static <T extends DeviceAction> T action(String method, Class<?>[] parameterTypes, Object... args) {
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod(method, parameterTypes);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (T) factory.invoke(null, args);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("Motor action factories require athena-mechanisms on the classpath.", exception);
        }
    }
}
