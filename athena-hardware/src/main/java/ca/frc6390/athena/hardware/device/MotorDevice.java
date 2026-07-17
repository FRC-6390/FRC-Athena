package ca.frc6390.athena.hardware.device;

import java.util.Objects;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorControllerKind;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.vendor.VendorOptions;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
import ca.frc6390.athena.hardware.signal.MotorCommandSnapshot;

/**
 * Reusable motor declaration for robot constants.
 *
 * @param kind physical motor and controller pairing
 * @param id controller CAN identifier
 * @param canbus CAN bus name
 * @param isInverted whether positive output is inverted
 * @param neutralMode behavior when no output is commanded
 * @param currentLimitAmps portable current limit, or zero when disabled
 * @param supplyCurrentLimitAmps explicit supply-side limit, or zero when unset
 * @param statorCurrentLimitAmps explicit stator-side limit, or zero when disabled
 * @param vendorOptions typed vendor-specific configuration
 * @param follower follower relationship, or {@code null} for an independent motor
 * @param isDisabled whether Athena should suppress this motor
 */
public record MotorDevice(
        MotorKind kind,
        int id,
        String canbus,
        boolean isInverted,
        MotorNeutralMode neutralMode,
        int currentLimitAmps,
        int supplyCurrentLimitAmps,
        int statorCurrentLimitAmps,
        VendorOptions vendorOptions,
        MotorFollowerBinding follower,
        boolean isDisabled,
        FailurePolicy failurePolicy) {
    private static final RuntimeBindings<MotorDevice, RuntimeMotor> RUNTIMES = new RuntimeBindings<>();
    /**
     * Creates a motor ref on the default roboRIO CAN bus.
     *
     * @param kind motor kind
     * @param id device id
     * @return motor ref
     */
    public static MotorDevice of(MotorKind kind, int id) {
        return new MotorDevice(kind, id, "rio", false, MotorNeutralMode.COAST,
                0, 0, 0, VendorOptions.empty(), null, false, FailurePolicy.DISABLE_MECHANISM);
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
        if (currentLimitAmps < 0 || supplyCurrentLimitAmps < 0 || statorCurrentLimitAmps < 0) {
            throw new IllegalArgumentException("Motor current limits cannot be negative.");
        }
        vendorOptions = vendorOptions == null ? VendorOptions.empty() : vendorOptions;
        failurePolicy = failurePolicy == null ? FailurePolicy.DISABLE_MECHANISM : failurePolicy;
    }

    public MotorDevice(
            MotorKind kind, int id, String canbus, boolean isInverted,
            MotorNeutralMode neutralMode, int currentLimitAmps,
            VendorOptions vendorOptions, MotorFollowerBinding follower) {
        this(kind, id, canbus, isInverted, neutralMode, currentLimitAmps, vendorOptions, follower, false);
    }

    public MotorDevice(
            MotorKind kind, int id, String canbus, boolean isInverted,
            MotorNeutralMode neutralMode, int currentLimitAmps,
            VendorOptions vendorOptions, MotorFollowerBinding follower, boolean isDisabled) {
        this(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                0, 0, vendorOptions, follower, isDisabled, FailurePolicy.DISABLE_MECHANISM);
    }

    public MotorDevice(
            MotorKind kind, int id, String canbus, boolean isInverted,
            MotorNeutralMode neutralMode, int currentLimitAmps,
            int supplyCurrentLimitAmps, int statorCurrentLimitAmps,
            VendorOptions vendorOptions, MotorFollowerBinding follower, boolean isDisabled) {
        this(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower,
                isDisabled, FailurePolicy.DISABLE_MECHANISM);
    }

    public MotorDevice disabled() {
        return disabled(true);
    }

    public MotorDevice disabled(boolean disabled) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode,
                currentLimitAmps, supplyCurrentLimitAmps, statorCurrentLimitAmps,
                vendorOptions, follower, disabled, failurePolicy);
    }

    /** Selects how Athena responds if this motor cannot be created or stops responding. */
    public MotorDevice failurePolicy(FailurePolicy policy) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode,
                currentLimitAmps, supplyCurrentLimitAmps, statorCurrentLimitAmps,
                vendorOptions, follower, isDisabled, Objects.requireNonNull(policy, "policy"));
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
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true if inverted
     * @return updated ref
     */
    public MotorDevice inverted(boolean inverted) {
        return new MotorDevice(
                kind, id, canbus, inverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy);
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
                kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy);
    }

    /**
     * Sets the portable controller current limit. CTRE maps this to supply current and
     * REV maps it to the smart phase-current limit. An explicit
     * {@link #supplyCurrentLimit(int)} or vendor-specific supply limit overrides it on
     * CTRE. A value of zero disables the portable limit.
     *
     * @param amps non-negative current limit in amps
     * @return updated motor declaration
     */
    public MotorDevice currentLimit(int amps) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, amps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy);
    }

    /**
     * Sets the controller supply-side current limit. CTRE TalonFX-family controllers
     * enforce this directly. REV SPARK controllers have no supply-current limiter, so
     * Athena conservatively includes this value when selecting the smart phase-current
     * limit. It overrides {@link #currentLimit(int)}; a vendor option overrides this value.
     * A value of zero leaves the explicit supply limit unset.
     *
     * @param amps non-negative supply current limit in amps
     * @return updated motor declaration
     */
    public MotorDevice supplyCurrentLimit(int amps) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                amps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy);
    }

    /**
     * Sets the motor stator-side current limit. This limits current through the motor
     * windings and therefore limits produced torque. CTRE TalonFX-family controllers
     * enforce it directly; REV maps it to the smart phase-current limit. When both
     * supply and stator limits are declared for REV, Athena uses the lower value. A
     * vendor option overrides this mapping. A value of zero disables the stator limit.
     *
     * @param amps non-negative stator current limit in amps
     * @return updated motor declaration
     */
    public MotorDevice statorCurrentLimit(int amps) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, amps, vendorOptions, follower, isDisabled, failurePolicy);
    }

    /** Returns the effective supply and stator limits used by Athena. */
    public MotorCurrentLimits currentLimits() {
        int supply = supplyCurrentLimitAmps > 0 ? supplyCurrentLimitAmps : currentLimitAmps;
        MotorControllerKind controller = kind.controllerKind();
        String controllerKey = controller == null ? kind.key() : controller.key();
        if (controllerKey.startsWith("rev:spark-")) {
            int smartLimit = supply <= 0
                    ? statorCurrentLimitAmps
                    : statorCurrentLimitAmps <= 0
                            ? supply
                            : Math.min(supply, statorCurrentLimitAmps);
            // A SPARK Smart Current Limit regulates motor phase current, so expose
            // the mapped limit in Athena's stator domain for telemetry and stalls.
            return new MotorCurrentLimits(0, smartLimit);
        }
        return new MotorCurrentLimits(supply, statorCurrentLimitAmps);
    }

    /** Creates a stall detector derived from this motor's configured current limits. */
    public MotorStallSignal stall() {
        return new MotorStallSignal(this, 0.9, 1.0, 2.0, 0.15, 0.25);
    }

    /** Returns the motor's latest applied voltage snapshot. */
    public double appliedVoltage() { return runtime().handle().appliedVoltage(); }

    /** Returns the motor's latest supply-current snapshot. */
    public double supplyCurrentAmps() { return runtime().handle().supplyCurrentAmps(); }

    /** Returns the motor's latest stator-current snapshot. */
    public double statorCurrentAmps() { return runtime().handle().statorCurrentAmps(); }

    /** Returns the motor's latest integrated velocity snapshot. */
    public double velocityRotationsPerSecond() { return runtime().handle().integratedVelocityRotationsPerSecond(); }

    /** Returns the motor's latest integrated position snapshot. */
    public double positionRotations() { return runtime().handle().integratedPositionRotations(); }

    /** Returns the latest command Athena applied to this motor. */
    public MotorCommandSnapshot command() { return runtime().command(); }

    /** Updates the runtime command snapshot after Athena applies an output. */
    public void recordCommand(MotorCommandSnapshot command) {
        RuntimeMotor runtime = RUNTIMES.find(this);
        if (runtime != null) {
            runtime.command(Objects.requireNonNull(command, "command"));
        }
    }

    /** Binds this declaration to its owning runtime handle. */
    public AutoCloseable bindRuntime(RuntimeScope scope, MotorHandle handle) {
        return RUNTIMES.bind(this, scope, new RuntimeMotor(handle));
    }

    private RuntimeMotor runtime() {
        return RUNTIMES.get(this, "Motor " + defaultName());
    }

    private static final class RuntimeMotor {
        private final MotorHandle handle;
        private volatile MotorCommandSnapshot command = MotorCommandSnapshot.neutral();

        private RuntimeMotor(MotorHandle handle) {
            this.handle = Objects.requireNonNull(handle, "handle");
        }

        private MotorHandle handle() { return handle; }

        private MotorCommandSnapshot command() { return command; }

        private void command(MotorCommandSnapshot value) { command = value; }
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
        return new MotorDevice(
                kind,
                id,
                canbus,
                isInverted,
                neutralMode,
                currentLimitAmps,
                supplyCurrentLimitAmps,
                statorCurrentLimitAmps,
                vendorOptions,
                new MotorFollowerBinding(leader),
                isDisabled,
                failurePolicy);
    }

    /** Returns this motor without a hardware follower declaration. */
    public MotorDevice independent() {
        return new MotorDevice(
                kind,
                id,
                canbus,
                isInverted,
                neutralMode,
                currentLimitAmps,
                supplyCurrentLimitAmps,
                statorCurrentLimitAmps,
                vendorOptions,
                null,
                isDisabled,
                failurePolicy);
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
                    supplyCurrentLimitAmps,
                    statorCurrentLimitAmps,
                    vendorOptions.with(optionType, options),
                    follower,
                    isDisabled,
                    failurePolicy);
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
