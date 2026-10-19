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
import ca.frc6390.athena.hardware.runtime.DeviceActionFactories;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
import ca.frc6390.athena.hardware.signal.MotorCommandSnapshot;
import ca.frc6390.athena.hardware.power.MotorThermalSpec;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.power.ThermalModel;

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
 * @param failurePolicy how Athena responds to failures of this motor
 * @param power shared electrical budget profile, or {@code null}
 * @param thermal thermal derate curve, or {@code null} for the motor kind default
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
        FailurePolicy failurePolicy,
        PowerProfile power,
        Thermal thermal) {
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
                0, 0, 0, VendorOptions.empty(), null, false, FailurePolicy.DISABLE_MECHANISM, null, null);
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
                0, 0, vendorOptions, follower, isDisabled, FailurePolicy.DISABLE_MECHANISM, null, null);
    }

    public MotorDevice(
            MotorKind kind, int id, String canbus, boolean isInverted,
            MotorNeutralMode neutralMode, int currentLimitAmps,
            int supplyCurrentLimitAmps, int statorCurrentLimitAmps,
            VendorOptions vendorOptions, MotorFollowerBinding follower, boolean isDisabled) {
        this(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower,
                isDisabled, FailurePolicy.DISABLE_MECHANISM, null, null);
    }

    public MotorDevice disabled() {
        return disabled(true);
    }

    public MotorDevice disabled(boolean disabled) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode,
                currentLimitAmps, supplyCurrentLimitAmps, statorCurrentLimitAmps,
                vendorOptions, follower, disabled, failurePolicy, power, thermal);
    }

    /** Selects how Athena responds if this motor cannot be created or stops responding. */
    public MotorDevice failurePolicy(FailurePolicy policy) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode,
                currentLimitAmps, supplyCurrentLimitAmps, statorCurrentLimitAmps,
                vendorOptions, follower, isDisabled, Objects.requireNonNull(policy, "policy"), power, thermal);
    }

    /**
     * Attaches a shared electrical budget profile. The profile may be shared by several motors and
     * changed at runtime; see {@link PowerProfile}.
     *
     * @param profile profile, or {@code null} to remove it
     * @return updated ref
     */
    public MotorDevice power(PowerProfile profile) {
        if (profile != null) {
            profile.markDeclared();
        }
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled,
                failurePolicy, profile, thermal);
    }

    /**
     * Sets the thermal derate curve. When absent, {@link MotorThermalSpec#defaultDerate()} for the motor
     * kind applies. Thermal only scales current limits; it never brakes or locks out the motor.
     *
     * @param thermal derate curve, or {@code null} to use the kind default
     * @return updated ref
     */
    public MotorDevice thermal(Thermal thermal) {
        return new MotorDevice(kind, id, canbus, isInverted, neutralMode, currentLimitAmps,
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled,
                failurePolicy, power, thermal);
    }

    /** Returns the derate curve in effect: the declared curve or the motor kind default. */
    public Thermal thermalCurve() {
        return thermal != null ? thermal : MotorThermalSpec.forKind(kind).defaultDerate();
    }

    /** Returns the fused winding temperature estimate in Celsius. */
    public double temperatureCelsius() {
        RuntimeMotor runtime = RUNTIMES.find(this);
        return runtime == null ? MotorThermalSpec.GENERIC.ambientCelsius() : runtime.thermal().celsius();
    }

    /** Returns the thermal load in {@code [0, 1]}: 0 at the derate start, 1 at the derate end. */
    public double thermalLoad() {
        return thermalCurve().load(temperatureCelsius());
    }

    /** Returns the thermal model bound to this motor, or {@code null} when unbound. */
    public ThermalModel thermalModel() {
        RuntimeMotor runtime = RUNTIMES.find(this);
        return runtime == null ? null : runtime.thermal();
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

    @SuppressWarnings("unchecked")
    public <T extends DeviceAction> T percent(double percent) {
        return (T) DeviceActionFactories.get().motorPercent(this, percent);
    }

    @SuppressWarnings("unchecked")
    public <T extends DeviceAction> T percent(DoubleSupplier percent) {
        return (T) DeviceActionFactories.get().motorPercent(this, percent);
    }

    @SuppressWarnings("unchecked")
    public <T extends DeviceAction> T voltage(double volts) {
        return (T) DeviceActionFactories.get().motorVoltage(this, volts);
    }

    @SuppressWarnings("unchecked")
    public <T extends DeviceAction> T voltage(DoubleSupplier volts) {
        return (T) DeviceActionFactories.get().motorVoltage(this, volts);
    }

    /** Stops commanding this motor and applies its configured neutral mode. */
    @SuppressWarnings("unchecked")
    public <T extends DeviceAction> T neutral() {
        return (T) DeviceActionFactories.get().motorNeutral(this);
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
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
                supplyCurrentLimitAmps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
                amps, statorCurrentLimitAmps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
                supplyCurrentLimitAmps, amps, vendorOptions, follower, isDisabled, failurePolicy, power, thermal);
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
        var constants = MotorConstants.forKind(kind);
        if (constants.isPresent() && currentLimits().supportsStallDetection()) {
            return MotorStallSignal.modelBased(this, constants.get());
        }
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

    /** Updates the runtime command snapshot, reusing it when the command is unchanged. */
    public void recordCommand(MotorCommandSnapshot.Mode mode, double value) {
        RuntimeMotor runtime = RUNTIMES.find(this);
        if (runtime != null) runtime.command(Objects.requireNonNull(mode, "mode"), value);
    }

    /** Binds this declaration to its owning runtime handle. */
    public AutoCloseable bindRuntime(RuntimeScope scope, MotorHandle handle) {
        return RUNTIMES.bind(this, scope, new RuntimeMotor(handle, MotorThermalSpec.forKind(kind)));
    }

    private RuntimeMotor runtime() {
        return RUNTIMES.get(this, "Motor " + defaultName());
    }

    private static final class RuntimeMotor {
        private final MotorHandle handle;
        private final ThermalModel thermal;
        private volatile MotorCommandSnapshot command = MotorCommandSnapshot.neutral();

        private RuntimeMotor(MotorHandle handle, MotorThermalSpec spec) {
            this.handle = Objects.requireNonNull(handle, "handle");
            this.thermal = new ThermalModel(spec);
        }

        private MotorHandle handle() { return handle; }

        private ThermalModel thermal() { return thermal; }

        private MotorCommandSnapshot command() { return command; }

        private void command(MotorCommandSnapshot value) { command = value; }

        private void command(MotorCommandSnapshot.Mode mode, double value) {
            MotorCommandSnapshot current = command;
            if (current.mode() != mode || Double.compare(current.value(), value) != 0) {
                command = new MotorCommandSnapshot(mode, value);
            }
        }
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
                failurePolicy, power, thermal);
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
                failurePolicy, power, thermal);
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
                    failurePolicy, power, thermal);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalArgumentException(
                    "Vendor option type must expose a no-argument constructor: " + optionType.getName(),
                    exception);
        }
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }
}
