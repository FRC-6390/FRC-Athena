package ca.frc6390.athena.hardware.config;

import java.util.Objects;
import java.util.function.Consumer;

import ca.frc6390.athena.api.hardware.MotorId;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.hardware.spec.NeutralMode;
import ca.frc6390.athena.hardware.spec.VendorOptions;

/**
 * Student-facing motor declaration.
 *
 * <p>This class is intentionally optimized for authoring. Call
 * {@link #toSpec(String, String)} to lower it into the immutable runtime model.</p>
 */
public final class MotorConfig {
    private MotorKind kind;
    private int id;
    private String canbus = "rio";
    private boolean inverted;
    private NeutralMode neutralMode = NeutralMode.COAST;
    private int currentLimitAmps = 40;
    private boolean integratedEncoder;
    private VendorOptions vendorOptions = VendorOptions.empty();

    private MotorConfig() {
    }

    /**
     * Creates an empty motor config.
     *
     * @return motor config
     */
    public static MotorConfig create() {
        return new MotorConfig();
    }

    /**
     * Sets hardware identity.
     *
     * @param kind motor kind
     * @param id device id
     * @return this config
     */
    public MotorConfig hardware(MotorKind kind, int id) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.id = id;
        return this;
    }

    /**
     * Sets hardware identity from a reusable robot hardware alias.
     *
     * @param motorId motor identity
     * @return this config
     */
    public MotorConfig hardware(MotorId motorId) {
        Objects.requireNonNull(motorId, "motorId");
        return hardware(motorId.kind(), motorId.id()).canbus(motorId.canbus());
    }

    /**
     * Sets hardware and common configuration from a reusable motor reference.
     *
     * @param motor motor reference
     * @return this config
     */
    public MotorConfig hardware(MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        kind = motor.kind();
        id = motor.id();
        canbus = motor.canbus();
        inverted = motor.isInverted();
        neutralMode = motor.neutralMode();
        currentLimitAmps = motor.currentLimitAmps();
        integratedEncoder = motor.hasIntegratedEncoder();
        vendorOptions = motor.vendorOptions();
        return this;
    }

    /**
     * Sets the CAN bus.
     *
     * @param canbus CAN bus name
     * @return this config
     */
    public MotorConfig canbus(String canbus) {
        this.canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        return this;
    }

    /**
     * Sets motor inversion.
     *
     * @param inverted true to invert output
     * @return this config
     */
    public MotorConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Inverts motor output.
     *
     * @return this config
     */
    public MotorConfig inverted() {
        return inverted(true);
    }

    /**
     * Sets brake neutral mode.
     *
     * @return this config
     */
    public MotorConfig brake() {
        this.neutralMode = NeutralMode.BRAKE;
        return this;
    }

    /**
     * Sets coast neutral mode.
     *
     * @return this config
     */
    public MotorConfig coast() {
        this.neutralMode = NeutralMode.COAST;
        return this;
    }

    /**
     * Sets current limit.
     *
     * @param amps limit in amps
     * @return this config
     */
    public MotorConfig currentLimit(int amps) {
        this.currentLimitAmps = amps;
        return this;
    }

    /**
     * Requests the integrated motor encoder.
     *
     * @return this config
     */
    public MotorConfig integratedEncoder() {
        this.integratedEncoder = true;
        return this;
    }

    /**
     * Adds typed vendor-specific options.
     *
     * <p>The option type comes from a vendor adapter artifact. Generic Athena
     * code stores it without importing vendor classes.</p>
     *
     * @param optionType option class with a no-argument constructor
     * @param configure option configuration callback
     * @param <T> option type
     * @return this config
     */
    public <T> MotorConfig vendor(Class<T> optionType, Consumer<T> configure) {
        Objects.requireNonNull(optionType, "optionType");
        Objects.requireNonNull(configure, "configure");
        try {
            T options = optionType.getDeclaredConstructor().newInstance();
            configure.accept(options);
            vendorOptions = vendorOptions.with(optionType, options);
            return this;
        } catch (ReflectiveOperationException exception) {
            throw new IllegalArgumentException(
                    "Vendor option type must expose a no-argument constructor: " + optionType.getName(),
                    exception);
        }
    }

    /**
     * Lowers this declaration into an immutable motor spec.
     *
     * @param ownerPath owning mechanism path
     * @param name motor name
     * @return motor spec
     */
    public MotorSpec toSpec(String ownerPath, String name) {
        if (kind == null) {
            throw new IllegalStateException("Motor hardware kind is required for " + ownerPath + "." + name);
        }
        return new MotorSpec(
                ownerPath,
                name,
                kind,
                id,
                canbus,
                inverted,
                neutralMode,
                currentLimitAmps,
                integratedEncoder,
                vendorOptions);
    }
}
