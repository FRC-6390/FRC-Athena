package ca.frc6390.athena.hardware.ref;

import java.util.Objects;
import java.util.Locale;
import java.util.function.Consumer;

import ca.frc6390.athena.api.hardware.MotorId;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.spec.NeutralMode;
import ca.frc6390.athena.hardware.spec.VendorOptions;

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
        boolean hasIntegratedEncoder,
        VendorOptions vendorOptions) {
    /**
     * Creates a motor ref on the default roboRIO CAN bus.
     *
     * @param kind motor kind
     * @param id device id
     * @return motor ref
     */
    public static MotorRef of(MotorKind kind, int id) {
        return new MotorRef(kind, id, "rio", false, NeutralMode.COAST, 40, false, VendorOptions.empty());
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
     * Moves this motor to another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated ref
     */
    public MotorRef canbus(String canbus) {
        return new MotorRef(kind, id, canbus, isInverted, neutralMode, currentLimitAmps, hasIntegratedEncoder, vendorOptions);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true if inverted
     * @return updated ref
     */
    public MotorRef inverted(boolean inverted) {
        return new MotorRef(kind, id, canbus, inverted, neutralMode, currentLimitAmps, hasIntegratedEncoder, vendorOptions);
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
        return new MotorRef(kind, id, canbus, isInverted, neutralMode, currentLimitAmps, hasIntegratedEncoder, vendorOptions);
    }

    /**
     * Sets current limit.
     *
     * @param amps current limit in amps
     * @return updated ref
     */
    public MotorRef currentLimit(int amps) {
        return new MotorRef(kind, id, canbus, isInverted, neutralMode, amps, hasIntegratedEncoder, vendorOptions);
    }

    /**
     * Requests the integrated motor encoder.
     *
     * @return updated ref
     */
    public MotorRef withIntegratedEncoder() {
        return new MotorRef(kind, id, canbus, isInverted, neutralMode, currentLimitAmps, true, vendorOptions);
    }

    /**
     * Creates an encoder ref for this motor's integrated encoder.
     *
     * @return integrated encoder ref
     */
    public EncoderRef integratedEncoder() {
        return EncoderRef.integrated(this);
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
                      hasIntegratedEncoder,
                      vendorOptions.with(optionType, options));
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
