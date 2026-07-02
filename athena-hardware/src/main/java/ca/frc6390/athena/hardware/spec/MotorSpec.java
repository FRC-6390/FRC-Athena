package ca.frc6390.athena.hardware.spec;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.MotorKind;

/**
 * Immutable normalized motor declaration consumed by validation and backends.
 *
 * @param ownerPath owning mechanism or subsystem path
 * @param name motor name inside the owner
 * @param kind Athena motor kind
 * @param id device id
 * @param canbus CAN bus name
 * @param inverted true when motor output should be inverted
 * @param neutralMode desired neutral mode
 * @param currentLimitAmps current limit in amps
 * @param integratedEncoder true when the integrated encoder is requested
 * @param vendorOptions typed vendor-specific options
 */
public record MotorSpec(
        String ownerPath,
        String name,
        MotorKind kind,
        int id,
        String canbus,
        boolean inverted,
        NeutralMode neutralMode,
        int currentLimitAmps,
        boolean integratedEncoder,
        VendorOptions vendorOptions) {
    public MotorSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "motor" : name;
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        neutralMode = neutralMode == null ? NeutralMode.COAST : neutralMode;
        vendorOptions = vendorOptions == null ? VendorOptions.empty() : vendorOptions;
    }

    /**
     * Creates a spec without vendor options.
     *
     * @param ownerPath owning mechanism or subsystem path
     * @param name motor name inside the owner
     * @param kind Athena motor kind
     * @param id device id
     * @param canbus CAN bus name
     * @param neutralMode desired neutral mode
     * @param currentLimitAmps current limit in amps
     * @param integratedEncoder true when the integrated encoder is requested
     */
    public MotorSpec(
            String ownerPath,
            String name,
            MotorKind kind,
            int id,
            String canbus,
            NeutralMode neutralMode,
            int currentLimitAmps,
            boolean integratedEncoder,
            VendorOptions vendorOptions) {
        this(
                ownerPath,
                name,
                kind,
                id,
                canbus,
                false,
                neutralMode,
                currentLimitAmps,
                integratedEncoder,
                vendorOptions);
    }

    /**
     * Creates a spec without vendor options.
     *
     * @param ownerPath owning mechanism or subsystem path
     * @param name motor name inside the owner
     * @param kind Athena motor kind
     * @param id device id
     * @param canbus CAN bus name
     * @param neutralMode desired neutral mode
     * @param currentLimitAmps current limit in amps
     * @param integratedEncoder true when the integrated encoder is requested
     */
    public MotorSpec(
            String ownerPath,
            String name,
            MotorKind kind,
            int id,
            String canbus,
            NeutralMode neutralMode,
            int currentLimitAmps,
            boolean integratedEncoder) {
        this(
                ownerPath,
                name,
                kind,
                id,
                canbus,
                false,
                neutralMode,
                currentLimitAmps,
                integratedEncoder,
                VendorOptions.empty());
    }

    /**
     * Returns a dotted path useful in validation errors.
     *
     * @return motor path
     */
    public String path() {
        return ownerPath + "." + name;
    }
}
