package ca.frc6390.athena.hardware.encoder;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderId;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.ref.EncoderRef;

/**
 * Student-facing encoder declaration.
 */
public final class EncoderConfig {
    private EncoderKind kind;
    private int id;
    private String canbus = "rio";
    private EncoderSignalType signalType = EncoderSignalType.RELATIVE_POSITION;
    private boolean inverted;
    private double gearRatio = 1.0;
    private double conversion = 1.0;
    private double offset;
    private EncoderUnit units = EncoderUnit.RAW;

    private EncoderConfig() {
    }

    /**
     * Creates an empty encoder config.
     *
     * @return encoder config
     */
    public static EncoderConfig create() {
        return new EncoderConfig();
    }

    /**
     * Sets hardware identity.
     *
     * @param kind encoder kind
     * @param id device id or channel
     * @return this config
     */
    public EncoderConfig hardware(EncoderKind kind, int id) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.id = id;
        return this;
    }

    /**
     * Sets hardware identity from a reusable alias.
     *
     * @param encoderId encoder identity
     * @return this config
     */
    public EncoderConfig hardware(EncoderId encoderId) {
        Objects.requireNonNull(encoderId, "encoderId");
        return hardware(encoderId.kind(), encoderId.id()).canbus(encoderId.canbus());
    }

    /**
     * Sets hardware and common configuration from a reusable encoder reference.
     *
     * @param encoder encoder reference
     * @return this config
     */
    public EncoderConfig hardware(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        kind = encoder.kind();
        id = encoder.id();
        canbus = encoder.canbus();
        signalType = encoder.signalType();
        inverted = encoder.isInverted();
        gearRatio = encoder.gearRatio();
        conversion = encoder.conversion();
        offset = encoder.offset();
        units = encoder.units();
        return this;
    }

    /**
     * Sets CAN bus.
     *
     * @param canbus bus name
     * @return this config
     */
    public EncoderConfig canbus(String canbus) {
        this.canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        return this;
    }

    /**
     * Marks this encoder as absolute position.
     *
     * @return this config
     */
    public EncoderConfig absolutePosition() {
        signalType = EncoderSignalType.ABSOLUTE_POSITION;
        return this;
    }

    /**
     * Marks this encoder as relative position.
     *
     * @return this config
     */
    public EncoderConfig relativePosition() {
        signalType = EncoderSignalType.RELATIVE_POSITION;
        return this;
    }

    /**
     * Marks this encoder as velocity.
     *
     * @return this config
     */
    public EncoderConfig velocity() {
        signalType = EncoderSignalType.VELOCITY;
        return this;
    }

    /**
     * Sets encoder inversion.
     *
     * @param inverted true to invert the sensor direction
     * @return this config
     */
    public EncoderConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Inverts the encoder direction.
     *
     * @return this config
     */
    public EncoderConfig inverted() {
        return inverted(true);
    }

    /**
     * Sets gear ratio.
     *
     * @param gearRatio mechanism-to-sensor gear ratio
     * @return this config
     */
    public EncoderConfig gearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     * Sets mechanism-unit conversion applied after gear ratio.
     *
     * @param conversion conversion factor
     * @return this config
     */
    public EncoderConfig conversion(double conversion) {
        this.conversion = conversion;
        return this;
    }

    public EncoderConfig wheelDiameterMeters(double diameterMeters) {
        if (!Double.isFinite(diameterMeters) || diameterMeters <= 0.0) {
            throw new IllegalArgumentException("Wheel diameter must be positive.");
        }
        return conversion(Math.PI * diameterMeters);
    }

    public EncoderConfig wheelDiameterInches(double diameterInches) {
        return wheelDiameterMeters(diameterInches * 0.0254);
    }

    public EncoderConfig units(EncoderUnit units) {
        this.units = units == null ? EncoderUnit.RAW : units;
        return this;
    }

    /**
     * Sets offset.
     *
     * @param offset offset in mechanism units
     * @return this config
     */
    public EncoderConfig offset(double offset) {
        this.offset = offset;
        return this;
    }

    /**
     * Lowers this declaration into an immutable spec.
     *
     * @param ownerPath owner path
     * @param name encoder name
     * @return encoder spec
     */
    public EncoderSpec toSpec(String ownerPath, String name) {
        if (kind == null) {
            throw new IllegalStateException("Encoder hardware kind is required for " + ownerPath + "." + name);
        }
        return new EncoderSpec(ownerPath, name, kind, id, canbus, signalType, inverted, gearRatio, conversion, offset, units);
    }
}
