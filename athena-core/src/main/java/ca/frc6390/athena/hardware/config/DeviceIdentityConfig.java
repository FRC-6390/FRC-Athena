package ca.frc6390.athena.hardware.config;

/**
 * Shared identity fields for device configs (type, id, canbus, inversion).
 */
public abstract class DeviceIdentityConfig<TType> {
    private TType type;
    private int id;
    private String canbus = "rio";
    private boolean inverted = false;

    protected DeviceIdentityConfig() {
    }

    protected DeviceIdentityConfig(TType type) {
        this.type = type;
    }

    protected DeviceIdentityConfig(TType type, int id) {
        this.type = type;
        applyId(id);
    }

    public final TType type() {
        return type;
    }

    public final int id() {
        return id;
    }

    public final String canbus() {
        return canbus;
    }

    public final boolean inverted() {
        return inverted;
    }

    protected void applyType(TType type) {
        this.type = type;
    }

    protected void applyId(int id) {
        this.id = normalizeId(id);
        Boolean derivedInversion = inferInvertedFromId(id);
        if (derivedInversion != null) {
            this.inverted = derivedInversion.booleanValue();
        }
    }

    protected void applyCanbus(String canbus) {
        this.canbus = canbus;
    }

    protected void applyInverted(boolean inverted) {
        this.inverted = inverted;
    }

    protected int normalizeId(int id) {
        return id;
    }

    protected Boolean inferInvertedFromId(int id) {
        return null;
    }
}
