package ca.frc6390.athena.hardware.imu;

import java.util.function.Consumer;

/**
 * Vendor-agnostic IMU configuration.
 */
public class ImuConfig {
    public ImuType type;
    public int id;
    public String canbus = "rio";
    public boolean inverted = false;

    /**
     */
    public ImuConfig(ImuType type, int id) {
        this.type = type;
        this.id = id;
    }

    /**
     */
    public ImuConfig(ImuType type) {
        this(type, -1);
    }

    public static ImuConfig create(ImuType type, int id) {
        return new ImuConfig(type, id);
    }

    public static ImuConfig create(ImuType type) {
        return new ImuConfig(type);
    }

    /**
     * Sectioned fluent API for identity and orientation settings.
     */
    public ImuConfig hardware(Consumer<HardwareSection> section) {
        if (section != null) {
            section.accept(new HardwareSection(this));
        }
        return this;
    }

    /**
     */
    public ImuConfig setCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    /**
     */
    public ImuConfig setId(int id) {
        this.id = id;
        return this;
    }

    /**
     */
    public ImuConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public ImuType type() {
        return type;
    }

    public int id() {
        return id;
    }

    public String canbus() {
        return canbus;
    }

    public boolean inverted() {
        return inverted;
    }

    public static final class HardwareSection {
        private final ImuConfig owner;

        private HardwareSection(ImuConfig owner) {
            this.owner = owner;
        }

        public HardwareSection type(ImuType type) {
            owner.type = type;
            return this;
        }

        public HardwareSection id(int id) {
            owner.setId(id);
            return this;
        }

        public HardwareSection canbus(String canbus) {
            owner.setCanbus(canbus);
            return this;
        }

        public HardwareSection inverted(boolean inverted) {
            owner.setInverted(inverted);
            return this;
        }
    }
}
