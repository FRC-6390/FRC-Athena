package ca.frc6390.athena.hardware.imu;

/**
 * Vendor-agnostic IMU configuration.
 */
public class ImuConfig {
    public ImuType type;
    public int id;
    public String canbus = "rio";
    public boolean inverted = false;

    public ImuConfig(ImuType type, int id) {
        this.type = type;
        this.id = id;
    }

    public ImuConfig(ImuType type) {
        this(type, -1);
    }

    public ImuConfig setCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    public ImuConfig setId(int id) {
        this.id = id;
        return this;
    }

    public ImuConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }
}
