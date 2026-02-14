package ca.frc6390.athena.hardware.imu;

import java.util.function.Consumer;

import ca.frc6390.athena.hardware.config.DeviceIdentityConfig;
import ca.frc6390.athena.hardware.config.DeviceIdentitySection;

/**
 * Vendor-agnostic IMU configuration.
 */
public class ImuConfig extends DeviceIdentityConfig<ImuType> {

    /**
     */
    public ImuConfig(ImuType type, int id) {
        super(type, id);
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

    public HardwareSection hardware() {
        return new HardwareSection(this);
    }

    public static final class HardwareSection extends DeviceIdentitySection<HardwareSection, ImuType> {
        private HardwareSection(ImuConfig owner) {
            super(owner::applyType, owner::applyId, owner::applyCanbus, owner::applyInverted);
        }

        @Override
        protected HardwareSection self() {
            return this;
        }
    }
}
