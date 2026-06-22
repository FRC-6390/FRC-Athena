package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;
import java.util.Objects;
import java.util.function.Function;

/**
 * Studica/NavX IMU backend backed by the Studica AHRS library.
 */
public final class StudicaImuBackend implements ImuBackend {
    private final Function<ImuSpec, ImuDevice> deviceFactory;

    /**
     * Creates a backend that constructs real Studica/NavX devices.
     */
    public StudicaImuBackend() {
        this(StudicaImuDevice::new);
    }

    StudicaImuBackend(Function<ImuSpec, ImuDevice> deviceFactory) {
        this.deviceFactory = Objects.requireNonNull(deviceFactory, "deviceFactory");
    }

    @Override
    public boolean supports(ImuKind kind) {
        return kind == AthenaImu.NAVX || kind.key().equals("studica:navx");
    }

    @Override
    public ImuDevice create(ImuSpec spec) {
        return deviceFactory.apply(spec);
    }
}
