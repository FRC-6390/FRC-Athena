package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import java.util.Objects;
import java.util.function.Function;

/**
 * Studica/NavX IMU backend backed by the Studica AHRS library.
 */
public final class StudicaImuBackend implements ImuBackend {
    private final Function<ImuDevice, ImuHandle> handleFactory;

    /**
     * Creates a backend that constructs real Studica/NavX devices.
     */
    public StudicaImuBackend() {
        this(StudicaImuHandle::new);
    }

    StudicaImuBackend(Function<ImuDevice, ImuHandle> handleFactory) {
        this.handleFactory = Objects.requireNonNull(handleFactory, "handleFactory");
    }

    @Override
    public boolean supports(ImuKind kind) {
        return kind == ImuKinds.NAVX || kind.key().equals("studica:navx");
    }

    @Override
    public ImuHandle create(ImuDevice device) {
        return handleFactory.apply(device);
    }
}
