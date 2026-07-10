package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;

/**
 * CTRE IMU backend for Pigeon2 devices.
 */
public final class CtreImuBackend implements ImuBackend {
    @Override
    public boolean supports(ImuKind kind) {
        return kind == ImuKinds.PIGEON_2 || kind.key().equals("ctre:pigeon-2");
    }

    @Override
    public boolean supports(ImuDevice device) {
        return supports(device.kind()) && device.connection() instanceof HardwareAddress.Can;
    }

    @Override
    public ImuHandle create(ImuDevice device) {
        return new CtrePigeon2Handle(device);
    }
}
