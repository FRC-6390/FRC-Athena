package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;

/**
 * CTRE IMU backend for Pigeon2 devices.
 */
public final class CtreImuBackend implements ImuBackend {
    @Override
    public boolean supports(ImuKind kind) {
        return kind == AthenaImu.PIGEON_2 || kind.key().equals("ctre:pigeon-2");
    }

    @Override
    public ImuDevice create(ImuSpec spec) {
        return new CtrePigeon2Device(spec);
    }
}
