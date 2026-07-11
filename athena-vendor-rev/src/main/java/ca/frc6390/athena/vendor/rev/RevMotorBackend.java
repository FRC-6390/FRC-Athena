package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * REV motor backend boundary.
 */
public final class RevMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.controllerKind() == MotorControllerKinds.SPARK_MAX
                || kind.controllerKind() == MotorControllerKinds.SPARK_FLEX
                || kind.key().startsWith("rev:spark-");
    }

    @Override
    public MotorHandle create(MotorDevice device) {
        return new RevMotorHandle(
                device, device.vendorOptions().find(RevMotorOptions.class).orElse(new RevMotorOptions()));
    }
}
