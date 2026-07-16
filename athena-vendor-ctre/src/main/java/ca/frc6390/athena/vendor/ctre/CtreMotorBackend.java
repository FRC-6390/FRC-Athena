package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * CTRE motor backend boundary.
 */
public final class CtreMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.controllerKind() == MotorControllerKinds.TALON_FX
                || kind.controllerKind() == MotorControllerKinds.TALON_FXS
                || kind.key().startsWith("ctre:talon-fx/")
                || kind.key().startsWith("ctre:talon-fxs/")
                || kind.key().equals("ctre:talon-fx")
                || kind.key().equals("ctre:kraken-x60")
                || kind.key().equals("ctre:kraken-x44");
    }

    @Override
    public MotorHandle create(MotorDevice device) {
        return new CtreMotorHandle(
                device, device.vendorOptions().find(CtreMotorOptions.class).orElse(new CtreMotorOptions()));
    }

    @Override
    public boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader) {
        return follower.canbus().equalsIgnoreCase(leader.canbus()) && supports(leader.kind());
    }
}
