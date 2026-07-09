package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Simulation motor backend for real robot motor declarations.
 */
public final class SimMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind == MotorKinds.TALON_FX
                || kind == MotorKinds.KRAKEN_X60
                || kind == MotorKinds.KRAKEN_X44
                || kind == MotorKinds.SPARK_MAX_BRUSHLESS
                || kind == MotorKinds.SPARK_MAX_BRUSHED
                || kind == MotorKinds.SPARK_FLEX_BRUSHLESS
                || kind == MotorKinds.SPARK_FLEX_BRUSHED;
    }

    @Override
    public MotorHandle create(MotorDevice device) {
        return new SimMotorHandle(device);
    }
}
