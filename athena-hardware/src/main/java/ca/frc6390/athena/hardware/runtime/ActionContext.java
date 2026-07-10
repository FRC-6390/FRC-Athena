package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Runtime access used by ref actions.
 */
public interface ActionContext {
    /**
     * Resolves an encoder declaration to its runtime handle.
     *
     * @param ref encoder declaration
     * @return runtime encoder
     */
    default EncoderHandle encoder(EncoderDevice ref) {
        throw new UnsupportedOperationException("Runtime encoder access is not available.");
    }

    /**
     * Resolves a motor declaration to its runtime handle.
     *
     * @param ref motor declaration
     * @return runtime motor
     */
    default MotorHandle motor(MotorDevice ref) {
        throw new UnsupportedOperationException("Runtime motor access is not available.");
    }

    default ImuHandle imu(ImuDevice ref) {
        throw new UnsupportedOperationException("Runtime IMU access is not available.");
    }

    /**
     * Empty context for tests that only execute pure runnable actions.
     *
     * @return empty action context
     */
    static ActionContext empty() {
        return new ActionContext() {
        };
    }
}
