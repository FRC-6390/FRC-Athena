package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.device.CameraDevice;
import java.util.List;

/**
 * Service provider for optional camera simulation engines.
 */
public interface VisionSimulationProvider {
    /**
     * Returns true when this provider can simulate the supplied cameras.
     *
     * @param cameras camera declarations
     * @return true when supported
     */
    boolean supports(List<CameraDevice> cameras);

    /**
     * Creates a vision simulation for the supplied cameras.
     *
     * @param cameras camera declarations
     * @return vision simulation
     */
    VisionSimulation create(List<CameraDevice> cameras);

    /**
     * Creates a vision simulation for the supplied cameras and field metadata.
     *
     * @param cameras camera declarations
     * @param field simulation field
     * @return vision simulation
     */
    default VisionSimulation create(List<CameraDevice> cameras, VisionSimulationField field) {
        return create(cameras);
    }
}
