package ca.frc6390.athena.sim.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuSpec;

class SimImuBackendTest {
    @Test
    void supportsOnlySimulationImus() {
        SimImuBackend backend = new SimImuBackend();

        assertTrue(backend.supports(AthenaImu.SIM));
        assertFalse(backend.supports(AthenaImu.PIGEON_2));
        assertFalse(backend.supports(AthenaImu.NAVX));
    }

    @Test
    void createsZeroedSimImu() {
        SimImuBackend backend = new SimImuBackend();
        ImuSpec spec = ImuConfig.create()
                .hardware(AthenaImu.SIM, 0)
                .toSpec("robot", "imu");

        ImuDevice device = backend.create(spec);

        assertInstanceOf(SimImuDevice.class, device);
        assertEquals(spec, device.spec());
        assertEquals(0.0, device.yawDegrees());
    }
}
