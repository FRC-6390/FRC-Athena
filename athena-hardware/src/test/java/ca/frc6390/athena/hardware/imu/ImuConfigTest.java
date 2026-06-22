package ca.frc6390.athena.hardware.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.ImuId;

class ImuConfigTest {
    @Test
    void lowersHardwareIdentityIntoSpec() {
        ImuSpec spec = ImuConfig.create()
                .hardware(AthenaImu.PIGEON_2, 9)
                .canbus("canivore")
                .mountPose(90.0, 1.5, -2.0)
                .toSpec("drive", "gyro");

        assertEquals("drive.gyro", spec.path());
        assertEquals(AthenaImu.PIGEON_2, spec.kind());
        assertEquals(9, spec.id());
        assertEquals("canivore", spec.canbus());
        assertEquals(new ImuMountPose(90.0, 1.5, -2.0), spec.mountPose());
    }

    @Test
    void hardwareAliasCarriesCanBus() {
        ImuSpec spec = ImuConfig.create()
                .hardware(ImuId.of(AthenaImu.NAVX, 1).canbus("rio-can"))
                .toSpec("robot", "imu");

        assertEquals(AthenaImu.NAVX, spec.kind());
        assertEquals(1, spec.id());
        assertEquals("rio-can", spec.canbus());
    }

    @Test
    void specNormalizesBlankNamesAndBus() {
        ImuSpec spec = ImuConfig.create()
                .hardware(AthenaImu.SIM, 0)
                .canbus(" ")
                .toSpec("", "");

        assertEquals("robot.imu", spec.path());
        assertEquals("rio", spec.canbus());
        assertEquals(ImuMountPose.identity(), spec.mountPose());
    }

    @Test
    void mountPoseReportsFiniteValues() {
        assertTrue(new ImuMountPose(0.0, 10.0, -10.0).isFinite());
        assertFalse(new ImuMountPose(Double.NaN, 0.0, 0.0).isFinite());
        assertFalse(new ImuMountPose(0.0, Double.POSITIVE_INFINITY, 0.0).isFinite());
    }

    @Test
    void hardwareKindIsRequired() {
        assertThrows(IllegalStateException.class, () -> ImuConfig.create().toSpec("robot", "imu"));
        assertThrows(NullPointerException.class, () -> ImuConfig.create().hardware((ImuId) null));
    }
}
