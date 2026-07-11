package ca.frc6390.athena.api.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class KindCatalogTest {
    @Test
    void builtInKindKeysStayStable() {
        assertEquals("ctre:talon-fx/kraken-x60", MotorKinds.KRAKEN_X60.key());
        assertEquals("kraken-x60", MotorKinds.KRAKEN_X60.motorKey());
        assertEquals("rev:through-bore", EncoderKinds.REV_THROUGH_BORE.key());
        assertEquals("studica:navx", ImuKinds.NAVX.key());
        assertEquals("limelight:camera", CameraKinds.LIMELIGHT.key());
    }

    @Test
    void builtInKindKeysAreUniqueWithinCatalogs() {
        assertUnique(Arrays.stream(MotorKinds.values()).map(MotorKinds::key).collect(Collectors.toSet()),
                MotorKinds.values().length);
        assertUnique(Arrays.stream(MotorControllerKinds.values())
                .map(MotorControllerKinds::key).collect(Collectors.toSet()), MotorControllerKinds.values().length);
        assertUnique(Arrays.stream(EncoderKinds.values()).map(EncoderKinds::key).collect(Collectors.toSet()),
                EncoderKinds.values().length);
        assertUnique(Arrays.stream(ImuKinds.values()).map(ImuKinds::key).collect(Collectors.toSet()),
                ImuKinds.values().length);
        assertUnique(Arrays.stream(CameraKinds.values()).map(CameraKinds::key).collect(Collectors.toSet()),
                CameraKinds.values().length);
    }

    @Test
    void builtInCatalogsImplementKindContracts() {
        assertInstanceOf(MotorKind.class, MotorKinds.KRAKEN_X60);
        assertInstanceOf(MotorControllerKind.class, MotorControllerKinds.TALON_FX);
        assertInstanceOf(EncoderKind.class, EncoderKinds.CANCODER);
        assertInstanceOf(ImuKind.class, ImuKinds.PIGEON_2);
        assertInstanceOf(CameraKind.class, CameraKinds.PHOTONVISION);
    }

    @Test
    void physicalMotorsExposeDefaultsAndAllowControllerOverrides() {
        assertEquals(MotorControllerKinds.SPARK_MAX, MotorKinds.NEO.controllerKind());
        assertEquals(MotorTechnology.BRUSHLESS, MotorKinds.NEO.technology());

        MotorKind flexNeo = MotorKinds.NEO.controlledBy(MotorControllerKinds.SPARK_FLEX);

        assertEquals(MotorControllerKinds.SPARK_FLEX, flexNeo.controllerKind());
        assertEquals(MotorKinds.NEO, flexNeo.motorKind());
        assertEquals("rev:spark-flex/neo", flexNeo.key());
        assertThrows(IllegalArgumentException.class,
                () -> MotorKinds.FALCON_500.controlledBy(MotorControllerKinds.SPARK_MAX));
    }

    private static void assertUnique(Set<String> keys, int expectedSize) {
        assertEquals(expectedSize, keys.size());
    }
}
