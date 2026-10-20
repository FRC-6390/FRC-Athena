package ca.frc6390.athena.hardware.device;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.power.MotorThermalSpec;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.sim.SimModel;
import edu.wpi.first.units.Units;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/** Derived values cached per declaration must match what the uncached code computed. */
class DeclarationCachingTest {
    @Test
    void motorHashCodeAgreesWithEqualsForSeparatelyBuiltDeclarations() {
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 4).canbus("canivore").inverted().brake()
                .statorCurrentLimit(Units.Amps.of(60.0)).thermal(Thermal.derate(
                        Units.Celsius.of(80.0), Units.Celsius.of(110.0), 0.4))
                .follow(MotorDevice.of(MotorKinds.KRAKEN_X60, 5));
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 4).canbus("canivore").inverted().brake()
                .statorCurrentLimit(Units.Amps.of(60.0)).thermal(Thermal.derate(
                        Units.Celsius.of(80.0), Units.Celsius.of(110.0), 0.4))
                .follow(MotorDevice.of(MotorKinds.KRAKEN_X60, 5));

        assertEquals(first, second);
        assertEquals(first.hashCode(), second.hashCode());
        assertNotEquals(first, second.statorCurrentLimit(Units.Amps.of(40.0)));
        assertNotEquals(first, second.independent());
        assertNotEquals(first, second.disabled());
        assertNotEquals(first, second.failurePolicy(FailurePolicy.PANIC));
        assertNotEquals(MotorDevice.of(MotorKinds.NEO, 1), MotorDevice.of(MotorKinds.NEO, 2));

        Set<MotorDevice> set = new HashSet<>();
        set.add(first);
        assertTrue(set.contains(second));
    }

    @Test
    void motorDefaultNameMatchesSanitizedKindAndIdAndIsComputedOnce() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 7).canbus("CANivore A");
        MotorDevice paired = MotorDevice.of(MotorControllerKinds.SPARK_FLEX, MotorKinds.NEO, 12);

        assertEquals("ctre_talon_fx/kraken_x60_7", motor.defaultName());
        assertEquals("rev_spark_flex/neo_12", paired.defaultName());
        assertSame(motor.defaultName(), motor.defaultName());
        assertEquals(motor.defaultName(), MotorDevice.of(MotorKinds.KRAKEN_X60, 7).inverted().defaultName());
    }

    @Test
    void encoderAndImuHashCodesAndNamesMatchTheirComponents() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 9);
        EncoderDevice integrated = motor.encoder();
        EncoderDevice sameIntegrated = MotorDevice.of(MotorKinds.NEO, 9).encoder();
        EncoderDevice absolute = motor.absoluteEncoder();
        EncoderDevice cancoder = EncoderDevice.of(EncoderKinds.CANCODER, 21);

        assertEquals(integrated, sameIntegrated);
        assertEquals(integrated.hashCode(), sameIntegrated.hashCode());
        assertEquals(cancoder.hashCode(), EncoderDevice.of(EncoderKinds.CANCODER, 21).hashCode());
        assertNotEquals(cancoder, cancoder.measures(Units.Degrees));
        assertEquals("athena_integrated_motor_9", integrated.defaultName());
        assertEquals("athena_integrated_motor_9", absolute.defaultName());
        assertEquals("ctre_cancoder_can_21", cancoder.defaultName());
        assertSame(cancoder.defaultName(), cancoder.defaultName());

        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 2);
        assertEquals("ctre_pigeon_2_can_2", imu.defaultName());
        assertSame(imu.defaultName(), imu.defaultName());
    }

    @Test
    void tableConstantsAreSharedAndEqualToFreshlyDerivedValues() {
        MotorSpec row = MotorSpec.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        MotorSpec copy = new MotorSpec(row.kind(), row.stallTorqueNm(), row.stallCurrentAmps(),
                row.freeCurrentAmps(), row.freeSpeedRpm(), row.heatCapacityJoulesPerCelsius(), row.coolingSeconds(),
                row.ambientCelsius(), row.maxWindingCelsius());

        assertSame(row, MotorSpec.forKind(MotorKinds.KRAKEN_X60).orElseThrow());
        assertSame(row.constants(), row.constants());
        assertSame(row.thermalSpec(), row.thermalSpec());
        assertEquals(copy.constants(), row.constants());
        assertEquals(copy.thermalSpec(), row.thermalSpec());
        assertSame(row.constants(), MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow());
        assertSame(row.thermalSpec(), MotorThermalSpec.forKind(MotorKinds.KRAKEN_X60));
        assertTrue(MotorConstants.forKind(MotorKinds.MINION).isEmpty());
        assertSame(MotorThermalSpec.GENERIC, MotorThermalSpec.forKind(MotorKinds.MINION));

        MotorThermalSpec spec = row.thermalSpec();
        Thermal expected = new Thermal(Units.Celsius.of(spec.maxWindingCelsius() - 30.0),
                Units.Celsius.of(spec.maxWindingCelsius()), 0.5);
        assertEquals(expected, spec.defaultDerate());
        assertSame(spec.defaultDerate(), spec.defaultDerate());
    }

    @Test
    void lazyRuntimeDescriptionIsBuiltOnlyWhenTheLookupFails() {
        RuntimeBindings<MotorDevice, String> bindings = new RuntimeBindings<>();
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 3);
        AtomicInteger described = new AtomicInteger();

        IllegalStateException unbound = assertThrows(IllegalStateException.class,
                () -> bindings.get(motor, device -> {
                    described.incrementAndGet();
                    return "Motor " + device.defaultName();
                }));
        assertEquals("Motor rev_spark_max/neo_3 is not runtime-bound.", unbound.getMessage());
        assertEquals(1, described.get());

        bindings.bind(motor, new RuntimeScope("test"), "bound");
        assertEquals("bound", bindings.get(motor, device -> {
            described.incrementAndGet();
            return "unused";
        }));
        assertEquals(1, described.get());
    }

    @Test
    void unboundMotorReadsStillNameTheMotor() {
        IllegalStateException failure = assertThrows(IllegalStateException.class,
                () -> MotorDevice.of(MotorKinds.NEO_550, 44).raw().supplyCurrentAmps());
        assertEquals("Motor rev_spark_max/neo_550_44 is not runtime-bound.", failure.getMessage());
    }

    @Test
    void simModelAggregatesAreStableAndDeduplicated() {
        MotorDevice left = MotorDevice.of(MotorKinds.NEO, 1);
        MotorDevice right = MotorDevice.of(MotorKinds.NEO, 2);
        SimModel model = SimModel.compose(SimModel.flywheel(left, right), SimModel.flywheel(
                MotorDevice.of(MotorKinds.NEO, 1)));

        assertEquals(List.of(left, right), model.motors());
        assertSame(model.motors(), model.motors());
        assertThrows(UnsupportedOperationException.class, () -> model.motors().clear());
    }
}
