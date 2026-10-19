package ca.frc6390.athena.runtime.measurement;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MeasurementStdDevsTest {
    @Test
    void headingInDegreesStoresTheSameRadiansAsPrimitiveInput() {
        MeasurementStdDevs typed = MeasurementStdDevs.of(Meters.of(0.25), Meters.of(0.5), Degrees.of(30.0));
        MeasurementStdDevs primitive = MeasurementStdDevs.Raw.of(0.25, 0.5, Math.toRadians(30.0));

        assertEquals(primitive.raw().headingRadians(), typed.raw().headingRadians(), 1.0e-12);
        assertEquals(primitive.raw().xMeters(), typed.raw().xMeters(), 1.0e-12);
        assertEquals(primitive.raw().yMeters(), typed.raw().yMeters(), 1.0e-12);
        assertEquals(primitive.raw().translationVarianceMetersSquared(),
                typed.raw().translationVarianceMetersSquared(), 1.0e-12);
    }

    @Test
    void headingAccessorsRoundTripDegreesAndRadians() {
        MeasurementStdDevs typed = MeasurementStdDevs.of(Meters.of(1.0), Meters.of(1.0), Degrees.of(30.0));

        assertEquals(Math.toRadians(30.0), typed.headingStdDev().in(Radians), 1.0e-12);
        assertEquals(30.0, typed.headingStdDev().in(Degrees), 1.0e-9);
        assertEquals(Math.toRadians(30.0), typed.raw().headingRadians(), 1.0e-12);
    }
}
