package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class ThermalTest {
    private static final double EPS = 1e-9;

    @Test
    void derateCurveIsFullBelowStartAndMinimumAboveEnd() {
        Thermal curve = Thermal.derate(Units.Celsius.of(80), Units.Celsius.of(120), 0.4);
        assertEquals(1.0, curve.derateFraction(60), EPS);
        assertEquals(1.0, curve.derateFraction(80), EPS);
        assertEquals(0.7, curve.derateFraction(100), EPS);
        assertEquals(0.4, curve.derateFraction(120), EPS);
        assertEquals(0.4, curve.derateFraction(200), EPS);
    }

    @Test
    void loadIsClampedRamp() {
        Thermal curve = new Thermal(80, 120, 0.4);
        assertEquals(0.0, curve.load(10), EPS);
        assertEquals(0.5, curve.load(100), EPS);
        assertEquals(1.0, curve.load(500), EPS);
    }

    @Test
    void unknownTemperatureDoesNotDerate() {
        Thermal curve = new Thermal(80, 120, 0.4);
        assertEquals(1.0, curve.derateFraction(Double.NaN), EPS);
        assertEquals(0.0, curve.load(Double.NaN), EPS);
    }

    @Test
    void invalidCurvesRejected() {
        assertThrows(IllegalArgumentException.class, () -> new Thermal(120, 80, 0.5));
        assertThrows(IllegalArgumentException.class, () -> new Thermal(80, 120, 1.5));
    }

    @Test
    void modelHeatsUnderCurrentAndCoolsToAmbient() {
        MotorThermalSpec spec = new MotorThermalSpec(0.05, 50.0, 100.0, 60.0, 25.0, 130.0);
        ThermalModel model = new ThermalModel(spec);
        assertEquals(25.0, model.celsius(), EPS);
        // Steady state under current: dT/dt = 0 when I^2 R / C = (T - Tamb) / tau.
        for (int i = 0; i < 60_000; i++) model.step(40.0, 0.02);
        double steady = 25.0 + (40.0 * 40.0 * 0.05 / 100.0) * 60.0;
        assertEquals(steady, model.modelCelsius(), 0.05);
        // Cool with no current.
        for (int i = 0; i < 60_000; i++) model.step(0.0, 0.02);
        assertEquals(25.0, model.modelCelsius(), 0.05);
    }

    @Test
    void vendorReadingIsPreferredAndPullsModelState() {
        MotorThermalSpec spec = new MotorThermalSpec(0.05, 50.0, 100.0, 60.0, 25.0, 130.0);
        ThermalModel model = new ThermalModel(spec);
        model.vendorReading(90.0);
        assertTrue(model.vendorFused());
        assertEquals(90.0, model.celsius(), EPS);
        for (int i = 0; i < 500; i++) model.step(0.0, 0.02);
        assertEquals(90.0, model.modelCelsius(), 5.0, "model converges toward vendor value");
        model.vendorReading(Double.NaN);
        assertTrue(!model.vendorFused());
        assertEquals(model.modelCelsius(), model.celsius(), EPS);
    }

    @Test
    void kindDefaultDerateStartsThirtyDegreesBelowRating() {
        MotorThermalSpec spec = MotorThermalSpec.GENERIC;
        Thermal derate = spec.defaultDerate();
        assertEquals(spec.maxWindingCelsius() - 30.0, derate.startCelsius(), EPS);
        assertEquals(spec.maxWindingCelsius(), derate.endCelsius(), EPS);
        assertEquals(0.5, derate.minFraction(), EPS);
    }
}
