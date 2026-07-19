package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.DoubleSupplier;
import java.util.Map;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.InterpolatedControlAction;
import ca.frc6390.athena.runtime.geometry.Point2d;
import org.junit.jupiter.api.Test;

class AxisSignalTest {
    @Test
    void buildsAnImmutableAxisPipelineAndExposesItAsASupplier() {
        double[] raw = {0.6};
        AxisSignal rawAxis = new AxisSignal(() -> raw[0]);
        DoubleSupplier processed = rawAxis.deadband(0.2).squared().inverted().toSupplier();

        assertEquals(0.6, rawAxis.toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(-0.25, processed.getAsDouble(), 1.0e-9);

        raw[0] = 0.1;
        assertEquals(0.0, processed.getAsDouble(), 1.0e-9);
    }

    @Test
    void clampsInvalidAndOutOfRangeInput() {
        double[] raw = {2.0};
        DoubleSupplier axis = new AxisSignal(() -> raw[0]).toSupplier();

        assertEquals(1.0, axis.getAsDouble(), 1.0e-9);
        raw[0] = Double.NaN;
        assertEquals(0.0, axis.getAsDouble(), 1.0e-9);
    }

    @Test
    void squaringPreservesNegativeInputSign() {
        DoubleSupplier axis = new AxisSignal(() -> -0.6)
                .deadband(0.2)
                .squared()
                .toSupplier();

        assertEquals(-0.25, axis.getAsDouble(), 1.0e-9);
    }

    @Test
    void thresholdSignalsSupportHysteresis() {
        double[] raw = {0.0};
        ControlSignal threshold = new AxisSignal(() -> raw[0]).above(0.6, 0.4);

        assertFalse(ControlSignalTest.sample(threshold, 0.0));
        raw[0] = 0.7;
        assertTrue(ControlSignalTest.sample(threshold, 0.02));
        raw[0] = 0.5;
        assertTrue(ControlSignalTest.sample(threshold, 0.04));
        raw[0] = 0.3;
        assertFalse(ControlSignalTest.sample(threshold, 0.06));
    }

    @Test
    void belowThresholdSupportsHysteresisAtBothBoundaries() {
        double[] raw = {0.0};
        ControlSignal threshold = new AxisSignal(() -> raw[0]).below(-0.6, -0.4);

        assertFalse(ControlSignalTest.sample(threshold, 0.0));
        raw[0] = -0.6;
        assertFalse(ControlSignalTest.sample(threshold, 0.02));
        raw[0] = -0.61;
        assertTrue(ControlSignalTest.sample(threshold, 0.04));
        raw[0] = -0.4;
        assertFalse(ControlSignalTest.sample(threshold, 0.06));
    }

    @Test
    void insideOutsideAndProcessedThresholdsUseProcessedAxisValues() {
        double[] raw = {0.5};
        AxisSignal axis = new AxisSignal(() -> raw[0]).deadband(0.2).inverted();
        ControlSignal inside = axis.inside(0.4);
        ControlSignal outside = axis.outside(0.3);
        ControlSignal below = axis.below(-0.3);

        assertTrue(ControlSignalTest.sample(inside, 0.0));
        assertTrue(ControlSignalTest.sample(outside, 0.02));
        assertTrue(ControlSignalTest.sample(below, 0.04));
        raw[0] = 0.2;
        assertTrue(ControlSignalTest.sample(inside, 0.06));
        assertFalse(ControlSignalTest.sample(outside, 0.08));
        assertFalse(ControlSignalTest.sample(below, 0.10));
    }

    @Test
    void thresholdArgumentsRejectNonFiniteMagnitudeAndInvalidHysteresis() {
        AxisSignal axis = new AxisSignal(() -> 0.0);

        assertThrows(IllegalArgumentException.class, () -> axis.above(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> axis.below(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> axis.above(0.4, 0.5));
        assertThrows(IllegalArgumentException.class, () -> axis.below(-0.4, -0.5));
        assertThrows(IllegalArgumentException.class, () -> axis.inside(-0.1));
        assertThrows(IllegalArgumentException.class, () -> axis.outside(1.1));
    }

    @Test
    void customMappingsRunAfterDeadbandAndRemainNormalized() {
        double[] raw = {0.6};
        AxisSignal axis = new AxisSignal(() -> raw[0])
                .deadband(0.2)
                .map(value -> value * 3.0);

        assertEquals(1.0, axis.value(), 1.0e-9);
        raw[0] = -0.4;
        assertEquals(-0.75, axis.value(), 1.0e-9);
        assertEquals(0.0, new AxisSignal(() -> 0.5).map(value -> Double.NaN).value(), 1.0e-9);
    }

    @Test
    void superRateIsSymmetricAndLiveTunable() {
        AxisCurves.SuperRateCurve curve = AxisCurves.superRate()
                .rcRate(1.0)
                .expo(0.4)
                .superRate(0.65);
        double initial = curve.apply(0.5);

        assertEquals(-initial, curve.apply(-0.5), 1.0e-9);
        assertEquals(1.0, curve.apply(1.0), 1.0e-9);
        curve.telemetry().get("Expo").set(0.0);
        assertTrue(Math.abs(curve.apply(0.5) - initial) > 1.0e-6);
    }

    @Test
    void sharedCurveUpdatesSeveralAxesWithoutSharingAxisState() {
        AxisCurves.ExpoCurve curve = AxisCurves.expo(0.0);
        double[] firstRaw = {0.5};
        double[] secondRaw = {-0.75};
        double[] time = {0.0};
        AxisSignal first = new AxisSignal(() -> firstRaw[0], () -> time[0])
                .curve(curve)
                .slew(1.0);
        AxisSignal second = new AxisSignal(() -> secondRaw[0], () -> time[0])
                .curve(curve)
                .slew(4.0);

        assertEquals(0.02, first.value(), 1.0e-9);
        assertEquals(-0.08, second.value(), 1.0e-9);

        first.telemetry().get("Curve/Config/Expo").set(1.0);
        assertEquals(1.0, curve.expo(), 1.0e-9);
        assertEquals(0.125,
                ((double[]) first.telemetry().get("Curve/Output").value())[30], 1.0e-9);
        assertEquals(-0.421875,
                ((double[]) second.telemetry().get("Curve/Output").value())[5], 1.0e-9);

        time[0] = 0.1;
        assertEquals(0.12, first.value(), 1.0e-9);
        assertEquals(-0.421875, second.value(), 1.0e-9);
    }

    @Test
    void oneCurveCanShapeAnAxisAndInterpolationWithSharedLiveTuning() {
        AxisCurves.ExpoCurve curve = AxisCurves.expo(0.0);
        AxisSignal axis = new AxisSignal(() -> 0.5).curve(curve);
        InterpolatedControlAction interpolation = Controls.position(
                        MotorDevice.of(MotorKinds.KRAKEN_X60, 41))
                .interpolate(curve, () -> 5.0)
                .at(0.0, 0.0)
                .at(10.0, 100.0);

        assertEquals(0.5, axis.value(), 1.0e-9);
        assertEquals(50.0, interpolation.target(), 1.0e-9);

        interpolation.telemetry().get("Config/Curve/Expo").set(1.0);
        assertEquals(0.125, axis.value(), 1.0e-9);
        assertEquals(12.5, interpolation.target(), 1.0e-9);
        Point2d[] axisGraph = (Point2d[]) axis.telemetry().get("Visualization/Curve").value();
        Point2d[] interpolationGraph =
                (Point2d[]) interpolation.telemetry().get("Visualization/Curve").value();
        assertEquals(0.125, axisGraph[30].y(), 1.0e-9);
        assertEquals(12.5, interpolationGraph[8].y(), 1.0e-9);
    }

    @Test
    void slewRatesAreAppliedUsingElapsedTime() {
        double[] raw = {1.0};
        double[] time = {0.0};
        AxisSignal axis = new AxisSignal(() -> raw[0], () -> time[0]).slew(2.0, 4.0);

        assertEquals(0.04, axis.value(), 1.0e-9);
        time[0] = 0.1;
        assertEquals(0.24, axis.value(), 1.0e-9);
        raw[0] = -1.0;
        time[0] = 0.2;
        assertEquals(-0.16, axis.value(), 1.0e-9);
    }

    @Test
    void telemetryProvidesEditableConfigAndSampledCurve() {
        AxisCurves.ExpoCurve curve = AxisCurves.expo(0.0);
        AxisSignal axis = new AxisSignal(() -> 0.5).deadband(0.1).curve(curve).slew(3.0);
        Map<String, ca.frc6390.athena.mechanism.core.TelemetryValue> telemetry = axis.telemetry();

        double[] input = (double[]) telemetry.get("Curve/Input").value();
        double[] linearOutput = (double[]) telemetry.get("Curve/Output").value();
        assertEquals(41, input.length);
        assertEquals(-1.0, input[0], 1.0e-9);
        assertEquals(1.0, input[40], 1.0e-9);

        telemetry.get("Curve/Config/Expo").set(1.0);
        double[] cubicOutput = (double[]) telemetry.get("Curve/Output").value();
        assertTrue(Math.abs(cubicOutput[30]) < Math.abs(linearOutput[30]));
        telemetry.get("Config/Deadband").set(0.2);
        assertEquals(0.2, telemetry.get("Config/Deadband").number(), 1.0e-9);
    }

    @Test
    void visualizationTracksRawMappedAndSlewLimitedOutputs() {
        double[] raw = {0.5};
        double[] time = {0.0};
        AxisSignal axis = new AxisSignal(() -> raw[0], () -> time[0])
                .curve(AxisCurves.power(2.0))
                .slew(1.0);
        axis.value();
        Map<String, ca.frc6390.athena.mechanism.core.TelemetryValue> telemetry = axis.telemetry();

        Point2d[] curve = (Point2d[]) telemetry.get("Visualization/Curve").value();
        Point2d rawMarker = ((Point2d[]) telemetry.get("Visualization/Raw").value())[0];
        Point2d mappedMarker = ((Point2d[]) telemetry.get("Visualization/Mapped").value())[0];
        Point2d outputMarker = ((Point2d[]) telemetry.get("Visualization/Output").value())[0];

        assertEquals(41, curve.length);
        assertEquals(0.5, rawMarker.x(), 1.0e-9);
        assertEquals(0.0, rawMarker.y(), 1.0e-9);
        assertEquals(0.25, mappedMarker.y(), 1.0e-9);
        assertEquals(0.02, outputMarker.y(), 1.0e-9);

        telemetry.get("Curve/Config/Exponent").set(3.0);
        Point2d[] updated = (Point2d[]) telemetry.get("Visualization/Curve").value();
        assertEquals(0.125, updated[30].y(), 1.0e-9);
    }
}
