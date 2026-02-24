package ca.frc6390.athena.util.examples;

import ca.frc6390.athena.util.InputRef;
import ca.frc6390.athena.util.SupplierUtil;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * Examples for InputRef and SupplierUtil wrappers.
 */
public final class InputUtilityExamples {
    private InputUtilityExamples() {}

    public static DoubleSupplier safeGainSupplier(DoubleSupplier configured) {
        return SupplierUtil.wrapDouble(configured, 0.0);
    }

    public static OptionalDouble optionalSafeTemperature(DoubleSupplier readingSupplier) {
        return SupplierUtil.optionalDouble(
                readingSupplier,
                value -> !Double.isFinite(value) || value < -40.0 || value > 140.0);
    }

    public static InputRef<Double> createSetpointRef(double initialSetpoint) {
        InputRef<Double> ref = InputRef.create();
        ref.set(initialSetpoint);
        return ref;
    }
}
