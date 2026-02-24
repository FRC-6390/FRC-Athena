package ca.frc6390.athena.filters.examples;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.filters.FilteredPose;
import ca.frc6390.athena.filters.FilteredValue;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Example filter pipelines for scalar and pose values.
 */
public final class FilterExamples {
    private FilterExamples() {}

    public static FilteredValue smoothedValue(DoubleSupplier source) {
        return new FilteredValue(source)
                .addMovingAverage(3)
                .addSlewRateLimtier(3.0);
    }

    public static FilteredValue offsetValue(DoubleSupplier source, double offset) {
        return new FilteredValue(source)
                .addFilter("offset", value -> value + offset);
    }

    public static FilteredPose smoothedPose(Supplier<Pose2d> source) {
        return new FilteredPose(source)
                .addMovingAverage(3)
                .addSlewRateLimtier(2.0);
    }
}
