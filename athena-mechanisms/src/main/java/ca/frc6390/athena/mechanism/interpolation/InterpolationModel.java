package ca.frc6390.athena.mechanism.interpolation;

/** Calculates one control target from an input and ordered interpolation data. */
@FunctionalInterface
public interface InterpolationModel {
    /** Returns the interpolated target. */
    double interpolate(double input, InterpolationData data);
}
