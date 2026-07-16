package ca.frc6390.athena.mechanism.interpolation;

/** Ordered interpolation points and their values for one runtime evaluation. */
public interface InterpolationData {
    /** Returns the number of points. */
    int size();

    /** Returns the input coordinate at an index. */
    double point(int index);

    /** Returns the value at an index, sampling dynamic values at most once per evaluation. */
    double value(int index);

    /** Returns the nearest point index at or below the input, clamped to the table. */
    int lowerIndex(double input);

    /** Returns the nearest point index at or above the input, clamped to the table. */
    int upperIndex(double input);
}
