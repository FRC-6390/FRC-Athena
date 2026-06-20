package ca.frc6390.athena.runtime.filter;

/**
 * Scalar filter function.
 */
@FunctionalInterface
public interface FilterFunction {
    /**
     * Applies the filter.
     *
     * @param value input value
     * @return output value
     */
    double apply(double value);
}
