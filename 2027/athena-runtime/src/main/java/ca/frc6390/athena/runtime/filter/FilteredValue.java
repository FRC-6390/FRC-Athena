package ca.frc6390.athena.runtime.filter;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Chainable scalar filter pipeline with cached last output.
 */
public final class FilteredValue {
    private final DoubleSupplier source;
    private final List<FilterFunction> filters = new ArrayList<>();
    private double lastValue;

    /**
     * Creates a filtered value.
     *
     * @param source raw value source
     */
    public FilteredValue(DoubleSupplier source) {
        this.source = source == null ? () -> 0.0 : source;
    }

    /**
     * Adds a filter to this pipeline.
     *
     * @param filter filter function
     * @return this filtered value
     */
    public FilteredValue addFilter(FilterFunction filter) {
        if (filter != null) {
            filters.add(filter);
        }
        return this;
    }

    /**
     * Adds a constant offset filter.
     *
     * @param offset offset to add
     * @return this filtered value
     */
    public FilteredValue addOffset(double offset) {
        return addFilter(value -> value + offset);
    }

    /**
     * Adds a moving average filter.
     *
     * @param taps number of retained samples
     * @return this filtered value
     */
    public FilteredValue addMovingAverage(int taps) {
        int window = Math.max(1, taps);
        ArrayDeque<Double> samples = new ArrayDeque<>();
        return addFilter(value -> {
            samples.addLast(value);
            while (samples.size() > window) {
                samples.removeFirst();
            }
            return samples.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        });
    }

    /**
     * Returns cached value unless runFilter is true.
     *
     * @param runFilter true to sample and filter
     * @return value
     */
    public double get(boolean runFilter) {
        return runFilter ? getFiltered() : lastValue;
    }

    /**
     * Samples source and runs the filter chain.
     *
     * @return filtered value
     */
    public double getFiltered() {
        double value = source.getAsDouble();
        if (!Double.isFinite(value)) {
            value = 0.0;
        }
        for (FilterFunction filter : filters) {
            value = filter.apply(value);
        }
        lastValue = value;
        return value;
    }
}
