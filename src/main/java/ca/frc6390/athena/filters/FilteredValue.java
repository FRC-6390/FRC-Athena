package ca.frc6390.athena.filters;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class FilteredValue extends FilterList {

    private DoubleSupplier supplier;
    private double preValue;

    public FilteredValue(DoubleSupplier supplier) {
        super();
        this.supplier = supplier;
    }

    public double get() {
        return get(false);
    }

    public double get(boolean runFilter) {
       return runFilter ? getFiltered() : preValue;
    }

    public double getFiltered(){
        double value = getUnfiltered();
        for (Function<Double, Double> filter : getFilterFunctions()) {
            value = filter.apply(value);
        }
        preValue = value;
        return value;
    }

    public double getUnfiltered() {
        return supplier.getAsDouble();
    }

    @Override
    public FilteredValue addMedianFilter(int window) {
        super.addMedianFilter(window);
        return this;
    }

    @Override
    public FilteredValue addLinearFilter(double[] ffgains, double[] fbgains) {
        super.addLinearFilter(ffgains, fbgains);
        return this;
    }

    @Override
    public FilteredValue addHighPass(double timeConstant, double period) {
        super.addHighPass(timeConstant, period);
        return this;
    }

    @Override
    public FilteredValue addFiniteDifference(int derivative, int[] stencil, double period) {
        super.addFiniteDifference(derivative, stencil, period);
        return this;
    }

    @Override
    public FilteredValue addMovingAverage(int taps) {
        super.addMovingAverage(taps);
        return this;
    }

    @Override
    public FilteredValue addBackwardsFiniteDifference(int derivative, int samples, double period) {
        super.addBackwardsFiniteDifference(derivative, samples, period);
        return this;
    }

    @Override
    public FilteredValue addSinglePoleIIR(double timeConstant, double period) {
        super.addSinglePoleIIR(timeConstant, period);
        return this;
    }

    @Override
    public FilteredValue addSlewRateLimtier(double rateLimiter) {
        super.addSlewRateLimtier(rateLimiter);
        return this;
    }

    @Override
    public FilteredValue addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit) {
        super.addSlewRateLimtier(positiveRateLimit, negativeRateLimit);
        return this;
    }

    @Override
    public FilteredValue addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        super.addSlewRateLimtier(positiveRateLimit, negativeRateLimit, initialValue);
        return this;
    }

    @Override
    public FilteredValue addFilter(Object filter, Function<Double, Double> func) {
        super.addFilter(filter, func);
        return this;
    }
}
