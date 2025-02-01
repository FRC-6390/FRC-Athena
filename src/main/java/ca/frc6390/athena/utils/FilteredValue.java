package ca.frc6390.athena.utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class FilteredValue {

    private DoubleSupplier supplier;

    private ArrayList<Function<Double, Double>> filtersFuncs;
    private ArrayList<Object> filters;

    public FilteredValue(DoubleSupplier supplier) {
        this.supplier = supplier;
        filtersFuncs = new ArrayList<>();
        filters = new ArrayList<>();
    }

    public FilteredValue addMedianFilter(int window) {
        MedianFilter filter = new MedianFilter(window);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addLinearFilter(double[] ffgains, double[] fbgains) {
        LinearFilter filter = new LinearFilter(ffgains,fbgains);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addHighPass(double timeConstant, double period) {
        LinearFilter filter = LinearFilter.highPass(timeConstant, period);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addFiniteDifference(int derivative, int[] stencil, double period) {
        LinearFilter filter = LinearFilter.finiteDifference(derivative, stencil, period);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addMovingAverage(int taps) {
        LinearFilter filter = LinearFilter.movingAverage(taps);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addBackwardsFiniteDifference(int derivative, int samples, double period) {
        LinearFilter filter = LinearFilter.backwardFiniteDifference(derivative, samples, period);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addSinglePoleIIR(double timeConstant, double period) {
        LinearFilter filter = LinearFilter.singlePoleIIR(timeConstant, period);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addSlewRateLimtier(double ratelimiter) {
        SlewRateLimiter filter = new SlewRateLimiter(ratelimiter);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit) {
        return addSlewRateLimtier(positiveRateLimit, negativeRateLimit, 0);
    }

    public FilteredValue addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        SlewRateLimiter filter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, initialValue);
        return addFilter(filter, filter::calculate);
    }

    public FilteredValue addFilter(Object filter, Function<Double, Double> func) {
        filters.add(filter);
        filtersFuncs.add(func);
        return this;
    }

    public double get() {
        double value = getUnfiltered();
        for (Function<Double, Double> filter : filtersFuncs) {
            value = filter.apply(value);
        }
        return getUnfiltered();
    }

    public double getUnfiltered() {
        return supplier.getAsDouble();
    }
}
