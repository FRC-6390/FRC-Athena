package ca.frc6390.athena.filters;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class FilterList {
    private ArrayList<Function<Double, Double>> filtersFuncs;
    private ArrayList<Object> filters;

    public FilterList() {
        filtersFuncs = new ArrayList<>();
        filters = new ArrayList<>();
    }

    public FilterList addMedianFilter(int window) {
        MedianFilter filter = new MedianFilter(window);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addLinearFilter(double[] ffgains, double[] fbgains) {
        LinearFilter filter = new LinearFilter(ffgains,fbgains);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addHighPass(double timeConstant, double period) {
        LinearFilter filter = LinearFilter.highPass(timeConstant, period);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addFiniteDifference(int derivative, int[] stencil, double period) {
        LinearFilter filter = LinearFilter.finiteDifference(derivative, stencil, period);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addMovingAverage(int taps) {
        LinearFilter filter = LinearFilter.movingAverage(taps);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addBackwardsFiniteDifference(int derivative, int samples, double period) {
        LinearFilter filter = LinearFilter.backwardFiniteDifference(derivative, samples, period);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addSinglePoleIIR(double timeConstant, double period) {
        LinearFilter filter = LinearFilter.singlePoleIIR(timeConstant, period);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addSlewRateLimtier(double ratelimiter) {
        SlewRateLimiter filter = new SlewRateLimiter(ratelimiter);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit) {
        return addSlewRateLimtier(positiveRateLimit, negativeRateLimit, 0);
    }

    public FilterList addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        SlewRateLimiter filter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, initialValue);
        return addFilter(filter, filter::calculate);
    }

    public FilterList addFilter(Object filter, Function<Double, Double> func) {
        filters.add(filter);
        filtersFuncs.add(func);
        return this;
    }

    public ArrayList<Function<Double, Double>> getFilterFunctions(){
        return filtersFuncs;
    }

    public double calculate(double value) {
        for (Function<Double, Double> filter : filtersFuncs) {
            value = filter.apply(value);
        }
        return value;
    }

}
