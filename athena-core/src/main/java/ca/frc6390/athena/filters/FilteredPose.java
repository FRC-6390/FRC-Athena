package ca.frc6390.athena.filters;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FilteredPose {

    private final Supplier<Pose2d> poseSupplier; 
    private final FilterList xFilter, yFilter, thetaFilter;
    private Pose2d preValue;

    public FilteredPose(Supplier<Pose2d> poseSupplier) {
        super();
        this.poseSupplier = poseSupplier;
        this.xFilter = new FilterList();
        this.yFilter = new FilterList();
        this.thetaFilter = new FilterList();
    }

    public Pose2d get() {
        return get(false);
    }

    public Pose2d get(boolean runFilter) {
       return runFilter ? getFiltered() : preValue;
    }

    public Pose2d getFiltered(){
        Pose2d value = getUnfiltered();
        double x = value.getX();
        double y = value.getY();
        double theta = value.getRotation().getRadians();

        for (Function<Double, Double> filter : xFilter.getFilterFunctions()) {
            x = filter.apply(x);
        }

        for (Function<Double, Double> filter : yFilter.getFilterFunctions()) {
            y = filter.apply(x);
        }

        for (Function<Double, Double> filter : thetaFilter.getFilterFunctions()) {
            theta = filter.apply(x);
        }

        value = new Pose2d(x, y, Rotation2d.fromRadians(theta));
        preValue = value;
        return value;
    }

    public Pose2d getUnfiltered() {
        return poseSupplier.get();
    }

    public FilteredPose addMedianFilter(int window) {
        xFilter.addMedianFilter(window);
        yFilter.addMedianFilter(window);
        thetaFilter.addMedianFilter(window);
        return this;
    }

    public FilteredPose addLinearFilter(double[] ffgains, double[] fbgains) {
        xFilter.addLinearFilter(ffgains, fbgains);
        yFilter.addLinearFilter(ffgains, fbgains);
        thetaFilter.addLinearFilter(ffgains, fbgains);
        return this;
    }

    public FilteredPose addHighPass(double timeConstant, double period) {
        xFilter.addHighPass(timeConstant, period);
        yFilter.addHighPass(timeConstant, period);
        thetaFilter.addHighPass(timeConstant, period);
        return this;
    }

    public FilteredPose addFiniteDifference(int derivative, int[] stencil, double period) {
        xFilter.addFiniteDifference(derivative, stencil, period);
        yFilter.addFiniteDifference(derivative, stencil, period);
        thetaFilter.addFiniteDifference(derivative, stencil, period);
        return this;
    }

    public FilteredPose addMovingAverage(int taps) {
        xFilter.addMovingAverage(taps);
        yFilter.addMovingAverage(taps);
        thetaFilter.addMovingAverage(taps);
        return this;
    }

    public FilteredPose addBackwardsFiniteDifference(int derivative, int samples, double period) {
        xFilter.addBackwardsFiniteDifference(derivative, samples, period);
        yFilter.addBackwardsFiniteDifference(derivative, samples, period);
        thetaFilter.addBackwardsFiniteDifference(derivative, samples, period);
        return this;
    }

    public FilteredPose addSinglePoleIIR(double timeConstant, double period) {
        xFilter.addSinglePoleIIR(timeConstant, period);
        yFilter.addSinglePoleIIR(timeConstant, period);
        thetaFilter.addSinglePoleIIR(timeConstant, period);
        return this;
    }

    public FilteredPose addSlewRateLimtier(double rateLimiter) {
        xFilter.addSlewRateLimtier(rateLimiter);
        yFilter.addSlewRateLimtier(rateLimiter);
        thetaFilter.addSlewRateLimtier(rateLimiter);
        return this;
    }

    public FilteredPose addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit) {
        xFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit);
        yFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit);
        thetaFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit);
        return this;
    }

    public FilteredPose addSlewRateLimtier(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        xFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit, initialValue);
        yFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit, initialValue);
        thetaFilter.addSlewRateLimtier(positiveRateLimit, negativeRateLimit, initialValue);
        return this;
    }

    public FilteredPose addFilter(Object filter, Function<Double, Double> func) {
        xFilter.addFilter(filter, func);
        yFilter.addFilter(filter, func);
        thetaFilter.addFilter(filter, func);
        return this;
    }
}
