package ca.frc6390.athena.runtime.filter;

import java.util.function.Supplier;

/**
 * Axis-wise filter pipeline for simple pose snapshots.
 */
public final class FilteredPose {
    private final Supplier<PoseSnapshot> source;
    private final FilteredValue x;
    private final FilteredValue y;
    private final FilteredValue heading;
    private PoseSnapshot last = new PoseSnapshot(0.0, 0.0, 0.0);

    /**
     * Creates a filtered pose pipeline.
     *
     * @param source pose source
     */
    public FilteredPose(Supplier<PoseSnapshot> source) {
        this.source = source == null ? () -> last : source;
        x = new FilteredValue(() -> this.source.get().xMeters());
        y = new FilteredValue(() -> this.source.get().yMeters());
        heading = new FilteredValue(() -> this.source.get().headingRadians());
    }

    /**
     * Adds a moving average to every pose axis.
     *
     * @param taps sample count
     * @return this filtered pose
     */
    public FilteredPose addMovingAverage(int taps) {
        x.addMovingAverage(taps);
        y.addMovingAverage(taps);
        heading.addMovingAverage(taps);
        return this;
    }

    /**
     * Returns cached pose unless runFilter is true.
     *
     * @param runFilter true to sample and filter
     * @return pose snapshot
     */
    public PoseSnapshot get(boolean runFilter) {
        return runFilter ? getFiltered() : last;
    }

    /**
     * Samples and filters the pose.
     *
     * @return filtered pose
     */
    public PoseSnapshot getFiltered() {
        last = new PoseSnapshot(x.getFiltered(), y.getFiltered(), heading.getFiltered());
        return last;
    }
}
