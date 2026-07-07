package ca.frc6390.athena.localization.ref;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.hardware.ref.ActionRef;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementRef;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;

/**
 * Composable localization result node.
 */
public final class LocalizationRef implements MeasurementRef {
    private final String estimatorName;
    private final LocalizationEstimatorRef estimator;
    private final List<MeasurementRef> inputs;
    private final List<LocalizationFilterRef> filters;
    private final String debugName;
    private final boolean publishNetworkTables;
    private final State state;

    public LocalizationRef(String estimatorName, LocalizationEstimatorRef estimator) {
        this(estimatorName, estimator, List.of(), List.of(), "", false, new State());
    }

    private LocalizationRef(
            String estimatorName,
            LocalizationEstimatorRef estimator,
            List<MeasurementRef> inputs,
            List<LocalizationFilterRef> filters,
            String debugName,
            boolean publishNetworkTables,
            State state) {
        this.estimatorName = estimatorName == null || estimatorName.isBlank() ? "custom" : estimatorName;
        this.estimator = Objects.requireNonNull(estimator, "estimator");
        this.inputs = List.copyOf(inputs == null ? List.of() : inputs);
        this.filters = List.copyOf(filters == null ? List.of() : filters);
        this.debugName = debugName == null ? "" : debugName;
        this.publishNetworkTables = publishNetworkTables;
        this.state = Objects.requireNonNull(state, "state");
    }

    /**
     * Adds an input.
     *
     * @param input input
     * @return updated ref
     */
    public LocalizationRef input(MeasurementRef input) {
        Objects.requireNonNull(input, "input");
        List<MeasurementRef> next = new ArrayList<>(inputs);
        next.add(input);
        return withInputs(next);
    }

    /**
     * Adds inputs.
     *
     * @param inputs inputs
     * @return updated ref
     */
    public LocalizationRef input(MeasurementRef... inputs) {
        LocalizationRef ref = this;
        if (inputs != null) {
            for (MeasurementRef input : inputs) {
                ref = ref.input(input);
            }
        }
        return ref;
    }

    /**
     * Adds a filter.
     *
     * @param filter filter
     * @return updated ref
     */
    public LocalizationRef filter(LocalizationFilterRef filter) {
        Objects.requireNonNull(filter, "filter");
        List<LocalizationFilterRef> next = new ArrayList<>(filters);
        next.add(filter);
        return withFilters(next);
    }

    /**
     * Adds filters.
     *
     * @param filters filters
     * @return updated ref
     */
    public LocalizationRef filter(LocalizationFilterRef... filters) {
        LocalizationRef ref = this;
        if (filters != null) {
            for (LocalizationFilterRef filter : filters) {
                ref = ref.filter(filter);
            }
        }
        return ref;
    }

    /**
     * Sets an optional debug name.
     *
     * @param name name
     * @return updated ref
     */
    public LocalizationRef name(String name) {
        return new LocalizationRef(estimatorName, estimator, inputs, filters, name, publishNetworkTables, state);
    }

    /**
     * Enables NetworkTables publishing metadata.
     *
     * @return updated ref
     */
    public LocalizationRef publishNetworkTables() {
        return publishNetworkTables(true);
    }

    /**
     * Sets NetworkTables publishing metadata.
     *
     * @param enabled true to publish
     * @return updated ref
     */
    public LocalizationRef publishNetworkTables(boolean enabled) {
        return new LocalizationRef(estimatorName, estimator, inputs, filters, debugName, enabled, state);
    }

    /**
     * Returns the current pose estimate.
     *
     * @return pose
     */
    public PoseSnapshot pose() {
        return latest().map(LocalizationResult::pose).orElseGet(() -> new PoseSnapshot(0.0, 0.0, 0.0));
    }

    /**
     * Returns current robot velocity.
     *
     * @return speeds
     */
    public RobotVelocity speeds() {
        return latest().map(LocalizationResult::speeds).orElseGet(RobotVelocity::zero);
    }

    /**
     * Returns the latest localization result.
     *
     * @return localization result
     */
    public Optional<LocalizationResult> latest() {
        List<LocalizationResult> acceptedResults = new ArrayList<>();
        List<Measurement> acceptedMeasurements = new ArrayList<>();
        List<Measurement> rejectedMeasurements = new ArrayList<>();

        for (MeasurementRef input : inputs) {
            if (input instanceof LocalizationRef localization) {
                localization.latest().ifPresent(result -> {
                    if (accept(input, result)) {
                        acceptedResults.add(result);
                    }
                });
                continue;
            }
            for (Measurement measurement : input.measurements()) {
                if (accept(input, measurement)) {
                    acceptedMeasurements.add(measurement);
                } else {
                    rejectedMeasurements.add(measurement);
                }
            }
        }

        LocalizationEstimateContext context = new LocalizationEstimateContext(
                this,
                inputs,
                acceptedResults,
                acceptedMeasurements,
                rejectedMeasurements,
                Optional.ofNullable(state.latest));
        Optional<LocalizationResult> estimated = estimator.estimate(context)
                .filter(result -> accept(null, result))
                .map(result -> result.withRejected(rejectedMeasurements));
        estimated.ifPresent(result -> state.latest = result);
        return estimated.or(() -> Optional.ofNullable(state.latest));
    }

    @Override
    public List<Measurement> measurements() {
        return latest()
                .map(result -> List.<Measurement>of(new PoseMeasurement(
                        result.pose(),
                        result.speeds(),
                        result.timestampSeconds(),
                        0.0,
                        0.0,
                        result.acceptedMeasurements().size(),
                        null,
                        this)))
                .orElseGet(List::of);
    }

    /**
     * Creates an action that resets this localization estimate.
     *
     * @param pose pose
     * @return action
     */
    public ActionRef reset(PoseSnapshot pose) {
        Objects.requireNonNull(pose, "pose");
        return context -> state.latest = LocalizationResult.of(pose);
    }

    /**
     * Creates an action that resets this localization estimate.
     *
     * @param xMeters x position
     * @param yMeters y position
     * @param headingRadians heading
     * @return action
     */
    public ActionRef reset(double xMeters, double yMeters, double headingRadians) {
        return reset(new PoseSnapshot(xMeters, yMeters, headingRadians));
    }

    /**
     * Creates an action that resets this estimate to another estimate.
     *
     * @param other other localization estimate
     * @return action
     */
    public ActionRef resetTo(LocalizationRef other) {
        Objects.requireNonNull(other, "other");
        return context -> state.latest = LocalizationResult.of(other.pose());
    }

    /**
     * Returns estimator name.
     *
     * @return estimator name
     */
    public String estimatorName() {
        return estimatorName;
    }

    /**
     * Returns inputs.
     *
     * @return inputs
     */
    public List<MeasurementRef> inputs() {
        return inputs;
    }

    /**
     * Returns filters.
     *
     * @return filters
     */
    public List<LocalizationFilterRef> filters() {
        return filters;
    }

    /**
     * Returns optional debug name.
     *
     * @return debug name
     */
    public String debugName() {
        return debugName;
    }

    /**
     * Returns whether NetworkTables publishing was requested.
     *
     * @return true when requested
     */
    public boolean publishNetworkTablesEnabled() {
        return publishNetworkTables;
    }

    private LocalizationRef withInputs(List<MeasurementRef> inputs) {
        return new LocalizationRef(estimatorName, estimator, inputs, filters, debugName, publishNetworkTables, state);
    }

    private LocalizationRef withFilters(List<LocalizationFilterRef> filters) {
        return new LocalizationRef(estimatorName, estimator, inputs, filters, debugName, publishNetworkTables, state);
    }

    private boolean accept(MeasurementRef input, Measurement measurement) {
        LocalizationFilterContext context = new LocalizationFilterContext(
                this, input, Optional.of(measurement), Optional.empty());
        return filters.stream().allMatch(filter -> filter.accept(context));
    }

    private boolean accept(MeasurementRef input, LocalizationResult result) {
        LocalizationFilterContext context = new LocalizationFilterContext(
                this, input, Optional.empty(), Optional.of(result));
        return filters.stream().allMatch(filter -> filter.accept(context));
    }

    private static final class State {
        private LocalizationResult latest;
    }
}
