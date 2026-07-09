package ca.frc6390.athena.localization.ref;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.Measurements;

/**
 * Runtime-owned localization pipeline declaration.
 */
public final class LocalizationPipeline implements MeasurementSignal {
    private final String estimatorName;
    private final LocalizationEstimator estimator;
    private final List<MeasurementSignal> inputs;
    private final List<LocalizationFilter> filters;
    private final String debugName;
    private final boolean publishNetworkTables;
    private final Action Action;

    LocalizationPipeline(String estimatorName, LocalizationEstimator estimator) {
        this(estimatorName, estimator, List.of(), List.of(), "", false, new Action());
    }

    private LocalizationPipeline(
            String estimatorName,
            LocalizationEstimator estimator,
            List<MeasurementSignal> inputs,
            List<LocalizationFilter> filters,
            String debugName,
            boolean publishNetworkTables,
            Action Action) {
        this.estimatorName = estimatorName == null || estimatorName.isBlank() ? "custom" : estimatorName;
        this.estimator = Objects.requireNonNull(estimator, "estimator");
        this.inputs = List.copyOf(inputs == null ? List.of() : inputs);
        this.filters = List.copyOf(filters == null ? List.of() : filters);
        this.debugName = debugName == null ? "" : debugName;
        this.publishNetworkTables = publishNetworkTables;
        this.Action = Objects.requireNonNull(Action, "Action");
    }

    public LocalizationPipeline input(MeasurementSignal input) {
        Objects.requireNonNull(input, "input");
        List<MeasurementSignal> next = new ArrayList<>(inputs);
        next.add(input);
        return withInputs(next);
    }

    public LocalizationPipeline input(MeasurementSignal... inputs) {
        LocalizationPipeline pipeline = this;
        if (inputs != null) {
            for (MeasurementSignal input : inputs) {
                pipeline = pipeline.input(input);
            }
        }
        return pipeline;
    }

    public LocalizationPipeline filter(LocalizationFilter filter) {
        Objects.requireNonNull(filter, "filter");
        List<LocalizationFilter> next = new ArrayList<>(filters);
        next.add(filter);
        return withFilters(next);
    }

    public LocalizationPipeline filter(LocalizationFilter... filters) {
        LocalizationPipeline pipeline = this;
        if (filters != null) {
            for (LocalizationFilter filter : filters) {
                pipeline = pipeline.filter(filter);
            }
        }
        return pipeline;
    }

    public LocalizationPipeline name(String name) {
        return new LocalizationPipeline(estimatorName, estimator, inputs, filters, name, publishNetworkTables, Action);
    }

    public LocalizationPipeline publishNetworkTables() {
        return publishNetworkTables(true);
    }

    public LocalizationPipeline publishNetworkTables(boolean enabled) {
        return new LocalizationPipeline(estimatorName, estimator, inputs, filters, debugName, enabled, Action);
    }

    public PoseSnapshot pose() {
        return latest().map(LocalizationResult::pose).orElseGet(() -> new PoseSnapshot(0.0, 0.0, 0.0));
    }

    public RobotVelocity speeds() {
        return latest().map(LocalizationResult::speeds).orElseGet(RobotVelocity::zero);
    }

    Optional<LocalizationResult> latest() {
        List<LocalizationResult> acceptedResults = new ArrayList<>();
        List<Measurement> acceptedMeasurements = new ArrayList<>();
        List<Measurement> rejectedMeasurements = new ArrayList<>();

        for (MeasurementSignal input : inputs) {
            if (input instanceof LocalizationPipeline pipeline) {
                pipeline.latest().ifPresent(result -> {
                    if (accept(null, result.pose())) {
                        acceptedResults.add(result);
                    }
                });
                continue;
            }
            for (Measurement measurement : input.measurements()) {
                if (accept(measurement, PoseSamples.from(measurement).map(PoseSample::pose).orElse(null))) {
                    acceptedMeasurements.add(measurement);
                } else {
                    rejectedMeasurements.add(measurement);
                }
            }
        }

        LocalizationEstimate estimate = new LocalizationEstimate(
                this,
                inputs,
                acceptedResults,
                acceptedMeasurements,
                rejectedMeasurements,
                Optional.ofNullable(Action.latest));
        Optional<LocalizationResult> estimated = estimator.estimate(estimate)
                .filter(result -> accept(null, result.pose()))
                .map(result -> result.withRejected(rejectedMeasurements));
        estimated.ifPresent(result -> Action.latest = result);
        return estimated.or(() -> Optional.ofNullable(Action.latest));
    }

    @Override
    public List<Measurement> measurements() {
        return latest()
                .map(result -> List.<Measurement>of(Measurements.poseAndSpeeds(result.pose(), result.speeds())))
                .orElseGet(List::of);
    }

    public ActionBinding reset(PoseSnapshot pose) {
        Objects.requireNonNull(pose, "pose");
        return context -> Action.latest = LocalizationResult.of(pose);
    }

    public ActionBinding reset(double xMeters, double yMeters, double headingRadians) {
        return reset(new PoseSnapshot(xMeters, yMeters, headingRadians));
    }

    public ActionBinding resetTo(LocalizationPipeline other) {
        Objects.requireNonNull(other, "other");
        return context -> Action.latest = LocalizationResult.of(other.pose());
    }

    public String estimatorName() {
        return estimatorName;
    }

    public List<MeasurementSignal> inputs() {
        return inputs;
    }

    public List<LocalizationFilter> filters() {
        return filters;
    }

    public String debugName() {
        return debugName;
    }

    public boolean publishNetworkTablesEnabled() {
        return publishNetworkTables;
    }

    private LocalizationPipeline withInputs(List<MeasurementSignal> inputs) {
        return new LocalizationPipeline(estimatorName, estimator, inputs, filters, debugName, publishNetworkTables, Action);
    }

    private LocalizationPipeline withFilters(List<LocalizationFilter> filters) {
        return new LocalizationPipeline(estimatorName, estimator, inputs, filters, debugName, publishNetworkTables, Action);
    }

    private boolean accept(Measurement measurement, PoseSnapshot pose) {
        return filters.stream().allMatch(filter -> filter.accept(this, measurement, pose));
    }

    private static final class Action {
        private LocalizationResult latest;
    }
}
