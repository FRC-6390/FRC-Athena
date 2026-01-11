package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.sensors.EnhancedDigitalInput;

/**
 * Declarative configuration for mechanism sensor simulation. Allows callers to specify how digital
 * sensors should respond to the underlying mechanism state when running in simulation.
 */
public final class MechanismSensorSimulationConfig {

    private final List<SensorDefinition> sensors;

    private MechanismSensorSimulationConfig(List<SensorDefinition> sensors) {
        this.sensors = sensors;
    }

    List<SensorDefinition> sensors() {
        return sensors;
    }

    public static Builder builder() {
        return new Builder();
    }

    @FunctionalInterface
    public interface SensorCondition {
        boolean test(EvaluationContext context);
    }

    /**
     * Bundles the mechanism and its current state (position/velocity) when evaluating sensor
     * conditions.
     */
    public record EvaluationContext(Mechanism mechanism, double position, double velocity) {}

    /**
     * Creates a condition that evaluates true when the mechanism position is at or above the supplied
     * threshold.
     */
    public static SensorCondition positionAtOrAbove(DoubleSupplier threshold) {
        Objects.requireNonNull(threshold);
        return ctx -> ctx.position() >= threshold.getAsDouble();
    }

    /**
     * Creates a condition that evaluates true when the mechanism position is at or below the supplied
     * threshold.
     */
    public static SensorCondition positionAtOrBelow(DoubleSupplier threshold) {
        Objects.requireNonNull(threshold);
        return ctx -> ctx.position() <= threshold.getAsDouble();
    }

    /**
     * Creates a condition that evaluates true when the mechanism position lies between the supplied
     * bounds (inclusive). The bounds are ordered internally, so callers may swap them freely.
     */
    public static SensorCondition positionBetween(DoubleSupplier lower, DoubleSupplier upper) {
        Objects.requireNonNull(lower);
        Objects.requireNonNull(upper);
        return ctx -> {
            double min = lower.getAsDouble();
            double max = upper.getAsDouble();
            double value = ctx.position();
            return value >= Math.min(min, max) && value <= Math.max(min, max);
        };
    }

    /**
     * Wraps an arbitrary {@link SensorCondition} for readability when constructing configs.
     */
    public static SensorCondition customSensorCondition(SensorCondition condition) {
        return Objects.requireNonNull(condition);
    }

    public static final class Builder {
        private final List<SensorDefinition> sensors = new ArrayList<>();

        private Builder() {}

        /**
         * Registers a digital sensor that toggles automatically when the provided condition evaluates
         * to true. The optional location defaults to {@code NaN}, which hides positional overlays.
         *
         * @param name display name for the sensor in dashboards/NT
         * @param accessor function that fetches the sensor instance from the mechanism
         * @param condition logic that determines when the sensor should read true
         */
        public Builder addDigitalSensor(String name,
                                        Function<Mechanism, ? extends EnhancedDigitalInput> accessor,
                                        SensorCondition condition) {
            return addDigitalSensor(name, accessor, condition, () -> Double.NaN);
        }

        /**
         * Registers a digital sensor with an explicit location supplier, allowing a dashboard overlay
         * to visualize its placement along the mechanism stroke.
         *
         * @param name display name for the sensor in dashboards/NT
         * @param accessor function that fetches the sensor instance from the mechanism
         * @param condition logic that determines when the sensor should read true
         * @param locationSupplier optional position (in mechanism units) for visualization
         */
        public Builder addDigitalSensor(String name,
                                        Function<Mechanism, ? extends EnhancedDigitalInput> accessor,
                                        SensorCondition condition,
                                        DoubleSupplier locationSupplier) {
            sensors.add(new SensorDefinition(
                    Objects.requireNonNull(name),
                    Objects.requireNonNull(accessor),
                    Objects.requireNonNull(condition),
                    locationSupplier != null ? locationSupplier : () -> Double.NaN));
            return this;
        }

        /**
         * Builds the immutable configuration containing all registered sensors.
         */
        public MechanismSensorSimulationConfig build() {
            return new MechanismSensorSimulationConfig(List.copyOf(sensors));
        }
    }

    static final class SensorDefinition {
        private final String name;
        private final Function<Mechanism, ? extends EnhancedDigitalInput> accessor;
        private final SensorCondition condition;
        private final DoubleSupplier locationSupplier;

        SensorDefinition(String name,
                         Function<Mechanism, ? extends EnhancedDigitalInput> accessor,
                         SensorCondition condition,
                         DoubleSupplier locationSupplier) {
            this.name = name;
            this.accessor = accessor;
            this.condition = condition;
            this.locationSupplier = locationSupplier;
        }

        String name() {
            return name;
        }

        Function<Mechanism, ? extends EnhancedDigitalInput> accessor() {
            return accessor;
        }

        SensorCondition condition() {
            return condition;
        }

        OptionalDouble location() {
            double value = locationSupplier != null ? locationSupplier.getAsDouble() : Double.NaN;
            return Double.isNaN(value) ? OptionalDouble.empty() : OptionalDouble.of(value);
        }
    }
}
