package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.input.TypedInputResolver.MutableInputs;
import ca.frc6390.athena.core.input.TypedInputResolver.ValueMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashMap;
import java.util.Map;

/**
 * Example typed-input resolver setups for strict, lenient, and mutable modes.
 */
public final class TypedInputResolverExamples {
    private TypedInputResolverExamples() {}

    public record ControlProfile(String name, double maxOutput) {}

    public static TypedInputResolver strictResolver() {
        return new TypedInputResolver(
                "ExampleStrictInputs",
                ValueMode.STRICT,
                TypedInputResolver.NO_MUTABLES,
                Map.of("enabled", () -> true),
                Map.of("gain", () -> 0.75),
                Map.of("slot", () -> 2),
                Map.of("mode", () -> "teleop"),
                Map.of("target2d", Pose2d::new),
                Map.of("target3d", Pose3d::new),
                Map.of("profile", () -> new ControlProfile("safe", 0.5)));
    }

    public static TypedInputResolver lenientResolver() {
        return new TypedInputResolver(
                "ExampleLenientInputs",
                ValueMode.LENIENT,
                TypedInputResolver.NO_MUTABLES,
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of());
    }

    public static TypedInputResolver resolverWithMutableOverrides(MutableInputs mutables) {
        return new TypedInputResolver(
                "ExampleMutableInputs",
                ValueMode.STRICT,
                mutables,
                Map.of("enabled", () -> true),
                Map.of("gain", () -> 0.25),
                Map.of("slot", () -> 1),
                Map.of("mode", () -> "default"),
                Map.of(),
                Map.of(),
                Map.of());
    }

    public static final class ExampleMutableInputs implements MutableInputs {
        private final Map<String, Boolean> bools = new HashMap<>();
        private final Map<String, Double> doubles = new HashMap<>();
        private final Map<String, Integer> ints = new HashMap<>();
        private final Map<String, String> strings = new HashMap<>();

        public ExampleMutableInputs bool(String key, boolean value) {
            bools.put(key, value);
            return this;
        }

        public ExampleMutableInputs dbl(String key, double value) {
            doubles.put(key, value);
            return this;
        }

        public ExampleMutableInputs intVal(String key, int value) {
            ints.put(key, value);
            return this;
        }

        public ExampleMutableInputs str(String key, String value) {
            strings.put(key, value);
            return this;
        }

        @Override
        public boolean hasBool(String key) {
            return bools.containsKey(key);
        }

        @Override
        public boolean bool(String key) {
            return bools.getOrDefault(key, false);
        }

        @Override
        public boolean hasDouble(String key) {
            return doubles.containsKey(key);
        }

        @Override
        public double dbl(String key) {
            return doubles.getOrDefault(key, Double.NaN);
        }

        @Override
        public boolean hasInt(String key) {
            return ints.containsKey(key);
        }

        @Override
        public int intVal(String key) {
            return ints.getOrDefault(key, 0);
        }

        @Override
        public boolean hasString(String key) {
            return strings.containsKey(key);
        }

        @Override
        public String str(String key) {
            return strings.getOrDefault(key, "");
        }
    }
}
