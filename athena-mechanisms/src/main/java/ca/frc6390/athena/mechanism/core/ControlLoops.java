package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.function.Function;

/**
 * Factories for custom control-loop declarations.
 */
public final class ControlLoops {
    private ControlLoops() {
    }

    /**
     * Creates a custom loop that must own final Athena-side output.
     *
     * @param factory runtime factory
     * @return control loop
     */
    public static ControlLoop software(Function<ControlLoopBinding, ControlLoopRuntime> factory) {
        return new Custom(ControlLoopRole.SOFTWARE_OUTPUT, factory);
    }

    /**
     * Creates a custom loop whose voltage output can be added to a device closed-loop request.
     *
     * @param factory runtime factory
     * @return control loop
     */
    public static ControlLoop arbitraryFeedforward(Function<ControlLoopBinding, ControlLoopRuntime> factory) {
        return new Custom(ControlLoopRole.ARBITRARY_FEEDFORWARD, factory);
    }

    private record Custom(ControlLoopRole role, Function<ControlLoopBinding, ControlLoopRuntime> factory)
            implements ControlLoop {
        private Custom {
            role = role == null ? ControlLoopRole.SOFTWARE_OUTPUT : role;
            Objects.requireNonNull(factory, "factory");
        }

        @Override
        public ControlLoopRuntime bind(ControlLoopBinding binding) {
            return Objects.requireNonNull(factory.apply(binding), "runtime");
        }
    }
}
