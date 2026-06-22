package ca.frc6390.athena.superstructure.runtime;

import ca.frc6390.athena.mechanism.runtime.MechanismController;
import ca.frc6390.athena.superstructure.spec.SuperstructureSpec;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Runtime controller that applies superstructure transition plans to mechanisms.
 */
public final class SuperstructureController {
    private final SuperstructurePlanner planner;
    private final Map<String, MechanismController> mechanisms;
    private String currentState;

    private SuperstructureController(
            SuperstructurePlanner planner,
            Map<String, MechanismController> mechanisms) {
        this.planner = Objects.requireNonNull(planner, "planner");
        this.mechanisms = Map.copyOf(mechanisms);
    }

    /**
     * Creates a controller for a superstructure spec.
     *
     * @param spec superstructure spec
     * @param mechanisms mechanism controllers keyed by planned leaf path
     * @return superstructure controller
     */
    public static SuperstructureController of(
            SuperstructureSpec spec,
            Map<String, MechanismController> mechanisms) {
        return new SuperstructureController(new SuperstructurePlanner(spec), mechanisms);
    }

    /**
     * Creates a builder for a superstructure controller.
     *
     * @param spec superstructure spec
     * @return builder
     */
    public static Builder builder(SuperstructureSpec spec) {
        return new Builder(spec);
    }

    /**
     * Returns the last applied state, or {@code null} before the first apply.
     *
     * @return current state
     */
    public String currentState() {
        return currentState;
    }

    /**
     * Plans and applies a target superstructure state.
     *
     * @param targetState target state
     * @return applied transition plan
     */
    public SuperstructureTransitionPlan applyState(String targetState) {
        SuperstructureTransitionPlan plan = planner.plan(currentState, targetState);
        for (SuperstructureMechanismTarget target : plan.targets()) {
            MechanismController mechanism = mechanisms.get(target.path());
            if (mechanism == null) {
                throw new IllegalArgumentException(
                        "No mechanism controller registered for superstructure path " + target.path() + ".");
            }
            mechanism.applyState(target.stateName());
        }
        currentState = targetState;
        return plan;
    }

    /**
     * Stops every registered mechanism controller.
     */
    public void stop() {
        for (MechanismController mechanism : mechanisms.values()) {
            mechanism.stop();
        }
    }

    /**
     * Builder for superstructure runtime controllers.
     */
    public static final class Builder {
        private final SuperstructureSpec spec;
        private final Map<String, MechanismController> mechanisms = new LinkedHashMap<>();

        private Builder(SuperstructureSpec spec) {
            this.spec = Objects.requireNonNull(spec, "spec");
        }

        /**
         * Registers a mechanism controller for a planned leaf path.
         *
         * @param path planned leaf path
         * @param mechanism mechanism controller
         * @return this builder
         */
        public Builder mechanism(String path, MechanismController mechanism) {
            if (path == null || path.isBlank()) {
                throw new IllegalArgumentException("Mechanism path is required.");
            }
            mechanisms.put(path, Objects.requireNonNull(mechanism, "mechanism"));
            return this;
        }

        /**
         * Builds the runtime controller.
         *
         * @return superstructure controller
         */
        public SuperstructureController build() {
            return SuperstructureController.of(spec, mechanisms);
        }
    }
}
