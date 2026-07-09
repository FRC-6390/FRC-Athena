package ca.frc6390.athena.commands;

import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Fluent builder for command behavior states.
 */
public final class CommandStateBuilder {
    private final String name;
    private Runnable onInitialize;
    private Runnable onExecute;
    private BooleanSupplier isFinished;
    private Runnable onEnd;
    private final Set<String> requirements = new LinkedHashSet<>();

    CommandStateBuilder(String name) {
        this.name = name;
    }

    /**
     * Sets the action run when the command state starts.
     *
     * @param action action
     * @return this builder
     */
    public CommandStateBuilder onInitialize(Runnable action) {
        onInitialize = action;
        return this;
    }

    /**
     * Sets the action run on each command state cycle.
     *
     * @param action action
     * @return this builder
     */
    public CommandStateBuilder onExecute(Runnable action) {
        onExecute = action;
        return this;
    }

    /**
     * Sets the finish condition.
     *
     * @param condition finish condition
     * @return this builder
     */
    public CommandStateBuilder until(BooleanSupplier condition) {
        isFinished = condition;
        return this;
    }

    /**
     * Sets the action run when the command state ends.
     *
     * @param action action
     * @return this builder
     */
    public CommandStateBuilder onEnd(Runnable action) {
        onEnd = action;
        return this;
    }

    /**
     * Adds named subsystem or resource requirements.
     *
     * @param requirementNames requirement names
     * @return this builder
     */
    public CommandStateBuilder requires(String... requirementNames) {
        if (requirementNames == null) {
            return this;
        }
        for (String requirement : requirementNames) {
            addRequirement(requirement);
        }
        return this;
    }

    /**
     * Adds named subsystem or resource requirements.
     *
     * @param requirementNames requirement names
     * @return this builder
     */
    public CommandStateBuilder requires(Collection<String> requirementNames) {
        Objects.requireNonNull(requirementNames, "requirementNames");
        requirementNames.forEach(this::addRequirement);
        return this;
    }

    /**
     * Builds the command state.
     *
     * @return command state
     */
    public CommandState build() {
        return new CommandState(name, onInitialize, onExecute, isFinished, onEnd, requirements);
    }

    private void addRequirement(String requirement) {
        if (requirement != null && !requirement.isBlank()) {
            requirements.add(requirement.trim());
        }
    }
}
