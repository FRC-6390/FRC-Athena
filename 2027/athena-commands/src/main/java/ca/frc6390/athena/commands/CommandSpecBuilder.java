package ca.frc6390.athena.commands;

import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Fluent builder for {@link CommandSpec}.
 */
public final class CommandSpecBuilder {
    private final String name;
    private AthenaAction onInitialize;
    private AthenaAction onExecute;
    private AthenaCondition isFinished;
    private AthenaAction onEnd;
    private final Set<String> requirements = new LinkedHashSet<>();

    CommandSpecBuilder(String name) {
        this.name = name;
    }

    /**
     * Sets initialize action.
     *
     * @param action action
     * @return this builder
     */
    public CommandSpecBuilder onInitialize(AthenaAction action) {
        onInitialize = action;
        return this;
    }

    /**
     * Sets execute action.
     *
     * @param action action
     * @return this builder
     */
    public CommandSpecBuilder onExecute(AthenaAction action) {
        onExecute = action;
        return this;
    }

    /**
     * Sets finish condition.
     *
     * @param condition condition
     * @return this builder
     */
    public CommandSpecBuilder until(AthenaCondition condition) {
        isFinished = condition;
        return this;
    }

    /**
     * Sets end action.
     *
     * @param action action
     * @return this builder
     */
    public CommandSpecBuilder onEnd(AthenaAction action) {
        onEnd = action;
        return this;
    }

    /**
     * Adds named subsystem or resource requirements.
     *
     * @param requirementNames requirement names
     * @return this builder
     */
    public CommandSpecBuilder requires(String... requirementNames) {
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
    public CommandSpecBuilder requires(Collection<String> requirementNames) {
        Objects.requireNonNull(requirementNames, "requirementNames");
        requirementNames.forEach(this::addRequirement);
        return this;
    }

    /**
     * Builds the immutable spec.
     *
     * @return command spec
     */
    public CommandSpec toSpec() {
        return new CommandSpec(name, onInitialize, onExecute, isFinished, onEnd, requirements);
    }

    private void addRequirement(String requirement) {
        if (requirement != null && !requirement.isBlank()) {
            requirements.add(requirement);
        }
    }
}
