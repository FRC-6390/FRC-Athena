package ca.frc6390.athena.commands;

import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Fluent builder for command behavior Actions.
 */
public final class CommandActionBuilder {
    private final String name;
    private Runnable onInitialize;
    private Runnable onExecute;
    private BooleanSupplier isFinished;
    private Runnable onEnd;
    private final Set<String> requirements = new LinkedHashSet<>();

    CommandActionBuilder(String name) {
        this.name = name;
    }

    /**
     * Sets the action run when the command Action starts.
     *
     * @param action action
     * @return this builder
     */
    public CommandActionBuilder onInitialize(Runnable action) {
        onInitialize = action;
        return this;
    }

    /**
     * Sets the action run on each command Action cycle.
     *
     * @param action action
     * @return this builder
     */
    public CommandActionBuilder onExecute(Runnable action) {
        onExecute = action;
        return this;
    }

    /**
     * Sets the finish condition.
     *
     * @param condition finish condition
     * @return this builder
     */
    public CommandActionBuilder until(BooleanSupplier condition) {
        isFinished = condition;
        return this;
    }

    /**
     * Sets the action run when the command Action ends.
     *
     * @param action action
     * @return this builder
     */
    public CommandActionBuilder onEnd(Runnable action) {
        onEnd = action;
        return this;
    }

    /**
     * Adds named subsystem or resource requirements.
     *
     * @param requirementNames requirement names
     * @return this builder
     */
    public CommandActionBuilder requires(String... requirementNames) {
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
    public CommandActionBuilder requires(Collection<String> requirementNames) {
        Objects.requireNonNull(requirementNames, "requirementNames");
        requirementNames.forEach(this::addRequirement);
        return this;
    }

    /**
     * Builds the command Action.
     *
     * @return command Action
     */
    public CommandAction build() {
        return new CommandAction(name, onInitialize, onExecute, isFinished, onEnd, requirements);
    }

    private void addRequirement(String requirement) {
        if (requirement != null && !requirement.isBlank()) {
            requirements.add(requirement.trim());
        }
    }
}
