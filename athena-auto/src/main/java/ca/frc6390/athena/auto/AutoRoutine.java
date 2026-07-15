package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import java.util.Objects;
import java.util.function.Supplier;

/** A named factory for a fresh ordinary Athena Action. */
public record AutoRoutine(String name, Supplier<Action> actionFactory) {
    public AutoRoutine {
        name = name == null || name.isBlank() ? "auto" : name.trim();
        Objects.requireNonNull(actionFactory, "actionFactory");
    }

    public AutoRoutine(String name, Action action) {
        this(name, () -> Objects.requireNonNull(action, "action"));
    }

    public Action action() {
        return Objects.requireNonNull(actionFactory.get(), "actionFactory returned null");
    }
}
