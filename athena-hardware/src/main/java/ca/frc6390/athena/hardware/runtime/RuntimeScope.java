package ca.frc6390.athena.hardware.runtime;

import java.util.Objects;
import java.util.function.Supplier;

/** Identifies one runtime when resolving directly readable device declarations. */
public final class RuntimeScope {
    private static final ThreadLocal<RuntimeScope> CURRENT = new ThreadLocal<>();

    private final String name;

    public RuntimeScope(String name) {
        this.name = name == null || name.isBlank() ? "runtime" : name;
    }

    public String name() {
        return name;
    }

    public void run(Runnable action) {
        Objects.requireNonNull(action, "action");
        call(() -> {
            action.run();
            return null;
        });
    }

    public <T> T call(Supplier<T> action) {
        Objects.requireNonNull(action, "action");
        RuntimeScope previous = CURRENT.get();
        CURRENT.set(this);
        try {
            return action.get();
        } finally {
            if (previous == null) {
                CURRENT.remove();
            } else {
                CURRENT.set(previous);
            }
        }
    }

    static RuntimeScope current() {
        return CURRENT.get();
    }
}
