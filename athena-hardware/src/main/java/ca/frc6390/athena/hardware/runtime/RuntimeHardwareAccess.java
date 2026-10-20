package ca.frc6390.athena.hardware.runtime;

import java.util.Objects;
import java.util.function.Supplier;

/** Scoped hardware access used while Athena evaluates runtime callbacks. */
public final class RuntimeHardwareAccess {
    private static final ThreadLocal<ActionContext> CURRENT = new ThreadLocal<>();

    private RuntimeHardwareAccess() {
    }

    public static <T> T call(ActionContext context, Supplier<T> action) {
        Objects.requireNonNull(context, "context");
        Objects.requireNonNull(action, "action");
        ActionContext previous = CURRENT.get();
        if (previous == context) {
            return action.get();
        }
        CURRENT.set(context);
        try {
            return action.get();
        } finally {
            // set(null) rather than remove(): remove() drops the thread-local entry, so every cycle's next call
            // would allocate it again. current() reads null either way.
            CURRENT.set(previous);
        }
    }

    public static void run(ActionContext context, Runnable action) {
        call(context, () -> {
            action.run();
            return null;
        });
    }

    public static ActionContext current() {
        return CURRENT.get();
    }
}
