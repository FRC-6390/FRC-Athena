package ca.frc6390.athena.mechanism.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Active Action request bridge for robot code.
 */
public final class ActionRequests {
    private static volatile ActionRequester requester;
    private static final ThreadLocal<List<Action>> CAPTURED = new ThreadLocal<>();

    private ActionRequests() {
    }

    public static void bind(ActionRequester requester) {
        ActionRequests.requester = Objects.requireNonNull(requester, "requester");
    }

    public static void clear() {
        requester = null;
    }

    public static void request(Action action) {
        Action safeAction = Objects.requireNonNull(action, "action");
        List<Action> captured = CAPTURED.get();
        if (captured != null) {
            captured.add(safeAction);
            return;
        }
        ActionRequester active = requester;
        if (active == null) {
            throw new IllegalStateException("No Athena Action requester is active.");
        }
        active.request(safeAction);
    }

    static List<Action> capture(Runnable callback) {
        Objects.requireNonNull(callback, "callback");
        List<Action> previous = CAPTURED.get();
        List<Action> captured = new ArrayList<>();
        CAPTURED.set(captured);
        try {
            callback.run();
            return List.copyOf(captured);
        } finally {
            if (previous == null) {
                CAPTURED.remove();
            } else {
                CAPTURED.set(previous);
            }
        }
    }
}
