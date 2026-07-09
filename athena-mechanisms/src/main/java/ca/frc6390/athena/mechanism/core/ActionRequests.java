package ca.frc6390.athena.mechanism.core;

import java.util.Objects;

/**
 * Active Action request bridge for robot code.
 */
public final class ActionRequests {
    private static volatile ActionRequester requester;

    private ActionRequests() {
    }

    public static void bind(ActionRequester requester) {
        ActionRequests.requester = Objects.requireNonNull(requester, "requester");
    }

    public static void clear() {
        requester = null;
    }

    public static void request(Action action) {
        ActionRequester active = requester;
        if (active == null) {
            throw new IllegalStateException("No Athena Action requester is active.");
        }
        active.request(Objects.requireNonNull(action, "action"));
    }
}
