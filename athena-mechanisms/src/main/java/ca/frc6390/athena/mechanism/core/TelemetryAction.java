package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/** A named, mechanism-owned action exposed to telemetry publishers. */
public final class TelemetryAction {
    private final String name;
    private final Action action;
    private final Runnable request;
    private final Runnable cancel;
    private final BooleanSupplier running;
    private final BooleanSupplier complete;

    TelemetryAction(String name, Action action, Runnable request, Runnable cancel,
            BooleanSupplier running, BooleanSupplier complete) {
        this.name = Objects.requireNonNull(name, "name");
        this.action = Objects.requireNonNull(action, "action");
        this.request = Objects.requireNonNull(request, "request");
        this.cancel = Objects.requireNonNull(cancel, "cancel");
        this.running = Objects.requireNonNull(running, "running");
        this.complete = Objects.requireNonNull(complete, "complete");
    }

    public String name() { return name; }
    public Action action() { return action; }
    public String type() { return action.getClass().getSimpleName(); }
    public void request() { request.run(); }
    public void cancel() { cancel.run(); }
    public boolean running() { return running.getAsBoolean(); }
    public boolean complete() { return complete.getAsBoolean(); }
}
