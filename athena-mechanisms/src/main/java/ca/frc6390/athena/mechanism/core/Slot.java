package ca.frc6390.athena.mechanism.core;

import java.util.Objects;

/**
 * Fillable mechanism-template slot.
 *
 * @param <O> owner type
 * @param <T> value type
 */
public class Slot<O, T> {
    private final O owner;
    private final String name;
    private final Runnable onFill;
    private T value;

    Slot(O owner, String name, Runnable onFill) {
        this.owner = Objects.requireNonNull(owner, "owner");
        this.name = name == null || name.isBlank() ? "slot" : name;
        this.onFill = onFill == null ? () -> { } : onFill;
    }

    public O fill(T value) {
        this.value = Objects.requireNonNull(value, name);
        onFill.run();
        return owner;
    }

    public boolean filled() {
        return value != null;
    }

    public T get() {
        if (value == null) {
            throw new IllegalStateException("Required slot '" + name + "' has not been filled.");
        }
        return value;
    }

    public String name() {
        return name;
    }
}
