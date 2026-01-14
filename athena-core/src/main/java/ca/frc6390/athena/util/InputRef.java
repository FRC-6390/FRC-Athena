package ca.frc6390.athena.util;

import java.util.function.Supplier;

/**
 * Mutable input reference that can be populated after configs are defined.
 */
public final class InputRef<T> implements Supplier<T> {
    private volatile T value;

    private InputRef() {
    }

    public static <T> InputRef<T> create() {
        return new InputRef<>();
    }

    public void set(T value) {
        this.value = value;
    }

    public void clear() {
        this.value = null;
    }

    public boolean isSet() {
        return value != null;
    }

    @Override
    public T get() {
        return value;
    }
}
