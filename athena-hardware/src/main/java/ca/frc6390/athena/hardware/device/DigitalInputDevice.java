package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Reusable digital input declaration.
 */
public record DigitalInputDevice(String name, int channel, boolean isInverted, BooleanSupplier reader) {
    private static final ConcurrentMap<DigitalInputDevice, BooleanSupplier> RUNTIME_READERS = new ConcurrentHashMap<>();
    private static final ConcurrentMap<DigitalInputDevice, SignalState> SIGNALS = new ConcurrentHashMap<>();
    private static final ThreadLocal<RuntimeScope> CURRENT_SCOPE = new ThreadLocal<>();

    /**
     * Creates a roboRIO digital input declaration.
     *
     * @param channel DIO channel
     * @return digital input ref
     */
    public static DigitalInputDevice rio(int channel) {
        return new DigitalInputDevice("dio" + channel, channel, false, null);
    }

    public DigitalInputDevice {
        name = name == null || name.isBlank() ? "dio" + channel : name;
    }

    /**
     * Marks this input as inverted.
     *
     * @return updated ref
     */
    public DigitalInputDevice inverted() {
        return inverted(true);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true when active-low
     * @return updated ref
     */
    public DigitalInputDevice inverted(boolean inverted) {
        return new DigitalInputDevice(name, channel, inverted, reader);
    }

    /**
     * Binds a runtime reader to this declaration.
     *
     * @param reader reader
     * @return bound ref
     */
    public DigitalInputDevice bind(BooleanSupplier reader) {
        return new DigitalInputDevice(name, channel, isInverted, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Binds a runtime reader to an existing ref instance.
     *
     * @param ref input ref
     * @param reader runtime reader
     */
    public static void bindRuntime(DigitalInputDevice ref, BooleanSupplier reader) {
        RUNTIME_READERS.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Creates an isolated runtime scope for digital input readers and sampled signal state.
     *
     * @return runtime scope
     */
    public static RuntimeScope runtimeScope() {
        return new RuntimeScope();
    }

    /**
     * Runs work with a scoped digital input runtime.
     *
     * @param scope runtime scope
     * @param work work
     */
    public static void withRuntime(RuntimeScope scope, Runnable work) {
        Objects.requireNonNull(work, "work");
        withRuntime(scope, () -> {
            work.run();
            return null;
        });
    }

    /**
     * Runs work with a scoped digital input runtime.
     *
     * @param scope runtime scope
     * @param work work
     * @param <T> result type
     * @return work result
     */
    public static <T> T withRuntime(RuntimeScope scope, Supplier<T> work) {
        Objects.requireNonNull(work, "work");
        if (scope == null) {
            return work.get();
        }
        RuntimeScope previous = CURRENT_SCOPE.get();
        CURRENT_SCOPE.set(scope);
        try {
            return work.get();
        } finally {
            if (previous == null) {
                CURRENT_SCOPE.remove();
            } else {
                CURRENT_SCOPE.set(previous);
            }
        }
    }

    /**
     * Reads the raw digital value.
     *
     * @return raw value
     */
    public boolean raw() {
        if (reader == null) {
            BooleanSupplier runtimeReader = runtimeReader(this);
            if (runtimeReader == null) {
                throw new IllegalStateException("Digital input " + defaultName() + " is not runtime-bound.");
            }
            return runtimeReader.getAsBoolean();
        }
        return reader.getAsBoolean();
    }

    /**
     * Reads the active value after inversion.
     *
     * @return active value
     */
    public boolean active() {
        boolean value = raw();
        return isInverted ? !value : value;
    }

    /**
     * Samples this input into its runtime signal state.
     *
     * <p>Fast signal loops can call this more often than the main robot loop. Edge state is
     * latched until hooks consume it.</p>
     */
    public void sample() {
        signal(this).sample(active());
    }

    /**
     * Returns the sampled active value when available, otherwise reads the input directly.
     *
     * @return sampled or direct active value
     */
    public boolean sampledActive() {
        SignalState signal = signalIfPresent(this);
        return signal == null || !signal.hasSample() ? active() : signal.active();
    }

    /**
     * Returns whether a rising edge has been latched since the last consume.
     *
     * @return true when a rising edge is latched
     */
    public boolean risingLatched() {
        SignalState signal = signalIfPresent(this);
        return signal != null && signal.risingLatched();
    }

    /**
     * Returns whether a falling edge has been latched since the last consume.
     *
     * @return true when a falling edge is latched
     */
    public boolean fallingLatched() {
        SignalState signal = signalIfPresent(this);
        return signal != null && signal.fallingLatched();
    }

    /**
     * Clears latched edge state after hooks have consumed it.
     */
    public void clearLatchedEdges() {
        SignalState signal = signalIfPresent(this);
        if (signal != null) {
            signal.clearLatchedEdges();
        }
    }

    /**
     * Returns a stable default name.
     *
     * @return default name
     */
    public String defaultName() {
        return sanitize(name);
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }

    private static SignalState signal(DigitalInputDevice input) {
        RuntimeScope scope = CURRENT_SCOPE.get();
        return scope == null ? SIGNALS.computeIfAbsent(input, ignored -> new SignalState()) : scope.signal(input);
    }

    private static SignalState signalIfPresent(DigitalInputDevice input) {
        RuntimeScope scope = CURRENT_SCOPE.get();
        return scope == null ? SIGNALS.get(input) : scope.signals.get(input);
    }

    private static BooleanSupplier runtimeReader(DigitalInputDevice input) {
        RuntimeScope scope = CURRENT_SCOPE.get();
        BooleanSupplier scoped = scope == null ? null : scope.readers.get(input);
        return scoped == null ? RUNTIME_READERS.get(input) : scoped;
    }

    /**
     * Scoped digital input runtime state.
     */
    public static final class RuntimeScope {
        private final ConcurrentMap<DigitalInputDevice, BooleanSupplier> readers = new ConcurrentHashMap<>();
        private final ConcurrentMap<DigitalInputDevice, SignalState> signals = new ConcurrentHashMap<>();

        private RuntimeScope() {
        }

        /**
         * Binds a reader in this scope.
         *
         * @param input input declaration
         * @param reader reader
         * @return this scope
         */
        public RuntimeScope bind(DigitalInputDevice input, BooleanSupplier reader) {
            readers.put(Objects.requireNonNull(input, "input"), Objects.requireNonNull(reader, "reader"));
            return this;
        }

        /**
         * Returns bound readers.
         *
         * @return readers
         */
        public Map<DigitalInputDevice, BooleanSupplier> readers() {
            return Map.copyOf(readers);
        }

        private SignalState signal(DigitalInputDevice input) {
            return signals.computeIfAbsent(input, ignored -> new SignalState());
        }
    }

    private static final class SignalState {
        private boolean sampled;
        private boolean active;
        private boolean risingLatched;
        private boolean fallingLatched;
        private long risingGeneration;
        private long fallingGeneration;
        private long observedRisingGeneration;
        private long observedFallingGeneration;

        private synchronized void sample(boolean nextActive) {
            if (sampled) {
                if (!active && nextActive) {
                    risingLatched = true;
                    risingGeneration++;
                }
                if (active && !nextActive) {
                    fallingLatched = true;
                    fallingGeneration++;
                }
            }
            active = nextActive;
            sampled = true;
        }

        private synchronized boolean hasSample() {
            return sampled;
        }

        private synchronized boolean active() {
            return active;
        }

        private synchronized boolean risingLatched() {
            if (risingLatched) {
                observedRisingGeneration = risingGeneration;
            }
            return risingLatched;
        }

        private synchronized boolean fallingLatched() {
            if (fallingLatched) {
                observedFallingGeneration = fallingGeneration;
            }
            return fallingLatched;
        }

        private synchronized void clearLatchedEdges() {
            if (observedRisingGeneration >= risingGeneration) {
                risingLatched = false;
            }
            if (observedFallingGeneration >= fallingGeneration) {
                fallingLatched = false;
            }
        }
    }
}
