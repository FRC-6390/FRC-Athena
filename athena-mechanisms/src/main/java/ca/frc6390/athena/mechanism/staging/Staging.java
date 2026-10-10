package ca.frc6390.athena.mechanism.staging;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.RuntimeEvents;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Team-defined health or operating stages for a mechanism, with caps that the runtime applies.
 *
 * <pre>{@code
 * public final Staging<ArmHealth> health = Staging.of(ArmHealth.NORMAL)
 *         .stage(ArmHealth.STRAINED, strained, angle.limitEffort(0.6))
 *         .stage(ArmHealth.HOT, hot, angle.limitSpeed(0.5))
 *         .stage(ArmHealth.SAFE_SHUTDOWN, shutdown, someAction)
 *         .exitAfter(Seconds.of(2));
 * }</pre>
 *
 * <ul>
 *   <li>The worst (highest ordinal) stage whose condition is true is entered at once.</li>
 *   <li>Leaving a stage requires its condition to stay false for {@link #exitAfter(Time)}; then the
 *       worst remaining true stage (or the initial stage) is entered.</li>
 *   <li>{@link #override(BooleanSupplier)} forces the initial stage while true.</li>
 *   <li>A stage with an Action requests it when entered and cancels it when left. A stage with modifiers
 *       caps the bindings it names while it is current.</li>
 *   <li>{@link Action#staged(Staging)} makes the staging evaluate, and its caps apply, only while that
 *       Action runs. Otherwise a declared staging is always evaluated.</li>
 * </ul>
 *
 * @param <E> stage enum
 */
public final class Staging<E extends Enum<E>> {
    /** Stage transition, for reporting. */
    public record Change<E>(E from, E to, double seconds) {
    }

    private static final int CHANGE_HISTORY = 32;

    private final E initial;
    private final Map<E, Definition<E>> definitions = new LinkedHashMap<>();
    private final Set<Action> gates = Collections.newSetFromMap(new IdentityHashMap<>());
    private final List<Change<E>> changes = new ArrayList<>();
    private double exitAfterSeconds;
    private BooleanSupplier override;
    private E current;
    private double exitSinceSeconds = Double.NaN;
    private boolean active = true;

    private Staging(E initial) {
        this.initial = Objects.requireNonNull(initial, "initial");
        this.current = initial;
    }

    /**
     * Creates a staging that starts in {@code initial}.
     *
     * @param initial initial stage
     * @param <E> stage enum
     * @return staging
     */
    public static <E extends Enum<E>> Staging<E> of(E initial) {
        return new Staging<>(initial);
    }

    /**
     * Declares a stage that requests an Action while it is current.
     *
     * @param stage stage
     * @param condition entry condition
     * @param action Action requested on entry and cancelled on exit
     * @return this staging
     */
    public Staging<E> stage(E stage, BooleanSupplier condition, Action action) {
        Objects.requireNonNull(action, "action");
        return define(stage, condition, action, new StageModifier[0]);
    }

    /**
     * Declares a stage that caps bindings while it is current.
     *
     * @param stage stage
     * @param condition entry condition
     * @param modifiers caps, typically from {@code binding.limitEffort(...)}
     * @return this staging
     */
    public Staging<E> stage(E stage, BooleanSupplier condition, StageModifier... modifiers) {
        return define(stage, condition, null, modifiers == null ? new StageModifier[0] : modifiers.clone());
    }

    /**
     * Sets how long every entry condition must stay false before a stage is left.
     *
     * @param time non-negative time
     * @return this staging
     */
    public Staging<E> exitAfter(Time time) {
        double seconds = Objects.requireNonNull(time, "time").in(Units.Seconds);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException("Exit delay must be a non-negative finite time.");
        }
        exitAfterSeconds = seconds;
        return this;
    }

    /**
     * Forces the initial stage while the condition is true.
     *
     * @param condition override condition, or {@code null} to remove it
     * @return this staging
     */
    public Staging<E> override(BooleanSupplier condition) {
        override = condition;
        return this;
    }

    /**
     * Returns a supplier of the current stage.
     *
     * @return current stage supplier
     */
    public Supplier<E> current() {
        return () -> current;
    }

    /** Returns the current stage. */
    public E currentStage() { return current; }

    /** Returns the initial stage. */
    public E initial() { return initial; }

    /** Returns whether the staging is evaluated in the current cycle (false while its gating Actions are idle). */
    public boolean active() { return active; }

    /**
     * Gates this staging on an Action. Used by {@link Action#staged(Staging)}.
     *
     * @param action action the staging is active for
     * @return this staging
     */
    public Staging<E> gate(Action action) {
        gates.add(Objects.requireNonNull(action, "action"));
        return this;
    }

    /**
     * Evaluates conditions, enters or leaves stages, and requests or cancels stage Actions.
     *
     * @param nowSeconds monotonic time
     * @param host scheduler services
     */
    public void evaluate(double nowSeconds, StageHost host) {
        Objects.requireNonNull(host, "host");
        active = gates.isEmpty() || anyGateRunning(host);
        if (!active) {
            exitSinceSeconds = Double.NaN;
            if (current != initial) transition(initial, nowSeconds, host);
            return;
        }
        if (override != null && override.getAsBoolean()) {
            exitSinceSeconds = Double.NaN;
            if (current != initial) transition(initial, nowSeconds, host);
            return;
        }
        E target = worstTrueStage();
        if (target == current) {
            exitSinceSeconds = Double.NaN;
            return;
        }
        if (target.ordinal() > current.ordinal()) {
            exitSinceSeconds = Double.NaN;
            transition(target, nowSeconds, host);
            return;
        }
        if (Double.isNaN(exitSinceSeconds)) exitSinceSeconds = nowSeconds;
        if (nowSeconds - exitSinceSeconds >= exitAfterSeconds) {
            exitSinceSeconds = Double.NaN;
            transition(target, nowSeconds, host);
        }
    }

    /**
     * Returns the caps the current stage places on bindings. Empty while inactive.
     *
     * @return modifiers of the current stage
     */
    public List<StageModifier> activeModifiers() {
        if (!active) return List.of();
        Definition<E> definition = definitions.get(current);
        return definition == null ? List.of() : definition.modifiers();
    }

    /**
     * Removes and returns recorded stage transitions, oldest first.
     *
     * @return transitions since the last call
     */
    public List<Change<E>> drainChanges() {
        List<Change<E>> drained = List.copyOf(changes);
        changes.clear();
        return drained;
    }

    private Staging<E> define(E stage, BooleanSupplier condition, Action action, StageModifier[] modifiers) {
        Objects.requireNonNull(stage, "stage");
        Objects.requireNonNull(condition, "condition");
        if (stage == initial) throw new IllegalArgumentException("The initial stage needs no condition.");
        if (definitions.containsKey(stage)) throw new IllegalArgumentException("Stage declared twice: " + stage);
        definitions.put(stage, new Definition<>(stage, condition, action, List.of(modifiers)));
        return this;
    }

    private E worstTrueStage() {
        E worst = initial;
        for (Definition<E> definition : definitions.values()) {
            if (definition.condition().getAsBoolean() && definition.stage().ordinal() > worst.ordinal()) {
                worst = definition.stage();
            }
        }
        return worst;
    }

    private boolean anyGateRunning(StageHost host) {
        for (Action gate : gates) {
            if (host.isRunning(gate)) return true;
        }
        return false;
    }

    private void transition(E to, double nowSeconds, StageHost host) {
        E from = current;
        Definition<E> leaving = definitions.get(from);
        if (leaving != null && leaving.action() != null) host.cancel(leaving.action());
        current = to;
        Definition<E> entering = definitions.get(to);
        if (entering != null && entering.action() != null) host.request(entering.action());
        changes.add(new Change<>(from, to, nowSeconds));
        if (changes.size() > CHANGE_HISTORY) changes.remove(0);
        RuntimeEvents.emit(RuntimeEvents.STAGE_CHANGE, from + " -> " + to + " t=" + nowSeconds);
    }

    private record Definition<E extends Enum<E>>(
            E stage, BooleanSupplier condition, Action action, List<StageModifier> modifiers) {
    }
}
