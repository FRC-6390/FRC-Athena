package ca.frc6390.athena.mechanism.core;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Set;

/**
 * Selects which output requests a rule applies to.
 */
public record OutputTarget(AxisRef axis, ControlRef control, Set<Kind> kinds, Direction direction) {
    public OutputTarget {
        kinds = kinds == null || kinds.isEmpty() ? Set.of() : Set.copyOf(kinds);
    }

    public static OutputTarget any() {
        return new OutputTarget(null, null, Set.of(), null);
    }

    public static OutputTarget axis(AxisRef axis) {
        return any().withAxis(axis);
    }

    public OutputTarget withAxis(AxisRef axis) {
        return new OutputTarget(Objects.requireNonNull(axis, "axis"), control, kinds, direction);
    }

    public static OutputTarget control(ControlRef control) {
        return any().withControl(control);
    }

    public OutputTarget withControl(ControlRef control) {
        return new OutputTarget(axis, Objects.requireNonNull(control, "control"), kinds, direction);
    }

    public OutputTarget openLoop() {
        return withKinds(Kind.PERCENT, Kind.VOLTAGE);
    }

    public OutputTarget closedLoop() {
        return withKinds(Kind.POSITION, Kind.VELOCITY);
    }

    public OutputTarget percent() {
        return withKinds(Kind.PERCENT);
    }

    public OutputTarget voltage() {
        return withKinds(Kind.VOLTAGE);
    }

    public OutputTarget position() {
        return withKinds(Kind.POSITION);
    }

    public OutputTarget velocity() {
        return withKinds(Kind.VELOCITY);
    }

    public OutputTarget positive() {
        return new OutputTarget(axis, control, kinds, Direction.POSITIVE);
    }

    public OutputTarget negative() {
        return new OutputTarget(axis, control, kinds, Direction.NEGATIVE);
    }

    public boolean matches(OutputRequest request) {
        if (request == null || request.output() == null) {
            return false;
        }
        if (axis != null && request.axis() != axis) {
            return false;
        }
        if (control != null && request.control() != control) {
            return false;
        }
        if (!kinds.isEmpty() && !kinds.contains(kind(request.output()))) {
            return false;
        }
        if (direction == Direction.POSITIVE && !request.positive()) {
            return false;
        }
        if (direction == Direction.NEGATIVE && !request.negative()) {
            return false;
        }
        return true;
    }

    private OutputTarget withKinds(Kind... nextKinds) {
        EnumSet<Kind> updated = EnumSet.noneOf(Kind.class);
        for (Kind kind : nextKinds) {
            updated.add(kind);
        }
        return new OutputTarget(axis, control, updated, direction);
    }

    private static Kind kind(Output output) {
        if (output instanceof Output.Percent) {
            return Kind.PERCENT;
        }
        if (output instanceof Output.Voltage) {
            return Kind.VOLTAGE;
        }
        if (output instanceof Output.Position) {
            return Kind.POSITION;
        }
        if (output instanceof Output.Velocity) {
            return Kind.VELOCITY;
        }
        return Kind.OTHER;
    }

    public enum Kind {
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY,
        OTHER
    }
}
