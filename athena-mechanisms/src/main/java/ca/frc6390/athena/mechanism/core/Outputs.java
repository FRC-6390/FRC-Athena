package ca.frc6390.athena.mechanism.core;

/**
 * Internal output value factories.
 */
final class Outputs {
    private static final Output NEUTRAL = new Neutral();

    private Outputs() {
    }

    static Output neutral() {
        return NEUTRAL;
    }

    static Output percent(double percent) {
        return new Percent(percent);
    }

    static Output voltage(double volts) {
        return new Voltage(volts);
    }

    static Output position(double position) {
        return new Position(position);
    }

    static Output velocity(double velocity) {
        return new Velocity(velocity);
    }

    /** True only for Athena's immutable scalar values, whose object can survive a no-op clamp unchanged. */
    static boolean isValue(Output output) {
        return output instanceof Percent || output instanceof Voltage || output instanceof Position
                || output instanceof Velocity;
    }

    private record Neutral() implements Output.Neutral {
    }

    private record Percent(double percent) implements Output.Percent {
    }

    private record Voltage(double volts) implements Output.Voltage {
    }

    private record Position(double position) implements Output.Position {
    }

    private record Velocity(double velocity) implements Output.Velocity {
    }
}
