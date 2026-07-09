package ca.frc6390.athena.mechanism.core;

/**
 * Internal output value factories.
 */
final class Outputs {
    private Outputs() {
    }

    static Output neutral() {
        return new Neutral();
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
