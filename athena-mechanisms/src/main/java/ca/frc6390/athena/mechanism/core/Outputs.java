package ca.frc6390.athena.mechanism.core;

/**
 * Factories for direct output commands.
 */
public final class Outputs {
    private Outputs() {
    }

    /**
     * Creates a neutral output.
     *
     * @return neutral output
     */
    public static Output neutral() {
        return new Neutral();
    }

    /**
     * Creates a percent output.
     *
     * @param percent percent output
     * @return percent output
     */
    public static Output percent(double percent) {
        return new Percent(percent);
    }

    /**
     * Creates a position output.
     *
     * @param position position target
     * @return position output
     */
    public static Output position(double position) {
        return new Position(position);
    }

    /**
     * Creates a velocity output.
     *
     * @param velocity velocity target
     * @return velocity output
     */
    public static Output velocity(double velocity) {
        return new Velocity(velocity);
    }

    /**
     * Creates a fault output.
     *
     * @param reason fault reason
     * @return fault output
     */
    public static Output fault(String reason) {
        return new Fault(reason);
    }

    private record Neutral() implements Output.Neutral {
    }

    private record Percent(double percent) implements Output.Percent {
    }

    private record Position(double position) implements Output.Position {
    }

    private record Velocity(double velocity) implements Output.Velocity {
    }

    private record Fault(String reason) implements Output.Fault {
        private Fault {
            reason = reason == null ? "" : reason;
        }
    }
}
