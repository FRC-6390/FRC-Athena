package ca.frc6390.athena.mechanism.core;

/**
 * Typed output request produced by a mechanism state.
 */
public interface Output {
    interface Neutral extends Output {
    }

    interface Percent extends Output {
        double percent();
    }

    interface Voltage extends Output {
        double volts();
    }

    interface Position extends Output {
        double position();
    }

    interface Velocity extends Output {
        double velocity();
    }

    interface Fault extends Output {
        String reason();
    }
}
