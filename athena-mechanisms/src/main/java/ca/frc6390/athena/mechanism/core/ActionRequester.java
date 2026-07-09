package ca.frc6390.athena.mechanism.core;

/**
 * Receives Action requests from robot-facing code.
 */
@FunctionalInterface
public interface ActionRequester {
    void request(Action action);
}
