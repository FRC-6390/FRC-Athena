package ca.frc6390.athena.mechanism.core;

import java.util.List;

/**
 * Inspectable result of one mechanism-runtime cycle.
 *
 * <p>The runtime creates this trace from values it already sampled or calculated. Reading a trace
 * never refreshes hardware and is therefore safe for telemetry exporters.</p>
 */
public record MechanismTraceSnapshot(
        String mechanism,
        double timestampSeconds,
        double timeInStateSeconds,
        boolean enabled,
        String requestedAction,
        String requestedActionType,
        String scheduledActionType,
        int schedulerStep,
        boolean schedulerComplete,
        int activeLeaseCount,
        List<ActionCandidate> candidates,
        List<Control> controls,
        List<Motor> motors,
        List<Hook> hooks) {
    public MechanismTraceSnapshot {
        mechanism = safe(mechanism);
        requestedAction = safe(requestedAction);
        requestedActionType = safe(requestedActionType);
        scheduledActionType = safe(scheduledActionType);
        candidates = candidates == null ? List.of() : List.copyOf(candidates);
        controls = controls == null ? List.of() : List.copyOf(controls);
        motors = motors == null ? List.of() : List.copyOf(motors);
        hooks = hooks == null ? List.of() : List.copyOf(hooks);
    }

    /** One output competing for ownership of its motors. */
    public record ActionCandidate(
            String source,
            String actionType,
            long recency,
            int order,
            boolean selected,
            String outputMode,
            double requestedValue,
            List<String> motors) {
        public ActionCandidate {
            source = safe(source);
            actionType = safe(actionType);
            outputMode = safe(outputMode);
            motors = motors == null ? List.of() : List.copyOf(motors);
        }
    }

    /** One control request after target transforms, planning, profiling, and output application. */
    public record Control(
            String name,
            String requestedMode,
            double requestedValue,
            double transformedValue,
            double goal,
            double referencePosition,
            double referenceVelocity,
            double referenceAcceleration,
            double measuredPosition,
            double measuredVelocity,
            double error,
            double proportionalVolts,
            double integralVolts,
            double derivativeVolts,
            double staticFeedforwardVolts,
            double velocityFeedforwardVolts,
            double accelerationFeedforwardVolts,
            double gravityFeedforwardVolts,
            String appliedMode,
            double appliedValue,
            String route,
            boolean constrained,
            boolean blocked,
            boolean saturated) {
        public Control {
            name = safe(name);
            requestedMode = safe(requestedMode);
            appliedMode = safe(appliedMode);
            route = safe(route);
        }
    }

    /** Cached motor feedback paired with the command selected during this cycle. */
    public record Motor(
            String name,
            String commandMode,
            double commandValue,
            double positionRotations,
            double velocityRotationsPerSecond,
            double appliedVoltage,
            double supplyCurrentAmps,
            double statorCurrentAmps) {
        public Motor {
            name = safe(name);
            commandMode = safe(commandMode);
        }
    }

    /** Latest sampled state of one named event hook. */
    public record Hook(String name, boolean sourceActive, boolean active, boolean triggeredThisCycle) {
        public Hook {
            name = safe(name);
        }
    }

    private static String safe(String value) {
        return value == null ? "" : value;
    }
}
