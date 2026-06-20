package ca.frc6390.athena.mechanism.spec;

import java.util.Optional;

/**
 * Immutable control declaration for a mechanism.
 *
 * @param mode control mode
 * @param pid optional PID gains
 * @param feedforward optional feedforward gains
 */
public record ControlSpec(ControlMode mode, PidSpec pid, FeedforwardSpec feedforward) {
    public ControlSpec {
        mode = mode == null ? ControlMode.NONE : mode;
    }

    /**
     * Creates an empty control declaration.
     *
     * @return control spec
     */
    public static ControlSpec none() {
        return new ControlSpec(ControlMode.NONE, null, null);
    }

    /**
     * Returns optional PID gains.
     *
     * @return optional PID gains
     */
    public Optional<PidSpec> pidSpec() {
        return Optional.ofNullable(pid);
    }

    /**
     * Returns optional feedforward gains.
     *
     * @return optional feedforward gains
     */
    public Optional<FeedforwardSpec> feedforwardSpec() {
        return Optional.ofNullable(feedforward);
    }
}
