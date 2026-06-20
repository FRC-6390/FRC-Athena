package ca.frc6390.athena.mechanism.config;

import java.util.function.Consumer;

import ca.frc6390.athena.mechanism.spec.ControlSpec;
import ca.frc6390.athena.mechanism.spec.ControlMode;

/**
 * Student-facing control declaration for a mechanism.
 */
public final class ControlConfig {
    private ControlMode mode = ControlMode.NONE;
    private PidConfig pid;
    private FeedforwardConfig feedforward;

    /**
     * Requests percent output control.
     *
     * @return this config
     */
    public ControlConfig percentOutput() {
        mode = ControlMode.PERCENT_OUTPUT;
        return this;
    }

    /**
     * Requests position closed-loop control.
     *
     * @return this config
     */
    public ControlConfig position() {
        mode = ControlMode.POSITION;
        return this;
    }

    /**
     * Requests position closed-loop control with PID gains.
     *
     * @param configure PID configuration callback
     * @return this config
     */
    public ControlConfig position(Consumer<PidConfig> configure) {
        position();
        pid(configure);
        return this;
    }

    /**
     * Requests velocity closed-loop control.
     *
     * @return this config
     */
    public ControlConfig velocity() {
        mode = ControlMode.VELOCITY;
        return this;
    }

    /**
     * Requests velocity closed-loop control with PID gains.
     *
     * @param configure PID configuration callback
     * @return this config
     */
    public ControlConfig velocity(Consumer<PidConfig> configure) {
        velocity();
        pid(configure);
        return this;
    }

    /**
     * Configures PID gains for the selected control mode.
     *
     * @param configure PID configuration callback
     * @return this config
     */
    public ControlConfig pid(Consumer<PidConfig> configure) {
        pid = new PidConfig();
        if (configure != null) {
            configure.accept(pid);
        }
        return this;
    }

    /**
     * Configures feedforward gains.
     *
     * @param configure feedforward configuration callback
     * @return this config
     */
    public ControlConfig feedforward(Consumer<FeedforwardConfig> configure) {
        feedforward = new FeedforwardConfig();
        if (configure != null) {
            configure.accept(feedforward);
        }
        return this;
    }

    /**
     * Returns the lowered control mode.
     *
     * @return control mode
     */
    public ControlSpec toSpec() {
        return new ControlSpec(
                mode,
                pid == null ? null : pid.toSpec(),
                feedforward == null ? null : feedforward.toSpec());
    }
}
