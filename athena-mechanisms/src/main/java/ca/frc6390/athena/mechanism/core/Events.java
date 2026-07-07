package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.BooleanRef;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Factories for hook events.
 */
public final class Events {
    private Events() {
    }

    /**
     * Starts a signal event from a boolean supplier.
     *
     * @param source source supplier
     * @return signal event builder
     */
    public static SignalBuilder when(BooleanSupplier source) {
        return new SignalBuilder("signal", source);
    }

    /**
     * Starts a signal event from a digital input.
     *
     * @param input digital input
     * @return signal event builder
     */
    public static SignalBuilder when(DigitalInputRef input) {
        Objects.requireNonNull(input, "input");
        return new SignalBuilder(input.defaultName(), input::active);
    }

    /**
     * Starts a signal event from a boolean ref.
     *
     * @param signal boolean signal
     * @return signal event builder
     */
    public static SignalBuilder when(BooleanRef signal) {
        Objects.requireNonNull(signal, "signal");
        return new SignalBuilder(signal.defaultName(), signal::active);
    }

    public static EventRef robotInit() {
        return lifecycle("robotInit", LifecycleMode.ROBOT, LifecyclePhase.INIT);
    }

    public static EventRef robotPeriodic() {
        return lifecycle("robotPeriodic", LifecycleMode.ROBOT, LifecyclePhase.PERIODIC);
    }

    public static EventRef robotExit() {
        return lifecycle("robotExit", LifecycleMode.ROBOT, LifecyclePhase.EXIT);
    }

    public static EventRef disabledInit() {
        return lifecycle("disabledInit", LifecycleMode.DISABLED, LifecyclePhase.INIT);
    }

    public static EventRef disabledPeriodic() {
        return lifecycle("disabledPeriodic", LifecycleMode.DISABLED, LifecyclePhase.PERIODIC);
    }

    public static EventRef disabledExit() {
        return lifecycle("disabledExit", LifecycleMode.DISABLED, LifecyclePhase.EXIT);
    }

    public static EventRef disableInit() {
        return disabledInit();
    }

    public static EventRef disablePeriodic() {
        return disabledPeriodic();
    }

    public static EventRef disableExit() {
        return disabledExit();
    }

    public static EventRef autonomousInit() {
        return lifecycle("autonomousInit", LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT);
    }

    public static EventRef autonomousPeriodic() {
        return lifecycle("autonomousPeriodic", LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC);
    }

    public static EventRef autonomousExit() {
        return lifecycle("autonomousExit", LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT);
    }

    public static EventRef autoInit() {
        return autonomousInit();
    }

    public static EventRef autoPeriodic() {
        return autonomousPeriodic();
    }

    public static EventRef autoExit() {
        return autonomousExit();
    }

    public static EventRef teleopInit() {
        return lifecycle("teleopInit", LifecycleMode.TELEOP, LifecyclePhase.INIT);
    }

    public static EventRef teleopPeriodic() {
        return lifecycle("teleopPeriodic", LifecycleMode.TELEOP, LifecyclePhase.PERIODIC);
    }

    public static EventRef teleopExit() {
        return lifecycle("teleopExit", LifecycleMode.TELEOP, LifecyclePhase.EXIT);
    }

    public static EventRef testInit() {
        return lifecycle("testInit", LifecycleMode.TEST, LifecyclePhase.INIT);
    }

    public static EventRef testPeriodic() {
        return lifecycle("testPeriodic", LifecycleMode.TEST, LifecyclePhase.PERIODIC);
    }

    public static EventRef testExit() {
        return lifecycle("testExit", LifecycleMode.TEST, LifecyclePhase.EXIT);
    }

    public static EventRef simulationInit() {
        return lifecycle("simulationInit", LifecycleMode.SIMULATION, LifecyclePhase.INIT);
    }

    public static EventRef simulationPeriodic() {
        return lifecycle("simulationPeriodic", LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC);
    }

    public static EventRef simulationExit() {
        return lifecycle("simulationExit", LifecycleMode.SIMULATION, LifecyclePhase.EXIT);
    }

    public static EventRef simInit() {
        return simulationInit();
    }

    public static EventRef simPeriodic() {
        return simulationPeriodic();
    }

    public static EventRef simExit() {
        return simulationExit();
    }

    static EventRef lifecycle(String name, LifecycleMode mode, LifecyclePhase phase) {
        return new LifecycleEvent(name, mode, phase);
    }

    /**
     * Builder for boolean signal events.
     */
    public static final class SignalBuilder {
        private final String name;
        private final BooleanSupplier source;

        private SignalBuilder(String name, BooleanSupplier source) {
            this.name = name == null || name.isBlank() ? "signal" : name;
            this.source = Objects.requireNonNull(source, "source");
        }

        /**
         * Creates a level event.
         *
         * @return event active while the source is true
         */
        public EventRef active() {
            return new SignalEvent(name, source, EventActivation.LEVEL);
        }

        /**
         * Creates a rising-edge event.
         *
         * @return event active when the source changes false to true
         */
        public EventRef rising() {
            return new SignalEvent(name, source, EventActivation.RISING);
        }

        /**
         * Creates a falling-edge event.
         *
         * @return event active when the source changes true to false
         */
        public EventRef falling() {
            return new SignalEvent(name, source, EventActivation.FALLING);
        }
    }

    public record SignalEvent(String name, BooleanSupplier source, EventActivation activation) implements EventRef {
        public SignalEvent {
            name = name == null || name.isBlank() ? "signal" : name;
            Objects.requireNonNull(source, "source");
            activation = activation == null ? EventActivation.LEVEL : activation;
        }

        @Override
        public boolean sourceActive(EventContext context) {
            return source.getAsBoolean();
        }
    }

    public record LifecycleEvent(String name, LifecycleMode mode, LifecyclePhase phase) implements EventRef {
        public LifecycleEvent {
            name = name == null || name.isBlank() ? "lifecycle" : name;
            mode = mode == null ? LifecycleMode.ROBOT : mode;
            phase = phase == null ? LifecyclePhase.PERIODIC : phase;
        }

        @Override
        public EventActivation activation() {
            return EventActivation.PULSE;
        }

        @Override
        public boolean sourceActive(EventContext context) {
            EventContext safeContext = context == null ? EventContext.empty() : context;
            return safeContext.mode() == mode && safeContext.phase() == phase;
        }
    }
}
