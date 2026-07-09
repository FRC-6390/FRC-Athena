package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Factories for hook events.
 */
public final class Events {
    private Events() {
    }

    public static SignalBuilder when(BooleanSupplier source) {
        return new SignalBuilder("signal", source);
    }

    public static SignalBuilder when(DigitalInputDevice input) {
        Objects.requireNonNull(input, "input");
        return new SignalBuilder(input.defaultName(), input::active);
    }

    public static EventBinding robotInit() { return lifecycle("robotInit", LifecycleMode.ROBOT, LifecyclePhase.INIT); }
    public static EventBinding robotPeriodic() { return lifecycle("robotPeriodic", LifecycleMode.ROBOT, LifecyclePhase.PERIODIC); }
    public static EventBinding robotExit() { return lifecycle("robotExit", LifecycleMode.ROBOT, LifecyclePhase.EXIT); }
    public static EventBinding disabledInit() { return lifecycle("disabledInit", LifecycleMode.DISABLED, LifecyclePhase.INIT); }
    public static EventBinding disabledPeriodic() { return lifecycle("disabledPeriodic", LifecycleMode.DISABLED, LifecyclePhase.PERIODIC); }
    public static EventBinding disabledExit() { return lifecycle("disabledExit", LifecycleMode.DISABLED, LifecyclePhase.EXIT); }
    public static EventBinding disableInit() { return disabledInit(); }
    public static EventBinding disablePeriodic() { return disabledPeriodic(); }
    public static EventBinding disableExit() { return disabledExit(); }
    public static EventBinding autonomousInit() { return lifecycle("autonomousInit", LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT); }
    public static EventBinding autonomousPeriodic() { return lifecycle("autonomousPeriodic", LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC); }
    public static EventBinding autonomousExit() { return lifecycle("autonomousExit", LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT); }
    public static EventBinding autoInit() { return autonomousInit(); }
    public static EventBinding autoPeriodic() { return autonomousPeriodic(); }
    public static EventBinding autoExit() { return autonomousExit(); }
    public static EventBinding teleopInit() { return lifecycle("teleopInit", LifecycleMode.TELEOP, LifecyclePhase.INIT); }
    public static EventBinding teleopPeriodic() { return lifecycle("teleopPeriodic", LifecycleMode.TELEOP, LifecyclePhase.PERIODIC); }
    public static EventBinding teleopExit() { return lifecycle("teleopExit", LifecycleMode.TELEOP, LifecyclePhase.EXIT); }
    public static EventBinding testInit() { return lifecycle("testInit", LifecycleMode.TEST, LifecyclePhase.INIT); }
    public static EventBinding testPeriodic() { return lifecycle("testPeriodic", LifecycleMode.TEST, LifecyclePhase.PERIODIC); }
    public static EventBinding testExit() { return lifecycle("testExit", LifecycleMode.TEST, LifecyclePhase.EXIT); }
    public static EventBinding simulationInit() { return lifecycle("simulationInit", LifecycleMode.SIMULATION, LifecyclePhase.INIT); }
    public static EventBinding simulationPeriodic() { return lifecycle("simulationPeriodic", LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC); }
    public static EventBinding simulationExit() { return lifecycle("simulationExit", LifecycleMode.SIMULATION, LifecyclePhase.EXIT); }
    public static EventBinding simInit() { return simulationInit(); }
    public static EventBinding simPeriodic() { return simulationPeriodic(); }
    public static EventBinding simExit() { return simulationExit(); }

    static EventBinding lifecycle(String name, LifecycleMode mode, LifecyclePhase phase) {
        return new LifecycleEvent(name, mode, phase);
    }

    public static final class SignalBuilder {
        private final String name;
        private final BooleanSupplier source;

        private SignalBuilder(String name, BooleanSupplier source) {
            this.name = name == null || name.isBlank() ? "signal" : name;
            this.source = Objects.requireNonNull(source, "source");
        }

        public EventBinding active() {
            return new SignalEvent(name, source, Edge.LEVEL);
        }

        public EventBinding rising() {
            return new SignalEvent(name, source, Edge.RISING);
        }

        public EventBinding falling() {
            return new SignalEvent(name, source, Edge.FALLING);
        }
    }

    enum Edge { LEVEL, RISING, FALLING }

    public record SignalEvent(String name, BooleanSupplier source, Edge edge) implements EventBinding {
        public SignalEvent {
            name = name == null || name.isBlank() ? "signal" : name;
            Objects.requireNonNull(source, "source");
            edge = edge == null ? Edge.LEVEL : edge;
        }

        @Override
        public boolean sourceActive(EventContext context) {
            return source.getAsBoolean();
        }

        @Override
        public boolean active(EventContext context, boolean previousSourceActive) {
            boolean active = sourceActive(context);
            return switch (edge) {
                case LEVEL -> active;
                case RISING -> active && !previousSourceActive;
                case FALLING -> !active && previousSourceActive;
            };
        }

        @Override
        public boolean pulse() {
            return edge != Edge.LEVEL;
        }
    }

    public record LifecycleEvent(String name, LifecycleMode mode, LifecyclePhase phase) implements EventBinding {
        public LifecycleEvent {
            name = name == null || name.isBlank() ? "lifecycle" : name;
            mode = mode == null ? LifecycleMode.ROBOT : mode;
            phase = phase == null ? LifecyclePhase.PERIODIC : phase;
        }

        @Override
        public boolean sourceActive(EventContext context) {
            EventContext safeContext = context == null ? EventContext.empty() : context;
            return safeContext.mode() == mode && safeContext.phase() == phase;
        }

        @Override
        public boolean pulse() {
            return true;
        }
    }
}
