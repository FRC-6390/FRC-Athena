package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;

/**
 * Factories for hook events.
 */
public final class Events {
    private Events() {
    }

    public static SignalBuilder when(BooleanSupplier source) {
        return new SignalBuilder("signal", source, null);
    }

    public static SignalBuilder when(DigitalInputDevice input) {
        Objects.requireNonNull(input, "input");
        return new SignalBuilder(input.defaultName(), input::sampledActive, input);
    }

    /** Creates events from a debounced motor stall detector. */
    public static StallSignalBuilder when(MotorStallSignal signal) {
        return new StallSignalBuilder(signal);
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
        private final DigitalInputDevice input;

        private SignalBuilder(String name, BooleanSupplier source, DigitalInputDevice input) {
            this.name = name == null || name.isBlank() ? "signal" : name;
            this.source = Objects.requireNonNull(source, "source");
            this.input = input;
        }

        public EventBinding active() {
            return event(Edge.LEVEL);
        }

        public EventBinding rising() {
            return event(Edge.RISING);
        }

        public EventBinding falling() {
            return event(Edge.FALLING);
        }

        private EventBinding event(Edge edge) {
            return input == null ? new SignalEvent(name, source, edge) : new DigitalInputEvent(name, input, edge);
        }
    }

    public static final class StallSignalBuilder {
        private final MotorStallSignal signal;

        private StallSignalBuilder(MotorStallSignal signal) {
            this.signal = Objects.requireNonNull(signal, "signal");
        }

        public EventBinding active() { return new MotorStallEvent(signal, Edge.LEVEL); }

        public EventBinding rising() { return new MotorStallEvent(signal, Edge.RISING); }

        public EventBinding falling() { return new MotorStallEvent(signal, Edge.FALLING); }
    }

    static final class MotorStallEvent implements EventBinding {
        private final MotorStallSignal signal;
        private final Edge edge;
        private double candidateSince = Double.NaN;
        private double inactiveSince = Double.NaN;
        private boolean armed = true;

        private MotorStallEvent(MotorStallSignal signal, Edge edge) {
            this.signal = signal;
            this.edge = edge;
        }

        @Override
        public String name() { return signal.motor().defaultName() + "Stall"; }

        @Override
        public boolean sourceActive(EventContext context) {
            double now = context.nowSeconds();
            boolean raw = signal.instantaneousActive();
            if (!armed) {
                if (raw) {
                    inactiveSince = Double.NaN;
                } else if (Double.isNaN(inactiveSince)) {
                    inactiveSince = now;
                } else if (now - inactiveSince >= signal.rearmSeconds()) {
                    armed = true;
                }
                return false;
            }
            if (!raw) {
                candidateSince = Double.NaN;
                return false;
            }
            if (Double.isNaN(candidateSince)) {
                candidateSince = now;
            }
            return now - candidateSince >= signal.durationSeconds();
        }

        @Override
        public boolean active(EventContext context, boolean previous, boolean current) {
            boolean result = switch (edge) {
                case LEVEL -> current;
                case RISING -> current && !previous;
                case FALLING -> !current && previous;
            };
            if (edge == Edge.RISING && result) {
                armed = false;
                candidateSince = Double.NaN;
                inactiveSince = Double.NaN;
            }
            return result;
        }

        @Override
        public boolean pulse() { return edge != Edge.LEVEL; }

        @Override
        public List<Object> declarations() { return List.of(signal.motor()); }
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
            return active(context, previousSourceActive, sourceActive(context));
        }

        @Override
        public boolean active(EventContext context, boolean previousSourceActive, boolean currentSourceActive) {
            boolean active = currentSourceActive;
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

    public record DigitalInputEvent(String name, DigitalInputDevice input, Edge edge) implements EventBinding {
        public DigitalInputEvent {
            name = name == null || name.isBlank() ? "digitalInput" : name;
            Objects.requireNonNull(input, "input");
            edge = edge == null ? Edge.LEVEL : edge;
        }

        @Override
        public boolean sourceActive(EventContext context) {
            return input.sampledActive();
        }

        @Override
        public boolean active(EventContext context, boolean previousSourceActive) {
            return active(context, previousSourceActive, sourceActive(context));
        }

        @Override
        public boolean active(EventContext context, boolean previousSourceActive, boolean currentSourceActive) {
            boolean active = currentSourceActive;
            return switch (edge) {
                case LEVEL -> active;
                case RISING -> input.risingLatched() || active && !previousSourceActive;
                case FALLING -> input.fallingLatched() || !active && previousSourceActive;
            };
        }

        @Override
        public boolean pulse() {
            return edge != Edge.LEVEL;
        }

        @Override
        public void afterRun(EventContext context) {
            input.clearLatchedEdges();
        }

        @Override
        public List<Object> declarations() {
            return List.of(input);
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
            boolean matchingMode = safeContext.mode() == mode
                    || mode == LifecycleMode.ROBOT && phase == LifecyclePhase.PERIODIC;
            return matchingMode && safeContext.phase() == phase;
        }

        @Override
        public boolean pulse() {
            return true;
        }
    }
}
