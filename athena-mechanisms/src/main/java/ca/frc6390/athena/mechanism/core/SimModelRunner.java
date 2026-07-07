package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.hardware.ref.RuntimeBoolean;
import ca.frc6390.athena.hardware.ref.SimLimitRef;
import ca.frc6390.athena.hardware.ref.SimRef;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

final class SimModelRunner {
    private final Map<SimRef, ModelState> states = new IdentityHashMap<>();

    void step(
            Collection<SimRef> simulations,
            Map<MotorRef, SimMotor> motors,
            Map<EncoderRef, SimEncoder> encoders,
            Map<DigitalInputRef, RuntimeBoolean> digitalInputs,
            double dtSeconds) {
        if (dtSeconds <= 0.0 || simulations.isEmpty()) {
            return;
        }
        for (SimRef simulation : simulations) {
            step(simulation, motors, encoders, digitalInputs, dtSeconds);
        }
    }

    private void step(
            SimRef simulation,
            Map<MotorRef, SimMotor> motors,
            Map<EncoderRef, SimEncoder> encoders,
            Map<DigitalInputRef, RuntimeBoolean> digitalInputs,
            double dtSeconds) {
        List<EncoderRef> linkedEncoders = linkedEncoders(simulation);
        if (simulation.motors().isEmpty() || linkedEncoders.isEmpty()) {
            return;
        }

        ModelState state = states.computeIfAbsent(simulation, ignored -> initialState(linkedEncoders, encoders));
        double targetVelocity = targetVelocity(simulation, motors, state.position);
        double responseTime = responseTime(simulation);
        double blend = Math.min(1.0, dtSeconds / responseTime);
        state.velocity += (targetVelocity - state.velocity) * blend;

        double nextPosition = state.position + state.velocity * dtSeconds;
        RangeRef range = range(simulation);
        if (range != null) {
            double clamped = range.clamp(nextPosition);
            if (clamped != nextPosition) {
                nextPosition = clamped;
                state.velocity = 0.0;
            }
        }
        state.position = nextPosition;

        for (EncoderRef encoderRef : linkedEncoders) {
            SimEncoder encoder = encoders.get(encoderRef);
            if (encoder != null) {
                encoder.set(state.position);
                encoder.setVelocity(state.velocity);
            }
        }

        for (SimLimitRef limit : limits(simulation)) {
            RuntimeBoolean input = digitalInputs.get(limit.sensor());
            if (input != null) {
                boolean active = Math.abs(state.position - limit.position()) <= limit.tolerance();
                input.set(limit.sensor().isInverted() ? !active : active);
            }
        }
    }

    private static ModelState initialState(List<EncoderRef> linkedEncoders, Map<EncoderRef, SimEncoder> encoders) {
        for (EncoderRef encoderRef : linkedEncoders) {
            SimEncoder encoder = encoders.get(encoderRef);
            if (encoder != null) {
                return new ModelState(encoder.position(), encoder.velocity());
            }
        }
        return new ModelState(0.0, 0.0);
    }

    private static double targetVelocity(SimRef simulation, Map<MotorRef, SimMotor> motors, double position) {
        List<MotorCommand> commands = new ArrayList<>();
        for (MotorRef motor : simulation.motors()) {
            MotorCommand command = command(motor, motors);
            if (command != null) {
                commands.add(command);
            }
        }
        if (commands.isEmpty()) {
            return 0.0;
        }

        double sum = 0.0;
        for (MotorCommand command : commands) {
            sum += switch (command.kind()) {
                case NEUTRAL -> 0.0;
                case PERCENT -> command.value() * freeSpeed(simulation);
                case VOLTAGE -> command.value() / 12.0 * freeSpeed(simulation);
                case POSITION -> (command.value() - position) * closedLoopGain(simulation);
                case VELOCITY -> command.value();
            };
        }
        return sum / commands.size();
    }

    private static MotorCommand command(MotorRef motor, Map<MotorRef, SimMotor> motors) {
        Objects.requireNonNull(motor, "motor");
        SimMotor runtime = motors.get(motor);
        if (runtime != null && runtime.commandKind() != SimMotor.CommandKind.NEUTRAL) {
            return new MotorCommand(runtime.commandKind(), runtime.commandValue());
        }
        if (motor.follower() != null) {
            MotorCommand leader = command(motor.follower().leader(), motors);
            if (leader == null) {
                return null;
            }
            if (motor.isInverted()
                    && (leader.kind() == SimMotor.CommandKind.PERCENT
                    || leader.kind() == SimMotor.CommandKind.VOLTAGE
                    || leader.kind() == SimMotor.CommandKind.VELOCITY)) {
                return new MotorCommand(leader.kind(), -leader.value());
            }
            return leader;
        }
        if (runtime != null) {
            return new MotorCommand(runtime.commandKind(), runtime.commandValue());
        }
        return null;
    }

    private static double freeSpeed(SimRef simulation) {
        double speed = switch (simulation.kind()) {
            case ARM -> 120.0;
            case FLYWHEEL -> 120.0;
            case MOTOR -> 60.0;
        };
        if (simulation.gearRatio() != null) {
            speed *= Math.abs(simulation.gearRatio().ratio());
        }
        return Math.max(1.0, speed);
    }

    private static double closedLoopGain(SimRef simulation) {
        return switch (simulation.kind()) {
            case ARM -> 8.0;
            case FLYWHEEL -> 12.0;
            case MOTOR -> 10.0;
        };
    }

    private static double responseTime(SimRef simulation) {
        double base = switch (simulation.kind()) {
            case ARM -> 0.22;
            case FLYWHEEL -> 0.16;
            case MOTOR -> 0.12;
        };
        return base + simulation.momentOfInertia().orElse(0.0) * 2.0;
    }

    private static List<EncoderRef> linkedEncoders(SimRef simulation) {
        Set<EncoderRef> refs = new LinkedHashSet<>(simulation.encoders());
        for (MotorRef motor : simulation.motors()) {
            refs.add(motor.encoder());
        }
        for (Object ref : simulation.refs()) {
            if (ref instanceof EncoderRef encoder) {
                refs.add(encoder);
            } else if (ref instanceof AxisRef axis) {
                refs.addAll(axis.encoders());
            }
        }
        return List.copyOf(refs);
    }

    private static RangeRef range(SimRef simulation) {
        for (Object ref : simulation.refs()) {
            if (ref instanceof RangeRef range) {
                return range;
            } else if (ref instanceof AxisRef axis && axis.range() != null) {
                return axis.range();
            }
        }
        return null;
    }

    private static List<SimLimitRef> limits(SimRef simulation) {
        List<SimLimitRef> limits = new ArrayList<>();
        for (Object ref : simulation.refs()) {
            if (ref instanceof SimLimitRef limit) {
                limits.add(limit);
            }
        }
        return limits;
    }

    private record MotorCommand(SimMotor.CommandKind kind, double value) {
    }

    private static final class ModelState {
        private double position;
        private double velocity;

        private ModelState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}
