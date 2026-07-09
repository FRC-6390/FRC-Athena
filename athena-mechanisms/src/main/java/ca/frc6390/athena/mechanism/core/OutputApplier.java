package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.ControlRoute;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Applies resolved mechanism outputs to runtime hardware handles.
 */
final class OutputApplier {
    private final ActionContext context;
    private final Map<ControlBinding, ControlRuntimeState> controlRuntimes = new HashMap<>();
    private final AppliedOutput appliedOutput = new AppliedOutput();

    private OutputApplier(ActionContext context) {
        this.context = Objects.requireNonNull(context, "context");
    }

    static OutputApplier using(ActionContext context) {
        return new OutputApplier(context);
    }

    public void applyAll(List<ResolvedOutput> outputs) {
        applyAll(outputs, MechanismContext.empty());
    }

    public void applyAll(List<ResolvedOutput> outputs, MechanismContext context) {
        Objects.requireNonNull(outputs, "outputs");
        applyAll(outputs, 0, context);
    }

    void applyAll(List<ResolvedOutput> outputs, int startIndex, MechanismContext context) {
        Objects.requireNonNull(outputs, "outputs");
        int safeStart = Math.max(0, startIndex);
        MechanismContext safeContext = context == null ? MechanismContext.empty() : context;
        for (int i = safeStart; i < outputs.size(); i++) {
            apply(outputs.get(i), safeContext);
        }
    }

    public void apply(ResolvedOutput output) {
        apply(output, MechanismContext.empty());
    }

    public void apply(ResolvedOutput output, MechanismContext mechanismContext) {
        Objects.requireNonNull(output, "output");
        AppliedOutput applied = resolveControlOutput(
                output,
                mechanismContext == null ? MechanismContext.empty() : mechanismContext);
        for (MotorDevice motor : motors(output.request())) {
            apply(context.motor(motor), applied);
        }
    }

    private AppliedOutput resolveControlOutput(ResolvedOutput output, MechanismContext mechanismContext) {
        OutputRequest request = output.request();
        ControlBinding control = request.control();
        if (control == null || control.loops().isEmpty()) {
            return appliedOutput.set(output.output(), ControlRoute.OPEN_LOOP, null);
        }
        ControlLoopContext loopContext = new AppliedControlLoopContext(control, output.output(), mechanismContext);
        ControlRuntimeState state = controlRuntimes(control);
        Class<?> requestType = output.output().getClass();
        double requestValue = outputValue(output.output());
        if (state.requestChanged(requestType, requestValue)) {
            state.runtimes.forEach(runtime -> runtime.reset(loopContext));
            state.lastRequestType = requestType;
            state.lastRequestValue = requestValue;
            state.hasLastRequest = true;
        }
        AppliedOutput offloaded = resolveDeviceControl(control, output.output(), loopContext, state);
        if (offloaded != null) {
            return offloaded;
        }
        Output applied = null;
        for (ControlLoopRuntime runtime : state.runtimes) {
            ControlOutput controlOutput = runtime.calculate(loopContext);
            if (controlOutput != null) {
                applied = combine(applied, controlOutput.output());
            }
        }
        return appliedOutput.set(applied == null ? output.output() : applied, ControlRoute.OPEN_LOOP, null);
    }

    private ControlRuntimeState controlRuntimes(ControlBinding control) {
        return controlRuntimes.computeIfAbsent(control, ControlRuntimeState::new);
    }

    private AppliedOutput resolveDeviceControl(
            ControlBinding control,
            Output request,
            ControlLoopContext loopContext,
            ControlRuntimeState state) {
        if (!(request instanceof Output.Position || request instanceof Output.Velocity) || control.output() == null) {
            return null;
        }
        if (state.config == null) {
            return null;
        }
        MotorHandle motor = context.motor(control.output());
        MotorControlCapabilities capabilities = motor.controlCapabilities();
        if ((request instanceof Output.Position && !capabilities.supportsPosition())
                || (request instanceof Output.Velocity && !capabilities.supportsVelocity())
                || !capabilities.supportsSlot(state.config.slot())
                || !usesDeviceNativeFeedback(control)) {
            return null;
        }
        boolean hybrid = false;
        double arbitraryFeedforwardVolts = 0.0;
        for (int i = 0; i < state.runtimes.size(); i++) {
            ControlLoopRole role = state.roles[i];
            if (role == ControlLoopRole.DEVICE_CONFIGURABLE) {
                continue;
            }
            if (role != ControlLoopRole.ARBITRARY_FEEDFORWARD || !capabilities.arbitraryVoltageFeedforward()) {
                return null;
            }
            ControlOutput controlOutput = state.runtimes.get(i).calculate(loopContext);
            if (controlOutput != null) {
                arbitraryFeedforwardVolts += volts(controlOutput.output());
                hybrid = true;
            }
        }
        ControlRoute route = hybrid ? ControlRoute.HYBRID_CLOSED_LOOP : ControlRoute.DEVICE_CLOSED_LOOP;
        MotorClosedLoopRequest closedLoopRequest = state.closedLoopRequest(route, arbitraryFeedforwardVolts);
        return appliedOutput.set(
                request,
                route,
                closedLoopRequest);
    }

    private static List<MotorDevice> motors(OutputRequest request) {
        if (request.control() != null) {
            return request.control().motors();
        }
        if (request.motor() != null) {
            return List.of(request.motor());
        }
        return List.of();
    }

    private static void apply(MotorHandle motor, AppliedOutput applied) {
        Output output = applied.output();
        if (output instanceof Output.Percent percent) {
            motor.setPercentOutput(percent.percent());
        } else if (output instanceof Output.Voltage voltage) {
            motor.setVoltage(voltage.volts());
        } else if (output instanceof Output.Position position) {
            if (applied.closedLoopRequest() == null) {
                motor.setPositionTargetRotations(position.position());
            } else {
                motor.setPositionTargetRotations(position.position(), applied.closedLoopRequest());
            }
        } else if (output instanceof Output.Velocity velocity) {
            if (applied.closedLoopRequest() == null) {
                motor.setVelocityTargetRotationsPerSecond(velocity.velocity());
            } else {
                motor.setVelocityTargetRotationsPerSecond(velocity.velocity(), applied.closedLoopRequest());
            }
        } else if (output instanceof Output.Neutral || output instanceof Output.Fault) {
            motor.stop();
        }
    }

    private static boolean usesDeviceNativeFeedback(ControlBinding control) {
        if (control.feedback().isEmpty()) {
            return true;
        }
        EncoderDevice feedback = control.feedback().get(0);
        return isNativeFeedback(control.output(), feedback);
    }

    private static boolean isNativeFeedback(MotorDevice output, EncoderDevice feedback) {
        if (output == null || feedback == null || feedback.isInverted()
                || feedback.gearRatio() != 1.0 || feedback.conversion() != 1.0
                || feedback.offset() != 0.0 || feedback.units() != EncoderUnit.RAW) {
            return false;
        }
        return feedback.equals(output.encoder()) || feedback.equals(output.absoluteEncoder());
    }

    private static Output combine(Output current, Output next) {
        if (next == null) {
            return current;
        }
        if (current == null || current instanceof Output.Neutral || current instanceof Output.Fault
                || next instanceof Output.Neutral || next instanceof Output.Fault) {
            return next;
        }
        if (current instanceof Output.Voltage || next instanceof Output.Voltage) {
            return Outputs.voltage(volts(current) + volts(next));
        }
        if (current instanceof Output.Percent currentPercent && next instanceof Output.Percent nextPercent) {
            return Outputs.percent(currentPercent.percent() + nextPercent.percent());
        }
        if (current instanceof Output.Position currentPosition && next instanceof Output.Position nextPosition) {
            return Outputs.position(currentPosition.position() + nextPosition.position());
        }
        if (current instanceof Output.Velocity currentVelocity && next instanceof Output.Velocity nextVelocity) {
            return Outputs.velocity(currentVelocity.velocity() + nextVelocity.velocity());
        }
        return next;
    }

    private static double volts(Output output) {
        if (output instanceof Output.Voltage voltage) {
            return voltage.volts();
        }
        if (output instanceof Output.Percent percent) {
            return percent.percent() * 12.0;
        }
        return 0.0;
    }

    private static MotorClosedLoopConfig closedLoopConfig(ControlBinding control) {
        PidGains pid = null;
        FeedforwardGains feedforward = null;
        for (ControlLoop loop : control.loops()) {
            if (loop.role() != ControlLoopRole.DEVICE_CONFIGURABLE) {
                continue;
            }
            if (loop instanceof PidGains gains) {
                if (pid != null) {
                    return null;
                }
                pid = gains;
            } else if (loop instanceof FeedforwardGains gains) {
                if (feedforward != null) {
                    return null;
                }
                feedforward = gains;
            } else {
                return null;
            }
        }
        if (pid == null && feedforward == null) {
            return null;
        }
        return new MotorClosedLoopConfig(
                control.slot(),
                pid == null ? 0.0 : pid.p(),
                pid == null ? 0.0 : pid.i(),
                pid == null ? 0.0 : pid.d(),
                pid == null ? 0.0 : pid.iZone(),
                pid == null ? 0.0 : pid.tolerance(),
                feedforward == null ? 0.0 : feedforward.staticGain(),
                feedforward == null ? 0.0 : feedforward.velocityGain(),
                feedforward == null ? 0.0 : feedforward.gravityGain(),
                null);
    }

    private final class AppliedControlLoopContext implements ControlLoopContext {
        private final ControlBinding control;
        private final Output request;
        private final MechanismContext mechanismContext;

        private AppliedControlLoopContext(ControlBinding control, Output request, MechanismContext mechanismContext) {
            this.control = control;
            this.request = request;
            this.mechanismContext = mechanismContext;
        }

        @Override
        public Output request() {
            return request;
        }

        @Override
        public double position() {
            return firstFeedbackPosition(control);
        }

        @Override
        public double velocity() {
            return firstFeedbackVelocity(control);
        }

        @Override
        public double dtSeconds() {
            return mechanismContext.dtSeconds();
        }

        @Override
        public ActionContext actionContext() {
            return context;
        }
    }

    private double firstFeedbackPosition(ControlBinding control) {
        if (!control.feedback().isEmpty()) {
            EncoderHandle encoder = context.encoder(control.feedback().get(0));
            return readOrZero(encoder::positionRotations);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrZero(motor::integratedPositionRotations);
        }
        return 0.0;
    }

    private double firstFeedbackVelocity(ControlBinding control) {
        if (!control.feedback().isEmpty()) {
            EncoderHandle encoder = context.encoder(control.feedback().get(0));
            return readOrZero(encoder::velocityRotationsPerSecond);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrZero(motor::integratedVelocityRotationsPerSecond);
        }
        return 0.0;
    }

    private static double readOrZero(DoubleSupplier read) {
        try {
            return read.getAsDouble();
        } catch (UnsupportedOperationException ignored) {
            return 0.0;
        }
    }

    private interface DoubleSupplier {
        double getAsDouble();
    }

    private final class ControlRuntimeState {
        private final List<ControlLoopRuntime> runtimes;
        private final ControlLoopRole[] roles;
        private final MotorClosedLoopConfig config;
        private boolean hasLastRequest;
        private Class<?> lastRequestType;
        private double lastRequestValue;
        private ControlRoute cachedRequestRoute;
        private double cachedRequestFeedforwardVolts;
        private MotorClosedLoopRequest cachedRequest;

        private ControlRuntimeState(ControlBinding binding) {
            List<ControlLoop> loops = binding.loops();
            roles = new ControlLoopRole[loops.size()];
            ControlLoopRuntime[] runtimeArray = new ControlLoopRuntime[loops.size()];
            ControlLoopBinding loopBinding = new ControlLoopBinding(binding, context);
            for (int i = 0; i < loops.size(); i++) {
                ControlLoop loop = loops.get(i);
                roles[i] = loop.role();
                runtimeArray[i] = loop.bind(loopBinding);
            }
            runtimes = List.of(runtimeArray);
            config = closedLoopConfig(binding);
        }

        private boolean requestChanged(Class<?> type, double value) {
            return !hasLastRequest
                    || lastRequestType != type
                    || Double.compare(lastRequestValue, value) != 0;
        }

        private MotorClosedLoopRequest closedLoopRequest(ControlRoute route, double feedforwardVolts) {
            if (cachedRequest != null
                    && cachedRequestRoute == route
                    && Double.compare(cachedRequestFeedforwardVolts, feedforwardVolts) == 0) {
                return cachedRequest;
            }
            cachedRequest = new MotorClosedLoopRequest(route, config, feedforwardVolts);
            cachedRequestRoute = route;
            cachedRequestFeedforwardVolts = feedforwardVolts;
            return cachedRequest;
        }
    }

    private static final class AppliedOutput {
        private Output output;
        private ControlRoute route;
        private MotorClosedLoopRequest closedLoopRequest;

        private AppliedOutput set(Output output, ControlRoute route, MotorClosedLoopRequest closedLoopRequest) {
            this.output = output;
            this.route = route;
            this.closedLoopRequest = closedLoopRequest;
            return this;
        }

        private Output output() {
            return output;
        }

        @SuppressWarnings("unused")
        private ControlRoute route() {
            return route;
        }

        private MotorClosedLoopRequest closedLoopRequest() {
            return closedLoopRequest;
        }
    }

    private static double outputValue(Output output) {
        if (output instanceof Output.Percent percent) {
            return percent.percent();
        }
        if (output instanceof Output.Voltage voltage) {
            return voltage.volts();
        }
        if (output instanceof Output.Position position) {
            return position.position();
        }
        if (output instanceof Output.Velocity velocity) {
            return velocity.velocity();
        }
        return 0.0;
    }
}
