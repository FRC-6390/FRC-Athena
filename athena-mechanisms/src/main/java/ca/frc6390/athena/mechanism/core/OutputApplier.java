package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.ControlRoute;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.signal.MotorCommandSnapshot;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.motion.MotionProfileRuntime;
import ca.frc6390.athena.mechanism.motion.MotionReference;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.ArrayList;

/**
 * Applies resolved mechanism outputs to runtime hardware handles.
 */
final class OutputApplier {
    private final ActionContext context;
    private final Map<ControlBinding, ControlRuntimeState> controlRuntimes = new IdentityHashMap<>();
    private final Set<ControlBinding> controlsAppliedThisCycle =
            java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<MotorHandle> drivenMotors = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final AppliedOutput appliedOutput = new AppliedOutput();
    private final List<MechanismTraceSnapshot.Control> controlTraces = new ArrayList<>();
    private final Map<MotorDevice, AppliedMotorCommand> appliedMotorCommands = new IdentityHashMap<>();

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
        if (output.request().control() != null) {
            controlsAppliedThisCycle.add(output.request().control());
        }
        AppliedOutput applied = RuntimeHardwareAccess.call(context, () -> resolveControlOutput(
                output,
                mechanismContext == null ? MechanismContext.empty() : mechanismContext));
        if (output.request().control() != null) {
            ControlBinding control = output.request().control();
            controlTraces.add(applied.trace(control, controlRuntimes.get(control)));
        }
        if (output.output() instanceof Actions.ControlSysIdVoltage sysId) {
            double fallbackVoltage = applied.output() instanceof Output.Voltage voltage ? voltage.volts() : 0.0;
            RuntimeHardwareAccess.call(context, () -> {
                sysId.routine().record(context, sysId.state(), fallbackVoltage);
                return null;
            });
        }
        for (MotorDevice motor : motors(output.request())) {
            MotorHandle handle = context.motor(motor);
            drivenMotors.add(handle);
            apply(handle, applied);
            motor.recordCommand(command(applied.output()));
            appliedMotorCommands.put(motor, new AppliedMotorCommand(mode(applied.output()), value(applied.output())));
        }
    }

    void stopAll() {
        drivenMotors.forEach(MotorHandle::stop);
        appliedMotorCommands.keySet().forEach(motor -> motor.recordCommand(MotorCommandSnapshot.neutral()));
        resetControls();
    }

    void resetControls() {
        controlRuntimes.clear();
        controlsAppliedThisCycle.clear();
    }

    void beginCycle() {
        controlsAppliedThisCycle.clear();
        controlTraces.clear();
        appliedMotorCommands.clear();
    }

    void endCycle() {
        controlRuntimes.keySet().removeIf(control -> !controlsAppliedThisCycle.contains(control));
        controlsAppliedThisCycle.clear();
    }

    List<MechanismTraceSnapshot.Control> controlTraces() {
        return List.copyOf(controlTraces);
    }

    AppliedMotorCommand appliedMotorCommand(MotorDevice motor) {
        return appliedMotorCommands.get(motor);
    }

    private AppliedOutput resolveControlOutput(ResolvedOutput output, MechanismContext mechanismContext) {
        OutputRequest request = output.request();
        ControlBinding control = request.control();
        if (control == null) {
            return appliedOutput.set(output.output(), ControlRoute.OPEN_LOOP, null).withoutTrace();
        }
        ControlRuntimeState state = controlRuntimes(control);
        double position = firstFeedbackPosition(control);
        double velocity = firstFeedbackVelocity(control);
        Output transformed = applyTargetTransforms(
                control,
                output.output(),
                mechanismContext,
                state,
                position,
                velocity);
        if (transformed instanceof Output.Neutral || transformed instanceof Output.Fault) {
            resetControlState(state, transformed, mechanismContext, position, velocity);
            return appliedOutput.set(Outputs.neutral(), ControlRoute.OPEN_LOOP, null)
                    .withTrace(output.output(), transformed, null, position, velocity, true, true, false);
        }
        StagedRequest staged = stageRequest(control, transformed, mechanismContext, state, position, velocity);
        if (staged == null) {
            resetControlState(state, Outputs.neutral(), mechanismContext, position, velocity);
            return appliedOutput.set(Outputs.neutral(), ControlRoute.OPEN_LOOP, null)
                    .withTrace(output.output(), transformed, null, position, velocity, false, true, false);
        }

        ControlLoopContext loopContext = new AppliedControlLoopContext(
                control,
                staged.output(),
                mechanismContext,
                staged.reference());
        Class<?> requestType = staged.output().getClass();
        if (state.requestTypeChanged(requestType)) {
            resetLoops(state, loopContext);
            state.lastRequestType = requestType;
            state.hasLastRequest = true;
        }
        AppliedOutput offloaded = resolveDeviceControl(control, staged.output(), loopContext, state);
        if (offloaded != null) {
            return offloaded.withTrace(
                    output.output(), transformed, staged, position, velocity, staged.constrained(), false, false);
        }
        Output applied = null;
        if (staged.output() instanceof Output.Position || staged.output() instanceof Output.Velocity) {
            for (int index = 0; index < state.runtimes.size(); index++) {
                if (state.roles[index] == ControlLoopRole.TARGET_TRANSFORM) {
                    continue;
                }
                ControlOutput controlOutput = state.runtimes.get(index).calculate(loopContext);
                if (controlOutput != null) {
                    applied = combine(applied, controlOutput.output());
                }
            }
        }
        Output resolved = applied == null ? staged.output() : applied;
        if (!isFinite(resolved)) {
            resetControlState(state, Outputs.neutral(), mechanismContext, position, velocity);
            return appliedOutput.set(Outputs.neutral(), ControlRoute.OPEN_LOOP, null)
                    .withTrace(output.output(), transformed, staged, position, velocity, staged.constrained(), true, false);
        }
        Output saturated = saturate(resolved);
        notifyApplied(state, loopContext, resolved, saturated);
        Output guarded = guardFinalOutput(
                control,
                saturated,
                mechanismContext,
                position,
                velocity);
        if (guarded instanceof Output.Neutral && !(saturated instanceof Output.Neutral)) {
            resetControlState(state, guarded, mechanismContext, position, 0.0);
        }
        return appliedOutput.set(guarded, ControlRoute.OPEN_LOOP, null)
                .withTrace(
                        output.output(),
                        transformed,
                        staged,
                        position,
                        velocity,
                        staged.constrained(),
                        guarded instanceof Output.Neutral && !(saturated instanceof Output.Neutral),
                        !sameOutput(resolved, saturated));
    }

    private Output applyTargetTransforms(
            ControlBinding control,
            Output request,
            MechanismContext mechanismContext,
            ControlRuntimeState state,
            double position,
            double velocity) {
        if (!(request instanceof Output.Position || request instanceof Output.Velocity)) {
            return request;
        }
        Output transformed = request;
        for (int index = 0; index < state.runtimes.size(); index++) {
            if (state.roles[index] != ControlLoopRole.TARGET_TRANSFORM) {
                continue;
            }
            MotionReference reference = referenceFor(transformed, position, velocity);
            ControlOutput result = state.runtimes.get(index).calculate(new AppliedControlLoopContext(
                    control,
                    transformed,
                    mechanismContext,
                    reference));
            if (result == null) {
                continue;
            }
            Output next = result.output();
            if ((transformed instanceof Output.Position && next instanceof Output.Position)
                    || (transformed instanceof Output.Velocity && next instanceof Output.Velocity)) {
                transformed = next;
            } else {
                return Outputs.neutral();
            }
        }
        return transformed;
    }

    private StagedRequest stageRequest(
            ControlBinding control,
            Output request,
            MechanismContext mechanismContext,
            ControlRuntimeState state,
            double position,
            double velocity) {
        if (request instanceof Output.Position target) {
            if ((!control.constraints().isEmpty() || control.planner() != null)
                    && !Double.isFinite(position)) {
                return null;
            }
            if (control.profile() != null
                    && (!Double.isFinite(position) || !Double.isFinite(velocity))) {
                return null;
            }
            ConstraintContext<Double> constraintContext = new ConstraintContext<>(
                    position,
                    target.position(),
                    mechanismContext);
            ConstraintResult<Double> result = control.planner() == null
                    ? Constraints.evaluate(control.constraints(), constraintContext)
                    : control.planner().plan(constraintContext, control.constraints());
            if (!result.accepted()) {
                return null;
            }
            double goal = result.value();
            if (control.profile() == null) {
                return new StagedRequest(
                        Outputs.position(goal),
                        MotionReference.stationary(goal),
                        goal,
                        Double.compare(goal, target.position()) != 0);
            }
            MotionReference reference = state.profileRuntime.step(
                    position,
                    velocity,
                    goal,
                    mechanismContext.dtSeconds());
            return new StagedRequest(
                    Outputs.position(reference.position()),
                    reference,
                    goal,
                    Double.compare(goal, target.position()) != 0);
        }
        if (request instanceof Output.Velocity target) {
            if (!control.constraints().isEmpty() && !Double.isFinite(position)) {
                return null;
            }
            if (!allowsDirection(control, position, Math.signum(target.velocity()), mechanismContext)) {
                return null;
            }
            return new StagedRequest(
                    request,
                    new MotionReference(finiteOrZero(position), target.velocity(), 0.0),
                    target.velocity(),
                    false);
        }
        if (request instanceof Output.Percent target) {
            if (!control.constraints().isEmpty() && !Double.isFinite(position)) {
                return null;
            }
            return allowsDirection(control, position, Math.signum(target.percent()), mechanismContext)
                    ? new StagedRequest(
                            request,
                            new MotionReference(finiteOrZero(position), finiteOrZero(velocity), 0.0),
                            target.percent(),
                            false)
                    : null;
        }
        if (request instanceof Output.Voltage target) {
            if (!control.constraints().isEmpty() && !Double.isFinite(position)) {
                return null;
            }
            return allowsDirection(control, position, Math.signum(target.volts()), mechanismContext)
                    ? new StagedRequest(
                            request,
                            new MotionReference(finiteOrZero(position), finiteOrZero(velocity), 0.0),
                            target.volts(),
                            false)
                    : null;
        }
        return new StagedRequest(request, MotionReference.stationary(finiteOrZero(position)), 0.0, false);
    }

    private ControlRuntimeState controlRuntimes(ControlBinding control) {
        return controlRuntimes.computeIfAbsent(control, ControlRuntimeState::new);
    }

    private static MotionReference referenceFor(Output output, double position, double velocity) {
        if (output instanceof Output.Position target) {
            return MotionReference.stationary(target.position());
        }
        if (output instanceof Output.Velocity target) {
            return new MotionReference(finiteOrZero(position), target.velocity(), 0.0);
        }
        return new MotionReference(finiteOrZero(position), finiteOrZero(velocity), 0.0);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private boolean allowsDirection(
            ControlBinding control,
            double position,
            double direction,
            MechanismContext context) {
        if (direction == 0.0 || control.constraints().isEmpty()) {
            return true;
        }
        ConstraintResult<Double> result = Constraints.evaluate(
                control.constraints(),
                new ConstraintContext<>(position, position + direction, context));
        if (!result.accepted()) {
            return false;
        }
        return direction > 0.0 ? result.value() > position : result.value() < position;
    }

    private Output guardFinalOutput(
            ControlBinding control,
            Output output,
            MechanismContext context,
            double position,
            double velocity) {
        double direction = outputDirection(output);
        if (direction == 0.0 || control.constraints().isEmpty()) {
            return output;
        }
        if (!allowsDirection(control, position, direction, context)) {
            return Outputs.neutral();
        }
        if (control.profile() == null || velocity == 0.0 || Math.signum(velocity) != direction) {
            return output;
        }
        double reactionSeconds = Math.max(0.0, context.dtSeconds());
        double stoppingDistance = velocity * velocity / (2.0 * control.profile().maxAcceleration())
                + Math.abs(velocity) * reactionSeconds;
        double stoppingPosition = position + Math.copySign(stoppingDistance, velocity);
        ConstraintResult<Double> stopping = Constraints.evaluate(
                control.constraints(),
                new ConstraintContext<>(position, stoppingPosition, context));
        if (!stopping.accepted()
                || stopping instanceof ConstraintResult.Corrected<?>) {
            return Outputs.neutral();
        }
        return output;
    }

    private static double outputDirection(Output output) {
        if (output instanceof Output.Percent percent) {
            return Math.signum(percent.percent());
        }
        if (output instanceof Output.Voltage voltage) {
            return Math.signum(voltage.volts());
        }
        if (output instanceof Output.Velocity velocity) {
            return Math.signum(velocity.velocity());
        }
        return 0.0;
    }

    private static Output saturate(Output output) {
        if (output instanceof Output.Percent percent) {
            return Outputs.percent(Math.max(-1.0, Math.min(1.0, percent.percent())));
        }
        if (output instanceof Output.Voltage voltage) {
            return Outputs.voltage(Math.max(-12.0, Math.min(12.0, voltage.volts())));
        }
        return output;
    }

    private static boolean isFinite(Output output) {
        if (output instanceof Output.Percent percent) {
            return Double.isFinite(percent.percent());
        }
        if (output instanceof Output.Voltage voltage) {
            return Double.isFinite(voltage.volts());
        }
        if (output instanceof Output.Position position) {
            return Double.isFinite(position.position());
        }
        if (output instanceof Output.Velocity velocity) {
            return Double.isFinite(velocity.velocity());
        }
        return true;
    }

    private static boolean sameOutput(Output first, Output second) {
        return mode(first).equals(mode(second)) && Double.compare(value(first), value(second)) == 0;
    }

    private static String mode(Output output) {
        if (output instanceof Output.Percent) return "percent";
        if (output instanceof Output.Voltage) return "voltage";
        if (output instanceof Output.Position) return "position";
        if (output instanceof Output.Velocity) return "velocity";
        if (output instanceof Output.Fault) return "fault";
        return "neutral";
    }

    private static double value(Output output) {
        if (output instanceof Output.Percent percent) return percent.percent();
        if (output instanceof Output.Voltage voltage) return voltage.volts();
        if (output instanceof Output.Position position) return position.position();
        if (output instanceof Output.Velocity velocity) return velocity.velocity();
        return 0.0;
    }

    private static void resetLoops(ControlRuntimeState state, ControlLoopContext context) {
        for (int index = 0; index < state.runtimes.size(); index++) {
            if (state.roles[index] != ControlLoopRole.TARGET_TRANSFORM) {
                state.runtimes.get(index).reset(context);
            }
        }
    }

    private static void notifyApplied(
            ControlRuntimeState state,
            ControlLoopContext context,
            Output requested,
            Output applied) {
        for (int index = 0; index < state.runtimes.size(); index++) {
            if (state.roles[index] != ControlLoopRole.TARGET_TRANSFORM) {
                state.runtimes.get(index).applied(context, requested, applied);
            }
        }
    }

    private void resetControlState(
            ControlRuntimeState state,
            Output request,
            MechanismContext mechanismContext,
            double position,
            double velocity) {
        double safePosition = finiteOrZero(position);
        double safeVelocity = finiteOrZero(velocity);
        resetLoops(state, new AppliedControlLoopContext(
                state.binding,
                request,
                mechanismContext,
                new MotionReference(safePosition, safeVelocity, 0.0)));
        state.hasLastRequest = false;
        state.lastRequestType = null;
        if (state.profileRuntime != null) {
            state.profileRuntime.reset(safePosition, safeVelocity);
        }
    }

    private AppliedOutput resolveDeviceControl(
            ControlBinding control,
            Output request,
            ControlLoopContext loopContext,
            ControlRuntimeState state) {
        if (!(request instanceof Output.Position || request instanceof Output.Velocity) || control.output() == null) {
            return null;
        }
        if (!control.constraints().isEmpty() || control.profile() != null || control.planner() != null) {
            return null;
        }
        MotorClosedLoopConfig config = closedLoopConfig(control);
        if (config == null) {
            return null;
        }
        // Athena's iZone is enforced by the software PID runtime. Do not silently
        // offload it to a controller adapter that only accepts the basic PID terms.
        if (config.iZone() > 0.0) {
            return null;
        }
        if (request instanceof Output.Position && config.staticFeedforward() != 0.0) {
            return null;
        }
        if (!usesDeviceNativeFeedback(control)) {
            return null;
        }
        MotorHandle motor = context.motor(control.output());
        boolean allSupportArbitraryFeedforward = true;
        for (MotorDevice device : control.motors()) {
            MotorControlCapabilities capabilities = context.motor(device).controlCapabilities();
            if (request instanceof Output.Position && !capabilities.supportsPosition()
                    || request instanceof Output.Velocity && !capabilities.supportsVelocity()
                    || !capabilities.supportsSlot(config.slot())) {
                return null;
            }
            allSupportArbitraryFeedforward &= capabilities.arbitraryVoltageFeedforward();
        }
        MotorControlCapabilities capabilities = motor.controlCapabilities();
        boolean hybrid = false;
        double arbitraryFeedforwardVolts = 0.0;
        for (int i = 0; i < state.runtimes.size(); i++) {
            ControlLoopRole role = state.roles[i];
            if (role == ControlLoopRole.DEVICE_CONFIGURABLE || role == ControlLoopRole.TARGET_TRANSFORM) {
                continue;
            }
            if (role != ControlLoopRole.ARBITRARY_FEEDFORWARD
                    || !capabilities.arbitraryVoltageFeedforward()
                    || !allSupportArbitraryFeedforward) {
                return null;
            }
            ControlOutput controlOutput = state.runtimes.get(i).calculate(loopContext);
            if (controlOutput != null) {
                arbitraryFeedforwardVolts += volts(controlOutput.output());
                hybrid = true;
            }
        }
        ControlRoute route = hybrid ? ControlRoute.HYBRID_CLOSED_LOOP : ControlRoute.DEVICE_CLOSED_LOOP;
        MotorClosedLoopRequest closedLoopRequest = state.closedLoopRequest(
                route, config, arbitraryFeedforwardVolts);
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

    private static MotorCommandSnapshot command(Output output) {
        if (output instanceof Output.Percent percent) {
            return new MotorCommandSnapshot(MotorCommandSnapshot.Mode.PERCENT, percent.percent());
        }
        if (output instanceof Output.Voltage voltage) {
            return new MotorCommandSnapshot(MotorCommandSnapshot.Mode.VOLTAGE, voltage.volts());
        }
        if (output instanceof Output.Position position) {
            return new MotorCommandSnapshot(MotorCommandSnapshot.Mode.POSITION, position.position());
        }
        if (output instanceof Output.Velocity velocity) {
            return new MotorCommandSnapshot(MotorCommandSnapshot.Mode.VELOCITY, velocity.velocity());
        }
        return MotorCommandSnapshot.neutral();
    }

    private static boolean usesDeviceNativeFeedback(ControlBinding control) {
        FeedbackBinding feedback = control.feedback();
        if (feedback == null) {
            return true;
        }
        if (!(feedback.position() instanceof EncoderDevice position)
                || !(feedback.velocity() instanceof EncoderDevice velocity)
                || !position.equals(velocity)) {
            return false;
        }
        return isNativeFeedback(control.output(), position);
    }

    private static boolean isNativeFeedback(MotorDevice output, EncoderDevice feedback) {
        if (output == null || feedback == null || feedback.isInverted()
                || feedback.gearRatio() != 1.0 || feedback.conversion() != 1.0
                || feedback.offset() != 0.0 || feedback.units() != EncoderUnit.RAW) {
            return false;
        }
        return feedback.equals(output.encoder());
    }

    private static Output combine(Output current, Output next) {
        if (next == null) {
            return current;
        }
        if (current == null) {
            return next;
        }
        if (current instanceof Output.Neutral || current instanceof Output.Fault) {
            return current;
        }
        if (next instanceof Output.Neutral || next instanceof Output.Fault) {
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
                pid == null || pid.isDisabled() ? 0.0 : pid.p(),
                pid == null || pid.isDisabled() ? 0.0 : pid.i(),
                pid == null || pid.isDisabled() ? 0.0 : pid.d(),
                pid == null || pid.isDisabled() ? 0.0 : pid.iZone(),
                feedforward == null || feedforward.isDisabled() ? 0.0 : feedforward.staticGain(),
                feedforward == null || feedforward.isDisabled() ? 0.0 : feedforward.velocityGain(),
                feedforward == null || feedforward.isDisabled() ? 0.0 : feedforward.accelerationGain(),
                feedforward == null || feedforward.isDisabled() ? 0.0 : feedforward.gravityGain(),
                null);
    }

    private final class AppliedControlLoopContext implements ControlLoopContext {
        private final ControlBinding control;
        private final Output request;
        private final MechanismContext mechanismContext;
        private final MotionReference reference;

        private AppliedControlLoopContext(
                ControlBinding control,
                Output request,
                MechanismContext mechanismContext,
                MotionReference reference) {
            this.control = control;
            this.request = request;
            this.mechanismContext = mechanismContext;
            this.reference = reference;
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
        public MotionReference reference() {
            return reference;
        }

        @Override
        public double dtSeconds() {
            return mechanismContext.dtSeconds();
        }

    }

    private double firstFeedbackPosition(ControlBinding control) {
        if (control.feedback() != null) {
            return readOrNaN(control.feedback().position()::position);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrNaN(motor::integratedPositionRotations);
        }
        return 0.0;
    }

    private double firstFeedbackVelocity(ControlBinding control) {
        if (control.feedback() != null) {
            return readOrNaN(control.feedback().velocity()::velocity);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrNaN(motor::integratedVelocityRotationsPerSecond);
        }
        return 0.0;
    }

    private static double readOrNaN(DoubleSupplier read) {
        try {
            return read.getAsDouble();
        } catch (UnsupportedOperationException ignored) {
            return Double.NaN;
        }
    }

    private interface DoubleSupplier {
        double getAsDouble();
    }

    private final class ControlRuntimeState {
        private final ControlBinding binding;
        private final List<ControlLoopRuntime> runtimes;
        private final ControlLoopRole[] roles;
        private final MotionProfileRuntime profileRuntime;
        private boolean hasLastRequest;
        private Class<?> lastRequestType;
        private ControlRoute cachedRequestRoute;
        private double cachedRequestFeedforwardVolts;
        private MotorClosedLoopConfig cachedConfig;
        private MotorClosedLoopRequest cachedRequest;

        private ControlRuntimeState(ControlBinding binding) {
            this.binding = binding;
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
            profileRuntime = binding.profile() == null ? null : new MotionProfileRuntime(binding.profile());
        }

        private boolean requestTypeChanged(Class<?> type) {
            return !hasLastRequest || lastRequestType != type;
        }

        private MotorClosedLoopRequest closedLoopRequest(
                ControlRoute route, MotorClosedLoopConfig config, double feedforwardVolts) {
            if (cachedRequest != null
                    && cachedRequestRoute == route
                    && Objects.equals(cachedConfig, config)
                    && Double.compare(cachedRequestFeedforwardVolts, feedforwardVolts) == 0) {
                return cachedRequest;
            }
            cachedRequest = new MotorClosedLoopRequest(route, config, feedforwardVolts);
            cachedRequestRoute = route;
            cachedConfig = config;
            cachedRequestFeedforwardVolts = feedforwardVolts;
            return cachedRequest;
        }
    }

    private static final class AppliedOutput {
        private Output output;
        private ControlRoute route;
        private MotorClosedLoopRequest closedLoopRequest;
        private Output requested;
        private Output transformed;
        private StagedRequest staged;
        private double measuredPosition;
        private double measuredVelocity;
        private boolean constrained;
        private boolean blocked;
        private boolean saturated;

        private AppliedOutput set(Output output, ControlRoute route, MotorClosedLoopRequest closedLoopRequest) {
            this.output = output;
            this.route = route;
            this.closedLoopRequest = closedLoopRequest;
            return this;
        }

        private AppliedOutput withoutTrace() {
            requested = null;
            transformed = null;
            staged = null;
            measuredPosition = Double.NaN;
            measuredVelocity = Double.NaN;
            constrained = false;
            blocked = false;
            saturated = false;
            return this;
        }

        private AppliedOutput withTrace(
                Output requested,
                Output transformed,
                StagedRequest staged,
                double measuredPosition,
                double measuredVelocity,
                boolean constrained,
                boolean blocked,
                boolean saturated) {
            this.requested = requested;
            this.transformed = transformed;
            this.staged = staged;
            this.measuredPosition = measuredPosition;
            this.measuredVelocity = measuredVelocity;
            this.constrained = constrained;
            this.blocked = blocked;
            this.saturated = saturated;
            return this;
        }

        private MechanismTraceSnapshot.Control trace(ControlBinding control, ControlRuntimeState state) {
            MotionReference reference = staged == null
                    ? MotionReference.stationary(Double.isFinite(measuredPosition) ? measuredPosition : 0.0)
                    : staged.reference();
            double requestedValue = value(requested);
            double transformedValue = value(transformed);
            double goal = staged == null ? transformedValue : staged.goal();
            double measurement = requested instanceof Output.Velocity ? measuredVelocity : measuredPosition;
            ControlLoopTrace loopTrace = ControlLoopTrace.ZERO;
            if (state != null) {
                for (ControlLoopRuntime runtime : state.runtimes) {
                    loopTrace = loopTrace.plus(runtime.trace());
                }
            }
            return new MechanismTraceSnapshot.Control(
                    control.output() == null ? "control" : control.output().defaultName(),
                    mode(requested),
                    requestedValue,
                    transformedValue,
                    goal,
                    reference.position(),
                    reference.velocity(),
                    reference.acceleration(),
                    measuredPosition,
                    measuredVelocity,
                    goal - measurement,
                    loopTrace.proportionalVolts(),
                    loopTrace.integralVolts(),
                    loopTrace.derivativeVolts(),
                    loopTrace.staticFeedforwardVolts(),
                    loopTrace.velocityFeedforwardVolts(),
                    loopTrace.accelerationFeedforwardVolts(),
                    loopTrace.gravityFeedforwardVolts(),
                    mode(output),
                    value(output),
                    route == null ? "" : route.name(),
                    constrained,
                    blocked,
                    saturated);
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

    private record StagedRequest(Output output, MotionReference reference, double goal, boolean constrained) {
        private StagedRequest {
            Objects.requireNonNull(output, "output");
            Objects.requireNonNull(reference, "reference");
        }
    }

    record AppliedMotorCommand(String mode, double value) {
    }

}
