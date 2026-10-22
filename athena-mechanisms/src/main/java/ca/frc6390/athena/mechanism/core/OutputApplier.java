package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;
import ca.frc6390.athena.hardware.backend.ControlRoute;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.hardware.signal.MotorCommandSnapshot;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.constraint.ConstraintStage;
import ca.frc6390.athena.mechanism.motion.MotionProfileRuntime;
import ca.frc6390.athena.mechanism.motion.MotionReference;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import ca.frc6390.athena.mechanism.staging.StageCaps;
import ca.frc6390.athena.mechanism.effort.EffortCarrier;
import ca.frc6390.athena.mechanism.effort.EffortRuntime;
import ca.frc6390.athena.mechanism.effort.EffortUnits;
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
    private final RuntimeOverrides overrides;
    private final Map<ControlBinding<?>, ControlRuntimeState> controlRuntimes = new IdentityHashMap<>();
    /** The values of {@link #controlRuntimes} in creation order, iterated without allocating entries. */
    private final List<ControlRuntimeState> controlStates = new ArrayList<>();
    /** Reused motor and limit lists of {@link #followerLimits} and {@link #limitOpenLoop}. */
    private final List<MotorDevice> followerDevicesScratch = new ArrayList<>();
    private double[] followerAmpsScratch = new double[4];
    private MotorHandle[] followerHandlesScratch = new MotorHandle[4];
    private final List<MotorDevice> openLoopMotorsScratch = new ArrayList<>();
    private double[] openLoopLimitsScratch = new double[4];
    private MotorHandle[] openLoopHandlesScratch = new MotorHandle[4];
    /** Power pass of the mechanism; its per-motor caps join the effort clamp. Null before the first cycle. */
    private PowerRuntime electricalPass;
    private final Set<ControlBinding<?>> controlsAppliedThisCycle =
            java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<MotorHandle> drivenMotors = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final AppliedOutput appliedOutput = new AppliedOutput();
    private final List<MechanismTraceSnapshot.Control> controlTraces = new ArrayList<>();
    private final Map<MotorDevice, AppliedMotorCommand> appliedMotorCommands = new IdentityHashMap<>();
    private final Set<MotorHandle> appliedHandles = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<ca.frc6390.athena.runtime.control.ControlSink> activeSinks =
            java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<MotorDevice> heldMotors = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    /** Motors whose neutral mode is overridden by a {@code Release.neutral(mode)} until next commanded. */
    private final Set<MotorDevice> neutralOverrides = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    /** Motors commanded by any output in the current cycle (used to tell an idle gap from a takeover). */
    private final Set<MotorDevice> motorsDrivenThisCycle = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private boolean captureTrace;
    private Map<ControlBinding<?>, StageCaps> stageCaps = java.util.Collections.emptyMap();
    private Map<ControlBinding<?>, Enum<?>> stageNames = java.util.Collections.emptyMap();
    private String blockedBy = "";
    private String guardEdge = "";
    private Enum<?> activeStage;
    private boolean applyingHold;
    /** Stable command slots survive the robot-wide prepare/allocate/commit boundary without recalculating loops. */
    private final List<PreparedOutput> preparedPool = new ArrayList<>();
    private int preparedCount;
    private boolean preparing;
    private boolean committingPrepared;
    private boolean preparedPowerValidated;
    private final List<Runnable> deferredReleases = new ArrayList<>();

    private OutputApplier(ActionContext context, RuntimeOverrides overrides) {
        this.context = Objects.requireNonNull(context, "context");
        this.overrides = Objects.requireNonNull(overrides, "overrides");
    }

    static OutputApplier using(ActionContext context) {
        return new OutputApplier(context, new RuntimeOverrides());
    }

    static OutputApplier using(ActionContext context, RuntimeOverrides overrides) {
        return new OutputApplier(context, overrides);
    }

    void captureTrace(boolean enabled) {
        captureTrace = enabled;
        if (!enabled) {
            controlTraces.clear();
            appliedMotorCommands.clear();
        }
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
        MechanismContext safeContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        AppliedOutput applied = RuntimeHardwareAccess.current() == context
                ? resolveControlOutput(output, safeContext)
                : RuntimeHardwareAccess.call(context, () -> resolveControlOutput(output, safeContext));
        List<MotorDevice> requestMotors = output.request().motors();
        for (int index = 0; index < requestMotors.size(); index++) {
            motorsDrivenThisCycle.add(requestMotors.get(index));
        }
        if (output.request().control() != null) {
            ControlRuntimeState recorded = controlRuntimes(output.request().control());
            recorded.lastRelease = releaseFor(output.request().control(), output.request().motionPolicy());
            recorded.lastRetarget = retargetFor(output.request().control(), output.request().motionPolicy());
            recorded.holdable = (output.output() instanceof Output.Position
                    || output.output() instanceof Output.Velocity)
                    && !(applied.output() instanceof Output.Neutral);
            if (!applyingHold) {
                recorded.holdGoal = Double.NaN;
            }
        }
        if (preparing) {
            PreparedOutput prepared;
            if (preparedCount == preparedPool.size()) {
                prepared = new PreparedOutput();
                preparedPool.add(prepared);
            } else {
                prepared = preparedPool.get(preparedCount);
            }
            prepared.output = output;
            prepared.context = safeContext;
            prepared.valid = true;
            prepared.applied.copyFrom(applied);
            registerCommand(prepared);
            preparedCount++;
            return;
        }
        commit(output, applied, safeContext);
    }

    /** Starts a cycle whose hardware writes wait until every mechanism has contributed its demand. */
    void beginPreparation() {
        beginCycle();
        preparedCount = 0;
        deferredReleases.clear();
        preparing = true;
        preparedPowerValidated = false;
    }

    /** Validates allocated directions for every output before any mechanism commits motor commands. */
    void validatePreparedPower() {
        if (preparedPowerValidated) return;
        for (int index = 0; index < preparedCount; index++) {
            PreparedOutput prepared = preparedPool.get(index);
            if (!prepared.valid) continue;
            AppliedOutput applied = prepared.applied;
            Output finalOutput = applied.output();
            if (electricalPass != null) {
                if (electricalPass.commandSuppressed(prepared)) {
                    finalOutput = Outputs.neutral();
                    applied.blocked = true;
                } else if (Double.isFinite(prepared.candidateVolts)) {
                    double planned = electricalPass.plannedVoltage(prepared, prepared.candidateVolts);
                    finalOutput = planned == prepared.candidateVolts ? applied.output() : Outputs.voltage(planned);
                    ControlBinding<?> control = prepared.output.request().control();
                    if (control != null && !sameOutput(finalOutput, applied.output())) {
                        Output guarded = guardFinalOutput(control, finalOutput, prepared.context,
                                applied.measuredPosition, applied.measuredVelocity);
                        if (guarded instanceof Output.Neutral && !(finalOutput instanceof Output.Neutral)) {
                            electricalPass.suppressCommandGroup(prepared, "Guard rejected allocated output");
                            applied.blocked = true;
                        }
                        finalOutput = guarded;
                    }
                }
            }
            prepared.plannedOutput = finalOutput;
        }
        preparedPowerValidated = true;
    }

    /** Applies cached controller results exactly once, after the robot-wide allocation is complete. */
    void commitPrepared() {
        validatePreparedPower();
        preparing = false;
        committingPrepared = true;
        appliedHandles.clear();
        try {
            for (int index = 0; index < deferredReleases.size(); index++) deferredReleases.get(index).run();
            for (int index = 0; index < preparedCount; index++) {
                PreparedOutput prepared = preparedPool.get(index);
                if (!prepared.valid) continue;
                AppliedOutput applied = prepared.applied;
                Output finalOutput = prepared.plannedOutput;
                if (electricalPass != null && electricalPass.commandSuppressed(prepared)) {
                    // A guard in a later mechanism can invalidate a group whose outputs were validated earlier.
                    finalOutput = Outputs.neutral();
                    applied.blocked = true;
                }
                applied.saturated |= !sameOutput(applied.output(), finalOutput);
                applied.output = finalOutput;
                commit(prepared.output, applied, prepared.context);
            }
        } finally {
            preparedCount = 0;
            deferredReleases.clear();
            committingPrepared = false;
        }
    }

    private void registerCommand(PreparedOutput prepared) {
        prepared.candidateVolts = scalarVolts(prepared.applied.output());
        OutputRequest request = prepared.output.request();
        if (electricalPass == null) return;
        if (!Double.isFinite(prepared.candidateVolts)) {
            ControlBinding<?> control = request.control();
            boolean actionLimit = prepared.output.output() instanceof EffortCarrier carrier
                    && Double.isFinite(carrier.effort().limitValue());
            if (control != null && !control.motors().isEmpty()
                    && (prepared.applied.output() instanceof Output.Position
                            || prepared.applied.output() instanceof Output.Velocity)
                    && (actionLimit || requiresHostPowerClamp(control))) {
                throw new IllegalStateException("Power-limited control for "
                        + control.output().defaultName()
                        + " requires a host PID/feedforward voltage loop; a device target cannot enforce its power allocation.");
            }
        }
        prepared.motors.clear();
        double actionLimit = prepared.output.output() instanceof EffortCarrier carrier
                ? carrier.effort().limitValue() : Double.POSITIVE_INFINITY;
        List<MotorDevice> roots = request.motors();
        ControlRuntimeState state = request.control() == null ? null : controlRuntimes(request.control());
        for (int index = 0; index < roots.size(); index++) {
            MotorDevice motor = roots.get(index);
            prepared.add(motor, context.motor(motor), state == null ? actionLimit : state.effort.activeLimitAmps(index));
            List<? extends ActionContext.MotorFollower> followers = context.followerConstraints(motor);
            for (int followerIndex = 0; followerIndex < followers.size(); followerIndex++) {
                ActionContext.MotorFollower follower = followers.get(followerIndex);
                prepared.add(follower.device(), follower.handle(), actionLimit);
            }
        }
        if (!prepared.motors.isEmpty()) {
            try {
                electricalPass.registerCommand(prepared, prepared.motors, prepared.handles,
                        prepared.candidateVolts, prepared.localAmps);
            } catch (RuntimeException exception) {
                electricalPass.discardCommand(prepared);
                prepared.valid = false;
                throw exception;
            }
        }
    }

    private static double scalarVolts(Output output) {
        return output instanceof Output.Voltage voltage ? voltage.volts()
                : output instanceof Output.Percent percent ? percent.percent() * EffortUnits.NOMINAL_VOLTS
                : Double.NaN;
    }

    private void commit(ResolvedOutput output, AppliedOutput applied, MechanismContext safeContext) {
        ControlBinding<?> appliedControl = output.request().control();
        if (appliedControl != null) {
            ControlRuntimeState state = controlRuntimes(appliedControl);
            state.lastVolts = applied.output() instanceof Output.Voltage volts ? volts.volts() : Double.NaN;
            state.effort.appliedVolts(scalarVolts(applied.output()));
            if (applied.notificationState != null) {
                notifyApplied(applied.notificationState, applied.notificationContext,
                        applied.notificationRequested, applied.output());
            }
        }
        if (output.request().control() != null) {
            ControlBinding<?> control = output.request().control();
            if (captureTrace) {
                controlTraces.add(applied.trace(control, controlRuntimes.get(control)));
            }
            recordSample(control, applied);
            if (control.sink() != null) {
                applySink(control.sink(), applied.output());
            }
        }
        if (!committingPrepared) appliedHandles.clear();
        ControlBinding<?> control = output.request().control();
        if (control != null) {
            List<MotorDevice> motors = control.motors();
            for (int index = 0; index < motors.size(); index++) {
                MotorDevice motor = motors.get(index);
                applyMotorAndSoftwareFollowers(motor, context.motor(motor), applied, appliedHandles);
            }
        } else if (output.request().motor() != null) {
            MotorDevice motor = output.request().motor();
            applyMotorAndSoftwareFollowers(motor, context.motor(motor), applied, appliedHandles);
        }
        if (output.output() instanceof CharacterizationRoutine.Voltage sysId) {
            double fallbackVoltage = applied.output() instanceof Output.Voltage voltage ? voltage.volts() : 0.0;
            RuntimeHardwareAccess.call(context, () -> {
                sysId.routine().record(context, sysId.state(), fallbackVoltage);
                return null;
            });
        }
    }

    private void applyMotorAndSoftwareFollowers(
            MotorDevice motor,
            MotorHandle handle,
            AppliedOutput applied,
            Set<MotorHandle> appliedHandles) {
        applyMotorAndSoftwareFollowers(motor, handle, applied, appliedHandles, true);
    }

    private void applyMotorAndSoftwareFollowers(MotorDevice motor, MotorHandle handle, AppliedOutput applied,
            Set<MotorHandle> appliedHandles, boolean mirrorFollowers) {
        MotorHandle target = handle.commandTarget();
        if (!appliedHandles.add(target)) {
            return;
        }
        if (committingPrepared && electricalPass != null && !electricalPass.claimCommandTarget(target)) return;
        drivenMotors.add(target);
        if (neutralOverrides.remove(motor)) {
            try {
                handle.applyRuntimeConfiguration(MotorRuntimeConfig.declared(motor));
            } catch (RuntimeException exception) {
                if (motor.failurePolicy() == FailurePolicy.PANIC) throw FaultIsolation.panic(exception);
                context.hardwareFailure(motor, exception);
            }
        }
        try {
            apply(handle, applied);
        } catch (RuntimeException exception) {
            if (motor.failurePolicy() == FailurePolicy.PANIC) {
                throw FaultIsolation.panic(exception);
            }
            context.hardwareFailure(motor, exception);
            motor.recordCommand(MotorCommandSnapshot.neutral());
            return;
        }
        recordCommand(motor, applied.output());
        if (captureTrace) {
            appliedMotorCommands.put(motor, new AppliedMotorCommand(mode(applied.output()), value(applied.output())));
        }
        if (!mirrorFollowers) return;
        List<? extends ActionContext.MotorFollower> followers = context.followerConstraints(motor);
        for (int index = 0; index < followers.size(); index++) {
            ActionContext.MotorFollower follower = followers.get(index);
            if (follower.softwareCommand()) {
                applyMotorAndSoftwareFollowers(follower.device(), follower.handle(), applied, appliedHandles, false);
            }
        }
    }

    void stopAll() {
        preparing = false;
        for (int index = 0; index < preparedCount; index++) discard(preparedPool.get(index));
        preparedCount = 0;
        deferredReleases.clear();
        drivenMotors.forEach(motor -> {
            try {
                motor.stop();
            } catch (RuntimeException exception) {
                if (motor.device().failurePolicy() == FailurePolicy.PANIC) throw FaultIsolation.panic(exception);
                context.hardwareFailure(motor.device(), exception);
            }
        });
        appliedMotorCommands.keySet().forEach(motor -> motor.recordCommand(MotorCommandSnapshot.neutral()));
        for (int index = 0; index < controlStates.size(); index++) {
            ControlBinding<?> control = controlStates.get(index).binding;
            if (control.sink() != null) control.sink().release();
        }
        resetControls();
    }

    void resetControls() {
        for (int index = 0; index < controlStates.size(); index++) {
            ControlRuntimeState state = controlStates.get(index);
            ControlBinding<?> control = state.binding;
            if (!state.active) {
                continue;
            }
            resetFromCurrentFeedback(control, state);
            state.active = false;
        }
        controlsAppliedThisCycle.clear();
    }

    /**
     * Neutralizes the hardware of an output whose application threw, so a faulted control never
     * keeps its previous command, and lets its control state reset at the end of the cycle.
     */
    void abandon(OutputRequest request) {
        for (int index = 0; index < preparedCount; index++) {
            PreparedOutput prepared = preparedPool.get(index);
            if (prepared.output.request() == request) discard(prepared);
        }
        ControlBinding<?> control = request.control();
        if (control != null) {
            controlsAppliedThisCycle.remove(control);
            if (control.sink() != null) {
                try {
                    control.sink().release();
                } catch (RuntimeException ignored) {
                    // Best effort while already handling a fault.
                }
            }
        }
        for (MotorDevice motor : request.motors()) {
            try {
                context.motor(motor).stop();
            } catch (RuntimeException ignored) {
                // Best effort while already handling a fault.
            }
            motor.recordCommand(MotorCommandSnapshot.neutral());
        }
    }

    private void discard(PreparedOutput prepared) {
        prepared.valid = false;
        if (electricalPass != null) electricalPass.discardCommand(prepared);
    }

    /**
     * Re-seeds profile and loop state for every control whose feedback is the encoder that was just
     * re-zeroed, so a running profiled/PID control does not chase a stale reference. The requested
     * position is used directly because cached hardware inputs may not reflect the new value yet.
     */
    void encoderPositionSet(EncoderDevice encoder, double position) {
        for (int index = 0; index < controlStates.size(); index++) {
            ControlRuntimeState state = controlStates.get(index);
            ControlBinding<?> control = state.binding;
            FeedbackBinding feedback = control.feedback();
            boolean direct = feedback != null && encoder.equals(feedback.position());
            boolean fedBy = direct
                    || feedback != null && feedback.dependencies().contains(encoder)
                    || feedback == null && control.output() != null && encoder.equals(control.output().encoder());
            if (!fedBy) continue;
            state.reseedPending = true;
            state.reseedPosition = direct && Double.isFinite(position) ? position : Double.NaN;
        }
    }

    /**
     * Installs the staging caps that apply in the coming cycle, and the stage that set each capped binding
     * (used for the {@code State/Reason} explanation).
     */
    /** Installs the power pass whose per-motor caps bound every voltage this applier commands (B6). */
    void electrical(PowerRuntime power) {
        electricalPass = power;
    }

    void stageCaps(Map<ControlBinding<?>, StageCaps> caps, Map<ControlBinding<?>, Enum<?>> stages) {
        stageCaps = caps == null ? java.util.Collections.emptyMap() : caps;
        stageNames = stages == null ? java.util.Collections.emptyMap() : stages;
    }

    /** Constraint kind that refused this cycle's request, or empty when none did. */
    String blockedBy() {
        return blockedBy;
    }

    /** Range edge ({@code max} or {@code min}) that clipped this cycle's output, or empty. */
    String guardEdge() {
        return guardEdge;
    }

    /** Stage constant that capped an applied output this cycle, or null. */
    Enum<?> activeStage() {
        return activeStage;
    }

    private StageCaps stageCapsFor(ControlBinding<?> control) {
        if (stageCaps.isEmpty()) return StageCaps.NONE;
        StageCaps caps = stageCaps.getOrDefault(control, StageCaps.NONE);
        if (!caps.equals(StageCaps.NONE)) {
            Enum<?> stage = stageNames.get(control);
            if (stage != null) activeStage = stage;
        }
        return caps;
    }

    /** Motors driven by held (released with HOLD or DECELERATE) controls in the last ended cycle. */
    Set<MotorDevice> heldMotors() {
        return heldMotors;
    }

    void beginCycle() {
        blockedBy = "";
        guardEdge = "";
        activeStage = null;
        heldMotors.clear();
        motorsDrivenThisCycle.clear();
        controlsAppliedThisCycle.clear();
        controlTraces.clear();
        appliedMotorCommands.clear();
    }

    void endCycle() {
        if (controlRuntimes.isEmpty()) {
            controlsAppliedThisCycle.clear();
            return;
        }
        activeSinks.clear();
        // Applied controls normally all have a runtime state; walking the states avoids an iterator.
        int appliedWithState = 0;
        for (int index = 0; index < controlStates.size(); index++) {
            ControlBinding<?> control = controlStates.get(index).binding;
            if (!controlsAppliedThisCycle.contains(control)) continue;
            appliedWithState++;
            if (control.sink() != null) activeSinks.add(control.sink());
        }
        if (appliedWithState != controlsAppliedThisCycle.size()) {
            for (ControlBinding<?> control : controlsAppliedThisCycle) {
                if (control.sink() != null) activeSinks.add(control.sink());
            }
        }
        for (int index = 0; index < controlStates.size(); index++) {
            ControlRuntimeState state = controlStates.get(index);
            ControlBinding<?> control = state.binding;
            boolean applied = controlsAppliedThisCycle.contains(control);
            if (!applied && state.active && holdsOnRelease(control, state)) {
                holdReleased(control, state);
                applied = true;
            } else if (!applied && state.active && state.lastRetarget == Retarget.SMOOTH
                    && !anyDriven(control.motors(), motorsDrivenThisCycle)) {
                // A SMOOTH binding keeps its reference through an idle gap, so a takeover in a later cycle
                // inherits it. A cycle in which another output drives the motor is a takeover by that
                // output, and the reference restarts from measured feedback. Loop state always restarts.
                softRelease(control, state);
                if (control.sink() != null && !activeSinks.contains(control.sink())) {
                    releaseSink(control.sink());
                }
                if (state.lastRelease.neutralMode() != null) {
                    overrideNeutralMode(control, state.lastRelease.neutralMode());
                }
            } else if (!applied && state.active) {
                resetFromCurrentFeedback(control, state);
                if (control.sink() != null && !activeSinks.contains(control.sink())) {
                    releaseSink(control.sink());
                }
                if (state.lastRelease.neutralMode() != null) {
                    overrideNeutralMode(control, state.lastRelease.neutralMode());
                }
            }
            state.active = applied;
            if (!applied) {
                overrides.controlSample(control).clear();
            }
        }
        controlsAppliedThisCycle.clear();
    }

    private static boolean anyDriven(List<MotorDevice> motors, Set<MotorDevice> driven) {
        for (int index = 0; index < motors.size(); index++) {
            if (driven.contains(motors.get(index))) return true;
        }
        return false;
    }

    private void softRelease(ControlBinding<?> control, ControlRuntimeState state) {
        MotionReference reference = state.lastReference != null
                ? state.lastReference
                : MotionReference.stationary(state.lastPosition);
        resetLoops(state, new AppliedControlLoopContext(
                control, Outputs.neutral(), state.lastContext, reference));
        state.hasLastRequest = false;
        state.lastRequestType = null;
        state.lastVolts = Double.NaN;
        state.holdGoal = Double.NaN;
    }

    private boolean holdsOnRelease(ControlBinding<?> control, ControlRuntimeState state) {
        if (!state.holdable || !state.lastRelease.holds()) return false;
        if (overrides.disabled(control)) return false;
        if (control.output() != null && overrides.disabled(control.output())) return false;
        List<MotorDevice> motors = control.motors();
        for (int index = 0; index < motors.size(); index++) {
            if (overrides.disabled(motors.get(index))) return false;
        }
        return true;
    }

    /**
     * Keeps a released closed-loop control under control without a lease. HOLD keeps the last reference
     * position (or zero velocity) and stops the profile immediately. DECELERATE sets the goal at the
     * braking distance of the current reference, so the profile stops there at its maximum deceleration.
     */
    private void holdReleased(ControlBinding<?> control, ControlRuntimeState state) {
        Release release = state.lastRelease;
        Output target;
        if (control.mode() == ControlMode.POSITION) {
            if (Double.isNaN(state.holdGoal)) {
                MotionReference reference = state.lastReference != null
                        ? state.lastReference
                        : MotionReference.stationary(state.lastPosition);
                MotionProfile profile = effectiveProfile(control);
                double goal = reference.position();
                if (profile != null && release.decelerates()) {
                    double velocity = reference.velocity();
                    goal += Math.copySign(velocity * velocity / (2.0 * profile.maxAcceleration()), velocity);
                } else if (profile != null) {
                    state.profileRuntime(profile).reset(reference.position(), 0.0);
                }
                state.holdGoal = clampTunedPosition(overrides.tuning(control), goal);
            }
            target = Outputs.position(state.holdGoal);
        } else {
            if (!release.decelerates()) {
                state.velocityReference = 0.0;
                state.velocityReferenceInitialized = true;
            }
            target = Outputs.velocity(0.0);
        }
        heldMotors.addAll(control.motors());
        ResolvedOutput hold = new ResolvedOutput(
                OutputRequest.Basic.of(control, null, target, new MotionPolicy(null, release)), target);
        applyingHold = true;
        try {
            apply(hold, state.lastContext);
        } finally {
            applyingHold = false;
        }
    }

    /**
     * Applies a temporary neutral mode to the control's motors. The declared configuration returns the
     * next time one of these motors is commanded. Backends without runtime configuration keep their mode.
     */
    private void overrideNeutralMode(ControlBinding<?> control, MotorNeutralMode mode) {
        if (preparing) {
            deferredReleases.add(() -> overrideNeutralMode(control, mode));
            return;
        }
        for (MotorDevice motor : control.motors()) {
            MotorHandle handle = context.motor(motor);
            if (!handle.supportsRuntimeConfiguration()) continue;
            MotorRuntimeConfig declared = MotorRuntimeConfig.declared(motor);
            try {
                handle.applyRuntimeConfiguration(new MotorRuntimeConfig(
                        mode,
                        declared.inverted(),
                        declared.supplyCurrentLimitAmps(),
                        declared.statorCurrentLimitAmps()));
                neutralOverrides.add(motor);
            } catch (RuntimeException exception) {
                if (motor.failurePolicy() == FailurePolicy.PANIC) throw FaultIsolation.panic(exception);
                context.hardwareFailure(motor, exception);
            }
        }
    }

    private void releaseSink(ca.frc6390.athena.runtime.control.ControlSink sink) {
        if (preparing) deferredReleases.add(sink::release);
        else sink.release();
    }

    private static Release releaseFor(ControlBinding<?> control, MotionPolicy policy) {
        return policy.release() != null ? policy.release() : control.release();
    }

    private static Retarget retargetFor(ControlBinding<?> control, MotionPolicy policy) {
        return policy.retarget() != null ? policy.retarget() : control.retarget();
    }

    private void resetFromCurrentFeedback(ControlBinding<?> control, ControlRuntimeState state) {
        RuntimeHardwareAccess.call(context, () -> {
            double position = firstFeedbackPosition(control);
            double velocity = firstFeedbackVelocity(control);
            state.lastPosition = finiteOrZero(position);
            state.lastVelocity = finiteOrZero(velocity);
            resetControlState(
                    state,
                    Outputs.neutral(),
                    state.lastContext,
                    state.lastPosition,
                    state.lastVelocity);
            return null;
        });
    }

    List<MechanismTraceSnapshot.Control> controlTraces() {
        return List.copyOf(controlTraces);
    }

    AppliedMotorCommand appliedMotorCommand(MotorDevice motor) {
        return appliedMotorCommands.get(motor);
    }

    private AppliedOutput resolveControlOutput(ResolvedOutput output, MechanismContext mechanismContext) {
        OutputRequest request = output.request();
        ControlBinding<?> control = request.control();
        if (control == null) {
            return appliedOutput.set(limitOpenLoop(request, output.output()), ControlRoute.OPEN_LOOP, null)
                    .withoutTrace();
        }
        ControlRuntimeState state = controlRuntimes(control);
        state.unappliedCapAmps = -1;
        double position = firstFeedbackPosition(control);
        double velocity = firstFeedbackVelocity(control);
        if (state.reseedPending) {
            state.reseedPending = false;
            if (Double.isFinite(state.reseedPosition)) position = state.reseedPosition;
            resetControlState(state, Outputs.neutral(), mechanismContext, position, velocity);
        }
        state.lastContext = mechanismContext;
        state.lastPosition = finiteOrZero(position);
        state.lastVelocity = finiteOrZero(velocity);
        Output transformed = applyTargetTransforms(
                control,
                output.output(),
                mechanismContext,
                state,
                position,
                velocity);
        transformed = applyStageSpeed(control, transformed);
        if (transformed instanceof Output.Neutral || transformed instanceof Output.Fault) {
            resetControlState(state, transformed, mechanismContext, position, velocity);
            return appliedOutput.set(Outputs.neutral(), ControlRoute.OPEN_LOOP, null)
                    .withTrace(output.output(), transformed, null, position, velocity, true, true, false);
        }
        // A change of request kind is a takeover: SMOOTH inherits the reference, IMMEDIATE re-plans
        // from the measured state. Decided before staging so the profile is not stepped from a stale state.
        Class<?> requestType = requestKind(transformed);
        boolean typeChanged = state.requestTypeChanged(requestType);
        if (typeChanged) {
            takeover(control, state, requestType, request.motionPolicy(), position, velocity);
        }
        StagedRequest staged = stageRequest(
                control, transformed, mechanismContext, state, position, velocity, request.motionPolicy());
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
        if (staged.output() instanceof Output.Position || staged.output() instanceof Output.Velocity) {
            state.lastReference = staged.reference();
        }
        if (typeChanged) {
            resetLoops(state, loopContext);
            state.lastRequestType = requestType;
            state.hasLastRequest = true;
        }
        // Staging produces plain targets, so inspect the original Action before its effort metadata is lost.
        boolean actionNeedsClamp = output.output() instanceof EffortCarrier carrier
                && Double.isFinite(carrier.effort().limitValue());
        AppliedOutput offloaded = actionNeedsClamp ? null
                : resolveDeviceControl(control, staged.output(), loopContext, state);
        if (offloaded != null) {
            // Offload is used only while no active power cap requires the host voltage clamp.
            return offloaded.withTrace(
                    output.output(), transformed, staged, position, velocity, staged.constrained(), false, false);
        }
        Output applied = null;
        boolean gainsChanged = false;
        for (int index = 0; index < state.runtimes.size(); index++) {
            gainsChanged |= state.runtimes.get(index).consumeGainChange();
        }
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
        if (gainsChanged && applied instanceof Output.Voltage raw && Double.isFinite(state.lastVolts)
                && Double.isFinite(raw.volts())) {
            // Bumpless gain transfer: the integrating loop re-seeds its integral so the combined output
            // equals the previous output at the change instant (see ControlLoopRuntime#bumpTo).
            for (int index = 0; index < state.runtimes.size(); index++) {
                if (state.roles[index] != ControlLoopRole.TARGET_TRANSFORM
                        && state.runtimes.get(index).bumpTo(state.lastVolts, raw.volts())) {
                    applied = Outputs.voltage(state.lastVolts);
                    break;
                }
            }
        }
        Output resolved = applied == null ? staged.output() : applied;
        if (!isFinite(resolved)) {
            resetControlState(state, Outputs.neutral(), mechanismContext, position, velocity);
            return appliedOutput.set(Outputs.neutral(), ControlRoute.OPEN_LOOP, null)
                    .withTrace(output.output(), transformed, staged, position, velocity, staged.constrained(), true, false);
        }
        Output constrainedOutput = applyStageEffort(
                control, applyOutputConstraints(control, resolved, mechanismContext));
        Output saturated = saturate(control, constrainedOutput);
        // EFFORT HOOK (B5/B6): current-limit voltage clamp and load responses. See the effort package.
        saturated = limitEffort(state, output.output(), saturated, mechanismContext.dtSeconds(), position, velocity);
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
                .notifyOnCommit(state, loopContext, resolved)
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
            ControlBinding<?> control,
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
        RuntimeOverrides.ControlTuning tuning = overrides.tuning(control);
        if (transformed instanceof Output.Position positionTarget && overrides.hasTuning(control)) {
            transformed = Outputs.position(clampTunedPosition(tuning, positionTarget.position()));
        } else if (transformed instanceof Output.Velocity velocityTarget) {
            transformed = Outputs.velocity(Math.max(-tuning.maxVelocity(),
                    Math.min(tuning.maxVelocity(), velocityTarget.velocity())));
        }
        return transformed;
    }

    private StagedRequest stageRequest(
            ControlBinding<?> control,
            Output request,
            MechanismContext mechanismContext,
            ControlRuntimeState state,
            double position,
            double velocity,
            MotionPolicy policy) {
        if (request instanceof Output.Position target) {
            if ((hasTargetConstraints(control) || control.planner() != null)
                    && !Double.isFinite(position)) {
                return null;
            }
            MotionProfile profile = effectiveProfile(control);
            if (profile != null
                    && (!Double.isFinite(position) || !Double.isFinite(velocity))) {
                return null;
            }
            double goal;
            if (control.planner() == null && !hasTargetConstraints(control)) {
                // No target constraint and no planner: evaluation would allow the request unchanged.
                goal = target.position();
            } else {
                ConstraintContext<Double> constraintContext = new ConstraintContext<>(
                        position,
                        target.position(),
                        mechanismContext);
                ConstraintResult<Double> result = control.planner() == null
                        ? Constraints.evaluate(control.constraints(), constraintContext)
                        : control.planner().plan(constraintContext, control.constraints());
                if (!result.accepted()) {
                    blockedBy = result.rejectedBy();
                    return null;
                }
                goal = result.value();
            }
            if (overrides.hasTuning(control)) {
                // Dashboard-tuned limits must also bound what a planner returns (a planner may pick
                // a wrap candidate outside the tuned window).
                goal = clampTunedPosition(overrides.tuning(control), goal);
            }
            if (profile == null) {
                return new StagedRequest(
                        Outputs.position(goal),
                        MotionReference.stationary(goal),
                        goal,
                        Double.compare(goal, target.position()) != 0);
            }
            MotionProfileRuntime runtime = state.profileRuntime(profile);
            if (retargetFor(control, policy) == Retarget.IMMEDIATE && Double.compare(goal, state.lastGoal) != 0) {
                // IMMEDIATE: a new target re-plans from the measured state, braking at full profile decel.
                runtime.reset(position, velocity);
            }
            state.lastGoal = goal;
            MotionReference reference = runtime.step(
                    position,
                    velocity,
                    goal,
                    mechanismContext.dtSeconds());
            // The tuning range starts as the declared constraint range and narrows with dashboard
            // overrides, so profile references never leave either window.
            double boundedPosition = clampTunedPosition(overrides.tuning(control), reference.position());
            if (Double.compare(boundedPosition, reference.position()) != 0) {
                state.profileRuntime(profile).reset(boundedPosition, 0.0);
                reference = MotionReference.stationary(boundedPosition);
            }
            return new StagedRequest(
                    Outputs.position(reference.position()),
                    reference,
                    goal,
                    Double.compare(goal, target.position()) != 0);
        }
        if (request instanceof Output.Velocity target) {
            boolean constrained = hasTargetConstraints(control);
            if (constrained && !Double.isFinite(velocity)) {
                return null;
            }
            double requestedVelocity;
            if (!constrained) {
                // No target constraint: evaluation would allow the request unchanged.
                requestedVelocity = target.velocity();
            } else {
                ConstraintResult<Double> result = Constraints.evaluate(
                        control.constraints(),
                        new ConstraintContext<>(velocity, target.velocity(), mechanismContext));
                if (!result.accepted()) {
                    blockedBy = result.rejectedBy();
                    return null;
                }
                requestedVelocity = result.value();
            }
            double maximumAcceleration = overrides.tuning(control).maxAcceleration();
            double referenceVelocity = requestedVelocity;
            double referenceAcceleration = 0.0;
            if (retargetFor(control, policy) == Retarget.IMMEDIATE
                    && Double.compare(requestedVelocity, state.lastGoal) != 0) {
                state.velocityReference = finiteOrZero(velocity);
                state.velocityReferenceInitialized = true;
            }
            state.lastGoal = requestedVelocity;
            if (maximumAcceleration < Double.MAX_VALUE) {
                if (!state.velocityReferenceInitialized) {
                    state.velocityReference = finiteOrZero(velocity);
                    state.velocityReferenceInitialized = true;
                }
                double dt = Math.max(0.0, mechanismContext.dtSeconds());
                double maximumDelta = maximumAcceleration * dt;
                double delta = Math.max(-maximumDelta,
                        Math.min(maximumDelta, requestedVelocity - state.velocityReference));
                referenceVelocity = state.velocityReference + delta;
                referenceAcceleration = dt > 0.0 ? delta / dt : 0.0;
                state.velocityReference = referenceVelocity;
            }
            return new StagedRequest(
                    Outputs.velocity(referenceVelocity),
                    new MotionReference(finiteOrZero(position), referenceVelocity, referenceAcceleration),
                    requestedVelocity,
                    Double.compare(referenceVelocity, requestedVelocity) != 0);
        }
        if (request instanceof Output.Percent target) {
            double feedback = constraintFeedback(control, position, velocity);
            if (hasTargetConstraints(control) && !Double.isFinite(feedback)) {
                return null;
            }
            return allowsEdge(control, feedback, Math.signum(target.percent()), mechanismContext)
                    ? new StagedRequest(
                            request,
                            new MotionReference(finiteOrZero(position), finiteOrZero(velocity), 0.0),
                            target.percent(),
                            false)
                    : null;
        }
        if (request instanceof Output.Voltage target) {
            double feedback = constraintFeedback(control, position, velocity);
            if (hasTargetConstraints(control) && !Double.isFinite(feedback)) {
                return null;
            }
            return allowsEdge(control, feedback, Math.signum(target.volts()), mechanismContext)
                    ? new StagedRequest(
                            request,
                            new MotionReference(finiteOrZero(position), finiteOrZero(velocity), 0.0),
                            target.volts(),
                            false)
                    : null;
        }
        return new StagedRequest(request, MotionReference.stationary(finiteOrZero(position)), 0.0, false);
    }

    /**
     * Starts a request kind (position or velocity) on a binding whose previous kind differs. SMOOTH
     * continues from the inherited reference; IMMEDIATE, or no inherited reference, starts from the
     * measured state.
     */
    private void takeover(
            ControlBinding<?> control,
            ControlRuntimeState state,
            Class<?> kind,
            MotionPolicy policy,
            double position,
            double velocity) {
        MotionReference prior = retargetFor(control, policy) == Retarget.SMOOTH ? state.lastReference : null;
        double referencePosition = prior != null ? prior.position() : finiteOrZero(position);
        double referenceVelocity = prior != null ? prior.velocity() : finiteOrZero(velocity);
        if (kind == Output.Position.class) {
            MotionProfile profile = effectiveProfile(control);
            if (profile != null) {
                state.profileRuntime(profile).reset(referencePosition, referenceVelocity);
            }
        } else if (kind == Output.Velocity.class) {
            state.velocityReference = referenceVelocity;
            state.velocityReferenceInitialized = true;
        }
    }

    private static Class<?> requestKind(Output output) {
        if (output instanceof Output.Position) return Output.Position.class;
        if (output instanceof Output.Velocity) return Output.Velocity.class;
        return output.getClass();
    }

    private ControlRuntimeState controlRuntimes(ControlBinding<?> control) {
        ControlRuntimeState state = controlRuntimes.get(control);
        if (state == null) {
            state = new ControlRuntimeState(control);
            controlRuntimes.put(control, state);
            controlStates.add(state);
        }
        return state;
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

    private static double clampTunedPosition(RuntimeOverrides.ControlTuning tuning, double position) {
        return Math.max(tuning.minimumPosition(), Math.min(tuning.maximumPosition(), position));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /** Like {@link #allowsDirection}, and records the range edge that clipped the output when it does not allow it. */
    private boolean allowsEdge(
            ControlBinding<?> control,
            double current,
            double direction,
            MechanismContext context) {
        if (allowsDirection(control, current, direction, context)) return true;
        guardEdge = direction > 0.0 ? "max" : "min";
        return false;
    }

    private boolean allowsDirection(
            ControlBinding<?> control,
            double current,
            double direction,
            MechanismContext context) {
        if (direction == 0.0) {
            return true;
        }
        RuntimeOverrides.ControlTuning tuning = overrides.tuning(control);
        if (control.mode() == ControlMode.POSITION
                && Double.isFinite(current)
                && (direction < 0.0 && current <= tuning.minimumPosition()
                || direction > 0.0 && current >= tuning.maximumPosition())) {
            return false;
        }
        if (!hasTargetConstraints(control)) return true;
        // Probe an infinitesimal move in the output direction. A fixed probe distance (for example
        // one unit) depends on the control's units: in radians it reaches far past nearby edges and
        // blocks outputs long before the mechanism is at its limit.
        double probe = current + direction * directionProbe(current);
        ConstraintResult<Double> result = Constraints.evaluate(
                control.constraints(),
                new ConstraintContext<>(current, probe, context));
        if (!result.accepted()) {
            blockedBy = result.rejectedBy();
            return false;
        }
        return direction > 0.0 ? result.value() > current : result.value() < current;
    }

    private static double directionProbe(double current) {
        return Math.max(1.0, Math.abs(current)) * 1.0e-9;
    }

    private Output guardFinalOutput(
            ControlBinding<?> control,
            Output output,
            MechanismContext context,
            double position,
            double velocity) {
        double direction = outputDirection(output);
        if (direction == 0.0 || !hasTargetConstraints(control)) {
            return output;
        }
        double feedback = constraintFeedback(control, position, velocity);
        if (!allowsEdge(control, feedback, direction, context)) {
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

    private Output saturate(ControlBinding<?> control, Output output) {
        if (output instanceof Output.Percent percent) {
            return Outputs.percent(Math.max(-1.0, Math.min(1.0, percent.percent())));
        }
        if (output instanceof Output.Voltage voltage) {
            return Outputs.voltage(Math.max(-12.0, Math.min(12.0, voltage.volts())));
        }
        return output;
    }

    private static double constraintFeedback(ControlBinding<?> control, double position, double velocity) {
        return control.mode() == ControlMode.VELOCITY ? velocity : position;
    }

    private static boolean hasTargetConstraints(ControlBinding<?> control) {
        return Constraints.hasStage(control.constraints(), ConstraintStage.TARGET);
    }

    private Output applyOutputConstraints(
            ControlBinding<?> control,
            Output output,
            MechanismContext context) {
        if (!isScalar(output)) return output;
        double value;
        if (!Constraints.hasStage(control.constraints(), ConstraintStage.OUTPUT)) {
            // No output constraint: evaluation would allow the value unchanged.
            value = value(output);
        } else {
            Double requested = scalarValue(output);
            ConstraintResult<Double> result = Constraints.evaluateOutput(
                    control.constraints(),
                    new ConstraintContext<>(requested, requested, context));
            if (!result.accepted()) {
                blockedBy = result.rejectedBy();
                return Outputs.neutral();
            }
            value = result.value();
        }
        if (output instanceof Output.Percent) return Outputs.percent(value);
        if (output instanceof Output.Voltage) return Outputs.voltage(value);
        if (output instanceof Output.Position) return Outputs.position(value);
        if (output instanceof Output.Velocity) return Outputs.velocity(value);
        return output;
    }

    private static boolean isScalar(Output output) {
        return output instanceof Output.Percent || output instanceof Output.Voltage
                || output instanceof Output.Position || output instanceof Output.Velocity;
    }

    private static Double scalarValue(Output output) {
        if (output instanceof Output.Percent value) return value.percent();
        if (output instanceof Output.Voltage value) return value.volts();
        if (output instanceof Output.Position value) return value.position();
        if (output instanceof Output.Velocity value) return value.velocity();
        return null;
    }

    private static void applySink(ca.frc6390.athena.runtime.control.ControlSink sink, Output output) {
        if (!isScalar(output)) {
            sink.release();
        } else {
            sink.apply(value(output));
        }
    }

    private MotionProfile effectiveProfile(ControlBinding<?> control) {
        if (!overrides.hasProfile(control)) return null;
        MotionProfile profile = overrides.profile(control);
        double effortSpeed = effortSpeedScale(control);
        if (stageCaps.isEmpty() && effortSpeed == 1.0) return profile;
        StageCaps caps = stageCapsFor(control);
        if (caps.speed() == 1.0 && caps.acceleration() == 1.0 && effortSpeed == 1.0) return profile;
        return new MotionProfile(
                Math.max(Double.MIN_VALUE, profile.maxVelocity() * caps.speed() * effortSpeed),
                Math.max(Double.MIN_VALUE, profile.maxAcceleration() * caps.acceleration()));
    }

    /** Scales a velocity target by the staging speed cap. Position targets are limited by the profile instead. */
    private Output applyStageSpeed(ControlBinding<?> control, Output output) {
        if (!(output instanceof Output.Velocity velocity)) return output;
        double speed = (stageCaps.isEmpty() ? 1.0 : stageCapsFor(control).speed())
                * effortSpeedScale(control);
        return speed == 1.0 ? output : Outputs.velocity(velocity.velocity() * speed);
    }

    /**
     * EFFORT HOOK (B5/B6). Advances the binding's effort state once per applied cycle and clamps a
     * voltage (or percent, at nominal bus voltage) to the current the motors may draw. The per-motor limit is
     * the smallest of the binding ceiling, the Action effort and the power/thermal cap. Every follower's
     * current window joins the intersection in the shared command frame.
     */
    private Output limitEffort(ControlRuntimeState state, Output requested, Output candidate, double dtSeconds,
            double position, double velocity) {
        double actionLimit = requested instanceof EffortCarrier carrier
                ? carrier.effort().limitValue()
                : Double.POSITIVE_INFINITY;
        List<MotorDevice> leaders = state.binding.motors();
        for (int i = 0; i < state.powerCaps.length; i++) {
            state.powerCaps[i] = powerCapAmps(leaders.get(i));
        }
        double volts = candidate instanceof Output.Voltage voltage ? voltage.volts()
                : candidate instanceof Output.Percent percent ? percent.percent() * EffortUnits.NOMINAL_VOLTS
                : Double.NaN;
        boolean planned = preparing && electricalPass != null;
        state.effort.advance(actionLimit, planned ? null : state.powerCaps, dtSeconds, volts, position, velocity);
        if (Double.isNaN(volts)) {
            return candidate;
        }
        if (planned) return candidate;
        Limits followers = followerLimits(leaders, actionLimit);
        double limited = state.effort.clampVolts(context, volts, followers.motors(), followers.amps(), followers.handles());
        return limited == volts ? candidate : Outputs.voltage(limited);
    }

    /**
     * EFFORT HOOK (B6) for open-loop motor outputs, which have no binding. A percent or voltage command to one
     * motor is clamped with the motor's power cap and every follower's cap, as a binding's command is.
     */
    private Output limitOpenLoop(OutputRequest request, Output output) {
        MotorDevice motor = request.motor();
        if (motor == null || !(output instanceof Output.Percent || output instanceof Output.Voltage)) {
            return output;
        }
        double actionLimit = output instanceof EffortCarrier carrier
                ? carrier.effort().limitValue()
                : Double.POSITIVE_INFINITY;
        double volts = output instanceof Output.Voltage voltage ? voltage.volts()
                : ((Output.Percent) output).percent() * EffortUnits.NOMINAL_VOLTS;
        if (!Double.isFinite(volts)) {
            return output;
        }
        if (preparing && electricalPass != null) return output;
        List<MotorDevice> motors = openLoopMotorsScratch;
        motors.clear();
        motors.add(motor);
        int followerCount = addFollowerLimits(motor, actionLimit, 0);
        if (openLoopLimitsScratch.length < followerCount + 1) {
            openLoopLimitsScratch = new double[Math.max(followerCount + 1, openLoopLimitsScratch.length * 2)];
            openLoopHandlesScratch = new MotorHandle[openLoopLimitsScratch.length];
        }
        double[] limits = openLoopLimitsScratch;
        limits[0] = Math.min(actionLimit, powerCapAmps(motor));
        openLoopHandlesScratch[0] = context.motor(motor);
        for (int index = 0; index < followerCount; index++) {
            motors.add(followerDevicesScratch.get(index));
            limits[index + 1] = followerAmpsScratch[index];
            openLoopHandlesScratch[index + 1] = followerHandlesScratch[index];
        }
        try {
            double limited = EffortRuntime.clampToLimits(context, volts, motors, limits, openLoopHandlesScratch);
            return limited == volts ? output : Outputs.voltage(limited);
        } finally {
            motors.clear();
            followerDevicesScratch.clear();
        }
    }

    /**
     * Returns the per-motor limits of every follower sharing the leaders' command, including native followers.
     * Each constraint uses that motor's power cap and the Action limit in its actual feedback frame.
     */
    private Limits followerLimits(List<MotorDevice> leaders, double actionLimit) {
        int count = 0;
        for (int leader = 0; leader < leaders.size(); leader++) {
            count = addFollowerLimits(leaders.get(leader), actionLimit, count);
        }
        if (count == 0) {
            return Limits.NONE;
        }
        return new Limits(followerDevicesScratch, followerAmpsScratch, followerHandlesScratch);
    }

    /**
     * Appends the follower constraints of {@code leader} and their limits to the follower scratch lists,
     * starting at {@code count}, and returns the new count. The scratch is valid until the next call
     * with a zero count.
     */
    private int addFollowerLimits(MotorDevice leader, double actionLimit, int count) {
        if (count == 0) followerDevicesScratch.clear();
        List<? extends ActionContext.MotorFollower> followers = context.followerConstraints(leader);
        for (int index = 0; index < followers.size(); index++) {
            MotorDevice follower = followers.get(index).device();
            if (followerAmpsScratch.length == count) {
                followerAmpsScratch = java.util.Arrays.copyOf(followerAmpsScratch, count * 2);
                followerHandlesScratch = java.util.Arrays.copyOf(followerHandlesScratch, count * 2);
            }
            followerDevicesScratch.add(follower);
            followerHandlesScratch[count] = followers.get(index).handle();
            followerAmpsScratch[count++] = Math.min(actionLimit, powerCapAmps(follower));
        }
        return count;
    }

    /**
     * Per-motor current limits of a set of motors that share one command. {@code amps} may be longer than
     * {@code motors}; only the first {@code motors.size()} entries apply.
     */
    private record Limits(List<MotorDevice> motors, double[] amps, MotorHandle[] handles) {
        static final Limits NONE = new Limits(List.of(), new double[0], new MotorHandle[0]);
    }

    /**
     * Legacy State/Reason diagnostic. Automatic offload now falls back to host voltage control whenever
     * a power cap is active, so a successfully applied binding has no unapplied device-loop cap.
     */
    int deviceLoopUnappliedCapAmps(ControlBinding<?> control) {
        if (control == null) return -1;
        ControlRuntimeState state = controlRuntimes.get(control);
        return state == null ? -1 : state.unappliedCapAmps;
    }

    /** Power/thermal cap of a motor from this cycle's power pass, infinite when uncapped or unknown. */
    private double powerCapAmps(MotorDevice motor) {
        return electricalPass == null ? Double.POSITIVE_INFINITY : electricalPass.capAmps(motor);
    }

    /** Speed multiplier from the binding's {@code slow(...)} load response, 1 when none applies. */
    private double effortSpeedScale(ControlBinding<?> control) {
        ControlRuntimeState state = controlRuntimes.get(control);
        return state == null ? 1.0 : state.effort.speedScale();
    }

    /** Scales voltage and percent output by the staging effort cap. */
    private Output applyStageEffort(ControlBinding<?> control, Output output) {
        if (stageCaps.isEmpty()) return output;
        double effort = stageCapsFor(control).effort();
        if (effort == 1.0) return output;
        if (output instanceof Output.Voltage voltage) return Outputs.voltage(voltage.volts() * effort);
        if (output instanceof Output.Percent percent) return Outputs.percent(percent.percent() * effort);
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

    /**
     * Stores the control's target, reference, error and applied volts for telemetry. Reads only fields the cycle
     * already computed, so it allocates nothing. The error is goal minus measurement, as in the trace.
     */
    private void recordSample(ControlBinding<?> control, AppliedOutput applied) {
        ControlSample sample = overrides.controlSample(control);
        Output requested = applied.requested;
        if (requested == null) {
            sample.clear();
            return;
        }
        StagedRequest staged = applied.staged;
        double measured = requested instanceof Output.Velocity ? applied.measuredVelocity : applied.measuredPosition;
        double goal = staged == null ? value(applied.transformed) : staged.goal();
        double referencePosition = staged == null ? applied.measuredPosition : staged.reference().position();
        double referenceVelocity = staged == null ? 0.0 : staged.reference().velocity();
        double volts = applied.output() instanceof Output.Voltage voltage ? voltage.volts() : Double.NaN;
        sample.record(value(requested), referencePosition, referenceVelocity, goal - measured, volts);
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
        state.velocityReference = safeVelocity;
        state.velocityReferenceInitialized = false;
        state.lastReference = null;
        state.lastGoal = Double.NaN;
        state.lastVolts = Double.NaN;
        state.holdGoal = Double.NaN;
        if (state.profileRuntime != null) {
            state.profileRuntime.reset(safePosition, safeVelocity);
        }
    }

    private AppliedOutput resolveDeviceControl(
            ControlBinding<?> control,
            Output request,
            ControlLoopContext loopContext,
            ControlRuntimeState state) {
        if (!(request instanceof Output.Position || request instanceof Output.Velocity) || control.output() == null) {
            return null;
        }
        // Offload is an optimization: the already-bound PID/feedforward runtimes must calculate voltage here
        // whenever the electrical pass needs to clamp it. A device target would bypass that voltage clamp.
        if (requiresHostPowerClamp(control)) return null;
        if (!control.constraints().isEmpty() || effectiveProfile(control) != null
                || control.planner() != null) {
            return null;
        }
        // Device closed loops do not know Athena's continuous input range and would take the long
        // way around a wrapped mechanism. Keep continuous controls on the software loop.
        if (control.continuousRange() != null) {
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
        List<MotorDevice> devices = control.motors();
        for (int index = 0; index < devices.size(); index++) {
            MotorControlCapabilities capabilities = context.motor(devices.get(index)).controlCapabilities();
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

    private boolean requiresHostPowerClamp(ControlBinding<?> control) {
        List<MotorDevice> motors = control.motors();
        for (int index = 0; index < motors.size(); index++) {
            MotorDevice motor = motors.get(index);
            if (preparing && electricalPass != null && electricalPass.requiresHostClamp(motor)) return true;
            if (Double.isFinite(powerCapAmps(motor))) return true;
            List<? extends ActionContext.MotorFollower> followers = context.followerConstraints(motor);
            for (int follower = 0; follower < followers.size(); follower++) {
                if (preparing && electricalPass != null
                        && electricalPass.requiresHostClamp(followers.get(follower).device())) return true;
                if (Double.isFinite(powerCapAmps(followers.get(follower).device()))) return true;
            }
        }
        return false;
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

    private static void recordCommand(MotorDevice motor, Output output) {
        if (output instanceof Output.Percent percent) {
            motor.recordCommand(MotorCommandSnapshot.Mode.PERCENT, percent.percent());
            return;
        }
        if (output instanceof Output.Voltage voltage) {
            motor.recordCommand(MotorCommandSnapshot.Mode.VOLTAGE, voltage.volts());
            return;
        }
        if (output instanceof Output.Position position) {
            motor.recordCommand(MotorCommandSnapshot.Mode.POSITION, position.position());
            return;
        }
        if (output instanceof Output.Velocity velocity) {
            motor.recordCommand(MotorCommandSnapshot.Mode.VELOCITY, velocity.velocity());
            return;
        }
        motor.recordCommand(MotorCommandSnapshot.neutral());
    }

    private static boolean usesDeviceNativeFeedback(ControlBinding<?> control) {
        FeedbackBinding feedback = control.feedback();
        if (feedback == null) {
            return true;
        }
        if (!(feedback.position() instanceof EncoderDevice.PositionFeedback position)
                || !(feedback.velocity() instanceof EncoderDevice.VelocityFeedback velocity)
                || !position.encoder().equals(velocity.encoder())) {
            return false;
        }
        return isNativeFeedback(control.output(), position.encoder());
    }

    private static boolean isNativeFeedback(MotorDevice output, EncoderDevice feedback) {
        if (output == null || feedback == null || feedback.isInverted()
                || feedback.gearRatio() != 1.0 || feedback.conversion() != 1.0
                || feedback.offset() != 0.0 || feedback.measuresUnit() != Units.Rotations) {
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

    private static MotorClosedLoopConfig closedLoopConfig(ControlBinding<?> control) {
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
        private final ControlBinding<?> control;
        private final Output request;
        private final MechanismContext mechanismContext;
        private final MotionReference reference;

        private AppliedControlLoopContext(
                ControlBinding<?> control,
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

    private double firstFeedbackPosition(ControlBinding<?> control) {
        if (control.feedback() != null) {
            return readOrNaN(control.feedback().position()::position);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrNaN(motor::integratedPositionRotations);
        }
        return 0.0;
    }

    private double firstFeedbackVelocity(ControlBinding<?> control) {
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
        private final ControlBinding<?> binding;
        private final List<ControlLoopRuntime> runtimes;
        private final ControlLoopRole[] roles;
        /** Effort state of this binding (B5/B6); a limit-free runtime when the binding has no ceiling. */
        private final EffortRuntime effort;
        /** Per-motor power/thermal caps of this cycle, in the binding's motor order. */
        private final double[] powerCaps;
        /** Power cap not applied this cycle because the binding is device-offloaded; -1 when none. */
        private int unappliedCapAmps = -1;
        private MotionProfileRuntime profileRuntime;
        private MotionProfile activeProfile;
        private double velocityReference;
        private boolean velocityReferenceInitialized;
        private boolean hasLastRequest;
        private Class<?> lastRequestType;
        private ControlRoute cachedRequestRoute;
        private double cachedRequestFeedforwardVolts;
        private MotorClosedLoopConfig cachedConfig;
        private MotorClosedLoopRequest cachedRequest;
        private MechanismContext lastContext = MechanismContext.empty();
        private double lastPosition;
        private double lastVelocity;
        private boolean active;
        private boolean reseedPending;
        private double reseedPosition = Double.NaN;
        private MotionReference lastReference;
        private double lastGoal = Double.NaN;
        private double lastVolts = Double.NaN;
        private Release lastRelease = Release.NEUTRAL;
        private Retarget lastRetarget = Retarget.SMOOTH;
        private boolean holdable;
        private double holdGoal = Double.NaN;

        private ControlRuntimeState(ControlBinding<?> binding) {
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
            EffortRuntime found = null;
            for (ControlLoopRuntime runtime : runtimeArray) {
                if (runtime instanceof EffortRuntime effortRuntime) {
                    found = effortRuntime;
                }
            }
            effort = found == null ? EffortRuntime.unlimited(binding.motors()) : found;
            powerCaps = new double[binding.motors().size()];
            activeProfile = binding.profile();
            profileRuntime = activeProfile == null ? null : new MotionProfileRuntime(activeProfile);
        }

        private MotionProfileRuntime profileRuntime(MotionProfile profile) {
            if (!Objects.equals(activeProfile, profile)) {
                activeProfile = profile;
                profileRuntime = new MotionProfileRuntime(profile);
            }
            return profileRuntime;
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
        private ControlRuntimeState notificationState;
        private ControlLoopContext notificationContext;
        private Output notificationRequested;

        private AppliedOutput set(Output output, ControlRoute route, MotorClosedLoopRequest closedLoopRequest) {
            this.output = output;
            this.route = route;
            this.closedLoopRequest = closedLoopRequest;
            notificationState = null;
            notificationContext = null;
            notificationRequested = null;
            return this;
        }

        private AppliedOutput notifyOnCommit(ControlRuntimeState state, ControlLoopContext context, Output requested) {
            notificationState = state;
            notificationContext = context;
            notificationRequested = requested;
            return this;
        }

        private void copyFrom(AppliedOutput source) {
            output = source.output;
            route = source.route;
            closedLoopRequest = source.closedLoopRequest;
            requested = source.requested;
            transformed = source.transformed;
            staged = source.staged;
            measuredPosition = source.measuredPosition;
            measuredVelocity = source.measuredVelocity;
            constrained = source.constrained;
            blocked = source.blocked;
            saturated = source.saturated;
            notificationState = source.notificationState;
            notificationContext = source.notificationContext;
            notificationRequested = source.notificationRequested;
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

        private MechanismTraceSnapshot.Control trace(ControlBinding<?> control, ControlRuntimeState state) {
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

    private static final class PreparedOutput {
        final AppliedOutput applied = new AppliedOutput();
        final List<MotorDevice> motors = new ArrayList<>();
        MotorHandle[] handles = new MotorHandle[4];
        double[] localAmps = new double[4];
        ResolvedOutput output;
        MechanismContext context;
        double candidateVolts;
        boolean valid;
        Output plannedOutput;

        void add(MotorDevice motor, MotorHandle handle, double amps) {
            for (int index = 0; index < motors.size(); index++) {
                if (motors.get(index) == motor) {
                    localAmps[index] = Math.min(localAmps[index], amps);
                    return;
                }
            }
            int index = motors.size();
            if (index == handles.length) {
                handles = java.util.Arrays.copyOf(handles, index * 2);
                localAmps = java.util.Arrays.copyOf(localAmps, index * 2);
            }
            motors.add(motor);
            handles[index] = handle;
            localAmps[index] = amps;
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
