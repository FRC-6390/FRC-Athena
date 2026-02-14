package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.Locale;

import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.diagnostics.BoundedEventLog;
import ca.frc6390.athena.core.localization.RobotLocalization;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Aggregates named command and autonomous routine registration for a robot.
 */
public class RobotAuto {
    private static final int DEFAULT_AUTO_TRACE_LOG_CAPACITY = 512;

    private final Map<String, Supplier<Command>> namedCommandSuppliers;
    private final Map<String, AutoRoutine> autoRoutines;
    private final Map<String, AutoInputBinding<?>> autoInputs;
    private SendableChooser<AutoRoutine> chooser;
    private SendableChooser<Command> commandChooser;
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;
    private java.util.function.Function<Pose2d, Command> autoPlanResetter;
    private boolean registrationFinalized;
    private final Set<String> boundPathPlannerNamedCommands;
    private final Set<String> boundChoreoNamedCommands;
    private final BoundedEventLog<AutoTraceEvent> autoTraceLog;
    private final int autoTraceLogCapacity;
    private RobotCore<?> robotCore;
    private boolean autoTraceEnabled;
    private Consumer<String> autoTraceSink;

    public RobotAuto() {
        namedCommandSuppliers = new LinkedHashMap<>();
        autoRoutines = new LinkedHashMap<>();
        autoInputs = new LinkedHashMap<>();
        chooser = null;
        commandChooser = null;
        xController = null;
        yController = null;
        thetaController = null;
        autoPlanResetter = null;
        registrationFinalized = false;
        boundPathPlannerNamedCommands = new HashSet<>();
        boundChoreoNamedCommands = new HashSet<>();
        autoTraceLogCapacity = DEFAULT_AUTO_TRACE_LOG_CAPACITY;
        autoTraceLog = new BoundedEventLog<>(autoTraceLogCapacity);
        robotCore = null;
        autoTraceEnabled = false;
        autoTraceSink = message -> DriverStation.reportWarning(message, false);
    }

    /**
     * Identifier interface for named commands used inside autonomous routines.
     */
    public interface NamedCommandKey {
        String id();

        default String displayName() {
            return id();
        }

        static NamedCommandKey of(String id) {
            return of(id, id);
        }

        static NamedCommandKey of(String id, String displayName) {
            return new SimpleNamedCommandKey(id, displayName);
        }
    }

    /**
     * Identifier interface for autonomous routines exposed on the chooser.
     */
    public interface AutoKey {
        String id();

        default String displayName() {
            return id();
        }

        static AutoKey of(String id) {
            return of(id, id);
        }

        static AutoKey of(String id, String displayName) {
            return new SimpleAutoKey(id, displayName);
        }
    }

    /**
     * Typed key for external auto inputs.
     */
    public interface AutoInput<T> {
        String id();

        Class<T> type();

        static <T> AutoInput<T> of(String id, Class<T> type) {
            return new SimpleAutoInput<>(id, type);
        }
    }

    /**
     * Immutable auto-trace entry for debugging auto execution flow.
     */
    public record AutoTraceEvent(
            long sequence,
            double timestampSeconds,
            String systemKey,
            String level,
            String category,
            Double xMeters,
            Double yMeters,
            Double headingDeg,
            String message,
            String line) {}

    /**
     * Source enum for trajectory-backed autos.
     */
    public enum TrajectorySource {
        PATH_PLANNER(AutoSource.PATH_PLANNER),
        CHOREO(AutoSource.CHOREO);

        private final AutoSource autoSource;

        TrajectorySource(AutoSource autoSource) {
            this.autoSource = autoSource;
        }

        public AutoSource autoSource() {
            return autoSource;
        }
    }

    /**
     * Shared runtime context available to auto program phases.
     */
    public interface AutoRuntimeCtx {
        RobotCore<?> robot();

        RobotAuto autos();

        default RobotMechanisms mechanisms() {
            return robot().getMechanisms();
        }

        default RobotLocalization<?> localization() {
            return robot().getLocalization();
        }

        default RobotVision vision() {
            return robot().getVision();
        }

        default RobotDrivetrain<?> drivetrain() {
            return robot().getDrivetrain();
        }

        default boolean hasNamedCommand(String id) {
            return autos().hasNamedCommand(id);
        }

        default boolean hasAuto(String id) {
            return autos().hasAuto(id);
        }

        default boolean hasInput(String id) {
            return autos().hasInput(id);
        }

        default <T> Supplier<T> input(AutoInput<T> key) {
            return autos().inputSupplier(key);
        }

        default <T> T inputValue(AutoInput<T> key) {
            return input(key).get();
        }
    }

    /**
     * Registration-only context. Use this in {@link AutoProgram#register(AutoRegisterCtx)} to
     * declare named commands, inputs, and dependent autos.
     */
    public interface AutoRegisterCtx extends AutoRuntimeCtx {
        default AutoRegisterCtx registerNamedCommand(String id, Supplier<Command> supplier) {
            autos().registerNamedCommand(id, supplier);
            return this;
        }

        default AutoRegisterCtx registerNamedCommand(String id, Command command) {
            autos().registerNamedCommand(id, command);
            return this;
        }

        default AutoRegisterCtx registerNamedCommand(String id, Runnable action) {
            autos().registerNamedCommand(id, action);
            return this;
        }

        default <T> AutoRegisterCtx input(AutoInput<T> key, Supplier<T> supplier) {
            autos().registerInput(key, supplier);
            return this;
        }

        /**
         * Register a trajectory by file/reference name. The chooser id defaults to the same value.
         *
         * <p>Signature order: {@code path(reference, source)}.</p>
         */
        default AutoRegisterCtx path(String reference, TrajectorySource source) {
            autos().registerPath(reference, source);
            return this;
        }

        /**
         * Register a trajectory by file/reference name and alias it under a different chooser id.
         *
         * <p>Signature order: {@code path(reference, source, id)}.</p>
         */
        default AutoRegisterCtx path(String reference, TrajectorySource source, String id) {
            autos().registerPath(reference, source, id);
            return this;
        }

        default AutoRegisterCtx custom(String id, Supplier<Command> factory) {
            autos().registerAuto(id, factory);
            return this;
        }

        default AutoRegisterCtx registerAuto(AutoRoutine routine) {
            autos().registerAuto(routine);
            return this;
        }

        default AutoRegisterCtx requireNamedCommand(String id) {
            if (!hasNamedCommand(id)) {
                throw new IllegalStateException("Required named command not registered: " + id);
            }
            return this;
        }

        default AutoRegisterCtx requireAuto(String id) {
            if (!hasAuto(id)) {
                throw new IllegalStateException("Required auto not registered: " + id);
            }
            return this;
        }

        default AutoRegisterCtx requireInput(String id) {
            if (!hasInput(id)) {
                throw new IllegalStateException("Required auto input not registered: " + id);
            }
            return this;
        }

        default AutoRegisterCtx requireAutos(String... ids) {
            if (ids == null) {
                return this;
            }
            for (String id : ids) {
                if (id != null) {
                    requireAuto(id);
                }
            }
            return this;
        }

        default AutoRegisterCtx requireNamedCommands(String... ids) {
            if (ids == null) {
                return this;
            }
            for (String id : ids) {
                if (id != null) {
                    requireNamedCommand(id);
                }
            }
            return this;
        }
    }

    /**
     * Build-only context used in {@link AutoProgram#build(AutoBuildCtx)}.
     */
    public interface AutoBuildCtx extends AutoRuntimeCtx {
        default Command auto(String id) {
            Objects.requireNonNull(id, "id");
            return autos().deferredAuto(id, OptionalInt.empty());
        }

        default Command auto(AutoKey key) {
            Objects.requireNonNull(key, "key");
            return auto(key.id());
        }

        /**
         * Resolves either an explicitly registered split (id.index) or derives it from base path reference.
         */
        default Command auto(String id, int splitIndex) {
            Objects.requireNonNull(id, "id");
            return autos().deferredAuto(id, OptionalInt.of(splitIndex));
        }

        default Command sequence(Command... steps) {
            Command[] commands = nonNullCommands(steps);
            Command sequenceCommand = commands.length == 0 ? Commands.none() : Commands.sequence(commands);
            return traceBuild("sequence", "steps=" + commands.length, sequenceCommand);
        }

        default Command sequence(String... autoIds) {
            Command[] commands = commandsForIds(this, autoIds);
            Command sequenceCommand = commands.length == 0 ? Commands.none() : Commands.sequence(commands);
            return traceBuild(
                    "sequence(autoIds)",
                    "steps=" + commands.length + " ids=" + idsLabel(autoIds),
                    sequenceCommand);
        }

        default Command parallel(Command... steps) {
            Command[] commands = nonNullCommands(steps);
            Command parallelCommand = commands.length == 0 ? Commands.none() : Commands.parallel(commands);
            return traceBuild("parallel", "steps=" + commands.length, parallelCommand);
        }

        default Command parallel(String... autoIds) {
            Command[] commands = commandsForIds(this, autoIds);
            Command parallelCommand = commands.length == 0 ? Commands.none() : Commands.parallel(commands);
            return traceBuild(
                    "parallel(autoIds)",
                    "steps=" + commands.length + " ids=" + idsLabel(autoIds),
                    parallelCommand);
        }

        default Command race(Command... steps) {
            Command[] commands = nonNullCommands(steps);
            Command raceCommand = commands.length == 0 ? Commands.none() : Commands.race(commands);
            return traceBuild("race", "steps=" + commands.length, raceCommand);
        }

        default Command race(String... autoIds) {
            Command[] commands = commandsForIds(this, autoIds);
            Command raceCommand = commands.length == 0 ? Commands.none() : Commands.race(commands);
            return traceBuild(
                    "race(autoIds)",
                    "steps=" + commands.length + " ids=" + idsLabel(autoIds),
                    raceCommand);
        }

        default Command deadline(Command deadline, Command... others) {
            Objects.requireNonNull(deadline, "deadline");
            Command[] commands = nonNullCommands(others);
            Command deadlineCommand = commands.length == 0 ? deadline : Commands.deadline(deadline, commands);
            return traceBuild(
                    "deadline",
                    "deadline=" + commandLabel(deadline) + " others=" + commands.length,
                    deadlineCommand);
        }

        default Command deadline(String deadlineAutoId, String... otherAutoIds) {
            Objects.requireNonNull(deadlineAutoId, "deadlineAutoId");
            Command[] otherCommands = commandsForIds(this, otherAutoIds);
            Command deadlineCommand = otherCommands.length == 0
                    ? auto(deadlineAutoId)
                    : Commands.deadline(auto(deadlineAutoId), otherCommands);
            return traceBuild(
                    "deadline(autoIds)",
                    "deadline=auto(" + deadlineAutoId + ") others=" + otherCommands.length
                            + " ids=" + idsLabel(otherAutoIds),
                    deadlineCommand);
        }

        default Command select(BooleanSupplier condition, Command ifTrue, Command ifFalse) {
            return selectWithLabels(condition, ifTrue, ifFalse, commandLabel(ifTrue), commandLabel(ifFalse));
        }

        default Command select(BooleanSupplier condition, String ifTrueId, String ifFalseId) {
            Objects.requireNonNull(ifTrueId, "ifTrueId");
            Objects.requireNonNull(ifFalseId, "ifFalseId");
            return selectWithLabels(
                    condition,
                    auto(ifTrueId),
                    auto(ifFalseId),
                    "auto(" + ifTrueId + ")",
                    "auto(" + ifFalseId + ")");
        }

        private Command selectWithLabels(
                BooleanSupplier condition,
                Command ifTrue,
                Command ifFalse,
                String ifTrueLabel,
                String ifFalseLabel) {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(ifTrue, "ifTrue");
            Objects.requireNonNull(ifFalse, "ifFalse");
            Objects.requireNonNull(ifTrueLabel, "ifTrueLabel");
            Objects.requireNonNull(ifFalseLabel, "ifFalseLabel");
            BooleanSupplier tracedCondition = () -> {
                boolean result = condition.getAsBoolean();
                autos().traceAutoDecision(
                        "select",
                        "condition=" + result + " -> " + (result ? ifTrueLabel : ifFalseLabel));
                return result;
            };
            Command selectCommand = Commands.either(ifTrue, ifFalse, tracedCondition);
            return traceBuild(
                    "select",
                    "ifTrue=" + ifTrueLabel + " ifFalse=" + ifFalseLabel,
                    selectCommand);
        }

        default Command selectIndex(IntSupplier selector, Command fallback, Command... options) {
            Objects.requireNonNull(selector, "selector");
            Objects.requireNonNull(fallback, "fallback");
            List<Command> resolvedOptions = new ArrayList<>();
            List<String> optionLabels = new ArrayList<>();
            if (options != null) {
                for (Command option : options) {
                    if (option == null) {
                        continue;
                    }
                    resolvedOptions.add(option);
                    optionLabels.add(commandLabel(option));
                }
            }
            return selectIndexWithLabels(
                    selector,
                    fallback,
                    commandLabel(fallback),
                    resolvedOptions.toArray(Command[]::new),
                    optionLabels.toArray(String[]::new));
        }

        default Command selectIndex(IntSupplier selector, String fallbackAutoId, String... optionAutoIds) {
            Objects.requireNonNull(fallbackAutoId, "fallbackAutoId");
            Command fallback = auto(fallbackAutoId);
            List<Command> options = new ArrayList<>();
            List<String> optionLabels = new ArrayList<>();
            if (optionAutoIds != null) {
                for (String optionAutoId : optionAutoIds) {
                    if (optionAutoId == null || optionAutoId.isBlank()) {
                        continue;
                    }
                    options.add(auto(optionAutoId));
                    optionLabels.add("auto(" + optionAutoId + ")");
                }
            }
            return selectIndexWithLabels(
                    selector,
                    fallback,
                    "auto(" + fallbackAutoId + ")",
                    options.toArray(Command[]::new),
                    optionLabels.toArray(String[]::new));
        }

        private Command selectIndexWithLabels(
                IntSupplier selector,
                Command fallback,
                String fallbackLabel,
                Command[] options,
                String[] optionLabels) {
            Objects.requireNonNull(selector, "selector");
            Objects.requireNonNull(fallback, "fallback");
            Objects.requireNonNull(fallbackLabel, "fallbackLabel");
            Objects.requireNonNull(options, "options");
            Objects.requireNonNull(optionLabels, "optionLabels");
            if (optionLabels.length != options.length) {
                throw new IllegalArgumentException("optionLabels length must match options length");
            }
            Command selectIndexCommand = Commands.defer(() -> {
                int index = selector.getAsInt();
                if (index < 0 || index >= options.length) {
                    autos().traceAutoDecision(
                            "selectIndex",
                            "index=" + index + " outOfRange(size=" + options.length + ") -> " + fallbackLabel);
                    return fallback;
                }
                String selectedLabel = optionLabels[index];
                autos().traceAutoDecision(
                        "selectIndex",
                        "index=" + index + " -> " + selectedLabel);
                return options[index];
            }, Set.of());
            return traceBuild(
                    "selectIndex",
                    "options=" + options.length + " fallback=" + fallbackLabel,
                    selectIndexCommand);
        }

        private static String commandLabel(Command command) {
            if (command == null) {
                return "<null>";
            }
            String name = command.getName();
            if (name == null || name.isBlank()) {
                return command.getClass().getSimpleName();
            }
            return name;
        }

        private Command traceBuild(String operation, String detail, Command command) {
            Objects.requireNonNull(operation, "operation");
            Objects.requireNonNull(command, "command");
            return autos().traceAutoLifecycle("ctx." + operation, detail, command);
        }

        private static String idsLabel(String... ids) {
            if (ids == null || ids.length == 0) {
                return "[]";
            }
            StringBuilder out = new StringBuilder();
            out.append('[');
            boolean first = true;
            for (String id : ids) {
                if (id == null || id.isBlank()) {
                    continue;
                }
                if (!first) {
                    out.append(", ");
                }
                out.append(id);
                first = false;
            }
            if (first) {
                return "[]";
            }
            out.append(']');
            return out.toString();
        }

        /**
         * Waits for a fixed amount of time.
         */
        default Command waitSeconds(double seconds) {
            if (seconds < 0.0) {
                throw new IllegalArgumentException("seconds must be >= 0");
            }
            Command waitCommand = seconds == 0.0 ? Commands.none() : Commands.waitSeconds(seconds);
            return traceBuild("waitSeconds", "seconds=" + seconds, waitCommand);
        }

        /**
         * Waits until the condition becomes true.
         */
        default Command waitUntil(BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            Command waitCommand = Commands.waitUntil(condition);
            return traceBuild("waitUntil", "condition=" + condition, waitCommand);
        }

        /**
         * Wait for a condition, then continue. If timeout elapses first, execute fallback.
         */
        default Command waitFor(BooleanSupplier condition, double timeoutSeconds, Command onTimeout) {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(onTimeout, "onTimeout");
            if (timeoutSeconds <= 0.0) {
                throw new IllegalArgumentException("timeoutSeconds must be > 0");
            }
            AtomicBoolean timedOut = new AtomicBoolean(false);
            Command timeoutBranch = Commands.sequence(
                    Commands.waitSeconds(timeoutSeconds),
                    Commands.runOnce(() -> timedOut.set(true)),
                    onTimeout);
            Command waitCommand = Commands.race(
                    Commands.waitUntil(condition),
                    timeoutBranch);
            return traceBuild(
                    "waitFor",
                    "timeoutSeconds=" + timeoutSeconds + " fallback=" + commandLabel(onTimeout),
                    waitCommand).finallyDo(interrupted -> {
                        if (interrupted) {
                            autos().traceAutoDecision("waitFor", "result=interrupted");
                            return;
                        }
                        autos().traceAutoDecision(
                                "waitFor",
                                timedOut.get() ? "result=timeoutFallback" : "result=conditionMet");
                    });
        }

        default Command waitFor(BooleanSupplier condition, double timeoutSeconds, Runnable onTimeout) {
            Objects.requireNonNull(onTimeout, "onTimeout");
            return waitFor(condition, timeoutSeconds, Commands.runOnce(onTimeout));
        }

        default Command waitFor(BooleanSupplier condition, double timeoutSeconds) {
            Objects.requireNonNull(condition, "condition");
            if (timeoutSeconds <= 0.0) {
                throw new IllegalArgumentException("timeoutSeconds must be > 0");
            }
            AtomicBoolean timedOut = new AtomicBoolean(false);
            Command waitCommand = Commands.race(
                    Commands.waitUntil(condition),
                    Commands.sequence(
                            Commands.waitSeconds(timeoutSeconds),
                            Commands.runOnce(() -> timedOut.set(true))));
            return traceBuild(
                    "waitFor",
                    "timeoutSeconds=" + timeoutSeconds + " fallback=<none>",
                    waitCommand).finallyDo(interrupted -> {
                        if (interrupted) {
                            autos().traceAutoDecision("waitFor", "result=interrupted");
                            return;
                        }
                        autos().traceAutoDecision(
                                "waitFor",
                                timedOut.get() ? "result=timeout" : "result=conditionMet");
                    });
        }

        /**
         * Wait indefinitely until the condition becomes true.
         */
        default Command waitFor(BooleanSupplier condition) {
            return waitUntil(condition);
        }

        /**
         * Applies a timeout to an arbitrary command.
         */
        default Command timeout(Command command, double timeoutSeconds) {
            Objects.requireNonNull(command, "command");
            if (timeoutSeconds <= 0.0) {
                throw new IllegalArgumentException("timeoutSeconds must be > 0");
            }
            AtomicBoolean timedOut = new AtomicBoolean(false);
            Command timeoutCommand = Commands.race(
                    command,
                    Commands.sequence(
                            Commands.waitSeconds(timeoutSeconds),
                            Commands.runOnce(() -> timedOut.set(true))));
            return traceBuild(
                    "timeout",
                    "timeoutSeconds=" + timeoutSeconds + " command=" + commandLabel(command),
                    timeoutCommand).finallyDo(interrupted -> {
                        if (interrupted) {
                            autos().traceAutoDecision("timeout", "result=interrupted");
                            return;
                        }
                        autos().traceAutoDecision(
                                "timeout",
                                timedOut.get() ? "result=timedOut" : "result=completed");
                    });
        }

        /**
         * Applies a timeout to a registered auto by id.
         */
        default Command timeout(String autoId, double timeoutSeconds) {
            Objects.requireNonNull(autoId, "autoId");
            return timeout(auto(autoId), timeoutSeconds);
        }

        /**
         * Runs a command, and if it times out, runs fallback.
         */
        default Command timeout(Command command, double timeoutSeconds, Command onTimeout) {
            Objects.requireNonNull(command, "command");
            Objects.requireNonNull(onTimeout, "onTimeout");
            if (timeoutSeconds <= 0.0) {
                throw new IllegalArgumentException("timeoutSeconds must be > 0");
            }
            AtomicBoolean timedOut = new AtomicBoolean(false);
            Command timeoutCommand = Commands.race(
                    command,
                    Commands.sequence(
                            Commands.waitSeconds(timeoutSeconds),
                            Commands.runOnce(() -> timedOut.set(true)),
                            onTimeout));
            return traceBuild(
                    "timeoutWithFallback",
                    "timeoutSeconds=" + timeoutSeconds
                            + " command=" + commandLabel(command)
                            + " fallback=" + commandLabel(onTimeout),
                    timeoutCommand).finallyDo(interrupted -> {
                        if (interrupted) {
                            autos().traceAutoDecision("timeoutWithFallback", "result=interrupted");
                            return;
                        }
                        autos().traceAutoDecision(
                                "timeoutWithFallback",
                                timedOut.get() ? "result=timeoutFallback" : "result=completed");
                    });
        }

        /**
         * Runs an auto by id, and if it times out, runs fallback.
         */
        default Command timeout(String autoId, double timeoutSeconds, Command onTimeout) {
            Objects.requireNonNull(autoId, "autoId");
            return timeout(auto(autoId), timeoutSeconds, onTimeout);
        }

        default Command timeout(Command command, double timeoutSeconds, Runnable onTimeout) {
            Objects.requireNonNull(onTimeout, "onTimeout");
            return timeout(command, timeoutSeconds, Commands.runOnce(onTimeout));
        }

        default Command timeout(String autoId, double timeoutSeconds, Runnable onTimeout) {
            Objects.requireNonNull(autoId, "autoId");
            return timeout(auto(autoId), timeoutSeconds, onTimeout);
        }

        default AutoPlan.Context planContext() {
            return new AutoPlan.Context() {
                @Override
                public Command resolve(AutoPlan.StepRef ref) {
                    return ref.splitIndex().isPresent()
                            ? auto(ref.key().id(), ref.splitIndex().getAsInt())
                            : auto(ref.key().id());
                }

                @Override
                public Optional<Pose2d> startingPose(AutoPlan.StepRef ref) {
                    return autos().startingPoseFor(ref.key().id(), ref.splitIndex());
                }

                @Override
                public Command resetPose(Pose2d pose) {
                    return autos().resetPose(pose);
                }
            };
        }

        private static Command[] commandsForIds(AutoBuildCtx ctx, String... ids) {
            Objects.requireNonNull(ctx, "ctx");
            if (ids == null || ids.length == 0) {
                return new Command[0];
            }
            List<Command> commands = new ArrayList<>();
            for (String id : ids) {
                if (id == null || id.isBlank()) {
                    continue;
                }
                commands.add(ctx.auto(id));
            }
            return commands.toArray(Command[]::new);
        }

        private static Command[] nonNullCommands(Command... commands) {
            if (commands == null || commands.length == 0) {
                return new Command[0];
            }
            List<Command> filtered = new ArrayList<>();
            for (Command command : commands) {
                if (command != null) {
                    filtered.add(command);
                }
            }
            return filtered.toArray(Command[]::new);
        }

    }

    /**
     * One-file auto definition:
     * <ol>
     *   <li>{@link #id()} declares the chooser id for this auto.</li>
     *   <li>{@link #register(AutoRegisterCtx)} registers dependencies for this auto.</li>
     *   <li>{@link #build(AutoBuildCtx)} returns the primary routine to place on the chooser.</li>
     * </ol>
     */
    public interface AutoProgram {
        String id();

        default AutoKey key() {
            return AutoKey.of(id());
        }

        default void register(AutoRegisterCtx ctx) {}

        AutoRoutine build(AutoBuildCtx ctx);

        default AutoRoutine path(TrajectorySource source, String reference) {
            return RobotAuto.path(key(), source, reference);
        }

        default AutoRoutine path(TrajectorySource source) {
            return path(source, key().id());
        }

        default AutoRoutine custom(Supplier<Command> factory) {
            return RobotAuto.custom(key(), factory);
        }
    }

    private record AutoRegisterCtxImpl(
            RobotCore<?> robot,
            RobotAuto autos,
            String programId,
            Map<String, AutoRoutine> scopedAutos) implements AutoRegisterCtx {
        private AutoRegisterCtxImpl {
            Objects.requireNonNull(robot, "robot");
            Objects.requireNonNull(autos, "autos");
            Objects.requireNonNull(programId, "programId");
            Objects.requireNonNull(scopedAutos, "scopedAutos");
        }

        @Override
        public boolean hasAuto(String id) {
            return id != null && (scopedAutos.containsKey(id) || autos.hasAuto(id));
        }

        @Override
        public AutoRegisterCtx path(String reference, TrajectorySource source) {
            return path(reference, source, reference);
        }

        @Override
        public AutoRegisterCtx path(String reference, TrajectorySource source, String id) {
            autos.registerScopedPath(scopedAutos, programId, reference, source, id);
            return this;
        }
    }

    private record AutoBuildCtxImpl(
            RobotCore<?> robot,
            RobotAuto autos,
            Map<String, AutoRoutine> scopedAutos) implements AutoBuildCtx {
        private AutoBuildCtxImpl {
            Objects.requireNonNull(robot, "robot");
            Objects.requireNonNull(autos, "autos");
            Objects.requireNonNull(scopedAutos, "scopedAutos");
        }

        @Override
        public boolean hasAuto(String id) {
            return id != null && (scopedAutos.containsKey(id) || autos.hasAuto(id));
        }

        @Override
        public Command auto(String id) {
            Objects.requireNonNull(id, "id");
            return autos.deferredAuto(scopedAutos, id, OptionalInt.empty());
        }

        @Override
        public Command auto(String id, int splitIndex) {
            Objects.requireNonNull(id, "id");
            return autos.deferredAuto(scopedAutos, id, OptionalInt.of(splitIndex));
        }

        @Override
        public AutoPlan.Context planContext() {
            return new AutoPlan.Context() {
                @Override
                public Command resolve(AutoPlan.StepRef ref) {
                    return ref.splitIndex().isPresent()
                            ? auto(ref.key().id(), ref.splitIndex().getAsInt())
                            : auto(ref.key().id());
                }

                @Override
                public Optional<Pose2d> startingPose(AutoPlan.StepRef ref) {
                    return autos.startingPoseFor(scopedAutos, ref.key().id(), ref.splitIndex());
                }

                @Override
                public Command resetPose(Pose2d pose) {
                    return autos.resetPose(pose);
                }
            };
        }
    }

    private record SimpleNamedCommandKey(String id, String displayName) implements NamedCommandKey {
        private SimpleNamedCommandKey {
            Objects.requireNonNull(id, "id");
            Objects.requireNonNull(displayName, "displayName");
        }
    }

    private record SimpleAutoKey(String id, String displayName) implements AutoKey {
        private SimpleAutoKey {
            Objects.requireNonNull(id, "id");
            Objects.requireNonNull(displayName, "displayName");
        }
    }

    private record SimpleAutoInput<T>(String id, Class<T> type) implements AutoInput<T> {
        private SimpleAutoInput {
            Objects.requireNonNull(id, "id");
            Objects.requireNonNull(type, "type");
        }
    }

    private record AutoInputBinding<T>(AutoInput<T> key, Supplier<T> supplier) {
        private AutoInputBinding {
            Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
        }
    }

    public enum AutoSource {
        PATH_PLANNER,
        CHOREO,
        CUSTOM
    }

    /**
     * Immutable description for an autonomous routine option.
     */
    public record AutoRoutine(
            AutoKey key,
            AutoSource source,
            String reference,
            Supplier<Command> factory,
            Pose2d startingPose,
            boolean hasStartingPose,
            Boolean autoInitResetOverride) {

        public AutoRoutine {
            Objects.requireNonNull(key, "key");
            Objects.requireNonNull(source, "source");
            Objects.requireNonNull(factory, "factory");
            reference = reference != null ? reference : key.id();
            startingPose = startingPose != null ? startingPose : new Pose2d();
            autoInitResetOverride = autoInitResetOverride;
        }

        public Command createCommand() {
            return factory.get();
        }

        public AutoRoutine withStartingPose(Pose2d pose) {
            return new AutoRoutine(
                    key,
                    source,
                    reference,
                    factory,
                    pose,
                    true,
                    autoInitResetOverride);
        }

        public AutoRoutine withAutoInitResetOverride(Boolean override) {
            return new AutoRoutine(
                    key,
                    source,
                    reference,
                    factory,
                    startingPose,
                    hasStartingPose,
                    override);
        }
    }

    public RobotAuto registerNamedCommand(NamedCommandKey key, Command command) {
        return registerNamedCommand(key, () -> command);
    }

    public RobotAuto registerNamedCommand(String id, Command command) {
        return registerNamedCommand(NamedCommandKey.of(id), command);
    }

    public RobotAuto registerNamedCommand(NamedCommandKey key, Supplier<Command> supplier) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(supplier, "supplier");
        String id = requireNonBlank(key.id(), "named command id");
        if (namedCommandSuppliers.containsKey(id)) {
            throw new IllegalArgumentException("Named command already registered: " + id);
        }
        Supplier<Command> tracedSupplier = traceNamedCommandSupplier(id, supplier);
        namedCommandSuppliers.put(id, tracedSupplier);
        boolean bound = bindNamedCommandToAvailableBackends(id, tracedSupplier);
        if (!bound) {
            DriverStation.reportWarning(
                    "No auto backend available; named command '" + id + "' not bound to vendor API.",
                    false);
        }
        if (registrationFinalized) {
            DriverStation.reportWarning(
                    "Named command '" + id + "' registered after autos were finalized; prefer registering named commands first.",
                    false);
        }
        return this;
    }

    public RobotAuto registerNamedCommand(String id, Supplier<Command> supplier) {
        return registerNamedCommand(NamedCommandKey.of(id), supplier);
    }

    public RobotAuto registerNamedCommand(NamedCommandKey key, Runnable action) {
        Objects.requireNonNull(action, "action");
        return registerNamedCommand(key, new InstantCommand(action));
    }

    public RobotAuto registerNamedCommand(String id, Runnable action) {
        return registerNamedCommand(NamedCommandKey.of(id), action);
    }

    public <T> RobotAuto registerInput(AutoInput<T> key, Supplier<T> supplier) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(supplier, "supplier");
        String id = requireNonBlank(key.id(), "auto input id");
        AutoInputBinding<?> existing = autoInputs.get(id);
        if (existing != null) {
            throw new IllegalArgumentException("Auto input already registered: " + id);
        }
        autoInputs.put(id, new AutoInputBinding<>(key, supplier));
        return this;
    }

    public boolean hasNamedCommand(NamedCommandKey key) {
        return key != null && namedCommandSuppliers.containsKey(key.id());
    }

    public boolean hasNamedCommand(String id) {
        return id != null && namedCommandSuppliers.containsKey(id);
    }

    public boolean hasAuto(AutoKey key) {
        return key != null && autoRoutines.containsKey(key.id());
    }

    public boolean hasAuto(String id) {
        return id != null && autoRoutines.containsKey(id);
    }

    public boolean hasInput(AutoInput<?> key) {
        return key != null && autoInputs.containsKey(key.id());
    }

    public boolean hasInput(String id) {
        return id != null && autoInputs.containsKey(id);
    }

    public <T> Supplier<T> inputSupplier(AutoInput<T> key) {
        Objects.requireNonNull(key, "key");
        AutoInputBinding<?> binding = autoInputs.get(key.id());
        if (binding == null) {
            throw new IllegalStateException("Auto input not registered: " + key.id());
        }
        if (!key.type().isAssignableFrom(binding.key().type())) {
            throw new IllegalStateException(
                    "Auto input type mismatch for '" + key.id() + "': requested "
                            + key.type().getSimpleName() + " but registered "
                            + binding.key().type().getSimpleName());
        }
        @SuppressWarnings("unchecked")
        Supplier<T> typed = (Supplier<T>) binding.supplier();
        return typed;
    }

    public <T> T inputValue(AutoInput<T> key) {
        return inputSupplier(key).get();
    }

    /**
     * Attaches the owning {@link RobotCore}. This is called automatically by {@link RobotCore}.
     */
    public RobotAuto attachRobotCore(RobotCore<?> robot) {
        Objects.requireNonNull(robot, "robot");
        if (robotCore != null && robotCore != robot) {
            throw new IllegalStateException(
                    "RobotAuto is already attached to a different RobotCore instance.");
        }
        robotCore = robot;
        return this;
    }

    public Optional<RobotCore<?>> robotCore() {
        return Optional.ofNullable(robotCore);
    }

    private RobotCore<?> requireRobotCore() {
        RobotCore<?> robot = robotCore;
        if (robot == null) {
            throw new IllegalStateException(
                    "RobotAuto is not attached to a RobotCore. "
                            + "Use autos.attachRobotCore(robot) once.");
        }
        return robot;
    }

    public RobotAuto registerProgram(AutoProgram program) {
        RobotCore<?> robot = requireRobotCore();
        Objects.requireNonNull(program, "program");
        String programId = Objects.requireNonNull(program.id(), "AutoProgram.id() returned null");
        if (programId.isBlank()) {
            throw new IllegalArgumentException("AutoProgram.id() cannot be blank for " + program.getClass().getName());
        }
        boolean hadProgramAutoBeforeRegister = hasAuto(programId);

        Map<String, AutoRoutine> scopedAutos = new LinkedHashMap<>();

        AutoRegisterCtx regCtx = new AutoRegisterCtxImpl(robot, this, programId, scopedAutos);
        program.register(regCtx);

        AutoBuildCtx buildCtx = new AutoBuildCtxImpl(robot, this, Map.copyOf(scopedAutos));
        AutoRoutine routine = Objects.requireNonNull(
                program.build(buildCtx),
                "AutoProgram.build(ctx) returned null for " + program.getClass().getName());
        if (!programId.equals(routine.key().id())) {
            throw new IllegalStateException(
                    "AutoProgram id mismatch for " + program.getClass().getName()
                            + ": program.id()=\"" + programId
                            + "\", routine.key().id()=\"" + routine.key().id() + "\"");
        }
        if (hasAuto(programId)) {
            if (hadProgramAutoBeforeRegister) {
                throw new IllegalArgumentException(
                        "Auto already registered before program registration: " + programId
                                + " (program " + program.getClass().getName() + ")");
            }
            // register(...) may have registered dependencies that collide with the program id.
            // Program build output is authoritative for chooser behavior.
            autoRoutines.put(programId, routine);
            resetChoosers();
            return this;
        }
        return registerAuto(routine);
    }

    public final RobotAuto registerPrograms(AutoProgram... programs) {
        requireRobotCore();
        if (programs == null) {
            return this;
        }
        for (AutoProgram program : programs) {
            if (program == null) {
                continue;
            }
            registerProgram(program);
        }
        return this;
    }

    @SafeVarargs
    public final RobotAuto registerPrograms(Class<? extends AutoProgram>... programTypes) {
        requireRobotCore();
        if (programTypes == null) {
            return this;
        }
        for (Class<? extends AutoProgram> programType : programTypes) {
            if (programType == null) {
                continue;
            }
            AutoProgram program = newInstance(programType);
            registerProgram(program);
        }
        return this;
    }

    private static <T> T newInstance(Class<T> type) {
        try {
            return type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException ex) {
            throw new IllegalArgumentException(
                    "Failed to construct auto program " + type.getName()
                            + ". Ensure it has an accessible no-arg constructor.",
                    ex);
        }
    }

    public RobotAuto registerAuto(AutoRoutine routine) {
        Objects.requireNonNull(routine, "routine");
        String id = requireNonBlank(routine.key().id(), "auto id");
        if (autoRoutines.containsKey(id)) {
            throw new IllegalArgumentException("Auto already registered: " + id);
        }
        autoRoutines.put(id, routine);
        resetChoosers();
        return this;
    }

    public RobotAuto registerAutos(AutoRoutine... routines) {
        if (routines == null) {
            return this;
        }
        for (AutoRoutine routine : routines) {
            if (routine != null) {
                registerAuto(routine);
            }
        }
        return this;
    }

    public RobotAuto registerAuto(AutoKey key, Supplier<Command> factory) {
        return registerAuto(custom(key, factory));
    }

    public RobotAuto registerAuto(String id, Supplier<Command> factory) {
        return registerAuto(AutoKey.of(id), factory);
    }

    /**
     * Register a trajectory-backed auto. The chooser id defaults to {@code reference}.
     */
    public RobotAuto registerPath(String reference, TrajectorySource source) {
        return registerPath(reference, source, reference);
    }

    /**
     * Register a trajectory-backed auto with an alias id.
     */
    public RobotAuto registerPath(String reference, TrajectorySource source, String id) {
        String resolvedReference = requireNonBlank(reference, "reference");
        String resolvedId = requireNonBlank(id, "id");
        try {
            validatePathReference(source, resolvedReference);
            return registerAuto(path(AutoKey.of(resolvedId), source, resolvedReference));
        } catch (IllegalArgumentException referenceError) {
            // Common user mistake: path(aliasId, source, reference) instead of path(reference, source, aliasId).
            // If the third argument validates as a reference, auto-correct to keep startup fail-fast but ergonomic.
            try {
                validatePathReference(source, resolvedId);
            } catch (IllegalArgumentException idError) {
                throw referenceError;
            }
            DriverStation.reportWarning(
                    "Auto path registration arguments were likely swapped. "
                            + "Expected path(reference, source, id), but received reference=\""
                            + resolvedReference + "\" and id=\"" + resolvedId + "\". "
                            + "Using reference=\"" + resolvedId + "\" and id=\"" + resolvedReference + "\".",
                    false);
            return registerAuto(path(AutoKey.of(resolvedReference), source, resolvedId));
        }
    }

    public RobotAuto registerPath(AutoKey key, TrajectorySource source, String reference) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(source, "source");
        String resolvedReference = requireNonBlank(reference, "reference");
        validatePathReference(source, resolvedReference);
        return registerAuto(path(key, source, resolvedReference));
    }

    public RobotAuto setAutoPlanResetter(java.util.function.Function<Pose2d, Command> resetter) {
        this.autoPlanResetter = resetter;
        return this;
    }

    public RobotAuto setAutoTraceEnabled(boolean enabled) {
        this.autoTraceEnabled = enabled;
        return this;
    }

    public RobotAuto enableAutoTrace() {
        return setAutoTraceEnabled(true);
    }

    public RobotAuto disableAutoTrace() {
        return setAutoTraceEnabled(false);
    }

    public boolean isAutoTraceEnabled() {
        return autoTraceEnabled;
    }

    public RobotAuto setAutoTraceSink(Consumer<String> sink) {
        this.autoTraceSink = Objects.requireNonNull(sink, "sink");
        return this;
    }

    public List<AutoTraceEvent> getAutoTraceLog() {
        return autoTraceLog.snapshot();
    }

    public List<AutoTraceEvent> getAutoTraceLog(int limit) {
        return autoTraceLog.snapshot(limit);
    }

    public int getAutoTraceLogCount() {
        return autoTraceLog.count();
    }

    public int getAutoTraceLogCapacity() {
        return autoTraceLogCapacity;
    }

    public RobotAuto clearAutoTraceLog() {
        autoTraceLog.clear();
        return this;
    }

    public RobotAuto setAutoInitReset(AutoKey key, Boolean resetOnInit) {
        Objects.requireNonNull(key, "key");
        AutoRoutine routine = autoRoutines.get(key.id());
        if (routine == null) {
            throw new IllegalArgumentException("Auto not registered: " + key.id());
        }
        autoRoutines.put(key.id(), routine.withAutoInitResetOverride(resetOnInit));
        resetChoosers();
        return this;
    }

    public RobotAuto setAutoInitReset(String id, Boolean resetOnInit) {
        return setAutoInitReset(AutoKey.of(id), resetOnInit);
    }

    public Optional<AutoRoutine> getAuto(AutoKey key) {
        if (key == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(autoRoutines.get(key.id()));
    }

    public Optional<AutoRoutine> getAuto(String id) {
        if (id == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(autoRoutines.get(id));
    }

    public Collection<AutoRoutine> getAutos() {
        return Collections.unmodifiableCollection(autoRoutines.values());
    }

    public SendableChooser<AutoRoutine> createChooser(AutoKey defaultAuto) {
        Objects.requireNonNull(defaultAuto, "defaultAuto");
        if (autoRoutines.isEmpty()) {
            throw new IllegalStateException("No autos registered.");
        }
        AutoRoutine defaultRoutine = autoRoutines.get(defaultAuto.id());
        if (defaultRoutine == null) {
            throw new IllegalArgumentException("Default auto not registered: " + defaultAuto.id());
        }
        SendableChooser<AutoRoutine> newChooser = new SendableChooser<>();
        newChooser.setDefaultOption(defaultRoutine.key().displayName(), defaultRoutine);
        for (AutoRoutine routine : autoRoutines.values()) {
            if (routine.key().id().equals(defaultAuto.id())) {
                continue;
            }
            newChooser.addOption(routine.key().displayName(), routine);
        }
        chooser = newChooser;
        commandChooser = null;
        return chooser;
    }

    public SendableChooser<AutoRoutine> getAutoChooser() {
        return chooser;
    }

    public SendableChooser<Command> createCommandChooser(AutoKey defaultAuto) {
        createChooser(defaultAuto);
        AutoRoutine defaultRoutine = autoRoutines.get(defaultAuto.id());
        SendableChooser<Command> newChooser = new SendableChooser<>();
        newChooser.setDefaultOption(defaultRoutine.key().displayName(), deferredCommand(defaultRoutine));
        for (AutoRoutine routine : autoRoutines.values()) {
            if (routine.key().id().equals(defaultAuto.id())) {
                continue;
            }
            newChooser.addOption(routine.key().displayName(), deferredCommand(routine));
        }
        commandChooser = newChooser;
        return newChooser;
    }

    public SendableChooser<Command> getCommandChooser() {
        return commandChooser;
    }

    public Optional<AutoRoutine> getSelectedAuto() {
        return Optional.ofNullable(chooser).map(SendableChooser::getSelected);
    }

    public Optional<List<Pose2d>> getSelectedAutoPoses() {
        return getSelectedAuto().flatMap(this::getAutoPoses);
    }

    public Optional<List<Pose2d>> getAutoPoses(AutoRoutine routine) {
        if (routine == null || routine.source() == AutoSource.CUSTOM) {
            return Optional.empty();
        }
        return AutoBackends.forSource(routine.source())
                .flatMap(backend -> backend.getAutoPoses(routine.source(), routine.reference()));
    }

    public Optional<Command> buildSelectedCommand() {
        finalizeRegistration();
        if (commandChooser != null) {
            Command selected = commandChooser.getSelected();
            traceAuto("SELECTED command chooser -> " + commandLabel(selected));
            return Optional.ofNullable(selected);
        }
        return getSelectedAuto().map(routine -> {
            traceAuto("SELECTED auto=\"" + routine.key().id()
                    + "\" source=" + routine.source()
                    + " reference=\"" + routine.reference() + "\"");
            return routine.createCommand();
        });
    }

    public ProfiledPIDController getXController() {
        return xController;
    }

    public ProfiledPIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public RobotAuto setPoseControllers(ProfiledPIDController xController,
                                        ProfiledPIDController yController,
                                        ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        return this;
    }

    public RobotAuto setStartingPose(AutoKey key, Pose2d pose) {
        Objects.requireNonNull(key, "key");
        AutoRoutine routine = autoRoutines.get(key.id());
        if (routine == null) {
            throw new IllegalArgumentException("Auto not registered: " + key.id());
        }
        AutoRoutine updated = routine.withStartingPose(pose);
        autoRoutines.put(key.id(), updated);
        resetChoosers();
        return this;
    }

    public RobotAuto setStartingPose(String autoId, Pose2d pose) {
        return setStartingPose(AutoKey.of(autoId), pose);
    }

    public static AutoRoutine path(AutoKey key, TrajectorySource source, String reference) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(source, "source");
        String resolvedReference = reference != null ? reference : key.id();
        Supplier<Command> factory = () -> buildPathCommand(source.autoSource(), resolvedReference);
        return new AutoRoutine(key, source.autoSource(), resolvedReference, factory, new Pose2d(), false, null);
    }

    public static AutoRoutine pathPlanner(AutoKey key, String autoName) {
        return path(key, TrajectorySource.PATH_PLANNER, autoName);
    }

    public static AutoRoutine choreo(AutoKey key, String trajectoryName) {
        return path(key, TrajectorySource.CHOREO, trajectoryName);
    }

    public static AutoRoutine custom(AutoKey key, Supplier<Command> factory) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(factory, "factory");
        return new AutoRoutine(key, AutoSource.CUSTOM, key.id(), factory, new Pose2d(), false, null);
    }

    private Command deferredCommand(AutoRoutine routine) {
        return Commands.defer(routine::createCommand, Set.of());
    }

    /**
     * Finalizes registration for the current robot init cycle. This rebinding step ensures named commands
     * are bound to vendor APIs before any autonomous routine is built (some libraries cache missing
     * named-command references during auto construction).
     *
     * <p>This is safe to call multiple times.</p>
     */
    public void finalizeRegistration() {
        if (registrationFinalized) {
            return;
        }
        for (Map.Entry<String, Supplier<Command>> entry : namedCommandSuppliers.entrySet()) {
            bindNamedCommandToAvailableBackends(entry.getKey(), entry.getValue());
        }
        registrationFinalized = true;
    }

    private boolean bindNamedCommandToAvailableBackends(String id, Supplier<Command> supplier) {
        // Always attempt to bind to both backends if present. Some robots include both vendor deps.
        // Use deferred proxies so vendor APIs that eagerly call supplier.get() don't construct heavy commands at init.
        Supplier<Command> deferred = () -> Commands.deferredProxy(supplier);
        boolean boundAny = false;
        boundAny |= AutoBackends.forSource(AutoSource.PATH_PLANNER)
                .filter(backend -> boundPathPlannerNamedCommands.add(id))
                .map(backend -> backend.registerNamedCommand(id, deferred))
                .orElse(false);
        boundAny |= AutoBackends.forSource(AutoSource.CHOREO)
                .filter(backend -> boundChoreoNamedCommands.add(id))
                .map(backend -> backend.registerNamedCommand(id, deferred))
                .orElse(false);
        return boundAny;
    }

    private Command deferredAuto(String id, OptionalInt splitIndex) {
        return deferredAuto(Collections.emptyMap(), id, splitIndex);
    }

    private Command deferredAuto(Map<String, AutoRoutine> scopedAutos, String id, OptionalInt splitIndex) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(id, "id");
        if (splitIndex.isPresent()) {
            int split = splitIndex.getAsInt();
            if (split < 0) {
                throw new IllegalArgumentException("splitIndex must be >= 0");
            }
            String explicitSplitId = id + "." + split;
            AutoRoutine explicitSplit = scopedAutos.get(explicitSplitId);
            if (explicitSplit == null) {
                explicitSplit = autoRoutines.get(explicitSplitId);
            }
            if (explicitSplit != null) {
                return traceAutoLifecycle(
                        "auto(" + explicitSplitId + ")",
                        "splitOf=" + id
                                + " source=" + explicitSplit.source()
                                + " reference=\"" + explicitSplit.reference() + "\"",
                        deferredCommand(explicitSplit));
            }
            AutoRoutine resolvedBase = scopedAutos.get(id);
            if (resolvedBase == null) {
                resolvedBase = autoRoutines.get(id);
            }
            if (resolvedBase == null) {
                throw new IllegalStateException("Auto not registered: " + id + " (or split " + explicitSplitId + ")");
            }
            if (resolvedBase.source() == AutoSource.CUSTOM) {
                throw new IllegalStateException("Auto split requested for CUSTOM auto: " + id + "." + split);
            }
            String splitReference = resolvedBase.reference() + "." + split;
            AutoSource splitSource = resolvedBase.source();
            Command splitCommand = Commands.defer(() -> buildPathCommand(splitSource, splitReference), Set.of());
            return traceAutoLifecycle(
                    "auto(" + id + "." + split + ")",
                    "derivedFrom=" + id
                            + " source=" + splitSource
                            + " reference=\"" + splitReference + "\"",
                    splitCommand);
        }

        AutoRoutine routine = scopedAutos.get(id);
        if (routine == null) {
            routine = autoRoutines.get(id);
        }
        if (routine == null) {
            throw new IllegalStateException("Auto not registered: " + id);
        }
        return traceAutoLifecycle(
                "auto(" + id + ")",
                "source=" + routine.source() + " reference=\"" + routine.reference() + "\"",
                deferredCommand(routine));
    }

    private Optional<Pose2d> startingPoseFor(String id, OptionalInt splitIndex) {
        return startingPoseFor(Collections.emptyMap(), id, splitIndex);
    }

    private Optional<Pose2d> startingPoseFor(Map<String, AutoRoutine> scopedAutos, String id, OptionalInt splitIndex) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        AutoRoutine base = scopedAutos.get(id);
        if (base == null) {
            base = autoRoutines.get(id);
        }
        if (splitIndex.isPresent()) {
            String splitId = id + "." + splitIndex.getAsInt();
            AutoRoutine split = scopedAutos.get(splitId);
            if (split == null) {
                split = autoRoutines.get(splitId);
            }
            if (split != null && split.hasStartingPose()) {
                return Optional.ofNullable(split.startingPose());
            }
        }
        if (base == null || !base.hasStartingPose()) {
            return Optional.empty();
        }
        return Optional.ofNullable(base.startingPose());
    }

    private static Command buildPathCommand(AutoSource source, String reference) {
        return backendForSource(source)
                .buildAuto(source, reference)
                .orElseThrow(() -> new IllegalStateException(
                        "Auto backend could not build " + source + " routine for reference \"" + reference + "\"."));
    }

    private void validatePathReference(TrajectorySource source, String reference) {
        AutoSource autoSource = source.autoSource();
        AutoBackend backend = backendForSource(autoSource);
        Optional<List<Pose2d>> poses;
        try {
            poses = backend.getAutoPoses(autoSource, reference);
        } catch (RuntimeException ex) {
            throw new IllegalArgumentException(
                    "Failed to validate " + source + " reference \"" + reference + "\".", ex);
        }
        if (poses.isPresent() && !poses.get().isEmpty()) {
            return;
        }

        // For split references like "Name.1", some PathPlanner pose loaders may not expose split
        // poses reliably even when runtime execution supports them. If the base reference validates,
        // allow registration and defer split-index validity to runtime build.
        //
        // Keep CHOREO strict so invalid split indices fail-fast (e.g., Name.1 when only split 0
        // exists) instead of silently falling back to base behavior.
        int dot = reference.lastIndexOf('.');
        if (source == TrajectorySource.PATH_PLANNER && dot > 0 && dot < reference.length() - 1) {
            String maybeIndex = reference.substring(dot + 1);
            try {
                Integer.parseInt(maybeIndex);
                String baseReference = reference.substring(0, dot);
                Optional<List<Pose2d>> basePoses = backend.getAutoPoses(autoSource, baseReference);
                if (basePoses.isPresent() && !basePoses.get().isEmpty()) {
                    return;
                }
            } catch (NumberFormatException ignored) {
                // Not a split suffix; keep fail-fast behavior below.
            } catch (RuntimeException ex) {
                throw new IllegalArgumentException(
                        "Failed to validate base reference for split \"" + reference + "\".", ex);
            }
        }

        throw new IllegalArgumentException(
                "Auto reference \"" + reference + "\" could not be resolved for source " + source + ". "
                        + "Registration is fail-fast; verify the file exists and source is correct.");
    }

    private static AutoBackend backendForSource(AutoSource source) {
        return AutoBackends.forSource(source)
                .orElseThrow(() -> new IllegalStateException("No auto backend available for source: " + source));
    }

    private void registerScopedPath(
            Map<String, AutoRoutine> scopedAutos,
            String programId,
            String reference,
            TrajectorySource source,
            String id) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        String resolvedReference = requireNonBlank(reference, "reference");
        String resolvedId = requireNonBlank(id, "id");
        try {
            validatePathReference(source, resolvedReference);
            putScopedPath(scopedAutos, programId, path(AutoKey.of(resolvedId), source, resolvedReference));
        } catch (IllegalArgumentException referenceError) {
            // Common user mistake: path(aliasId, source, reference) instead of path(reference, source, aliasId).
            // If the third argument validates as a reference, auto-correct to keep startup fail-fast but ergonomic.
            try {
                validatePathReference(source, resolvedId);
            } catch (IllegalArgumentException idError) {
                throw referenceError;
            }
            DriverStation.reportWarning(
                    "Auto path registration arguments were likely swapped. "
                            + "Expected path(reference, source, id), but received reference=\""
                            + resolvedReference + "\" and id=\"" + resolvedId + "\". "
                            + "Using reference=\"" + resolvedId + "\" and id=\"" + resolvedReference + "\"."
                            + " Program=\"" + programId + "\".",
                    false);
            putScopedPath(scopedAutos, programId, path(AutoKey.of(resolvedReference), source, resolvedId));
        }
    }

    private static void putScopedPath(Map<String, AutoRoutine> scopedAutos, String programId, AutoRoutine routine) {
        String id = requireNonBlank(routine.key().id(), "auto id");
        if (scopedAutos.containsKey(id)) {
            throw new IllegalArgumentException(
                    "Auto path already registered in program \"" + programId + "\": " + id);
        }
        scopedAutos.put(id, routine);
    }

    private Command resetPose(Pose2d pose) {
        if (autoPlanResetter == null) {
            DriverStation.reportWarning("AutoPlan reset requested but no resetter configured.", false);
            return Commands.none();
        }
        return autoPlanResetter.apply(pose);
    }

    private static String requireNonBlank(String value, String fieldName) {
        Objects.requireNonNull(value, fieldName);
        String trimmed = value.trim();
        if (trimmed.isEmpty()) {
            throw new IllegalArgumentException(fieldName + " cannot be blank");
        }
        return trimmed;
    }

    private void traceAutoDecision(String decisionType, String detail) {
        traceAuto("DECISION " + decisionType + " | " + detail);
    }

    private Command traceAutoLifecycle(String label, String detail, Command command) {
        Objects.requireNonNull(label, "label");
        Objects.requireNonNull(command, "command");
        return command
                .beforeStarting(() -> {
                    String suffix = (detail == null || detail.isBlank()) ? "" : " | " + detail;
                    traceAuto("START " + label + suffix);
                })
                .finallyDo(interrupted -> traceAuto("END " + label + " interrupted=" + interrupted));
    }

    private void traceAuto(String message) {
        if (!autoTraceEnabled) {
            return;
        }
        String payload = "[AUTO TRACE] " + Objects.requireNonNullElse(message, "<null>");
        AutoTraceEvent event = appendAutoTraceEvent(payload);
        try {
            autoTraceSink.accept(formatAutoTraceLine(event));
        } catch (RuntimeException ex) {
            DriverStation.reportWarning("Auto trace sink failure: " + ex.getMessage(), false);
        }
    }

    private AutoTraceEvent appendAutoTraceEvent(String payload) {
        Pose2d pose = currentFieldPoseOrNull();
        Double x = pose != null ? pose.getX() : null;
        Double y = pose != null ? pose.getY() : null;
        Double headingDeg = pose != null ? pose.getRotation().getDegrees() : null;
        String level = "INFO";
        String category = "trace";
        String line = "[auto] "
                + level.toLowerCase(Locale.ROOT)
                + " " + category + ": " + payload;
        return autoTraceLog.append((sequence, timestampSeconds) -> new AutoTraceEvent(
                sequence,
                timestampSeconds,
                "auto",
                level,
                category,
                x,
                y,
                headingDeg,
                payload,
                line));
    }

    private String formatAutoTraceLine(AutoTraceEvent event) {
        StringBuilder out = new StringBuilder(160);
        out.append(event.message());
        out.append(" | t=").append(formatDouble(event.timestampSeconds(), 3));
        if (event.xMeters() != null && event.yMeters() != null && event.headingDeg() != null) {
            out.append(" pose=(")
                    .append(formatDouble(event.xMeters(), 3)).append(", ")
                    .append(formatDouble(event.yMeters(), 3)).append(", ")
                    .append(formatDouble(event.headingDeg(), 1)).append("deg)");
        } else {
            out.append(" pose=<unknown>");
        }
        return out.toString();
    }

    private static String formatDouble(double value, int decimals) {
        return String.format(Locale.US, "%." + decimals + "f", value);
    }

    private Pose2d currentFieldPoseOrNull() {
        RobotCore<?> core = robotCore;
        if (core == null) {
            return null;
        }
        try {
            RobotLocalization<?> localization = core.getLocalization();
            if (localization == null) {
                return null;
            }
            return localization.getFieldPose();
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    private Supplier<Command> traceNamedCommandSupplier(String id, Supplier<Command> supplier) {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(supplier, "supplier");
        return () -> {
            traceAuto("EVENT namedCommand(" + id + ") trigger");
            Command command = supplier.get();
            if (command == null) {
                traceAutoDecision("namedCommand", "id=" + id + " returned null; using none()");
                return Commands.none();
            }
            return traceAutoLifecycle("namedCommand(" + id + ")", null, command);
        };
    }

    private static String commandLabel(Command command) {
        if (command == null) {
            return "<none>";
        }
        String name = command.getName();
        if (name == null || name.isBlank()) {
            return command.getClass().getSimpleName();
        }
        return name;
    }

    private void resetChoosers() {
        chooser = null;
        commandChooser = null;
    }
}
