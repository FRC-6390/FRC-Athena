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
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.localization.RobotLocalization;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Aggregates named command and autonomous routine registration for a robot.
 */
public class RobotAuto {

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
         */
        default AutoRegisterCtx path(String reference, TrajectorySource source) {
            autos().registerPath(reference, source);
            return this;
        }

        /**
         * Register a trajectory by file/reference name and alias it under a different chooser id.
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
            return commands.length == 0 ? Commands.none() : Commands.sequence(commands);
        }

        default Command sequence(String... autoIds) {
            return sequence(commandsForIds(autos(), autoIds));
        }

        default Command parallel(Command... steps) {
            Command[] commands = nonNullCommands(steps);
            return commands.length == 0 ? Commands.none() : Commands.parallel(commands);
        }

        default Command parallel(String... autoIds) {
            return parallel(commandsForIds(autos(), autoIds));
        }

        default Command race(Command... steps) {
            Command[] commands = nonNullCommands(steps);
            return commands.length == 0 ? Commands.none() : Commands.race(commands);
        }

        default Command race(String... autoIds) {
            return race(commandsForIds(autos(), autoIds));
        }

        default Command deadline(Command deadline, Command... others) {
            Objects.requireNonNull(deadline, "deadline");
            Command[] commands = nonNullCommands(others);
            return commands.length == 0 ? deadline : Commands.deadline(deadline, commands);
        }

        default Command deadline(String deadlineAutoId, String... otherAutoIds) {
            return deadline(auto(deadlineAutoId), commandsForIds(autos(), otherAutoIds));
        }

        default Command select(BooleanSupplier condition, Command ifTrue, Command ifFalse) {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(ifTrue, "ifTrue");
            Objects.requireNonNull(ifFalse, "ifFalse");
            return Commands.either(ifTrue, ifFalse, condition);
        }

        default Command select(BooleanSupplier condition, String ifTrueId, String ifFalseId) {
            return select(condition, auto(ifTrueId), auto(ifFalseId));
        }

        default Command selectIndex(IntSupplier selector, Command fallback, Command... options) {
            Objects.requireNonNull(selector, "selector");
            Objects.requireNonNull(fallback, "fallback");
            Command[] resolvedOptions = nonNullCommands(options);
            return Commands.defer(() -> {
                int index = selector.getAsInt();
                if (index < 0 || index >= resolvedOptions.length) {
                    return fallback;
                }
                return resolvedOptions[index];
            }, Set.of());
        }

        default Command selectIndex(IntSupplier selector, String fallbackAutoId, String... optionAutoIds) {
            return selectIndex(selector, auto(fallbackAutoId), commandsForIds(autos(), optionAutoIds));
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
            return Commands.race(
                    Commands.waitUntil(condition),
                    Commands.sequence(Commands.waitSeconds(timeoutSeconds), onTimeout));
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
            return Commands.waitUntil(condition).withTimeout(timeoutSeconds);
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

        private static Command[] commandsForIds(RobotAuto autos, String... ids) {
            if (ids == null || ids.length == 0) {
                return new Command[0];
            }
            List<Command> commands = new ArrayList<>();
            for (String id : ids) {
                if (id == null || id.isBlank()) {
                    continue;
                }
                commands.add(autos.deferredAuto(id, OptionalInt.empty()));
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

    private record AutoRegisterCtxImpl(RobotCore<?> robot, RobotAuto autos) implements AutoRegisterCtx {
        private AutoRegisterCtxImpl {
            Objects.requireNonNull(robot, "robot");
            Objects.requireNonNull(autos, "autos");
        }
    }

    private record AutoBuildCtxImpl(RobotCore<?> robot, RobotAuto autos) implements AutoBuildCtx {
        private AutoBuildCtxImpl {
            Objects.requireNonNull(robot, "robot");
            Objects.requireNonNull(autos, "autos");
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
        namedCommandSuppliers.put(id, supplier);
        boolean bound = bindNamedCommandToAvailableBackends(id, supplier);
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

    public RobotAuto registerProgram(RobotCore<?> robot, AutoProgram program) {
        Objects.requireNonNull(robot, "robot");
        Objects.requireNonNull(program, "program");
        String programId = Objects.requireNonNull(program.id(), "AutoProgram.id() returned null");
        if (programId.isBlank()) {
            throw new IllegalArgumentException("AutoProgram.id() cannot be blank for " + program.getClass().getName());
        }

        AutoRegisterCtx regCtx = new AutoRegisterCtxImpl(robot, this);
        program.register(regCtx);

        AutoBuildCtx buildCtx = new AutoBuildCtxImpl(robot, this);
        AutoRoutine routine = Objects.requireNonNull(
                program.build(buildCtx),
                "AutoProgram.build(ctx) returned null for " + program.getClass().getName());
        if (!programId.equals(routine.key().id())) {
            throw new IllegalStateException(
                    "AutoProgram id mismatch for " + program.getClass().getName()
                            + ": program.id()=\"" + programId
                            + "\", routine.key().id()=\"" + routine.key().id() + "\"");
        }
        return registerAuto(routine);
    }

    public final RobotAuto registerPrograms(RobotCore<?> robot, AutoProgram... programs) {
        Objects.requireNonNull(robot, "robot");
        if (programs == null) {
            return this;
        }
        for (AutoProgram program : programs) {
            if (program == null) {
                continue;
            }
            registerProgram(robot, program);
        }
        return this;
    }

    @SafeVarargs
    public final RobotAuto registerPrograms(RobotCore<?> robot, Class<? extends AutoProgram>... programTypes) {
        Objects.requireNonNull(robot, "robot");
        if (programTypes == null) {
            return this;
        }
        for (Class<? extends AutoProgram> programType : programTypes) {
            if (programType == null) {
                continue;
            }
            AutoProgram program = newInstance(programType);
            registerProgram(robot, program);
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
        return registerPath(AutoKey.of(resolvedId), source, resolvedReference);
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
            return Optional.ofNullable(commandChooser.getSelected());
        }
        return getSelectedAuto().map(AutoRoutine::createCommand);
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
        Objects.requireNonNull(id, "id");
        if (splitIndex.isPresent()) {
            int split = splitIndex.getAsInt();
            if (split < 0) {
                throw new IllegalArgumentException("splitIndex must be >= 0");
            }
            String explicitSplitId = id + "." + split;
            AutoRoutine explicitSplit = autoRoutines.get(explicitSplitId);
            if (explicitSplit != null) {
                return deferredCommand(explicitSplit);
            }
            AutoRoutine base = autoRoutines.get(id);
            if (base == null) {
                throw new IllegalStateException("Auto not registered: " + id + " (or split " + explicitSplitId + ")");
            }
            if (base.source() == AutoSource.CUSTOM) {
                throw new IllegalStateException("Auto split requested for CUSTOM auto: " + id + "." + split);
            }
            String splitReference = base.reference() + "." + split;
            return Commands.defer(() -> buildPathCommand(base.source(), splitReference), Set.of());
        }

        AutoRoutine routine = autoRoutines.get(id);
        if (routine == null) {
            throw new IllegalStateException("Auto not registered: " + id);
        }
        return deferredCommand(routine);
    }

    private Optional<Pose2d> startingPoseFor(String id, OptionalInt splitIndex) {
        AutoRoutine base = autoRoutines.get(id);
        if (splitIndex.isPresent()) {
            String splitId = id + "." + splitIndex.getAsInt();
            AutoRoutine split = autoRoutines.get(splitId);
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
        if (poses.isEmpty() || poses.get().isEmpty()) {
            throw new IllegalArgumentException(
                    "Auto reference \"" + reference + "\" could not be resolved for source " + source + ". "
                            + "Registration is fail-fast; verify the file exists and source is correct.");
        }
    }

    private static AutoBackend backendForSource(AutoSource source) {
        return AutoBackends.forSource(source)
                .orElseThrow(() -> new IllegalStateException("No auto backend available for source: " + source));
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

    private void resetChoosers() {
        chooser = null;
        commandChooser = null;
    }
}
