package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.auto.AutoBackends;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Aggregates named command and autonomous routine registration for a robot.
 * <p>
 * The lifecycle is:
 * <ol>
 *     <li>Register all named commands via {@link #registerNamedCommand(NamedCommandKey, Supplier)}.</li>
 *     <li>Register autonomous routines via {@link #registerAuto(AutoRoutine)}.</li>
 *     <li>Create a chooser with {@link #createChooser(AutoKey)} and expose it to the dashboard.</li>
 * </ol>
 */
public class RobotAuto {

    private final Map<String, Supplier<Command>> namedCommandSuppliers;
    private final Map<String, AutoRoutine> autoRoutines;
    private SendableChooser<AutoRoutine> chooser;
    private SendableChooser<Command> commandChooser;
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;
    private java.util.function.Function<Pose2d, Command> autoPlanResetter;

    public RobotAuto() {
        namedCommandSuppliers = new LinkedHashMap<>();
        autoRoutines = new LinkedHashMap<>();
        chooser = null;
        commandChooser = null;
        xController = null;
        yController = null;
        thetaController = null;
        autoPlanResetter = null;
    }

    /**
     * Identifier interface for named commands used inside PathPlanner autos.
     * Implementers are encouraged to use enums to provide compile-time safety.
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
     * Implement as an enum for type-safety when selecting routines elsewhere in code.
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
        String id = key.id();
        if (namedCommandSuppliers.containsKey(id)) {
            throw new IllegalArgumentException("Named command already registered: " + id);
        }
        namedCommandSuppliers.put(id, supplier);
        boolean bound = AutoBackends.forSource(AutoSource.PATH_PLANNER)
                .map(backend -> backend.registerNamedCommand(id, () -> Commands.deferredProxy(supplier)))
                .orElse(false);
        if (!bound) {
            bound = AutoBackends.forSource(AutoSource.CHOREO)
                    .map(backend -> backend.registerNamedCommand(id, () -> Commands.deferredProxy(supplier)))
                    .orElse(false);
        }
        if (!bound) {
            DriverStation.reportWarning(
                    "No auto backend available; named command '" + id + "' not bound to vendor API.",
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

    public boolean hasNamedCommand(NamedCommandKey key) {
        return key != null && namedCommandSuppliers.containsKey(key.id());
    }

    public RobotAuto registerAuto(AutoRoutine routine) {
        Objects.requireNonNull(routine, "routine");
        String id = routine.key().id();
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

    public RobotAuto registerAutoPlan(AutoKey key, AutoPlan plan) {
        Objects.requireNonNull(plan, "plan");
        ensurePlanAutos(plan);
        return registerAuto(custom(key, () -> plan.build(createPlanContext())));
    }

    public RobotAuto registerAutoPlan(String id, AutoPlan plan) {
        return registerAutoPlan(AutoKey.of(id), plan);
    }

    public RobotAuto setAutoPlanResetter(java.util.function.Function<Pose2d, Command> resetter) {
        this.autoPlanResetter = resetter;
        return this;
    }

    public RobotAuto registerAutoSequence(AutoKey key, AutoKey... steps) {
        return registerAuto(custom(key, () -> buildSequence(steps)));
    }

    public RobotAuto registerAutoSequence(String id, AutoKey... steps) {
        return registerAutoSequence(AutoKey.of(id), steps);
    }

    public RobotAuto registerAutoSequence(String id, String... stepIds) {
        return registerAutoSequence(AutoKey.of(id), toAutoKeys(stepIds));
    }

    public RobotAuto registerAutoBranch(AutoKey key, BooleanSupplier condition, AutoKey ifTrue, AutoKey ifFalse) {
        Objects.requireNonNull(condition, "condition");
        Objects.requireNonNull(ifTrue, "ifTrue");
        Objects.requireNonNull(ifFalse, "ifFalse");
        return registerAuto(custom(key,
                () -> Commands.either(deferredAuto(ifTrue), deferredAuto(ifFalse), condition)));
    }

    public RobotAuto registerAutoBranch(String id, BooleanSupplier condition, AutoKey ifTrue, AutoKey ifFalse) {
        return registerAutoBranch(AutoKey.of(id), condition, ifTrue, ifFalse);
    }

    public RobotAuto registerAutoBranch(String id, BooleanSupplier condition, String ifTrueId, String ifFalseId) {
        return registerAutoBranch(AutoKey.of(id), condition, AutoKey.of(ifTrueId), AutoKey.of(ifFalseId));
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

    public RobotAuto registerPathPlannerAuto(AutoKey key) {
        return registerPathPlannerAuto(key, key.id());
    }

    public RobotAuto registerPathPlannerAuto(String id) {
        return registerPathPlannerAuto(AutoKey.of(id));
    }

    public RobotAuto registerPathPlannerAuto(AutoKey key, String autoName) {
        return registerAuto(pathPlanner(key, autoName));
    }

    public RobotAuto registerPathPlannerAuto(String id, String autoName) {
        return registerPathPlannerAuto(AutoKey.of(id), autoName);
    }

    public RobotAuto registerChoreoAuto(AutoKey key, String trajectoryName) {
        return registerAuto(choreo(key, trajectoryName));
    }

    public RobotAuto registerChoreoAuto(String id, String trajectoryName) {
        return registerChoreoAuto(AutoKey.of(id), trajectoryName);
    }

    public Optional<AutoRoutine> getAuto(AutoKey key) {
        if (key == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(autoRoutines.get(key.id()));
    }

    public Optional<AutoRoutine> getAuto(String id) {
        return getAuto(AutoKey.of(id));
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

    public Optional<Command> buildSelectedCommand() {
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

    public static AutoRoutine pathPlanner(AutoKey key, String autoName) {
        Objects.requireNonNull(key, "key");
        String reference = autoName != null ? autoName : key.id();
        Supplier<Command> factory = () -> AutoBackends.forSource(AutoSource.PATH_PLANNER)
                .flatMap(backend -> backend.buildAuto(AutoSource.PATH_PLANNER, reference))
                .orElseGet(() -> {
                    DriverStation.reportError("Path planner backend missing; auto \"" + reference + "\" unavailable.", false);
                    return Commands.none();
                });
        return new AutoRoutine(key, AutoSource.PATH_PLANNER, reference, factory, new Pose2d(), false, null);
    }

    public static AutoRoutine choreo(AutoKey key, String trajectoryName) {
        Objects.requireNonNull(key, "key");
        String reference = trajectoryName != null ? trajectoryName : key.id();
        Supplier<Command> factory = () -> AutoBackends.forSource(AutoSource.CHOREO)
                .flatMap(backend -> backend.buildAuto(AutoSource.CHOREO, reference))
                .orElseGet(() -> {
                    DriverStation.reportError("Choreo backend missing; auto \"" + reference + "\" unavailable.", false);
                    return Commands.none();
                });
        return new AutoRoutine(key, AutoSource.CHOREO, reference, factory, new Pose2d(), false, null);
    }

    public static AutoRoutine custom(AutoKey key, Supplier<Command> factory) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(factory, "factory");
        return new AutoRoutine(key, AutoSource.CUSTOM, key.id(), factory, new Pose2d(), false, null);
    }

    private Command deferredCommand(AutoRoutine routine) {
        return Commands.defer(routine::createCommand, Set.of());
    }

    private Command deferredAuto(AutoKey key) {
        Objects.requireNonNull(key, "key");
        AutoRoutine routine = autoRoutines.get(key.id());
        if (routine == null) {
            DriverStation.reportError("Auto not registered: " + key.id(), false);
            return Commands.none();
        }
        return deferredCommand(routine);
    }

    private Command deferredAuto(AutoPlan.StepRef ref) {
        AutoRoutine routine = autoRoutines.get(ref.key().id());
        if (routine != null && !ref.splitIndex().isPresent()) {
            return deferredCommand(routine);
        }
        if (ref.splitIndex().isPresent()) {
            AutoRoutine splitRoutine = autoRoutines.get(splitId(ref));
            if (splitRoutine != null) {
                return deferredCommand(splitRoutine);
            }
            if (routine != null) {
                return buildSplitCommand(routine, ref.splitIndex().getAsInt());
            }
        }
        DriverStation.reportError("Auto not registered: " + ref.key().id(), false);
        return Commands.none();
    }

    private Optional<Pose2d> startingPoseFor(AutoPlan.StepRef ref) {
        AutoRoutine routine = autoRoutines.get(ref.key().id());
        if (ref.splitIndex().isPresent()) {
            AutoRoutine splitRoutine = autoRoutines.get(splitId(ref));
            if (splitRoutine != null && splitRoutine.hasStartingPose()) {
                return Optional.ofNullable(splitRoutine.startingPose());
            }
        }
        if (routine == null || !routine.hasStartingPose()) {
            return Optional.empty();
        }
        return Optional.ofNullable(routine.startingPose());
    }

    private Command resetPose(Pose2d pose) {
        if (autoPlanResetter == null) {
            DriverStation.reportWarning("AutoPlan reset requested but no resetter configured.", false);
            return Commands.none();
        }
        return autoPlanResetter.apply(pose);
    }

    private AutoPlan.Context createPlanContext() {
        return new AutoPlan.Context() {
            @Override
            public Command resolve(AutoPlan.StepRef ref) {
                return deferredAuto(ref);
            }

            @Override
            public Optional<Pose2d> startingPose(AutoPlan.StepRef ref) {
                return startingPoseFor(ref);
            }

            @Override
            public Command resetPose(Pose2d pose) {
                return RobotAuto.this.resetPose(pose);
            }
        };
    }

    private void ensurePlanAutos(AutoPlan plan) {
        for (AutoPlan.StepRef ref : plan.stepRefs()) {
            if (!ref.splitIndex().isPresent()) {
                continue;
            }
            String splitId = splitId(ref);
            if (autoRoutines.containsKey(splitId)) {
                continue;
            }
            AutoRoutine base = autoRoutines.get(ref.key().id());
            if (base == null) {
                DriverStation.reportWarning("AutoPlan step \"" + splitId
                        + "\" could not be derived (base auto missing).", false);
                continue;
            }
            if (base.source() == AutoSource.CUSTOM) {
                DriverStation.reportWarning("AutoPlan step \"" + splitId
                        + "\" could not be derived from CUSTOM auto.", false);
                continue;
            }
            AutoRoutine derived = splitRoutine(base, splitId, ref.splitIndex().getAsInt());
            autoRoutines.put(splitId, derived);
        }
    }

    private AutoRoutine splitRoutine(AutoRoutine base, String splitId, int index) {
        String reference = base.reference() + "." + index;
        Supplier<Command> factory = () -> AutoBackends.forSource(base.source())
                .flatMap(backend -> backend.buildAuto(base.source(), reference))
                .orElseGet(() -> {
                    DriverStation.reportError("Auto backend missing; auto \"" + reference + "\" unavailable.", false);
                    return Commands.none();
                });
        return new AutoRoutine(AutoKey.of(splitId), base.source(), reference, factory, base.startingPose(), base.hasStartingPose(), null);
    }

    private Command buildSplitCommand(AutoRoutine base, int index) {
        String reference = base.reference() + "." + index;
        return AutoBackends.forSource(base.source())
                .flatMap(backend -> backend.buildAuto(base.source(), reference))
                .orElseGet(() -> {
                    DriverStation.reportError("Auto backend missing; auto \"" + reference + "\" unavailable.", false);
                    return Commands.none();
                });
    }

    private String splitId(AutoPlan.StepRef ref) {
        return ref.key().id() + "." + ref.splitIndex().getAsInt();
    }

    private Command buildSequence(AutoKey... steps) {
        Objects.requireNonNull(steps, "steps");
        List<Command> commands = new ArrayList<>();
        for (AutoKey step : steps) {
            if (step == null) {
                continue;
            }
            commands.add(deferredAuto(step));
        }
        if (commands.isEmpty()) {
            return Commands.none();
        }
        return Commands.sequence(commands.toArray(Command[]::new));
    }

    private AutoKey[] toAutoKeys(String... stepIds) {
        if (stepIds == null || stepIds.length == 0) {
            return new AutoKey[0];
        }
        AutoKey[] keys = new AutoKey[stepIds.length];
        for (int i = 0; i < stepIds.length; i++) {
            String id = stepIds[i];
            keys[i] = id == null ? null : AutoKey.of(id);
        }
        return keys;
    }

    private void resetChoosers() {
        chooser = null;
        commandChooser = null;
    }
}
