package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.IdentityHashMap;
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
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.Locale;

import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.diagnostics.BoundedEventLog;
import ca.frc6390.athena.core.localization.RobotLocalization;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Aggregates named command and autonomous routine registration for a robot.
 */
public class RobotAuto {
    private static final int DEFAULT_AUTO_TRACE_LOG_CAPACITY = 512;
    private static final String GLOBAL_INPUT_SCOPE = "<global>";

    // Command overloads may intentionally reuse a prebuilt instance. Clear any prior
    // composition bookkeeping before handing that instance back to a deferred wrapper.
    static Supplier<Command> reusableCommandSupplier(Command command) {
        Command reusable = Objects.requireNonNull(command, "command");
        return () -> {
            CommandScheduler.getInstance().removeComposedCommand(reusable);
            return reusable;
        };
    }

    private final Map<String, Supplier<Command>> namedCommandSuppliers;
    private final Map<String, AutoRoutine> autoRoutines;
    private final Map<String, InputBinding<?>> autoInputs;
    private final Map<String, Map<String, RuntimeInputOverride>> runtimeInputOverrides;
    private final Map<String, Map<String, InputBinding<?>>> programScopedInputs;
    private final Map<String, List<AutoRoutine>> programPathRoutines;
    private final Map<String, Optional<List<Pose2d>>> autoPoseCache;
    private final Map<String, Optional<List<Pose2d>>> programPoseCache;
    private final Map<Command, AutoRoutine> commandChooserRoutineByCommand;
    private SendableChooser<AutoRoutine> chooser;
    private SendableChooser<Command> commandChooser;
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;
    private java.util.function.Function<Pose2d, Command> autoPlanResetter;
    private boolean registrationFinalized;
    private final Set<String> boundPathPlannerNamedCommands;
    private final Set<String> boundChoreoNamedCommands;
    private final Set<AutoSource> warmedAutoSources;
    private final BoundedEventLog<AutoTraceEvent> autoTraceLog;
    private final int autoTraceLogCapacity;
    private final RegistrySection registrySection = new RegistrySection();
    private final CommandsSection commandsSection = new CommandsSection();
    private final InputsSection inputsSection = new InputsSection();
    private final RoutinesSection routinesSection = new RoutinesSection();
    private final SelectionSection selectionSection = new SelectionSection();
    private final ExecutionSection executionSection = new ExecutionSection();
    private final TraceSection traceSection = new TraceSection();
    private final ResetSection resetSection = new ResetSection();
    private final ControllersSection controllersSection = new ControllersSection();
    private RobotCore<?> robotCore;
    private boolean autoTraceEnabled;
    private Consumer<String> autoTraceSink;
    private String prelightSignature;
    private Command prelightCommand;

    public RobotAuto() {
        namedCommandSuppliers = new LinkedHashMap<>();
        autoRoutines = new LinkedHashMap<>();
        autoInputs = new LinkedHashMap<>();
        runtimeInputOverrides = new LinkedHashMap<>();
        programScopedInputs = new LinkedHashMap<>();
        programPathRoutines = new LinkedHashMap<>();
        autoPoseCache = new LinkedHashMap<>();
        programPoseCache = new LinkedHashMap<>();
        commandChooserRoutineByCommand = new IdentityHashMap<>();
        chooser = null;
        commandChooser = null;
        xController = null;
        yController = null;
        thetaController = null;
        autoPlanResetter = null;
        registrationFinalized = false;
        boundPathPlannerNamedCommands = new HashSet<>();
        boundChoreoNamedCommands = new HashSet<>();
        warmedAutoSources = EnumSet.noneOf(AutoSource.class);
        autoTraceLogCapacity = DEFAULT_AUTO_TRACE_LOG_CAPACITY;
        autoTraceLog = new BoundedEventLog<>(autoTraceLogCapacity);
        robotCore = null;
        autoTraceEnabled = false;
        autoTraceSink = message -> DriverStation.reportWarning(message, false);
        prelightSignature = "";
        prelightCommand = null;
    }

    public RobotAuto controllers(Consumer<ControllersSection> section) {
        if (section != null) {
            section.accept(controllersSection);
        }
        return this;
    }

    public ControllersSection controllers() {
        return controllersSection;
    }

    public RobotAuto registry(Consumer<RegistrySection> section) {
        if (section != null) {
            section.accept(registrySection);
        }
        return this;
    }

    public RegistrySection registry() {
        return registrySection;
    }

    public CommandsSection commands() {
        return commandsSection;
    }

    public InputsSection inputs() {
        return inputsSection;
    }

    public RoutinesSection routines() {
        return routinesSection;
    }

    public SelectionSection selection() {
        return selectionSection;
    }

    public ExecutionSection execution() {
        return executionSection;
    }

    public TraceSection trace() {
        return traceSection;
    }

    public ResetSection reset() {
        return resetSection;
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
     * Read-only typed input access used by auto runtime contexts.
     */
    public interface RuntimeInputSection {
        boolean bool(String key);

        BooleanSupplier boolSupplier(String key);

        double dbl(String key);

        DoubleSupplier dblSupplier(String key);

        int integer(String key);

        IntSupplier integerSupplier(String key);

        String string(String key);

        Supplier<String> stringSupplier(String key);

        Pose2d pose2d(String key);

        Supplier<Pose2d> pose2dSupplier(String key);

        Pose3d pose3d(String key);

        Supplier<Pose3d> pose3dSupplier(String key);

        <V> V object(String key, Class<V> type);

        <V> Supplier<V> objectSupplier(String key, Class<V> type);
    }

    /**
     * Typed input registration access for {@link AutoRegisterCtx}.
     */
    public interface RegisterInputSection extends RuntimeInputSection {
        RegisterInputSection bool(String key, BooleanSupplier supplier);

        RegisterInputSection bool(String key, boolean value);

        RegisterInputSection dbl(String key, DoubleSupplier supplier);

        RegisterInputSection dbl(String key, double value);

        RegisterInputSection integer(String key, IntSupplier supplier);

        RegisterInputSection integer(String key, int value);

        RegisterInputSection string(String key, Supplier<String> supplier);

        RegisterInputSection string(String key, String value);

        RegisterInputSection pose2d(String key, Supplier<Pose2d> supplier);

        RegisterInputSection pose2d(String key, Pose2d value);

        RegisterInputSection pose3d(String key, Supplier<Pose3d> supplier);

        RegisterInputSection pose3d(String key, Pose3d value);

        <V> RegisterInputSection object(String key, Class<V> type, Supplier<V> supplier);
    }

    /**
     * Typed input runtime mutation access for {@link AutoBuildCtx}.
     */
    public interface BuildInputSection extends RuntimeInputSection {
        Command bool(String key, boolean value);

        Command resetBool(String key);

        Command dbl(String key, double value);

        Command resetDouble(String key);

        Command integer(String key, int value);

        Command resetInt(String key);

        Command string(String key, String value);

        Command resetString(String key);

        Command pose2d(String key, Pose2d value);

        Command resetPose2d(String key);

        Command pose3d(String key, Pose3d value);

        Command resetPose3d(String key);

        <V> Command object(String key, Class<V> type, V value);

        <V> Command resetObject(String key, Class<V> type);

        Command clear();
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
     *
     * <p>This context exposes program-scoped metadata and can resolve globally registered
     * autos. It is shared by {@link AutoRegisterCtx} and {@link AutoBuildCtx}.</p>
     */
    public interface AutoRuntimeCtx {
        RobotCore<?> robot();

        String programId();

        boolean hasMarker(String id);

        boolean hasAuto(String id);

        boolean hasInput(String id);

        boolean hasInput(String programId, String id);

        default RuntimeInputSection input() {
            return input(programId());
        }

        default RuntimeInputSection input(String programId) {
            AutoRuntimeCtx ctx = this;
            return new RuntimeInputSection() {
                @Override
                public boolean bool(String key) {
                    return ctx.bool(programId, key);
                }

                @Override
                public BooleanSupplier boolSupplier(String key) {
                    return ctx.boolSupplier(programId, key);
                }

                @Override
                public double dbl(String key) {
                    return ctx.doubleInput(programId, key);
                }

                @Override
                public DoubleSupplier dblSupplier(String key) {
                    return ctx.doubleInputSupplier(programId, key);
                }

                @Override
                public int integer(String key) {
                    return ctx.intVal(programId, key);
                }

                @Override
                public IntSupplier integerSupplier(String key) {
                    return ctx.intValSupplier(programId, key);
                }

                @Override
                public String string(String key) {
                    return ctx.stringVal(programId, key);
                }

                @Override
                public Supplier<String> stringSupplier(String key) {
                    return ctx.stringValSupplier(programId, key);
                }

                @Override
                public Pose2d pose2d(String key) {
                    return ctx.pose2dVal(programId, key);
                }

                @Override
                public Supplier<Pose2d> pose2dSupplier(String key) {
                    return ctx.pose2dValSupplier(programId, key);
                }

                @Override
                public Pose3d pose3d(String key) {
                    return ctx.pose3dVal(programId, key);
                }

                @Override
                public Supplier<Pose3d> pose3dSupplier(String key) {
                    return ctx.pose3dValSupplier(programId, key);
                }

                @Override
                public <V> V object(String key, Class<V> type) {
                    return ctx.objectInput(programId, key, type);
                }

                @Override
                public <V> Supplier<V> objectSupplier(String key, Class<V> type) {
                    return ctx.objectInputSupplier(programId, key, type);
                }
            };
        }

        <V> Supplier<V> input(String programId, String key, Class<V> type);

        default <V> Supplier<V> input(String key, Class<V> type) {
            return input(programId(), key, type);
        }

        default <V> V inputValue(String key, Class<V> type) {
            Objects.requireNonNull(type, "type");
            return inputValue(programId(), key, type);
        }

        default <V> V inputValue(String programId, String key, Class<V> type) {
            Objects.requireNonNull(type, "type");
            return input(programId, key, type).get();
        }

        default BooleanSupplier inputSupplier(String key) {
            return boolSupplier(key);
        }

        default BooleanSupplier inputSupplier(String programId, String key) {
            return boolSupplier(programId, key);
        }

        default boolean bool(String key) {
            return bool(programId(), key);
        }

        default boolean bool(String programId, String key) {
            return Boolean.TRUE.equals(inputValue(programId, key, Boolean.class));
        }

        default BooleanSupplier boolSupplier(String key) {
            return boolSupplier(programId(), key);
        }

        default BooleanSupplier boolSupplier(String programId, String key) {
            return () -> bool(programId, key);
        }

        default double doubleInput(String key) {
            return doubleInput(programId(), key);
        }

        default double doubleInput(String programId, String key) {
            Double value = inputValue(programId, key, Double.class);
            return value != null ? value.doubleValue() : Double.NaN;
        }

        default DoubleSupplier doubleInputSupplier(String key) {
            return doubleInputSupplier(programId(), key);
        }

        default DoubleSupplier doubleInputSupplier(String programId, String key) {
            return () -> doubleInput(programId, key);
        }

        default int intVal(String key) {
            return intVal(programId(), key);
        }

        default int intVal(String programId, String key) {
            Integer value = inputValue(programId, key, Integer.class);
            return value != null ? value.intValue() : 0;
        }

        default IntSupplier intValSupplier(String key) {
            return intValSupplier(programId(), key);
        }

        default IntSupplier intValSupplier(String programId, String key) {
            return () -> intVal(programId, key);
        }

        default String stringVal(String key) {
            return stringVal(programId(), key);
        }

        default String stringVal(String programId, String key) {
            return inputValue(programId, key, String.class);
        }

        default Supplier<String> stringValSupplier(String key) {
            return stringValSupplier(programId(), key);
        }

        default Supplier<String> stringValSupplier(String programId, String key) {
            return input(programId, key, String.class);
        }

        default Pose2d pose2dVal(String key) {
            return pose2dVal(programId(), key);
        }

        default Pose2d pose2dVal(String programId, String key) {
            return inputValue(programId, key, Pose2d.class);
        }

        default Supplier<Pose2d> pose2dValSupplier(String key) {
            return pose2dValSupplier(programId(), key);
        }

        default Supplier<Pose2d> pose2dValSupplier(String programId, String key) {
            return input(programId, key, Pose2d.class);
        }

        default Pose3d pose3dVal(String key) {
            return pose3dVal(programId(), key);
        }

        default Pose3d pose3dVal(String programId, String key) {
            return inputValue(programId, key, Pose3d.class);
        }

        default Supplier<Pose3d> pose3dValSupplier(String key) {
            return pose3dValSupplier(programId(), key);
        }

        default Supplier<Pose3d> pose3dValSupplier(String programId, String key) {
            return input(programId, key, Pose3d.class);
        }

        default <V> V objectInput(String key, Class<V> type) {
            return objectInput(programId(), key, type);
        }

        default <V> V objectInput(String programId, String key, Class<V> type) {
            if (!hasInput(programId, key)) {
                return null;
            }
            return inputValue(programId, key, type);
        }

        default <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            return objectInputSupplier(programId(), key, type);
        }

        default <V> Supplier<V> objectInputSupplier(String programId, String key, Class<V> type) {
            return input(programId, key, type);
        }
    }

    /**
     * Registration-only context. Use this in {@link AutoProgram#register(AutoRegisterCtx)} to
     * declare markers, inputs, and dependent autos.
     */
    public interface AutoRegisterCtx extends AutoRuntimeCtx {
        AutoRegisterCtx marker(String id, Supplier<Command> supplier);

        default AutoRegisterCtx marker(String id, Command command) {
            Objects.requireNonNull(command, "command");
            return marker(id, RobotAuto.reusableCommandSupplier(command));
        }

        default AutoRegisterCtx marker(String id, Runnable action) {
            Objects.requireNonNull(action, "action");
            return marker(id, new InstantCommand(action));
        }

        default <E extends Enum<E>> AutoRegisterCtx marker(String id, E state) {
            Objects.requireNonNull(state, "state");
            return marker(id, new Enum<?>[] {state});
        }

        default AutoRegisterCtx marker(String id, Enum<?>... states) {
            Objects.requireNonNull(states, "states");
            return marker(
                    id,
                    Commands.runOnce(() -> {
                        for (Enum<?> state : states) {
                            if (state != null) {
                                queueStateUnchecked(robot(), state);
                            }
                        }
                    }).ignoringDisable(true));
        }

        @SuppressWarnings({"rawtypes", "unchecked"})
        private static void queueStateUnchecked(RobotCore<?> robot, Enum<?> state) {
            robot.state().queue((Enum) state);
        }

        @Override
        default RegisterInputSection input() {
            AutoRegisterCtx ctx = this;
            return new RegisterInputSection() {
                @Override
                public RegisterInputSection bool(String key, BooleanSupplier supplier) {
                    ctx.registerInput(key, Boolean.class, () -> supplier.getAsBoolean());
                    return this;
                }

                @Override
                public RegisterInputSection bool(String key, boolean value) {
                    ctx.registerInput(key, Boolean.class, () -> value);
                    return this;
                }

                @Override
                public RegisterInputSection dbl(String key, DoubleSupplier supplier) {
                    ctx.registerInput(key, Double.class, () -> supplier.getAsDouble());
                    return this;
                }

                @Override
                public RegisterInputSection dbl(String key, double value) {
                    ctx.registerInput(key, Double.class, () -> value);
                    return this;
                }

                @Override
                public RegisterInputSection integer(String key, IntSupplier supplier) {
                    ctx.registerInput(key, Integer.class, () -> supplier.getAsInt());
                    return this;
                }

                @Override
                public RegisterInputSection integer(String key, int value) {
                    ctx.registerInput(key, Integer.class, () -> value);
                    return this;
                }

                @Override
                public RegisterInputSection string(String key, Supplier<String> supplier) {
                    ctx.registerInput(key, String.class, supplier);
                    return this;
                }

                @Override
                public RegisterInputSection string(String key, String value) {
                    ctx.registerInput(key, String.class, () -> value);
                    return this;
                }

                @Override
                public RegisterInputSection pose2d(String key, Supplier<Pose2d> supplier) {
                    ctx.registerInput(key, Pose2d.class, supplier);
                    return this;
                }

                @Override
                public RegisterInputSection pose2d(String key, Pose2d value) {
                    ctx.registerInput(key, Pose2d.class, () -> value);
                    return this;
                }

                @Override
                public RegisterInputSection pose3d(String key, Supplier<Pose3d> supplier) {
                    ctx.registerInput(key, Pose3d.class, supplier);
                    return this;
                }

                @Override
                public RegisterInputSection pose3d(String key, Pose3d value) {
                    ctx.registerInput(key, Pose3d.class, () -> value);
                    return this;
                }

                @Override
                public <V> RegisterInputSection object(String key, Class<V> type, Supplier<V> supplier) {
                    ctx.registerInput(key, type, supplier);
                    return this;
                }

                @Override
                public boolean bool(String key) {
                    return ctx.bool(key);
                }

                @Override
                public BooleanSupplier boolSupplier(String key) {
                    return ctx.boolSupplier(key);
                }

                @Override
                public double dbl(String key) {
                    return ctx.doubleInput(key);
                }

                @Override
                public DoubleSupplier dblSupplier(String key) {
                    return ctx.doubleInputSupplier(key);
                }

                @Override
                public int integer(String key) {
                    return ctx.intVal(key);
                }

                @Override
                public IntSupplier integerSupplier(String key) {
                    return ctx.intValSupplier(key);
                }

                @Override
                public String string(String key) {
                    return ctx.stringVal(key);
                }

                @Override
                public Supplier<String> stringSupplier(String key) {
                    return ctx.stringValSupplier(key);
                }

                @Override
                public Pose2d pose2d(String key) {
                    return ctx.pose2dVal(key);
                }

                @Override
                public Supplier<Pose2d> pose2dSupplier(String key) {
                    return ctx.pose2dValSupplier(key);
                }

                @Override
                public Pose3d pose3d(String key) {
                    return ctx.pose3dVal(key);
                }

                @Override
                public Supplier<Pose3d> pose3dSupplier(String key) {
                    return ctx.pose3dValSupplier(key);
                }

                @Override
                public <V> V object(String key, Class<V> type) {
                    return ctx.objectInput(key, type);
                }

                @Override
                public <V> Supplier<V> objectSupplier(String key, Class<V> type) {
                    return ctx.objectInputSupplier(key, type);
                }
            };
        }

        <V> AutoRegisterCtx registerInput(String key, Class<V> type, Supplier<V> supplier);

        /**
         * Register a trajectory by file/reference name. The chooser id defaults to the same value.
         *
         * <p>Signature order: {@code path(reference, source)}.</p>
         */
        AutoRegisterCtx path(String reference, TrajectorySource source);

        /**
         * Register a trajectory by file/reference name and alias it under a different chooser id.
         *
         * <p>Signature order: {@code path(reference, source, id)}.</p>
         */
        AutoRegisterCtx path(String reference, TrajectorySource source, String id);

        AutoRegisterCtx custom(String id, Supplier<Command> factory);

        default AutoRegisterCtx custom(String id, Command command) {
            Objects.requireNonNull(command, "command");
            return custom(id, RobotAuto.reusableCommandSupplier(command));
        }

        default AutoRegisterCtx custom(String id, Runnable action) {
            Objects.requireNonNull(action, "action");
            return custom(id, new InstantCommand(action));
        }

        default AutoRegisterCtx auto(AutoKey key, Supplier<Command> factory) {
            Objects.requireNonNull(key, "key");
            Objects.requireNonNull(factory, "factory");
            return auto(RobotAuto.custom(key, factory));
        }

        default AutoRegisterCtx auto(String id, Supplier<Command> factory) {
            Objects.requireNonNull(id, "id");
            return auto(AutoKey.of(id), factory);
        }

        default AutoRegisterCtx auto(AutoKey key, Command command) {
            Objects.requireNonNull(command, "command");
            return auto(key, RobotAuto.reusableCommandSupplier(command));
        }

        default AutoRegisterCtx auto(String id, Command command) {
            Objects.requireNonNull(command, "command");
            return auto(id, RobotAuto.reusableCommandSupplier(command));
        }

        default AutoRegisterCtx auto(AutoKey key, Runnable action) {
            Objects.requireNonNull(action, "action");
            return auto(key, new InstantCommand(action));
        }

        default AutoRegisterCtx auto(String id, Runnable action) {
            Objects.requireNonNull(action, "action");
            return auto(id, new InstantCommand(action));
        }

        AutoRegisterCtx auto(AutoRoutine routine);
    }

    /**
     * Build-only context used in {@link AutoProgram#build(AutoBuildCtx)}.
     *
     * <p>{@code auto(...)} resolves to deferred commands so autos are looked up when the command
     * is scheduled. Odometry reset settings are consumed by the next {@code auto(...)} call only,
     * then automatically disarmed.</p>
     */
    public interface AutoBuildCtx extends AutoRuntimeCtx {
        /**
         * Pose source used when applying the one-shot odometry reset.
         */
        enum OdometryResetTarget {
            /** Do not apply a reset. */
            NONE,
            /** Reset to the resolved auto path's starting pose. */
            PATH_START,
            /** Reset to the resolved auto path's ending pose. */
            PATH_END,
            /** Reset to an explicit pose supplied through {@link #odometryResetPose(Pose2d)}. */
            POSE
        }

        /**
         * Arms/disarms odometry reset for the next {@code auto(...)} call built by this ctx.
         *
         * <p>Reset is one-shot. After the next {@code auto(...)} call, this flag is cleared.</p>
         */
        default AutoBuildCtx odometryReset(boolean enabled) {
            return this;
        }

        /**
         * Returns whether the next {@code auto(...)} call will attempt an odometry reset.
         */
        default boolean odometryResetEnabled() {
            return true;
        }

        /**
         * Selects which pose source should be used when odometry reset is enabled.
         */
        default AutoBuildCtx odometryResetTarget(OdometryResetTarget target) {
            return this;
        }

        /**
         * Returns the configured pose source for the next reset operation.
         */
        default OdometryResetTarget odometryResetTarget() {
            return OdometryResetTarget.PATH_START;
        }

        /**
         * Sets an explicit reset pose and selects {@link OdometryResetTarget#POSE}.
         */
        default AutoBuildCtx odometryResetPose(Pose2d pose) {
            return this;
        }

        /**
         * Returns the explicit pose used when target is {@link OdometryResetTarget#POSE}.
         */
        default Optional<Pose2d> odometryResetPose() {
            return Optional.empty();
        }

        /**
         * Convenience for {@link #odometryResetTarget(OdometryResetTarget)} with {@code PATH_START}.
         */
        default AutoBuildCtx odometryResetToPathStart() {
            return odometryResetTarget(OdometryResetTarget.PATH_START);
        }

        /**
         * Convenience for {@link #odometryResetTarget(OdometryResetTarget)} with {@code PATH_END}.
         */
        default AutoBuildCtx odometryResetToPathEnd() {
            return odometryResetTarget(OdometryResetTarget.PATH_END);
        }

        /**
         * Convenience for {@link #odometryResetPose(Pose2d)}.
         */
        default AutoBuildCtx odometryResetToPose(Pose2d pose) {
            return odometryResetPose(pose);
        }

        /**
         * Disables the next one-shot odometry reset.
         */
        default AutoBuildCtx disableOdometryReset() {
            return odometryReset(false);
        }

        /**
         * Resolves a registered auto routine id to a deferred command.
         *
         * <p>Resolution checks the current program scope first, then globally registered autos.
         * If odometry reset is armed, it is applied to this call and then disarmed.</p>
         */
        default Command auto(String id) {
            Objects.requireNonNull(id, "id");
            return robot().autos().deferredAuto(id, OptionalInt.empty());
        }

        /**
         * Resolves a registered auto routine key to a deferred command.
         */
        default Command auto(AutoKey key) {
            Objects.requireNonNull(key, "key");
            return auto(key.id());
        }

        /**
         * Resolves a specific split of an auto routine to a deferred command.
         *
         * <p>Implementations may resolve either an explicitly registered split ({@code id.index}) or
         * derive it from a base trajectory reference, depending on backend capabilities. Resolution
         * checks the current program scope first, then globally registered autos.</p>
         *
         * <p>If odometry reset is armed, it is applied to this call and then disarmed.</p>
         */
        default Command auto(String id, int splitIndex) {
            Objects.requireNonNull(id, "id");
            return robot().autos().deferredAuto(id, OptionalInt.of(splitIndex));
        }

        /**
         * Queues a robot state via {@link RobotCore#state()} and waits until that state is reached.
         *
         * <p>This is the auto-build equivalent of "queue state, then block sequence progression until
         * done". If no unique owner can be resolved for the enum type, scheduling this command throws
         * an {@link IllegalStateException}.</p>
         */
        default <E extends Enum<E>> Command state(E state) {
            Objects.requireNonNull(state, "state");
            RobotCore<?> core = robot();
            String label = stateLabel(state);
            Command stateCommand = Commands.sequence(
                    Commands.runOnce(() -> {
                        if (!queueStateUnchecked(core, state)) {
                            throw new IllegalStateException(
                                    "AutoBuildCtx.state(" + label + ") failed: no unique owner was resolved by "
                                            + "RobotCore.state() for enum type "
                                            + state.getDeclaringClass().getName());
                        }
                    }),
                    Commands.waitUntil(() -> atStateUnchecked(core, state)));
            return traceBuild("state", "target=" + label, stateCommand);
        }

        @Override
        default BuildInputSection input() {
            return input(programId());
        }

        @Override
        default BuildInputSection input(String programId) {
            AutoBuildCtx ctx = this;
            String scopedProgramId = requireNonBlank(programId, "program id");
            return new BuildInputSection() {
                @Override
                public Command bool(String key, boolean value) {
                    return ctx.setInput(scopedProgramId, key, Boolean.class, value);
                }

                @Override
                public Command resetBool(String key) {
                    return ctx.clearInput(scopedProgramId, key, Boolean.class);
                }

                @Override
                public Command dbl(String key, double value) {
                    return ctx.setInput(scopedProgramId, key, Double.class, value);
                }

                @Override
                public Command resetDouble(String key) {
                    return ctx.clearInput(scopedProgramId, key, Double.class);
                }

                @Override
                public Command integer(String key, int value) {
                    return ctx.setInput(scopedProgramId, key, Integer.class, value);
                }

                @Override
                public Command resetInt(String key) {
                    return ctx.clearInput(scopedProgramId, key, Integer.class);
                }

                @Override
                public Command string(String key, String value) {
                    return ctx.setInput(scopedProgramId, key, String.class, value);
                }

                @Override
                public Command resetString(String key) {
                    return ctx.clearInput(scopedProgramId, key, String.class);
                }

                @Override
                public Command pose2d(String key, Pose2d value) {
                    return ctx.setInput(scopedProgramId, key, Pose2d.class, value);
                }

                @Override
                public Command resetPose2d(String key) {
                    return ctx.clearInput(scopedProgramId, key, Pose2d.class);
                }

                @Override
                public Command pose3d(String key, Pose3d value) {
                    return ctx.setInput(scopedProgramId, key, Pose3d.class, value);
                }

                @Override
                public Command resetPose3d(String key) {
                    return ctx.clearInput(scopedProgramId, key, Pose3d.class);
                }

                @Override
                public <V> Command object(String key, Class<V> type, V value) {
                    return ctx.setInput(scopedProgramId, key, type, value);
                }

                @Override
                public <V> Command resetObject(String key, Class<V> type) {
                    return ctx.clearInput(scopedProgramId, key, type);
                }

                @Override
                public Command clear() {
                    return ctx.clearInputs(scopedProgramId);
                }

                @Override
                public boolean bool(String key) {
                    return ctx.bool(scopedProgramId, key);
                }

                @Override
                public BooleanSupplier boolSupplier(String key) {
                    return ctx.boolSupplier(scopedProgramId, key);
                }

                @Override
                public double dbl(String key) {
                    return ctx.doubleInput(scopedProgramId, key);
                }

                @Override
                public DoubleSupplier dblSupplier(String key) {
                    return ctx.doubleInputSupplier(scopedProgramId, key);
                }

                @Override
                public int integer(String key) {
                    return ctx.intVal(scopedProgramId, key);
                }

                @Override
                public IntSupplier integerSupplier(String key) {
                    return ctx.intValSupplier(scopedProgramId, key);
                }

                @Override
                public String string(String key) {
                    return ctx.stringVal(scopedProgramId, key);
                }

                @Override
                public Supplier<String> stringSupplier(String key) {
                    return ctx.stringValSupplier(scopedProgramId, key);
                }

                @Override
                public Pose2d pose2d(String key) {
                    return ctx.pose2dVal(scopedProgramId, key);
                }

                @Override
                public Supplier<Pose2d> pose2dSupplier(String key) {
                    return ctx.pose2dValSupplier(scopedProgramId, key);
                }

                @Override
                public Pose3d pose3d(String key) {
                    return ctx.pose3dVal(scopedProgramId, key);
                }

                @Override
                public Supplier<Pose3d> pose3dSupplier(String key) {
                    return ctx.pose3dValSupplier(scopedProgramId, key);
                }

                @Override
                public <V> V object(String key, Class<V> type) {
                    return ctx.objectInput(scopedProgramId, key, type);
                }

                @Override
                public <V> Supplier<V> objectSupplier(String key, Class<V> type) {
                    return ctx.objectInputSupplier(scopedProgramId, key, type);
                }
            };
        }

        default <T> Command setInput(String key, Class<T> type, T value) {
            return setInput(programId(), key, type, value);
        }

        default <T> Command setInput(String programId, String key, Class<T> type, T value) {
            Objects.requireNonNull(type, "type");
            String scopedProgramId = requireNonBlank(programId, "program id");
            Command setCommand = Commands.runOnce(
                    () -> robot().autos().setRuntimeInputOverride(scopedProgramId, key, type, value));
            return traceBuild(
                    "setInput",
                    "scope=" + scopedProgramId + " id=" + key + " type=" + type.getSimpleName(),
                    setCommand);
        }

        default <T> Command clearInput(String key, Class<T> type) {
            return clearInput(programId(), key, type);
        }

        default <T> Command clearInput(String programId, String key, Class<T> type) {
            Objects.requireNonNull(type, "type");
            String scopedProgramId = requireNonBlank(programId, "program id");
            Command clearCommand = Commands.runOnce(
                    () -> robot().autos().clearRuntimeInputOverride(scopedProgramId, key));
            return traceBuild(
                    "clearInput",
                    "scope=" + scopedProgramId + " id=" + key + " type=" + type.getSimpleName(),
                    clearCommand);
        }

        default Command triggerInput(String key) {
            return triggerInput(programId(), key);
        }

        default Command triggerInput(String programId, String key) {
            return setInput(programId, key, Boolean.class, Boolean.TRUE);
        }

        default Command clearInputs() {
            return clearInputs(programId());
        }

        default Command clearInputs(String programId) {
            String scopedProgramId = requireNonBlank(programId, "program id");
            Command clearCommand = Commands.runOnce(
                    () -> robot().autos().clearRuntimeInputOverrides(scopedProgramId));
            return traceBuild("clearInputs", "scope=" + scopedProgramId, clearCommand);
        }

        default Command clearAllInputs() {
            Command clearAllCommand = Commands.runOnce(() -> robot().autos().clearRuntimeInputOverrides());
            return traceBuild("clearInputs", "scope=all", clearAllCommand);
        }

        default Command bool(String key, boolean value) {
            return setInput(key, Boolean.class, value);
        }

        default Command resetBool(String key) {
            return clearInput(key, Boolean.class);
        }

        default Command dbl(String key, double value) {
            return setInput(key, Double.class, value);
        }

        default Command resetDouble(String key) {
            return clearInput(key, Double.class);
        }

        default Command integer(String key, int value) {
            return setInput(key, Integer.class, value);
        }

        default Command resetInt(String key) {
            return clearInput(key, Integer.class);
        }

        default Command string(String key, String value) {
            return setInput(key, String.class, value);
        }

        default Command resetString(String key) {
            return clearInput(key, String.class);
        }

        default Command pose2d(String key, Pose2d value) {
            return setInput(key, Pose2d.class, value);
        }

        default Command resetPose2d(String key) {
            return clearInput(key, Pose2d.class);
        }

        default Command pose3d(String key, Pose3d value) {
            return setInput(key, Pose3d.class, value);
        }

        default Command resetPose3d(String key) {
            return clearInput(key, Pose3d.class);
        }

        default <V> Command object(String key, Class<V> type, V value) {
            return setInput(key, type, value);
        }

        default <V> Command resetObject(String key, Class<V> type) {
            return clearInput(key, type);
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

        default Command deadline(Command deadline, String firstOtherAutoId, String... otherAutoIds) {
            Objects.requireNonNull(deadline, "deadline");
            Objects.requireNonNull(firstOtherAutoId, "firstOtherAutoId");
            List<String> ids = new ArrayList<>();
            ids.add(firstOtherAutoId);
            if (otherAutoIds != null && otherAutoIds.length > 0) {
                ids.addAll(Arrays.asList(otherAutoIds));
            }
            Command[] otherCommands = commandsForIds(this, ids.toArray(String[]::new));
            Command deadlineCommand = otherCommands.length == 0
                    ? deadline
                    : Commands.deadline(deadline, otherCommands);
            return traceBuild(
                    "deadline(command+autoIds)",
                    "deadline=" + commandLabel(deadline) + " others=" + otherCommands.length
                            + " ids=" + idsLabel(ids.toArray(String[]::new)),
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

        default Command deadline(String deadlineAutoId, Command firstOther, Command... others) {
            Objects.requireNonNull(deadlineAutoId, "deadlineAutoId");
            Objects.requireNonNull(firstOther, "firstOther");
            Command[] trailing = nonNullCommands(others);
            Command[] combined = new Command[trailing.length + 1];
            combined[0] = firstOther;
            if (trailing.length > 0) {
                System.arraycopy(trailing, 0, combined, 1, trailing.length);
            }
            Command deadlineCommand = Commands.deadline(auto(deadlineAutoId), combined);
            return traceBuild(
                    "deadline(autoId+commands)",
                    "deadline=auto(" + deadlineAutoId + ") others=" + combined.length,
                    deadlineCommand);
        }

        default Command select(BooleanSupplier condition, Command ifTrue, Command ifFalse) {
            return selectWithLabels(condition, ifTrue, ifFalse, commandLabel(ifTrue), commandLabel(ifFalse));
        }

        default Command select(BooleanSupplier condition, Command ifTrue, String ifFalseId) {
            Objects.requireNonNull(ifFalseId, "ifFalseId");
            return selectWithLabels(
                    condition,
                    ifTrue,
                    auto(ifFalseId),
                    commandLabel(ifTrue),
                    "auto(" + ifFalseId + ")");
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

        default Command select(BooleanSupplier condition, String ifTrueId, Command ifFalse) {
            Objects.requireNonNull(ifTrueId, "ifTrueId");
            return selectWithLabels(
                    condition,
                    auto(ifTrueId),
                    ifFalse,
                    "auto(" + ifTrueId + ")",
                    commandLabel(ifFalse));
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
                robot().autos().traceAutoDecision(
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

        default Command selectIndex(
                IntSupplier selector,
                Command fallback,
                String firstOptionAutoId,
                String... optionAutoIds) {
            Objects.requireNonNull(selector, "selector");
            Objects.requireNonNull(fallback, "fallback");
            Objects.requireNonNull(firstOptionAutoId, "firstOptionAutoId");
            List<Command> options = new ArrayList<>();
            List<String> optionLabels = new ArrayList<>();
            options.add(auto(firstOptionAutoId));
            optionLabels.add("auto(" + firstOptionAutoId + ")");
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
                    commandLabel(fallback),
                    options.toArray(Command[]::new),
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

        default Command selectIndex(
                IntSupplier selector,
                String fallbackAutoId,
                Command firstOption,
                Command... options) {
            Objects.requireNonNull(selector, "selector");
            Objects.requireNonNull(fallbackAutoId, "fallbackAutoId");
            Objects.requireNonNull(firstOption, "firstOption");
            Command fallback = auto(fallbackAutoId);
            List<Command> resolvedOptions = new ArrayList<>();
            List<String> optionLabels = new ArrayList<>();
            resolvedOptions.add(firstOption);
            optionLabels.add(commandLabel(firstOption));
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
                    "auto(" + fallbackAutoId + ")",
                    resolvedOptions.toArray(Command[]::new),
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
                    robot().autos().traceAutoDecision(
                            "selectIndex",
                            "index=" + index + " outOfRange(size=" + options.length + ") -> " + fallbackLabel);
                    return fallback;
                }
                String selectedLabel = optionLabels[index];
                robot().autos().traceAutoDecision(
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

        @SuppressWarnings({"rawtypes", "unchecked"})
        private static boolean queueStateUnchecked(RobotCore<?> robot, Enum<?> state) {
            return robot.state().queue((Enum) state);
        }

        @SuppressWarnings({"rawtypes", "unchecked"})
        private static boolean atStateUnchecked(RobotCore<?> robot, Enum<?> state) {
            return robot.state().at((Enum) state);
        }

        private static String stateLabel(Enum<?> state) {
            return state.getDeclaringClass().getSimpleName() + "." + state.name();
        }

        private Command traceBuild(String operation, String detail, Command command) {
            Objects.requireNonNull(operation, "operation");
            Objects.requireNonNull(command, "command");
            return robot().autos().traceAutoLifecycle("ctx." + operation, detail, command);
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
                            robot().autos().traceAutoDecision("waitFor", "result=interrupted");
                            return;
                        }
                        robot().autos().traceAutoDecision(
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
                            robot().autos().traceAutoDecision("waitFor", "result=interrupted");
                            return;
                        }
                        robot().autos().traceAutoDecision(
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
                            robot().autos().traceAutoDecision("timeout", "result=interrupted");
                            return;
                        }
                        robot().autos().traceAutoDecision(
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
                            robot().autos().traceAutoDecision("timeoutWithFallback", "result=interrupted");
                            return;
                        }
                        robot().autos().traceAutoDecision(
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

        /**
         * Adapter to use this context with {@link AutoPlan} helpers.
         */
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
                    return robot().autos().startingPoseFor(ref.key().id(), ref.splitIndex());
                }

                @Override
                public Command resetPose(Pose2d pose) {
                    return robot().autos().resetPose(pose);
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
        /**
         * Unique chooser/registry id for this program.
         */
        String id();

        /**
         * Key form of {@link #id()}.
         */
        default AutoKey key() {
            return AutoKey.of(id());
        }

        /**
         * Registration phase. Use this to declare dependencies before build.
         */
        default void register(AutoRegisterCtx ctx) {}

        /**
         * Build phase. Returns the primary chooser routine for this program.
         */
        AutoRoutine build(AutoBuildCtx ctx);

        /**
         * Creates a trajectory-backed routine using this program id as chooser key.
         */
        default AutoRoutine path(TrajectorySource source, String reference) {
            return RobotAuto.path(key(), source, reference);
        }

        /**
         * Creates a trajectory-backed routine where reference defaults to {@link #id()}.
         */
        default AutoRoutine path(TrajectorySource source) {
            return path(source, key().id());
        }

        /**
         * Creates a custom command-backed routine using this program id as chooser key.
         */
        default AutoRoutine custom(Supplier<Command> factory) {
            return RobotAuto.custom(key(), factory);
        }
    }

    private record AutoRegisterCtxImpl(
            RobotCore<?> robot,
            RobotAuto autos,
            String programId,
            Map<String, AutoRoutine> scopedAutos,
            Map<String, Supplier<Command>> scopedMarkers,
            Map<String, InputBinding<?>> scopedInputs) implements AutoRegisterCtx {
        private AutoRegisterCtxImpl {
            Objects.requireNonNull(robot, "robot");
            Objects.requireNonNull(autos, "autos");
            Objects.requireNonNull(programId, "programId");
            Objects.requireNonNull(scopedAutos, "scopedAutos");
            Objects.requireNonNull(scopedMarkers, "scopedMarkers");
            Objects.requireNonNull(scopedInputs, "scopedInputs");
        }

        @Override
        public boolean hasMarker(String id) {
            return id != null && scopedMarkers.containsKey(id);
        }

        @Override
        public boolean hasAuto(String id) {
            return id != null && (scopedAutos.containsKey(id) || autos.hasAuto(id));
        }

        @Override
        public boolean hasInput(String id) {
            return id != null && scopedInputs.containsKey(id);
        }

        @Override
        public boolean hasInput(String programId, String id) {
            if (programId == null || id == null) {
                return false;
            }
            if (this.programId.equals(programId)) {
                return scopedInputs.containsKey(id);
            }
            return autos.hasProgramScopedInput(programId, id);
        }

        @Override
        public <V> Supplier<V> input(String key, Class<V> type) {
            return autos.inputSupplier(scopedInputs, programId, key, type);
        }

        @Override
        public <V> Supplier<V> input(String programId, String key, Class<V> type) {
            if (programId == null || programId.isBlank() || this.programId.equals(programId)) {
                return autos.inputSupplier(scopedInputs, this.programId, key, type);
            }
            return autos.inputSupplierForProgram(programId, key, type);
        }

        @Override
        public AutoRegisterCtx marker(String id, Supplier<Command> supplier) {
            autos.registerScopedMarker(scopedMarkers, programId, id, supplier);
            return this;
        }

        @Override
        public <V> AutoRegisterCtx registerInput(String key, Class<V> type, Supplier<V> supplier) {
            autos.registerScopedInput(scopedInputs, programId, key, type, supplier);
            return this;
        }

        @Override
        public AutoRegisterCtx path(String reference, TrajectorySource source) {
            return path(reference, source, reference);
        }

        @Override
        public AutoRegisterCtx path(String reference, TrajectorySource source, String id) {
            autos.registerScopedPath(scopedAutos, scopedMarkers, programId, reference, source, id);
            return this;
        }

        @Override
        public AutoRegisterCtx custom(String id, Supplier<Command> factory) {
            Objects.requireNonNull(factory, "factory");
            autos.registerScopedAuto(
                    scopedAutos,
                    scopedMarkers,
                    programId,
                    RobotAuto.custom(AutoKey.of(id), factory));
            return this;
        }

        @Override
        public AutoRegisterCtx auto(AutoRoutine routine) {
            autos.registerScopedAuto(scopedAutos, scopedMarkers, programId, routine);
            return this;
        }
    }

    private static final class AutoBuildCtxImpl implements AutoBuildCtx {
        private final RobotCore<?> robot;
        private final RobotAuto autos;
        private final String programId;
        private final Map<String, AutoRoutine> scopedAutos;
        private final Map<String, Supplier<Command>> scopedMarkers;
        private final Map<String, InputBinding<?>> scopedInputs;
        private boolean odometryResetEnabled = true;
        private OdometryResetTarget odometryResetTarget = OdometryResetTarget.PATH_START;
        private Pose2d odometryResetPose = null;

        private AutoBuildCtxImpl(
                RobotCore<?> robot,
                RobotAuto autos,
                String programId,
                Map<String, AutoRoutine> scopedAutos,
                Map<String, Supplier<Command>> scopedMarkers,
                Map<String, InputBinding<?>> scopedInputs) {
            this.robot = Objects.requireNonNull(robot, "robot");
            this.autos = Objects.requireNonNull(autos, "autos");
            this.programId = Objects.requireNonNull(programId, "programId");
            this.scopedAutos = Objects.requireNonNull(scopedAutos, "scopedAutos");
            this.scopedMarkers = Objects.requireNonNull(scopedMarkers, "scopedMarkers");
            this.scopedInputs = Objects.requireNonNull(scopedInputs, "scopedInputs");
        }

        @Override
        public RobotCore<?> robot() {
            return robot;
        }

        @Override
        public String programId() {
            return programId;
        }

        @Override
        public boolean hasMarker(String id) {
            return id != null && scopedMarkers.containsKey(id);
        }

        @Override
        public boolean hasAuto(String id) {
            return id != null && (scopedAutos.containsKey(id) || autos.hasAuto(id));
        }

        @Override
        public boolean hasInput(String id) {
            return id != null && scopedInputs.containsKey(id);
        }

        @Override
        public boolean hasInput(String programId, String id) {
            if (programId == null || id == null) {
                return false;
            }
            if (this.programId.equals(programId)) {
                return scopedInputs.containsKey(id);
            }
            return autos.hasProgramScopedInput(programId, id);
        }

        @Override
        public <V> Supplier<V> input(String key, Class<V> type) {
            return autos.inputSupplier(scopedInputs, programId, key, type);
        }

        @Override
        public <V> Supplier<V> input(String programId, String key, Class<V> type) {
            if (programId == null || programId.isBlank() || this.programId.equals(programId)) {
                return autos.inputSupplier(scopedInputs, this.programId, key, type);
            }
            return autos.inputSupplierForProgram(programId, key, type);
        }

        @Override
        public AutoBuildCtx odometryReset(boolean enabled) {
            this.odometryResetEnabled = enabled;
            return this;
        }

        @Override
        public boolean odometryResetEnabled() {
            return odometryResetEnabled;
        }

        @Override
        public AutoBuildCtx odometryResetTarget(OdometryResetTarget target) {
            this.odometryResetTarget = Objects.requireNonNull(target, "target");
            return this;
        }

        @Override
        public OdometryResetTarget odometryResetTarget() {
            return odometryResetTarget;
        }

        @Override
        public AutoBuildCtx odometryResetPose(Pose2d pose) {
            this.odometryResetPose = Objects.requireNonNull(pose, "pose");
            this.odometryResetTarget = OdometryResetTarget.POSE;
            return this;
        }

        @Override
        public Optional<Pose2d> odometryResetPose() {
            return Optional.ofNullable(odometryResetPose);
        }

        @Override
        public Command auto(String id) {
            Objects.requireNonNull(id, "id");
            Command command = autos.deferredScopedAuto(scopedAutos, scopedMarkers, id, OptionalInt.empty());
            if (!odometryResetEnabled) {
                return command;
            }
            Command wrapped = autos.applyScopedBuildCtxOdometryReset(
                    scopedAutos,
                    id,
                    OptionalInt.empty(),
                    command,
                    odometryResetTarget,
                    odometryResetPose);
            odometryResetEnabled = false;
            return wrapped;
        }

        @Override
        public Command auto(String id, int splitIndex) {
            Objects.requireNonNull(id, "id");
            OptionalInt split = OptionalInt.of(splitIndex);
            Command command = autos.deferredScopedAuto(scopedAutos, scopedMarkers, id, split);
            if (!odometryResetEnabled) {
                return command;
            }
            Command wrapped = autos.applyScopedBuildCtxOdometryReset(
                    scopedAutos,
                    id,
                    split,
                    command,
                    odometryResetTarget,
                    odometryResetPose);
            odometryResetEnabled = false;
            return wrapped;
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

    private record InputBinding<T>(String id, Class<T> type, Supplier<T> supplier) {
        private InputBinding {
            id = requireNonBlank(id, "auto input id");
            Objects.requireNonNull(type, "type");
            Objects.requireNonNull(supplier, "supplier");
        }
    }

    private record RuntimeInputOverride(Class<?> type, Object value) {
        private RuntimeInputOverride {
            Objects.requireNonNull(type, "type");
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

    private RobotAuto registerNamedCommand(NamedCommandKey key, Command command) {
        return registerNamedCommand(key, reusableCommandSupplier(command));
    }

    private RobotAuto registerNamedCommand(String id, Command command) {
        return registerNamedCommand(NamedCommandKey.of(id), command);
    }

    private RobotAuto registerNamedCommand(NamedCommandKey key, Supplier<Command> supplier) {
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

    private RobotAuto registerNamedCommand(String id, Supplier<Command> supplier) {
        return registerNamedCommand(NamedCommandKey.of(id), supplier);
    }

    private RobotAuto registerNamedCommand(NamedCommandKey key, Runnable action) {
        Objects.requireNonNull(action, "action");
        return registerNamedCommand(key, new InstantCommand(action));
    }

    private RobotAuto registerNamedCommand(String id, Runnable action) {
        return registerNamedCommand(NamedCommandKey.of(id), action);
    }

    private <T> RobotAuto registerInput(String key, Class<T> type, Supplier<T> supplier) {
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(supplier, "supplier");
        String id = requireNonBlank(key, "auto input id");
        InputBinding<?> existing = autoInputs.get(id);
        if (existing != null) {
            throw new IllegalArgumentException("Auto input already registered: " + id);
        }
        autoInputs.put(id, new InputBinding<>(id, type, supplier));
        return this;
    }

    private boolean hasNamedCommand(NamedCommandKey key) {
        return key != null && namedCommandSuppliers.containsKey(key.id());
    }

    private boolean hasNamedCommand(String id) {
        return id != null && namedCommandSuppliers.containsKey(id);
    }

    private boolean hasAuto(AutoKey key) {
        return key != null && autoRoutines.containsKey(key.id());
    }

    private boolean hasAuto(String id) {
        return id != null && autoRoutines.containsKey(id);
    }

    private boolean hasInput(String id) {
        return id != null && autoInputs.containsKey(id);
    }

    private boolean hasProgramScopedInput(String programId, String id) {
        if (programId == null || id == null) {
            return false;
        }
        Map<String, InputBinding<?>> inputs = programScopedInputs.get(programId);
        return inputs != null && inputs.containsKey(id);
    }

    private <T> Supplier<T> inputSupplier(String key, Class<T> type) {
        return inputSupplier(autoInputs, GLOBAL_INPUT_SCOPE, key, type);
    }

    private <T> Supplier<T> inputSupplierForProgram(String programId, String key, Class<T> type) {
        String scope = requireNonBlank(programId, "program id");
        Map<String, InputBinding<?>> inputs = programScopedInputs.get(scope);
        if (inputs == null) {
            throw new IllegalStateException("Auto program scope not registered: " + scope);
        }
        return inputSupplier(inputs, scope, key, type);
    }

    private <T> Supplier<T> inputSupplier(
            Map<String, InputBinding<?>> inputs,
            String scope,
            String key,
            Class<T> type) {
        Objects.requireNonNull(inputs, "inputs");
        Objects.requireNonNull(type, "type");
        String id = requireNonBlank(key, "auto input id");
        String resolvedScope = normalizeInputScope(scope);
        InputBinding<?> binding = inputs.get(id);
        if (binding == null) {
            throw new IllegalStateException("Auto input not registered: " + id);
        }
        if (!type.isAssignableFrom(binding.type())) {
            throw new IllegalStateException(
                    "Auto input type mismatch for '" + id + "': requested "
                            + type.getSimpleName() + " but registered "
                            + binding.type().getSimpleName());
        }
        @SuppressWarnings("unchecked")
        Supplier<T> fallbackSupplier = (Supplier<T>) binding.supplier();
        return () -> {
            RuntimeInputOverride override = runtimeInputOverride(resolvedScope, id);
            if (override == null) {
                return fallbackSupplier.get();
            }
            if (!type.isAssignableFrom(override.type())) {
                throw new IllegalStateException(
                        "Runtime auto input type mismatch for '" + id + "': requested "
                                + type.getSimpleName() + " but runtime value is "
                                + override.type().getSimpleName());
            }
            @SuppressWarnings("unchecked")
            T value = (T) override.value();
            return value;
        };
    }

    private <T> T inputValue(String key, Class<T> type) {
        return inputSupplier(key, type).get();
    }

    private static String normalizeInputScope(String scope) {
        if (scope == null) {
            return GLOBAL_INPUT_SCOPE;
        }
        String trimmed = scope.trim();
        return trimmed.isEmpty() ? GLOBAL_INPUT_SCOPE : trimmed;
    }

    private RuntimeInputOverride runtimeInputOverride(String scope, String key) {
        String resolvedScope = normalizeInputScope(scope);
        String id = requireNonBlank(key, "auto input id");
        Map<String, RuntimeInputOverride> scoped = runtimeInputOverrides.get(resolvedScope);
        RuntimeInputOverride value = scoped != null ? scoped.get(id) : null;
        if (value != null || GLOBAL_INPUT_SCOPE.equals(resolvedScope)) {
            return value;
        }
        Map<String, RuntimeInputOverride> global = runtimeInputOverrides.get(GLOBAL_INPUT_SCOPE);
        return global != null ? global.get(id) : null;
    }

    private <T> void setRuntimeInputOverride(String key, Class<T> type, T value) {
        setRuntimeInputOverride(GLOBAL_INPUT_SCOPE, key, type, value);
    }

    private <T> void setRuntimeInputOverride(String scope, String key, Class<T> type, T value) {
        Objects.requireNonNull(type, "type");
        String resolvedScope = normalizeInputScope(scope);
        String id = requireNonBlank(key, "auto input id");
        runtimeInputOverrides
                .computeIfAbsent(resolvedScope, ignored -> new LinkedHashMap<>())
                .put(id, new RuntimeInputOverride(type, value));
    }

    private void clearRuntimeInputOverride(String key) {
        clearRuntimeInputOverride(GLOBAL_INPUT_SCOPE, key);
    }

    private void clearRuntimeInputOverride(String scope, String key) {
        String resolvedScope = normalizeInputScope(scope);
        String id = requireNonBlank(key, "auto input id");
        Map<String, RuntimeInputOverride> scoped = runtimeInputOverrides.get(resolvedScope);
        if (scoped == null) {
            return;
        }
        scoped.remove(id);
        if (scoped.isEmpty()) {
            runtimeInputOverrides.remove(resolvedScope);
        }
    }

    private void clearRuntimeInputOverrides() {
        runtimeInputOverrides.clear();
    }

    private void clearRuntimeInputOverrides(String scope) {
        String resolvedScope = normalizeInputScope(scope);
        runtimeInputOverrides.remove(resolvedScope);
    }

    /**
     * Attaches the owning {@link RobotCore}. This is called automatically by {@link RobotCore}.
     */
    RobotAuto attachRobotCore(RobotCore<?> robot) {
        Objects.requireNonNull(robot, "robot");
        if (robotCore != null && robotCore != robot) {
            throw new IllegalStateException(
                    "RobotAuto is already attached to a different RobotCore instance.");
        }
        robotCore = robot;
        return this;
    }

    private Optional<RobotCore<?>> robotCore() {
        return Optional.ofNullable(robotCore);
    }

    private RobotCore<?> requireRobotCore() {
        RobotCore<?> robot = robotCore;
        if (robot == null) {
            throw new IllegalStateException(
                    "RobotAuto is not attached to a RobotCore. "
                            + "It must be attached by RobotCore during startup.");
        }
        return robot;
    }

    private RobotAuto registerProgram(AutoProgram program) {
        RobotCore<?> robot = requireRobotCore();
        Objects.requireNonNull(program, "program");
        String programId = Objects.requireNonNull(program.id(), "AutoProgram.id() returned null");
        if (programId.isBlank()) {
            throw new IllegalArgumentException("AutoProgram.id() cannot be blank for " + program.getClass().getName());
        }
        boolean hadProgramAutoBeforeRegister = hasAuto(programId);

        Map<String, AutoRoutine> scopedAutos = new LinkedHashMap<>();
        Map<String, Supplier<Command>> scopedMarkers = new LinkedHashMap<>();
        Map<String, InputBinding<?>> scopedInputs = new LinkedHashMap<>();

        AutoRegisterCtx regCtx = new AutoRegisterCtxImpl(
                robot,
                this,
                programId,
                scopedAutos,
                scopedMarkers,
                scopedInputs);
        program.register(regCtx);
        programScopedInputs.put(programId, Map.copyOf(scopedInputs));

        AutoBuildCtx buildCtx = new AutoBuildCtxImpl(
                robot,
                this,
                programId,
                Map.copyOf(scopedAutos),
                Map.copyOf(scopedMarkers),
                Map.copyOf(scopedInputs));
        AutoRoutine routine = bindScopedMarkersToRoutine(Objects.requireNonNull(
                program.build(buildCtx),
                "AutoProgram.build(ctx) returned null for " + program.getClass().getName()), Map.copyOf(scopedMarkers));
        if (!programId.equals(routine.key().id())) {
            throw new IllegalStateException(
                    "AutoProgram id mismatch for " + program.getClass().getName()
                            + ": program.id()=\"" + programId
                            + "\", routine.key().id()=\"" + routine.key().id() + "\"");
        }
        recordProgramPathRoutines(programId, scopedAutos, routine);
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

    private final RobotAuto registerPrograms(AutoProgram... programs) {
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
    private final RobotAuto registerPrograms(Class<? extends AutoProgram>... programTypes) {
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

    private RobotAuto registerAuto(AutoRoutine routine) {
        Objects.requireNonNull(routine, "routine");
        String id = requireNonBlank(routine.key().id(), "auto id");
        if (autoRoutines.containsKey(id)) {
            throw new IllegalArgumentException("Auto already registered: " + id);
        }
        autoRoutines.put(id, routine);
        resetChoosers();
        return this;
    }

    private RobotAuto registerAutos(AutoRoutine... routines) {
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

    private RobotAuto registerAuto(AutoKey key, Supplier<Command> factory) {
        return registerAuto(custom(key, factory));
    }

    private RobotAuto registerAuto(String id, Supplier<Command> factory) {
        return registerAuto(AutoKey.of(id), factory);
    }

    /**
     * Register a trajectory-backed auto. The chooser id defaults to {@code reference}.
     */
    private RobotAuto registerPath(String reference, TrajectorySource source) {
        return registerPath(reference, source, reference);
    }

    /**
     * Register a trajectory-backed auto with an alias id.
     */
    private RobotAuto registerPath(String reference, TrajectorySource source, String id) {
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

    private RobotAuto registerPath(AutoKey key, TrajectorySource source, String reference) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(source, "source");
        String resolvedReference = requireNonBlank(reference, "reference");
        validatePathReference(source, resolvedReference);
        return registerAuto(path(key, source, resolvedReference));
    }

    private RobotAuto setAutoPlanResetter(java.util.function.Function<Pose2d, Command> resetter) {
        this.autoPlanResetter = resetter;
        return this;
    }

    private RobotAuto setAutoTraceEnabled(boolean enabled) {
        this.autoTraceEnabled = enabled;
        return this;
    }

    private RobotAuto enableAutoTrace() {
        return setAutoTraceEnabled(true);
    }

    private RobotAuto disableAutoTrace() {
        return setAutoTraceEnabled(false);
    }

    private boolean isAutoTraceEnabled() {
        return autoTraceEnabled;
    }

    private RobotAuto setAutoTraceSink(Consumer<String> sink) {
        this.autoTraceSink = Objects.requireNonNull(sink, "sink");
        return this;
    }

    private List<AutoTraceEvent> getAutoTraceLog() {
        return autoTraceLog.snapshot();
    }

    private List<AutoTraceEvent> getAutoTraceLog(int limit) {
        return autoTraceLog.snapshot(limit);
    }

    private int getAutoTraceLogCount() {
        return autoTraceLog.count();
    }

    private int getAutoTraceLogCapacity() {
        return autoTraceLogCapacity;
    }

    private RobotAuto clearAutoTraceLog() {
        autoTraceLog.clear();
        return this;
    }

    private RobotAuto setAutoInitReset(AutoKey key, Boolean resetOnInit) {
        Objects.requireNonNull(key, "key");
        AutoRoutine routine = autoRoutines.get(key.id());
        if (routine == null) {
            throw new IllegalArgumentException("Auto not registered: " + key.id());
        }
        autoRoutines.put(key.id(), routine.withAutoInitResetOverride(resetOnInit));
        resetChoosers();
        return this;
    }

    private RobotAuto setAutoInitReset(String id, Boolean resetOnInit) {
        return setAutoInitReset(AutoKey.of(id), resetOnInit);
    }

    private Optional<AutoRoutine> getAuto(AutoKey key) {
        if (key == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(autoRoutines.get(key.id()));
    }

    private Optional<AutoRoutine> getAuto(String id) {
        if (id == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(autoRoutines.get(id));
    }

    private Collection<AutoRoutine> getAutos() {
        return Collections.unmodifiableCollection(autoRoutines.values());
    }

    private SendableChooser<AutoRoutine> createChooser(AutoKey defaultAuto) {
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
        commandChooserRoutineByCommand.clear();
        return chooser;
    }

    private SendableChooser<AutoRoutine> getAutoChooser() {
        return chooser;
    }

    private SendableChooser<Command> createCommandChooser(AutoKey defaultAuto) {
        createChooser(defaultAuto);
        AutoRoutine defaultRoutine = autoRoutines.get(defaultAuto.id());
        SendableChooser<Command> newChooser = new SendableChooser<>();
        commandChooserRoutineByCommand.clear();
        Command defaultCommand = deferredCommand(defaultRoutine);
        commandChooserRoutineByCommand.put(defaultCommand, defaultRoutine);
        newChooser.setDefaultOption(defaultRoutine.key().displayName(), defaultCommand);
        for (AutoRoutine routine : autoRoutines.values()) {
            if (routine.key().id().equals(defaultAuto.id())) {
                continue;
            }
            Command command = deferredCommand(routine);
            commandChooserRoutineByCommand.put(command, routine);
            newChooser.addOption(routine.key().displayName(), command);
        }
        commandChooser = newChooser;
        return newChooser;
    }

    private SendableChooser<Command> getCommandChooser() {
        return commandChooser;
    }

    private Optional<AutoRoutine> getSelectedAuto() {
        if (commandChooser != null) {
            Command selectedCommand = commandChooser.getSelected();
            if (selectedCommand != null) {
                AutoRoutine mapped = commandChooserRoutineByCommand.get(selectedCommand);
                if (mapped != null) {
                    return Optional.of(mapped);
                }
            }
        }
        AutoRoutine selectedRoutine = chooser != null ? chooser.getSelected() : null;
        if (selectedRoutine != null) {
            return Optional.of(selectedRoutine);
        }
        return Optional.empty();
    }

    private Optional<List<Pose2d>> getSelectedAutoPoses() {
        return getSelectedAuto().flatMap(this::getProgramPathPoses);
    }

    private Optional<List<Pose2d>> getProgramPathPoses(AutoRoutine selectedRoutine) {
        if (selectedRoutine == null) {
            return Optional.empty();
        }
        AutoKey key = selectedRoutine.key();
        String programId = key != null ? key.id() : null;
        String programCacheKey = programId != null ? allianceScopedPoseCacheKey(programId) : null;
        if (programId != null) {
            Optional<List<Pose2d>> cached = programPoseCache.get(programCacheKey);
            if (cached != null) {
                return cached;
            }
        }
        List<AutoRoutine> routines = programPathRoutines.get(selectedRoutine.key().id());
        Optional<List<Pose2d>> resolved;
        if (routines == null || routines.isEmpty()) {
            resolved = getAutoPoses(selectedRoutine);
        } else {
            List<Pose2d> merged = new ArrayList<>();
            for (AutoRoutine routine : routines) {
                getAutoPoses(routine)
                        .filter(poses -> poses != null && !poses.isEmpty())
                        .ifPresent(merged::addAll);
            }
            if (merged.isEmpty()) {
                resolved = Optional.empty();
            } else {
                resolved = Optional.of(List.copyOf(merged));
            }
        }
        if (programId != null) {
            programPoseCache.put(programCacheKey, resolved);
        }
        return resolved;
    }

    private void recordProgramPathRoutines(
            String programId,
            Map<String, AutoRoutine> scopedAutos,
            AutoRoutine programRoutine) {
        Objects.requireNonNull(programId, "programId");
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        List<AutoRoutine> routines = new ArrayList<>();
        for (AutoRoutine routine : scopedAutos.values()) {
            if (routine != null && routine.source() != AutoSource.CUSTOM) {
                routines.add(routine);
            }
        }
        if (programRoutine != null && programRoutine.source() != AutoSource.CUSTOM) {
            boolean alreadyIncluded = routines.stream()
                    .anyMatch(r -> r.key().id().equals(programRoutine.key().id()));
            if (!alreadyIncluded) {
                routines.add(programRoutine);
            }
        }
        programPathRoutines.put(programId, List.copyOf(routines));
    }

    private Optional<List<Pose2d>> getAutoPoses(AutoRoutine routine) {
        if (routine == null || routine.source() == AutoSource.CUSTOM) {
            return Optional.empty();
        }
        String cacheKey = allianceScopedPoseCacheKey(routine.source().name() + "|" + routine.reference());
        Optional<List<Pose2d>> cached = autoPoseCache.get(cacheKey);
        if (cached != null) {
            return cached;
        }
        Optional<List<Pose2d>> resolved = AutoBackends.forSource(routine.source())
                .flatMap(backend -> backend.getAutoPoses(routine.source(), routine.reference()));
        resolved = resolved.map(List::copyOf);
        autoPoseCache.put(cacheKey, resolved);
        return resolved;
    }

    private static String allianceScopedPoseCacheKey(String key) {
        String alliance = DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN");
        return key + "|" + alliance;
    }

    private Optional<Command> buildSelectedCommand() {
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

    private Optional<Command> prelightSelectedCommand() {
        Optional<AutoRoutine> selectedOpt = getSelectedAuto();
        if (selectedOpt.isEmpty()) {
            clearPrelightCache();
            return Optional.empty();
        }
        AutoRoutine selected = selectedOpt.get();
        String signature = prelightSignature(selected);
        if (prelightCommand != null && signature.equals(prelightSignature)) {
            return Optional.of(prelightCommand);
        }

        getProgramPathPoses(selected);
        warmupSource(selected.source());
        prelightProgramPathCommands(selected);

        Command built;
        try {
            built = selected.createCommand();
        } catch (RuntimeException ex) {
            DriverStation.reportWarning(
                    "Failed to prelight auto \"" + selected.key().id() + "\": " + ex.getMessage(),
                    ex.getStackTrace());
            prelightSignature = signature;
            prelightCommand = null;
            return Optional.empty();
        }
        if (built == null) {
            prelightSignature = signature;
            prelightCommand = null;
            return Optional.empty();
        }

        prelightSignature = signature;
        prelightCommand = built;
        return Optional.of(built);
    }

    private String prelightSignature(AutoRoutine routine) {
        if (routine == null) {
            return "";
        }
        String alliance = DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN");
        String id = routine.key() != null && routine.key().id() != null ? routine.key().id() : "";
        String reference = routine.reference() != null ? routine.reference() : "";
        return id + "|" + routine.source().name() + "|" + reference + "|" + alliance;
    }

    private void warmupSource(AutoSource source) {
        if (source == null || source == AutoSource.CUSTOM) {
            return;
        }
        if (!warmedAutoSources.add(source)) {
            return;
        }
        AutoBackends.forSource(source)
                .flatMap(backend -> backend.warmupCommand(source))
                .ifPresent(CommandScheduler.getInstance()::schedule);
    }

    private void prelightProgramPathCommands(AutoRoutine selectedRoutine) {
        if (selectedRoutine == null) {
            return;
        }
        List<AutoRoutine> routines = programPathRoutines.get(selectedRoutine.key().id());
        if (routines == null || routines.isEmpty()) {
            routines = List.of(selectedRoutine);
        }
        for (AutoRoutine routine : routines) {
            if (routine == null || routine.source() == AutoSource.CUSTOM) {
                continue;
            }
            try {
                buildPathCommand(routine.source(), routine.reference());
            } catch (RuntimeException ex) {
                DriverStation.reportWarning(
                        "Failed to prelight path \"" + routine.reference() + "\": " + ex.getMessage(),
                        ex.getStackTrace());
            }
        }
    }

    private ProfiledPIDController getXController() {
        return xController;
    }

    private ProfiledPIDController getYController() {
        return yController;
    }

    private ProfiledPIDController getThetaController() {
        return thetaController;
    }

    private RobotAuto setPoseControllers(ProfiledPIDController xController,
                                        ProfiledPIDController yController,
                                        ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        return this;
    }

    private RobotAuto setStartingPose(AutoKey key, Pose2d pose) {
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

    private RobotAuto setStartingPose(String autoId, Pose2d pose) {
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
    private void finalizeRegistration() {
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

    private Command deferredScopedAuto(
            Map<String, AutoRoutine> scopedAutos,
            Map<String, Supplier<Command>> scopedMarkers,
            String id,
            OptionalInt splitIndex) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
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
            boolean resolvedBaseScoped = true;
            if (resolvedBase == null) {
                resolvedBase = autoRoutines.get(id);
                resolvedBaseScoped = false;
            }
            if (resolvedBase == null) {
                throw new IllegalStateException(
                        "Auto not registered in program scope or global registry: "
                                + id + " (or split " + explicitSplitId + ")");
            }
            if (resolvedBase.source() == AutoSource.CUSTOM) {
                throw new IllegalStateException("Auto split requested for CUSTOM auto: " + id + "." + split);
            }
            String splitReference = resolvedBase.reference() + "." + split;
            AutoSource splitSource = resolvedBase.source();
            Command splitCommand = resolvedBaseScoped
                    ? Commands.defer(
                            () -> buildScopedPathCommand(scopedMarkers, splitSource, splitReference),
                            Set.of())
                    : Commands.defer(() -> buildPathCommand(splitSource, splitReference), Set.of());
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
            throw new IllegalStateException(
                    "Auto not registered in program scope or global registry: " + id);
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

        Optional<Pose2d> basePose = resolvedStartingPose(base);
        if (splitIndex.isPresent()) {
            AutoRoutine split = resolveRoutineFor(scopedAutos, id, splitIndex);
            Optional<Pose2d> splitPose = resolvedStartingPose(split);
            if (splitPose.isPresent()) {
                return splitPose;
            }
            if (basePose.isPresent()) {
                return basePose;
            }
            return Optional.empty();
        }
        if (basePose.isEmpty()) {
            return Optional.empty();
        }
        return basePose;
    }

    private Optional<Pose2d> resolvedStartingPose(AutoRoutine routine) {
        if (routine == null) {
            return Optional.empty();
        }
        if (routine.hasStartingPose()) {
            return Optional.ofNullable(routine.startingPose());
        }
        Optional<List<Pose2d>> poses = getAutoPoses(routine);
        if (poses.isEmpty() || poses.get().isEmpty()) {
            return Optional.empty();
        }
        return Optional.ofNullable(poses.get().get(0));
    }

    private AutoRoutine resolveRoutineFor(Map<String, AutoRoutine> scopedAutos, String id, OptionalInt splitIndex) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(id, "id");
        AutoRoutine base = scopedAutos.get(id);
        if (base == null) {
            base = autoRoutines.get(id);
        }
        if (splitIndex.isEmpty()) {
            return base;
        }
        int split = splitIndex.getAsInt();
        String splitId = id + "." + split;
        AutoRoutine explicitSplit = scopedAutos.get(splitId);
        if (explicitSplit == null) {
            explicitSplit = autoRoutines.get(splitId);
        }
        if (explicitSplit != null) {
            return explicitSplit;
        }
        if (base == null || base.source() == AutoSource.CUSTOM) {
            return null;
        }
        String splitReference = base.reference() + "." + split;
        AutoSource splitSource = base.source();
        Supplier<Command> splitFactory = () -> buildPathCommand(splitSource, splitReference);
        return new AutoRoutine(
                AutoKey.of(splitId),
                splitSource,
                splitReference,
                splitFactory,
                new Pose2d(),
                false,
                null);
    }

    private Optional<Pose2d> endingPoseFor(Map<String, AutoRoutine> scopedAutos, String id, OptionalInt splitIndex) {
        AutoRoutine routine = resolveRoutineFor(scopedAutos, id, splitIndex);
        if (routine == null) {
            return Optional.empty();
        }
        Optional<List<Pose2d>> poses = getAutoPoses(routine);
        if (poses.isEmpty() || poses.get().isEmpty()) {
            return Optional.empty();
        }
        List<Pose2d> path = poses.get();
        return Optional.ofNullable(path.get(path.size() - 1));
    }

    private Command applyBuildCtxOdometryReset(
            Map<String, AutoRoutine> scopedAutos,
            String id,
            OptionalInt splitIndex,
            Command command,
            AutoBuildCtx.OdometryResetTarget target,
            Pose2d explicitPose) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(command, "command");
        AutoBuildCtx.OdometryResetTarget resolvedTarget =
                target != null ? target : AutoBuildCtx.OdometryResetTarget.NONE;
        if (resolvedTarget == AutoBuildCtx.OdometryResetTarget.NONE) {
            return command;
        }

        Optional<Pose2d> resetPose;
        switch (resolvedTarget) {
            case PATH_START -> resetPose = startingPoseFor(scopedAutos, id, splitIndex);
            case PATH_END -> resetPose = endingPoseFor(scopedAutos, id, splitIndex);
            case POSE -> resetPose = Optional.ofNullable(explicitPose);
            case NONE -> resetPose = Optional.empty();
            default -> resetPose = Optional.empty();
        }

        if (resetPose.isEmpty()) {
            String splitLabel = splitIndex.isPresent() ? "." + splitIndex.getAsInt() : "";
            throw new IllegalStateException(
                    "Unable to resolve odometry reset pose for auto \"" + id + splitLabel
                            + "\" with target " + resolvedTarget + ".");
        }

        Pose2d pose = resetPose.get();
        String splitLabel = splitIndex.isPresent() ? "." + splitIndex.getAsInt() : "";
        String detail = "target=" + resolvedTarget
                + " x=" + String.format(Locale.US, "%.3f", pose.getX())
                + " y=" + String.format(Locale.US, "%.3f", pose.getY())
                + " deg=" + String.format(Locale.US, "%.2f", pose.getRotation().getDegrees());
        return traceAutoLifecycle(
                "ctx.odometryReset(" + id + splitLabel + ")",
                detail,
                Commands.sequence(resetPose(pose), command));
    }

    private Command applyScopedBuildCtxOdometryReset(
            Map<String, AutoRoutine> scopedAutos,
            String id,
            OptionalInt splitIndex,
            Command command,
            AutoBuildCtx.OdometryResetTarget target,
            Pose2d explicitPose) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(command, "command");
        AutoBuildCtx.OdometryResetTarget resolvedTarget =
                target != null ? target : AutoBuildCtx.OdometryResetTarget.NONE;
        if (resolvedTarget == AutoBuildCtx.OdometryResetTarget.NONE) {
            return command;
        }

        Optional<Pose2d> resetPose;
        switch (resolvedTarget) {
            case PATH_START -> resetPose = startingPoseFor(scopedAutos, id, splitIndex);
            case PATH_END -> resetPose = endingPoseFor(scopedAutos, id, splitIndex);
            case POSE -> resetPose = Optional.ofNullable(explicitPose);
            case NONE -> resetPose = Optional.empty();
            default -> resetPose = Optional.empty();
        }

        if (resetPose.isEmpty()) {
            String splitLabel = splitIndex.isPresent() ? "." + splitIndex.getAsInt() : "";
            throw new IllegalStateException(
                    "Unable to resolve odometry reset pose for auto \"" + id + splitLabel
                            + "\" with target " + resolvedTarget + ".");
        }

        Pose2d pose = resetPose.get();
        String splitLabel = splitIndex.isPresent() ? "." + splitIndex.getAsInt() : "";
        String detail = "target=" + resolvedTarget
                + " x=" + String.format(Locale.US, "%.3f", pose.getX())
                + " y=" + String.format(Locale.US, "%.3f", pose.getY())
                + " deg=" + String.format(Locale.US, "%.2f", pose.getRotation().getDegrees());
        return traceAutoLifecycle(
                "ctx.odometryReset(" + id + splitLabel + ")",
                detail,
                Commands.sequence(resetPose(pose), command));
    }

    private static Command buildPathCommand(AutoSource source, String reference) {
        return backendForSource(source)
                .buildAuto(source, reference)
                .orElseThrow(() -> new IllegalStateException(
                        "Auto backend could not build " + source + " routine for reference \"" + reference + "\"."));
    }

    private Command buildScopedPathCommand(
            Map<String, Supplier<Command>> scopedMarkers,
            AutoSource source,
            String reference) {
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(reference, "reference");
        for (Map.Entry<String, Supplier<Command>> entry : scopedMarkers.entrySet()) {
            bindScopedMarkerToAvailableBackends(entry.getKey(), entry.getValue());
        }
        return buildPathCommand(source, reference);
    }

    private void bindScopedMarkerToAvailableBackends(String id, Supplier<Command> supplier) {
        String resolvedId = requireNonBlank(id, "marker id");
        Objects.requireNonNull(supplier, "supplier");
        Supplier<Command> deferred = () -> Commands.deferredProxy(supplier);
        AutoBackends.forSource(AutoSource.PATH_PLANNER)
                .ifPresent(backend -> backend.registerNamedCommand(resolvedId, deferred));
        AutoBackends.forSource(AutoSource.CHOREO)
                .ifPresent(backend -> backend.registerNamedCommand(resolvedId, deferred));
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
            Map<String, Supplier<Command>> scopedMarkers,
            String programId,
            String reference,
            TrajectorySource source,
            String id) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
        String resolvedReference = requireNonBlank(reference, "reference");
        String resolvedId = requireNonBlank(id, "id");
        try {
            validatePathReference(source, resolvedReference);
            registerScopedAuto(
                    scopedAutos,
                    scopedMarkers,
                    programId,
                    path(AutoKey.of(resolvedId), source, resolvedReference));
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
            registerScopedAuto(
                    scopedAutos,
                    scopedMarkers,
                    programId,
                    path(AutoKey.of(resolvedReference), source, resolvedId));
        }
    }

    private void registerScopedAuto(
            Map<String, AutoRoutine> scopedAutos,
            Map<String, Supplier<Command>> scopedMarkers,
            String programId,
            AutoRoutine routine) {
        Objects.requireNonNull(scopedAutos, "scopedAutos");
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
        Objects.requireNonNull(programId, "programId");
        Objects.requireNonNull(routine, "routine");
        String id = requireNonBlank(routine.key().id(), "auto id");
        if (scopedAutos.containsKey(id)) {
            throw new IllegalArgumentException(
                    "Auto already registered in program \"" + programId + "\": " + id);
        }
        scopedAutos.put(id, bindScopedMarkersToRoutine(routine, scopedMarkers));
    }

    private AutoRoutine bindScopedMarkersToRoutine(
            AutoRoutine routine,
            Map<String, Supplier<Command>> scopedMarkers) {
        Objects.requireNonNull(routine, "routine");
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
        if (routine.source() == AutoSource.CUSTOM) {
            return routine;
        }
        Supplier<Command> wrappedFactory =
                () -> buildScopedPathCommand(scopedMarkers, routine.source(), routine.reference());
        return new AutoRoutine(
                routine.key(),
                routine.source(),
                routine.reference(),
                wrappedFactory,
                routine.startingPose(),
                routine.hasStartingPose(),
                routine.autoInitResetOverride());
    }

    private void registerScopedMarker(
            Map<String, Supplier<Command>> scopedMarkers,
            String programId,
            String id,
            Supplier<Command> supplier) {
        Objects.requireNonNull(scopedMarkers, "scopedMarkers");
        Objects.requireNonNull(programId, "programId");
        Objects.requireNonNull(supplier, "supplier");
        String resolvedId = requireNonBlank(id, "marker id");
        if (scopedMarkers.containsKey(resolvedId)) {
            throw new IllegalArgumentException(
                    "Marker already registered in program \"" + programId + "\": " + resolvedId);
        }
        Supplier<Command> tracedSupplier = traceNamedCommandSupplier(programId + "/" + resolvedId, supplier);
        scopedMarkers.put(resolvedId, tracedSupplier);
    }

    private <T> void registerScopedInput(
            Map<String, InputBinding<?>> scopedInputs,
            String programId,
            String key,
            Class<T> type,
            Supplier<T> supplier) {
        Objects.requireNonNull(scopedInputs, "scopedInputs");
        Objects.requireNonNull(programId, "programId");
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(supplier, "supplier");
        String id = requireNonBlank(key, "auto input id");
        InputBinding<?> existing = scopedInputs.get(id);
        if (existing != null) {
            throw new IllegalArgumentException(
                    "Auto input already registered in program \"" + programId + "\": " + id);
        }
        scopedInputs.put(id, new InputBinding<>(id, type, supplier));
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
            RobotLocalization<?> localization = core.localization();
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
        runtimeInputOverrides.clear();
        autoPoseCache.clear();
        programPoseCache.clear();
        clearPrelightCache();
    }

    private void clearPrelightCache() {
        prelightSignature = "";
        prelightCommand = null;
    }

    private void invalidateTrajectoryCaches() {
        autoPoseCache.clear();
        programPoseCache.clear();
        warmedAutoSources.clear();
        clearPrelightCache();
    }

    public final class RegistrySection {
        public RegistrySection program(AutoProgram program) {
            registerProgram(program);
            return this;
        }

        public RegistrySection programs(AutoProgram... programs) {
            registerPrograms(programs);
            return this;
        }

        @SafeVarargs
        public final RegistrySection programTypes(Class<? extends AutoProgram>... types) {
            registerPrograms(types);
            return this;
        }

        public RegistrySection command(NamedCommandKey key, Command command) {
            registerNamedCommand(key, command);
            return this;
        }

        public RegistrySection command(String id, Command command) {
            registerNamedCommand(id, command);
            return this;
        }

        public RegistrySection command(NamedCommandKey key, Supplier<Command> supplier) {
            registerNamedCommand(key, supplier);
            return this;
        }

        public RegistrySection command(String id, Supplier<Command> supplier) {
            registerNamedCommand(id, supplier);
            return this;
        }

        public RegistrySection command(NamedCommandKey key, Runnable action) {
            registerNamedCommand(key, action);
            return this;
        }

        public RegistrySection command(String id, Runnable action) {
            registerNamedCommand(id, action);
            return this;
        }

        public <V> RegistrySection input(String key, Class<V> type, Supplier<V> supplier) {
            registerInput(key, type, supplier);
            return this;
        }

        public RegistrySection bool(String key, BooleanSupplier supplier) {
            registerInput(key, Boolean.class, () -> supplier.getAsBoolean());
            return this;
        }

        public RegistrySection bool(String key, boolean value) {
            registerInput(key, Boolean.class, () -> value);
            return this;
        }

        public RegistrySection dbl(String key, DoubleSupplier supplier) {
            registerInput(key, Double.class, () -> supplier.getAsDouble());
            return this;
        }

        public RegistrySection dbl(String key, double value) {
            registerInput(key, Double.class, () -> value);
            return this;
        }

        public RegistrySection integer(String key, IntSupplier supplier) {
            registerInput(key, Integer.class, () -> supplier.getAsInt());
            return this;
        }

        public RegistrySection integer(String key, int value) {
            registerInput(key, Integer.class, () -> value);
            return this;
        }

        public RegistrySection string(String key, Supplier<String> supplier) {
            registerInput(key, String.class, supplier);
            return this;
        }

        public RegistrySection string(String key, String value) {
            registerInput(key, String.class, () -> value);
            return this;
        }

        public RegistrySection pose2d(String key, Supplier<Pose2d> supplier) {
            registerInput(key, Pose2d.class, supplier);
            return this;
        }

        public RegistrySection pose2d(String key, Pose2d value) {
            registerInput(key, Pose2d.class, () -> value);
            return this;
        }

        public RegistrySection pose3d(String key, Supplier<Pose3d> supplier) {
            registerInput(key, Pose3d.class, supplier);
            return this;
        }

        public RegistrySection pose3d(String key, Pose3d value) {
            registerInput(key, Pose3d.class, () -> value);
            return this;
        }

        public <V> RegistrySection object(String key, Class<V> type, Supplier<V> supplier) {
            registerInput(key, type, supplier);
            return this;
        }

        public <V> RegistrySection object(String key, Class<V> type, V value) {
            registerInput(key, type, () -> value);
            return this;
        }

        public RegistrySection routine(AutoRoutine routine) {
            registerAuto(routine);
            return this;
        }

        public RegistrySection routines(AutoRoutine... routines) {
            registerAutos(routines);
            return this;
        }

        public RegistrySection routine(AutoKey key, Supplier<Command> factory) {
            registerAuto(key, factory);
            return this;
        }

        public RegistrySection routine(String id, Supplier<Command> factory) {
            registerAuto(id, factory);
            return this;
        }

        public RegistrySection path(String reference, TrajectorySource source) {
            registerPath(reference, source);
            return this;
        }

        public RegistrySection path(String reference, TrajectorySource source, String id) {
            registerPath(reference, source, id);
            return this;
        }

        public RegistrySection path(AutoKey key, TrajectorySource source, String reference) {
            registerPath(key, source, reference);
            return this;
        }

        public boolean hasCommand(String id) {
            return hasNamedCommand(id);
        }

        public boolean hasRoutine(String id) {
            return hasAuto(id);
        }

        public boolean hasInput(String id) {
            return RobotAuto.this.hasInput(id);
        }
    }

    public final class CommandsSection {
        public RobotAuto put(NamedCommandKey key, Command command) {
            registry().command(key, command);
            return RobotAuto.this;
        }

        public RobotAuto put(String id, Command command) {
            registry().command(id, command);
            return RobotAuto.this;
        }

        public RobotAuto put(NamedCommandKey key, Supplier<Command> supplier) {
            registry().command(key, supplier);
            return RobotAuto.this;
        }

        public RobotAuto put(String id, Supplier<Command> supplier) {
            registry().command(id, supplier);
            return RobotAuto.this;
        }

        public RobotAuto put(NamedCommandKey key, Runnable action) {
            registry().command(key, action);
            return RobotAuto.this;
        }

        public RobotAuto put(String id, Runnable action) {
            registry().command(id, action);
            return RobotAuto.this;
        }

        public boolean has(String id) {
            return registry().hasCommand(id);
        }
    }

    public final class InputsSection {
        public <V> RobotAuto put(String key, Class<V> type, Supplier<V> supplier) {
            registry().input(key, type, supplier);
            return RobotAuto.this;
        }

        public InputsSection bool(String key, BooleanSupplier supplier) {
            registry().bool(key, supplier);
            return this;
        }

        public InputsSection dbl(String key, DoubleSupplier supplier) {
            registry().dbl(key, supplier);
            return this;
        }

        public InputsSection integer(String key, IntSupplier supplier) {
            registry().integer(key, supplier);
            return this;
        }

        public InputsSection string(String key, Supplier<String> supplier) {
            registry().string(key, supplier);
            return this;
        }

        public InputsSection pose2d(String key, Supplier<Pose2d> supplier) {
            registry().pose2d(key, supplier);
            return this;
        }

        public InputsSection pose3d(String key, Supplier<Pose3d> supplier) {
            registry().pose3d(key, supplier);
            return this;
        }

        public <V> InputsSection object(String key, Class<V> type, Supplier<V> supplier) {
            registry().object(key, type, supplier);
            return this;
        }

        /**
         * Overrides the runtime value for an auto input id.
         *
         * <p>The override is shared across nested/linked autos and takes precedence over
         * the registered supplier until cleared.</p>
         */
        public <T> InputsSection set(String key, Class<T> type, T value) {
            setRuntimeInputOverride(key, type, value);
            return this;
        }

        public <T> InputsSection clear(String key, Class<T> type) {
            Objects.requireNonNull(type, "type");
            clearRuntimeInputOverride(key);
            return this;
        }

        public InputsSection clear(String key) {
            clearRuntimeInputOverride(key);
            return this;
        }

        public InputsSection clear() {
            clearRuntimeInputOverrides();
            return this;
        }

        public InputsSection bool(String key, boolean value) {
            set(key, Boolean.class, value);
            return this;
        }

        public InputsSection resetBool(String key) {
            clear(key, Boolean.class);
            return this;
        }

        public InputsSection dbl(String key, double value) {
            set(key, Double.class, value);
            return this;
        }

        public InputsSection resetDouble(String key) {
            clear(key, Double.class);
            return this;
        }

        public InputsSection integer(String key, int value) {
            set(key, Integer.class, value);
            return this;
        }

        public InputsSection resetInt(String key) {
            clear(key, Integer.class);
            return this;
        }

        public InputsSection string(String key, String value) {
            set(key, String.class, value);
            return this;
        }

        public InputsSection resetString(String key) {
            clear(key, String.class);
            return this;
        }

        public InputsSection pose2d(String key, Pose2d value) {
            set(key, Pose2d.class, value);
            return this;
        }

        public InputsSection resetPose2d(String key) {
            clear(key, Pose2d.class);
            return this;
        }

        public InputsSection pose3d(String key, Pose3d value) {
            set(key, Pose3d.class, value);
            return this;
        }

        public InputsSection resetPose3d(String key) {
            clear(key, Pose3d.class);
            return this;
        }

        public <V> InputsSection object(String key, Class<V> type, V value) {
            set(key, type, value);
            return this;
        }

        public <V> InputsSection resetObject(String key, Class<V> type) {
            clear(key, type);
            return this;
        }

        public boolean has(String id) {
            return registry().hasInput(id);
        }

        public <T> Supplier<T> supplier(String key, Class<T> type) {
            return inputSupplier(key, type);
        }

        public <T> T value(String key, Class<T> type) {
            return inputValue(key, type);
        }

        public boolean bool(String key) {
            return Boolean.TRUE.equals(value(key, Boolean.class));
        }

        public double dbl(String key) {
            Double value = value(key, Double.class);
            return value != null ? value.doubleValue() : Double.NaN;
        }

        public int integer(String key) {
            Integer value = value(key, Integer.class);
            return value != null ? value.intValue() : 0;
        }

        public String string(String key) {
            return value(key, String.class);
        }

        public Pose2d pose2d(String key) {
            return value(key, Pose2d.class);
        }

        public Pose3d pose3d(String key) {
            return value(key, Pose3d.class);
        }

        public <V> V object(String key, Class<V> type) {
            return value(key, type);
        }
    }

    public final class RoutinesSection {
        public RobotAuto put(AutoRoutine routine) {
            registry().routine(routine);
            return RobotAuto.this;
        }

        public RobotAuto put(AutoRoutine... routines) {
            registry().routines(routines);
            return RobotAuto.this;
        }

        public RobotAuto put(AutoKey key, Supplier<Command> factory) {
            registry().routine(key, factory);
            return RobotAuto.this;
        }

        public RobotAuto put(String id, Supplier<Command> factory) {
            registry().routine(id, factory);
            return RobotAuto.this;
        }

        public RobotAuto path(String reference, TrajectorySource source) {
            registry().path(reference, source);
            return RobotAuto.this;
        }

        public RobotAuto path(String reference, TrajectorySource source, String id) {
            registry().path(reference, source, id);
            return RobotAuto.this;
        }

        public RobotAuto path(AutoKey key, TrajectorySource source, String reference) {
            registry().path(key, source, reference);
            return RobotAuto.this;
        }

        public boolean has(String id) {
            return registry().hasRoutine(id);
        }

        public Optional<AutoRoutine> one(String id) {
            return getAuto(id);
        }

        public Collection<AutoRoutine> all() {
            return getAutos();
        }

        public Optional<List<Pose2d>> poses(AutoRoutine routine) {
            return getAutoPoses(routine);
        }

        public RobotAuto startingPose(AutoKey key, Pose2d pose) {
            return setStartingPose(key, pose);
        }

        public RobotAuto startingPose(String id, Pose2d pose) {
            return setStartingPose(id, pose);
        }
    }

    public final class SelectionSection {
        public SendableChooser<AutoRoutine> programChooser(AutoKey defaultProgram) {
            return createChooser(defaultProgram);
        }

        public Optional<SendableChooser<AutoRoutine>> programChooser() {
            return Optional.ofNullable(chooser);
        }

        public Optional<AutoRoutine> selectedProgram() {
            return getSelectedAuto();
        }

        public Optional<List<Pose2d>> selectedProgramPoses() {
            return getSelectedAutoPoses();
        }

        public SendableChooser<AutoRoutine> chooser(AutoKey defaultAuto) {
            return programChooser(defaultAuto);
        }

        public SendableChooser<Command> commandChooser(AutoKey defaultAuto) {
            return createCommandChooser(defaultAuto);
        }

        public Optional<SendableChooser<AutoRoutine>> chooser() {
            return programChooser();
        }

        public Optional<SendableChooser<Command>> commandChooser() {
            return Optional.ofNullable(commandChooser);
        }

        public Optional<AutoRoutine> selected() {
            return selectedProgram();
        }

        public Optional<List<Pose2d>> selectedPoses() {
            return selectedProgramPoses();
        }
    }

    public final class ExecutionSection {
        public Optional<Command> selectedCommand() {
            return buildSelectedCommand();
        }

        public Optional<Command> prelightSelectedCommand() {
            return RobotAuto.this.prelightSelectedCommand();
        }

        public void invalidateTrajectoryCaches() {
            RobotAuto.this.invalidateTrajectoryCaches();
        }

        public void prepare() {
            finalizeRegistration();
        }
    }

    public final class TraceSection {
        public TraceSection enabled(boolean enabled) {
            autoTraceEnabled = enabled;
            return this;
        }

        public boolean enabled() {
            return autoTraceEnabled;
        }

        public TraceSection sink(Consumer<String> sink) {
            autoTraceSink = Objects.requireNonNull(sink, "sink");
            return this;
        }

        public List<AutoTraceEvent> events() {
            return autoTraceLog.snapshot();
        }

        public List<AutoTraceEvent> events(int limit) {
            return autoTraceLog.snapshot(limit);
        }

        public int count() {
            return autoTraceLog.count();
        }

        public int capacity() {
            return autoTraceLogCapacity;
        }

        public TraceSection clear() {
            autoTraceLog.clear();
            return this;
        }
    }

    public final class ResetSection {
        public RobotAuto poseResetter(java.util.function.Function<Pose2d, Command> resetter) {
            autoPlanResetter = resetter;
            return RobotAuto.this;
        }

        public RobotAuto autoInit(AutoKey key, Boolean resetOnInit) {
            return setAutoInitReset(key, resetOnInit);
        }

        public RobotAuto autoInit(String id, Boolean resetOnInit) {
            return setAutoInitReset(id, resetOnInit);
        }
    }

    public final class ControllersSection {
        public ProfiledPIDController x() {
            return xController;
        }

        public ProfiledPIDController y() {
            return yController;
        }

        public ProfiledPIDController theta() {
            return thetaController;
        }

        public ControllersSection pose(
                ProfiledPIDController x,
                ProfiledPIDController y,
                ProfiledPIDController theta) {
            xController = x;
            yController = y;
            thetaController = theta;
            return this;
        }
    }
}
