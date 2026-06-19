package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.ToDoubleFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * Runtime-owned mechanism configuration used by the V2 API path.
 *
 * <p>This holds the resolved runtime data the base {@link Mechanism} class needs without requiring
 * any authoring-time declaration object to exist.</p>
 */
public final class MechanismRuntimeConfig<T extends Mechanism> {
    @FunctionalInterface
    public interface MechanismControlLoop<T extends Mechanism>
            extends ToDoubleFunction<MechanismControlContext<T>> {
        default double calculate(MechanismControlContext<T> context) {
            return applyAsDouble(context);
        }
    }

    public record ControlLoopBinding<T extends Mechanism>(
            String name,
            double periodSeconds,
            MechanismControlLoop<T> loop) {
        public ControlLoopBinding {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(loop, "loop");
            if (!Double.isFinite(periodSeconds) || periodSeconds < 0.0) {
                throw new IllegalArgumentException("periodSeconds must be finite and >= 0");
            }
        }

    }

    public record PeriodicHookBinding<T extends Mechanism>(
            Consumer<T> hook,
            double periodMs) {
        public PeriodicHookBinding {
            Objects.requireNonNull(hook, "hook");
            if (!Double.isFinite(periodMs) || periodMs < 0.0) {
                throw new IllegalArgumentException("periodMs must be finite and >= 0");
            }
        }
    }

    public record PidAutotunerConfig(
            boolean enabled,
            String dashboardPath,
            MechanismPidAutotunerProgram program) {
        public static PidAutotunerConfig defaults() {
            return new PidAutotunerConfig(false, null, null);
        }

    }

    public record PidProfile(
            OutputType outputType,
            double kP,
            double kI,
            double kD,
            double iZone,
            double tolerance,
            double maxVelocity,
            double maxAcceleration,
            PidAutotunerConfig autotuner,
            MechanismInputSource inputSource,
            MechanismSetpointSource setpointSource) {
        public PidProfile {
            autotuner = autotuner != null ? autotuner : PidAutotunerConfig.defaults();
            inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

        public PidProfile(
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance,
                double maxVelocity,
                double maxAcceleration,
                PidAutotunerConfig autotuner) {
            this(outputType, kP, kI, kD, iZone, tolerance, maxVelocity, maxAcceleration, autotuner, null, null);
        }

    }

    public record BangBangProfile(
            OutputType outputType,
            double highOutput,
            double lowOutput,
            double tolerance,
            MechanismInputSource inputSource,
            MechanismSetpointSource setpointSource) {
        public BangBangProfile {
            inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

    }

    public enum FeedforwardType {
        SIMPLE,
        ARM,
        ELEVATOR;

    }

    public record FeedforwardProfile(
            OutputType outputType,
            FeedforwardType type,
            double kS,
            double kG,
            double kV,
            double kA,
            double tolerance,
            MechanismSetpointSource setpointSource) {
        public FeedforwardProfile {
            type = type != null ? type : FeedforwardType.SIMPLE;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

        public FeedforwardProfile(
                OutputType outputType,
                FeedforwardType type,
                double kS,
                double kG,
                double kV,
                double kA,
                double tolerance) {
            this(outputType, type, kS, kG, kV, kA, tolerance, null);
        }

        public SimpleMotorFeedforward simple() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        public ArmFeedforward arm() {
            return new ArmFeedforward(kS, kG, kV, kA);
        }

        public ElevatorFeedforward elevator() {
            return new ElevatorFeedforward(kS, kG, kV, kA);
        }
    }

    private final String name;
    private final Object sourceKey;
    private final boolean configDisabled;
    private final List<MotorControllerConfig> motors;
    private final Map<String, MechanismEncoderSource> encoderSources;
    private final String positionSourceName;
    private final String velocitySourceName;
    private final String absoluteSourceName;
    private final MechanismConfigRecord data;
    private final List<Consumer<T>> periodicHooks;
    private final List<PeriodicHookBinding<T>> periodicHookBindings;
    private final Map<String, BooleanSupplier> inputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;
    private final Map<String, Boolean> mutableBoolInputDefaults;
    private final Map<String, Double> mutableDoubleInputDefaults;
    private final Map<String, Integer> mutableIntInputDefaults;
    private final Map<String, String> mutableStringInputDefaults;
    private final Map<String, Pose2d> mutablePose2dInputDefaults;
    private final Map<String, Pose3d> mutablePose3dInputDefaults;
    private final Map<String, PidProfile> controlLoopPidProfiles;
    private final Map<String, BangBangProfile> controlLoopBangBangProfiles;
    private final Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles;
    private final List<ControlLoopBinding<T>> controlLoops;
    private final Set<String> initiallyDisabledControlLoops;
    private final MechanismSimulationConfig simulationConfig;
    private final MechanismVisualizationConfig visualizationConfig;
    private final MechanismSensorSimulationConfig sensorSimulationConfig;

    public MechanismRuntimeConfig(
            String name,
            Object sourceKey,
            boolean configDisabled,
            List<MotorControllerConfig> motors,
            Map<String, MechanismEncoderSource> encoderSources,
            String positionSourceName,
            String velocitySourceName,
            String absoluteSourceName,
            MechanismConfigRecord data,
            List<Consumer<T>> periodicHooks,
            List<PeriodicHookBinding<T>> periodicHookBindings,
            Map<String, BooleanSupplier> inputs,
            Map<String, DoubleSupplier> doubleInputs,
            Map<String, IntSupplier> intInputs,
            Map<String, Supplier<String>> stringInputs,
            Map<String, Supplier<Pose2d>> pose2dInputs,
            Map<String, Supplier<Pose3d>> pose3dInputs,
            Map<String, Supplier<?>> objectInputs,
            Map<String, Boolean> mutableBoolInputDefaults,
            Map<String, Double> mutableDoubleInputDefaults,
            Map<String, Integer> mutableIntInputDefaults,
            Map<String, String> mutableStringInputDefaults,
            Map<String, Pose2d> mutablePose2dInputDefaults,
            Map<String, Pose3d> mutablePose3dInputDefaults,
            Map<String, PidProfile> controlLoopPidProfiles,
            Map<String, BangBangProfile> controlLoopBangBangProfiles,
            Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles,
            List<ControlLoopBinding<T>> controlLoops,
            Set<String> initiallyDisabledControlLoops,
            MechanismSimulationConfig simulationConfig,
            MechanismVisualizationConfig visualizationConfig,
            MechanismSensorSimulationConfig sensorSimulationConfig) {
        this.name = name != null ? name : "";
        this.sourceKey = sourceKey;
        this.configDisabled = configDisabled;
        this.motors = List.copyOf(Objects.requireNonNull(motors, "motors"));
        this.encoderSources = Map.copyOf(Objects.requireNonNull(encoderSources, "encoderSources"));
        this.positionSourceName = positionSourceName;
        this.velocitySourceName = velocitySourceName;
        this.absoluteSourceName = absoluteSourceName;
        this.data = Objects.requireNonNull(data, "data");
        this.periodicHooks = List.copyOf(Objects.requireNonNull(periodicHooks, "periodicHooks"));
        this.periodicHookBindings = List.copyOf(Objects.requireNonNull(periodicHookBindings, "periodicHookBindings"));
        this.inputs = Map.copyOf(Objects.requireNonNull(inputs, "inputs"));
        this.doubleInputs = Map.copyOf(Objects.requireNonNull(doubleInputs, "doubleInputs"));
        this.intInputs = Map.copyOf(Objects.requireNonNull(intInputs, "intInputs"));
        this.stringInputs = Map.copyOf(Objects.requireNonNull(stringInputs, "stringInputs"));
        this.pose2dInputs = Map.copyOf(Objects.requireNonNull(pose2dInputs, "pose2dInputs"));
        this.pose3dInputs = Map.copyOf(Objects.requireNonNull(pose3dInputs, "pose3dInputs"));
        this.objectInputs = Map.copyOf(Objects.requireNonNull(objectInputs, "objectInputs"));
        this.mutableBoolInputDefaults = Map.copyOf(Objects.requireNonNull(mutableBoolInputDefaults, "mutableBoolInputDefaults"));
        this.mutableDoubleInputDefaults = Map.copyOf(Objects.requireNonNull(mutableDoubleInputDefaults, "mutableDoubleInputDefaults"));
        this.mutableIntInputDefaults = Map.copyOf(Objects.requireNonNull(mutableIntInputDefaults, "mutableIntInputDefaults"));
        this.mutableStringInputDefaults = Map.copyOf(Objects.requireNonNull(mutableStringInputDefaults, "mutableStringInputDefaults"));
        this.mutablePose2dInputDefaults = Map.copyOf(Objects.requireNonNull(mutablePose2dInputDefaults, "mutablePose2dInputDefaults"));
        this.mutablePose3dInputDefaults = Map.copyOf(Objects.requireNonNull(mutablePose3dInputDefaults, "mutablePose3dInputDefaults"));
        this.controlLoopPidProfiles = Map.copyOf(Objects.requireNonNull(controlLoopPidProfiles, "controlLoopPidProfiles"));
        this.controlLoopBangBangProfiles = Map.copyOf(Objects.requireNonNull(controlLoopBangBangProfiles, "controlLoopBangBangProfiles"));
        this.controlLoopFeedforwardProfiles = Map.copyOf(Objects.requireNonNull(controlLoopFeedforwardProfiles, "controlLoopFeedforwardProfiles"));
        this.controlLoops = List.copyOf(Objects.requireNonNull(controlLoops, "controlLoops"));
        this.initiallyDisabledControlLoops = Set.copyOf(Objects.requireNonNull(initiallyDisabledControlLoops, "initiallyDisabledControlLoops"));
        this.simulationConfig = simulationConfig;
        this.visualizationConfig = visualizationConfig;
        this.sensorSimulationConfig = sensorSimulationConfig;
    }

    public String name() {
        return name;
    }

    public Object sourceKey() {
        return sourceKey;
    }

    public boolean configDisabled() {
        return configDisabled;
    }

    public List<MotorControllerConfig> motors() {
        return motors;
    }

    public Map<String, MechanismEncoderSource> encoderSources() {
        return encoderSources;
    }

    public String positionSourceName() {
        return positionSourceName;
    }

    public String velocitySourceName() {
        return velocitySourceName;
    }

    public String absoluteSourceName() {
        return absoluteSourceName;
    }

    public MechanismConfigRecord data() {
        return data;
    }

    public List<Consumer<T>> periodicHooks() {
        return periodicHooks;
    }

    public List<PeriodicHookBinding<T>> periodicHookBindings() {
        return periodicHookBindings;
    }

    public Map<String, BooleanSupplier> inputs() {
        return inputs;
    }

    public Map<String, DoubleSupplier> doubleInputs() {
        return doubleInputs;
    }

    public Map<String, IntSupplier> intInputs() {
        return intInputs;
    }

    public Map<String, Supplier<String>> stringInputs() {
        return stringInputs;
    }

    public Map<String, Supplier<Pose2d>> pose2dInputs() {
        return pose2dInputs;
    }

    public Map<String, Supplier<Pose3d>> pose3dInputs() {
        return pose3dInputs;
    }

    public Map<String, Supplier<?>> objectInputs() {
        return objectInputs;
    }

    public Map<String, Boolean> mutableBoolInputDefaults() {
        return mutableBoolInputDefaults;
    }

    public Map<String, Double> mutableDoubleInputDefaults() {
        return mutableDoubleInputDefaults;
    }

    public Map<String, Integer> mutableIntInputDefaults() {
        return mutableIntInputDefaults;
    }

    public Map<String, String> mutableStringInputDefaults() {
        return mutableStringInputDefaults;
    }

    public Map<String, Pose2d> mutablePose2dInputDefaults() {
        return mutablePose2dInputDefaults;
    }

    public Map<String, Pose3d> mutablePose3dInputDefaults() {
        return mutablePose3dInputDefaults;
    }

    public Map<String, PidProfile> controlLoopPidProfiles() {
        return controlLoopPidProfiles;
    }

    public Map<String, BangBangProfile> controlLoopBangBangProfiles() {
        return controlLoopBangBangProfiles;
    }

    public Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles() {
        return controlLoopFeedforwardProfiles;
    }

    public List<ControlLoopBinding<T>> controlLoops() {
        return controlLoops;
    }

    public Set<String> initiallyDisabledControlLoops() {
        return initiallyDisabledControlLoops;
    }

    public MechanismSimulationConfig simulationConfig() {
        return simulationConfig;
    }

    public MechanismVisualizationConfig visualizationConfig() {
        return visualizationConfig;
    }

    public MechanismSensorSimulationConfig sensorSimulationConfig() {
        return sensorSimulationConfig;
    }

}
