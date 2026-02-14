package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.LoopTiming;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.core.diagnostics.BoundedEventLog;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.loop.TimedRunner;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulation;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationModel;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualization;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Mechanism extends SubsystemBase implements RobotSendableSystem, RegisterableMechanism{

    // Retain the builder instance that constructed this mechanism so RobotCore can resolve
    // mechanisms by config identity when desired (Constants-style configs).
    private final MechanismConfig<? extends Mechanism> sourceConfig;

    private final MotorControllerGroup motors;
    private final Encoder encoder;
    private final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;
    private boolean useAbsolute;
    private OutputType outputType = OutputType.PERCENT;
    private final GenericLimitSwitch[] limitSwitches;
    private static final String MOTION_AXIS_ID = "axis";
    private final MotionLimits motionLimits;
    private final TrapezoidProfile.Constraints baseProfiledConstraints;
    private boolean override, emergencyStopped, manualEmergencyStopped, connectivityFaultEmergencyStopped,
            pidEnabled, feedforwardEnabled, setpointIsOutput, customPIDCycle;
    private double setpoint, prevSetpoint, pidOutput, feedforwardOutput, output, nudge, pidPeriod; 
    private RobotMode prevRobotMode = RobotMode.DISABLE, robotMode = RobotMode.DISABLE;
    private final MechanismSimulationModel simulationModel;
    private final double simulationUpdatePeriodSeconds;
    private double lastSimulationTimestampSeconds = Double.NaN;
    private double hardwareUpdatePeriodSeconds = 0.02;
    private double lastHardwareUpdateSeconds = Double.NaN;
    private int hardwareUpdateSlot = 0;
    private double hardwareUpdatePhaseSeconds = 0.0;
    private double firstHardwareRefreshDueSeconds = Double.NaN;
    private static final double HARDWARE_REFRESH_QUANTUM_SECONDS = 0.02;
    private static final AtomicInteger NEXT_HARDWARE_UPDATE_SLOT = new AtomicInteger(0);
    private final boolean encoderOwnedByMotorGroup;
    private final MechanismVisualization visualization;
    private final MechanismSensorSimulation sensorSimulation;
    private boolean boundsEnabled = false;
    private double minBound = Double.NaN;
    private double maxBound = Double.NaN;
    private Pose3d visualizationRootOverride;
    private boolean simEncoderOverride;
    private double simEncoderPosition;
    private double simEncoderVelocity;
    private boolean manualOutputActive;
    private double manualOutput;
    private boolean manualOutputIsVoltage;
    private boolean lastOutputValid;
    private double lastOutput;
    private boolean lastOutputIsVoltage;
    private double networkTablesPeriodSecondsOverride = Double.NaN;
    private RobotNetworkTables.Node lastNetworkTablesNode;
    private RobotNetworkTables lastRobotNetworkTables;
    private boolean networkTablesPublishRequested;
    private double lastNetworkTablesPublishAttemptSeconds = Double.NaN;
    private static final double NETWORKTABLES_PUBLISH_RETRY_PERIOD_SECONDS = 0.5;
    private long lastNetworkTablesConfigRevision = -1;
    private String networkTablesOwnerPath;
    private boolean ntMotorsBuilt;
    private boolean ntEncodersBuilt;
    private boolean ntStatusBuilt;
    private boolean ntSetpointsBuilt;
    private boolean ntOutputsBuilt;
    private boolean ntVisualizationBuilt;
    private boolean ntSimulationBuilt;
    private boolean ntCommandsBuilt;
    private boolean ntConfigBuilt;
    private boolean ntLimitSwitchesBuilt;
    private boolean ntSysIdBuilt;
    private boolean ntControllersBuilt;
    private boolean ntHintBuilt;
    private boolean cachedEmergencyStopped;
    private boolean cachedOverride;
    private boolean cachedAtSetpoint;
    private double cachedSetpoint;
    private double cachedNudge;
    private double cachedOutput;
    private double cachedPidOutput;
    private double cachedFeedforwardOutput;
    private boolean cachedHasSimulation;
    private double cachedSimulationUpdatePeriodSeconds;
    private boolean cachedUseVoltage;
    private boolean cachedUseAbsolute;
    private boolean cachedSetpointAsOutput;
    private double cachedPidPeriod;
    private boolean cachedFeedforwardEnabled;
    private boolean cachedPidEnabled;
    private double cachedSysIdRampRate;
    private double cachedSysIdStepVoltage;
    private double cachedSysIdTimeoutSeconds;
    private boolean cachedSysIdActive;
    private double cachedHardwareUpdatePeriodSeconds;
    private double cachedHardwareUpdatePhaseSeconds;
    private double cachedHardwareLastRefreshAgeSeconds = Double.NaN;
    private double cachedHardwareNextRefreshInSeconds = Double.NaN;
    private int cachedHardwareUpdateSlot;
    private String cachedFaultReason = "";
    private String cachedLastFaultReason = "";
    private RobotCore<?> robotCore;
    private SysIdRoutine sysIdRoutine;
    private double sysIdRampRateVoltsPerSecond = 1.0;
    private double sysIdStepVoltage = 7.0;
    private double sysIdTimeoutSeconds = 10.0;
    private double sysIdLastVoltage = 0.0;
    private boolean sysIdActive = false;
    private boolean sysIdPreviousOverride = false;
    private final List<Consumer<?>> periodicHooks;
    private final List<TimedRunner<Consumer<Mechanism>>> timedPeriodicHooks;
    private final List<TimedRunner<MechanismConfig.MechanismControlLoop<Mechanism>>> controlLoops;
    private final Map<String, TimedRunner<MechanismConfig.MechanismControlLoop<Mechanism>>> controlLoopsByName;
    private final Set<String> disabledControlLoops;
    private final Map<String, BooleanSupplier> controlLoopInputs;
    private final Map<String, DoubleSupplier> controlLoopDoubleInputs;
    private final Map<String, java.util.function.IntSupplier> controlLoopIntInputs;
    private final Map<String, java.util.function.Supplier<String>> controlLoopStringInputs;
    private final Map<String, java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d>> controlLoopPose2dInputs;
    private final Map<String, java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d>> controlLoopPose3dInputs;
    private final Map<String, Supplier<?>> controlLoopObjectInputs;
    // Mutable inputs (declared with defaults in MechanismConfig.inputs(...)).
    private final Map<String, Boolean> mutableBoolInputDefaults;
    private final Map<String, Boolean> mutableBoolInputs;
    private final Map<String, Double> mutableDoubleInputDefaults;
    private final Map<String, Double> mutableDoubleInputs;
    private final Map<String, Integer> mutableIntInputDefaults;
    private final Map<String, Integer> mutableIntInputs;
    private final Map<String, String> mutableStringInputDefaults;
    private final Map<String, String> mutableStringInputs;
    private final Map<String, edu.wpi.first.math.geometry.Pose2d> mutablePose2dInputDefaults;
    private final Map<String, edu.wpi.first.math.geometry.Pose2d> mutablePose2dInputs;
    private final Map<String, edu.wpi.first.math.geometry.Pose3d> mutablePose3dInputDefaults;
    private final Map<String, edu.wpi.first.math.geometry.Pose3d> mutablePose3dInputs;
    private final Map<String, PIDController> controlLoopPids;
    private final Map<String, ProfiledPIDController> controlLoopProfiledPids;
    private final Map<String, OutputType> controlLoopPidOutputTypes;
    private final Map<String, MechanismConfig.BangBangProfile> controlLoopBangBangs;
    private final Map<String, SimpleMotorFeedforward> controlLoopFeedforwards;
    private final Map<String, OutputType> controlLoopFeedforwardOutputTypes;
    private final MechanismControlContextImpl controlContext = new MechanismControlContextImpl();
    private  boolean shouldCustomEncoder = false;
    private  DoubleSupplier customEncoderPos;
    private boolean networkTablesEnabled;
    private double lastNetworkTablesCacheUpdateSeconds = Double.NaN;
    private double lastEmergencyStopLogSeconds = Double.NaN;
    private String lastEmergencyStopReason = "";
    private String connectivityFaultReason = "";
    private String lastFaultReason = "";
    private final BoundedEventLog<DiagnosticsChannel.Event> diagnosticsLog =
            new BoundedEventLog<>(DIAGNOSTIC_LOG_CAPACITY);
    private final DiagnosticsView diagnosticsView = new DiagnosticsView();
    private static final double EMERGENCY_STOP_LOG_PERIOD_SECONDS = 1.0;
    private static final int DIAGNOSTIC_LOG_CAPACITY = 256;

    private enum RobotMode {
        TELE,
        AUTO,
        DISABLE,
        TEST,
    }

    public record MechanismLogEvent(
            long sequence,
            double timestampSeconds,
            String systemKey,
            String level,
            String category,
            String message,
            String line) {
    }

    private String diagnosticsSystemKey() {
        String mechanismName = getName();
        if (mechanismName == null || mechanismName.isBlank()) {
            mechanismName = getClass().getSimpleName();
        }
        return "mechanisms/" + mechanismName;
    }

    public Mechanism(MechanismConfig<? extends Mechanism> config){
        this.sourceConfig = config;
        this.motors =
                MotorControllerGroup.fromConfigs(config.data().motors().toArray(MotorControllerConfig[]::new));
        this.encoder = resolveEncoder(config.data().encoder(), this.motors);
        this.useAbsolute = config.data().useAbsolute();
        OutputType configOutputType = config.data().outputType();
        if (configOutputType == null) {
            this.outputType = config.data().useVoltage() ? OutputType.VOLTAGE : OutputType.PERCENT;
        } else {
            this.outputType = configOutputType;
        }
        this.pidController = config.data().pidController();
        this.profiledPIDController = config.data().profiledPIDController();
        this.override = false;
        this.limitSwitches =
                config.data().limitSwitches().stream().map(GenericLimitSwitch::fromConfig).toArray(GenericLimitSwitch[]::new);
        this.setpointIsOutput = config.data().useSetpointAsOutput();
        this.pidPeriod = config.data().pidPeriod();
        this.motionLimits = new MotionLimits();
        this.hardwareUpdatePeriodSeconds = sanitizeHardwareUpdatePeriod(config.data().hardwareUpdatePeriodSeconds());
        this.hardwareUpdateSlot = NEXT_HARDWARE_UPDATE_SLOT.getAndIncrement();
        recomputeHardwareRefreshPhase();
        this.encoderOwnedByMotorGroup = isEncoderOwnedByMotorGroup(this.encoder, this.motors);

        if (pidController != null && config.data().pidContinous()) {
            pidController.enableContinuousInput(config.data().continousMin(), config.data().continousMax());
        }
        if (profiledPIDController != null && config.data().pidContinous()) {
            profiledPIDController.enableContinuousInput(config.data().continousMin(), config.data().continousMax());
        }

        if(profiledPIDController != null){
            profiledPIDController.reset(getPosition(), getVelocity());
        }
        this.baseProfiledConstraints =
                profiledPIDController != null ? profiledPIDController.getConstraints() : null;

        pidEnabled = pidController != null || profiledPIDController != null;

        MechanismSimulationModel model = null;
        double updatePeriod = 0.02;
        if (config.simulationConfig() != null && RobotBase.isSimulation()) {
            model = config.simulationConfig().createSimulation(this);
            updatePeriod = config.simulationConfig().updatePeriodSeconds();
        }
        this.simulationModel = model;
        this.simulationUpdatePeriodSeconds = updatePeriod;
        if (simulationModel != null) {
            simulationModel.reset();
            lastSimulationTimestampSeconds = Timer.getFPGATimestamp();
        }
        MechanismVisualizationConfig resolvedVisualizationConfig =
                config.visualizationConfig() != null ? config.visualizationConfig() : MechanismVisualizationDefaults.forMechanism(this);
        this.visualization = resolvedVisualizationConfig != null ? new MechanismVisualization(resolvedVisualizationConfig) : null;
        if (RobotBase.isSimulation()) {
            if (config.sensorSimulationConfig() != null) {
                this.sensorSimulation = MechanismSensorSimulation.fromConfig(this, config.sensorSimulationConfig());
            } else {
                this.sensorSimulation = MechanismSensorSimulation.forLimitSwitches(this);
            }
        } else {
        this.sensorSimulation = MechanismSensorSimulation.empty();
        }
        this.periodicHooks = new ArrayList<>();
        this.timedPeriodicHooks = new ArrayList<>();
        MechanismConfigRecord cfg = config.data();
        setBounds(cfg.minBound(), cfg.maxBound());
        setMotionLimits(cfg.motionLimits());
        this.periodicHooks.addAll(config.periodicHooks());
        if (config.periodicHookBindings() != null) {
            for (MechanismConfig.PeriodicHookBinding<?> binding : config.periodicHookBindings()) {
                registerPeriodicHook(binding);
            }
        }
        this.shouldCustomEncoder = config.usesCustomEncoder();
        this.customEncoderPos = config.customEncoderPositionSupplier();
        this.controlLoopInputs = new HashMap<>(config.inputs());
        this.controlLoopDoubleInputs = new HashMap<>(config.doubleInputs());
        this.controlLoopIntInputs = new HashMap<>(config.intInputs());
        this.controlLoopStringInputs = new HashMap<>(config.stringInputs());
        this.controlLoopPose2dInputs = new HashMap<>(config.pose2dInputs());
        this.controlLoopPose3dInputs = new HashMap<>(config.pose3dInputs());
        this.controlLoopObjectInputs = new HashMap<>(config.objectInputs());
        this.mutableBoolInputDefaults = new HashMap<>(config.mutableBoolInputDefaults());
        this.mutableBoolInputs = new HashMap<>(this.mutableBoolInputDefaults);
        this.mutableDoubleInputDefaults = new HashMap<>(config.mutableDoubleInputDefaults());
        this.mutableDoubleInputs = new HashMap<>(this.mutableDoubleInputDefaults);
        this.mutableIntInputDefaults = new HashMap<>(config.mutableIntInputDefaults());
        this.mutableIntInputs = new HashMap<>(this.mutableIntInputDefaults);
        this.mutableStringInputDefaults = new HashMap<>(config.mutableStringInputDefaults());
        this.mutableStringInputs = new HashMap<>(this.mutableStringInputDefaults);
        this.mutablePose2dInputDefaults = new HashMap<>(config.mutablePose2dInputDefaults());
        this.mutablePose2dInputs = new HashMap<>(this.mutablePose2dInputDefaults);
        this.mutablePose3dInputDefaults = new HashMap<>(config.mutablePose3dInputDefaults());
        this.mutablePose3dInputs = new HashMap<>(this.mutablePose3dInputDefaults);
        this.controlLoopPids = new HashMap<>();
        this.controlLoopProfiledPids = new HashMap<>();
        this.controlLoopPidOutputTypes = new HashMap<>();
        this.controlLoopBangBangs = new HashMap<>();
        this.controlLoopFeedforwards = new HashMap<>();
        this.controlLoopFeedforwardOutputTypes = new HashMap<>();
        this.controlLoops = new ArrayList<>();
        this.controlLoopsByName = new HashMap<>();
        this.disabledControlLoops = new HashSet<>();
        if (config.controlLoopPidProfiles() != null) {
            for (Map.Entry<String, MechanismConfig.PidProfile> entry : config.controlLoopPidProfiles().entrySet()) {
                String name = entry.getKey();
                MechanismConfig.PidProfile profile = entry.getValue();
                if (name == null || name.isBlank() || profile == null) {
                    continue;
                }
                OutputType profileOutput = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
                if (profileOutput != OutputType.PERCENT && profileOutput != OutputType.VOLTAGE) {
                    throw new IllegalStateException("PID profile '" + name + "' output type must be PERCENT or VOLTAGE");
                }
                boolean hasProfiledVelocity = Double.isFinite(profile.maxVelocity()) && profile.maxVelocity() > 0.0;
                boolean hasProfiledAcceleration =
                        Double.isFinite(profile.maxAcceleration()) && profile.maxAcceleration() > 0.0;
                if (hasProfiledVelocity != hasProfiledAcceleration) {
                    throw new IllegalStateException(
                            "PID profile '" + name + "' must define both maxVelocity and maxAcceleration for profiled control");
                }
                if (hasProfiledVelocity) {
                    ProfiledPIDController pid = new ProfiledPIDController(
                            profile.kP(),
                            profile.kI(),
                            profile.kD(),
                            new TrapezoidProfile.Constraints(profile.maxVelocity(), profile.maxAcceleration()));
                    if (Double.isFinite(profile.iZone()) && profile.iZone() > 0.0) {
                        pid.setIZone(profile.iZone());
                    }
                    if (Double.isFinite(profile.tolerance()) && profile.tolerance() > 0.0) {
                        pid.setTolerance(profile.tolerance());
                    }
                    controlLoopProfiledPids.put(name, pid);
                } else {
                    PIDController pid = new PIDController(profile.kP(), profile.kI(), profile.kD());
                    if (Double.isFinite(profile.iZone()) && profile.iZone() > 0.0) {
                        pid.setIZone(profile.iZone());
                    }
                    if (Double.isFinite(profile.tolerance()) && profile.tolerance() > 0.0) {
                        pid.setTolerance(profile.tolerance());
                    }
                    controlLoopPids.put(name, pid);
                }
                controlLoopPidOutputTypes.put(name, profileOutput);
            }
        }
        if (config.controlLoopBangBangProfiles() != null) {
            for (Map.Entry<String, MechanismConfig.BangBangProfile> entry : config.controlLoopBangBangProfiles().entrySet()) {
                String name = entry.getKey();
                MechanismConfig.BangBangProfile profile = entry.getValue();
                if (name == null || name.isBlank() || profile == null) {
                    continue;
                }
                OutputType profileOutput = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
                if (profileOutput != OutputType.PERCENT && profileOutput != OutputType.VOLTAGE) {
                    throw new IllegalStateException("Bang-bang profile '" + name + "' output type must be PERCENT or VOLTAGE");
                }
                controlLoopBangBangs.put(
                        name,
                        new MechanismConfig.BangBangProfile(
                                profileOutput,
                                sanitizeBangBangLevel(profile.highOutput()),
                                sanitizeBangBangLevel(profile.lowOutput()),
                                sanitizeBangBangTolerance(profile.tolerance())));
            }
        }
        if (config.controlLoopFeedforwardProfiles() != null) {
            for (Map.Entry<String, MechanismConfig.FeedforwardProfile> entry : config.controlLoopFeedforwardProfiles().entrySet()) {
                String name = entry.getKey();
                MechanismConfig.FeedforwardProfile profile = entry.getValue();
                SimpleMotorFeedforward ff = profile != null ? profile.feedforward() : null;
                if (name == null || name.isBlank() || ff == null) {
                    continue;
                }
                OutputType profileOutput = profile.outputType() != null ? profile.outputType() : OutputType.VOLTAGE;
                if (profileOutput != OutputType.VOLTAGE) {
                    throw new IllegalStateException("Feedforward profile '" + name + "' output type must be VOLTAGE");
                }
                controlLoopFeedforwards.put(name, new SimpleMotorFeedforward(ff.getKs(), ff.getKv(), ff.getKa()));
                controlLoopFeedforwardOutputTypes.put(name, profileOutput);
            }
        }
        for (MechanismConfig.ControlLoopBinding<?> binding : config.controlLoops()) {
            registerControlLoop(binding);
        }

    }

    public MechanismConfig<? extends Mechanism> getSourceConfig() {
        return sourceConfig;
    }

    public void runLifecycleHooks(RobotCoreHooks.Phase phase) {
        if (sourceConfig == null || phase == null) {
            return;
        }
        sourceConfig.runPhaseHooks(this, phase);
    }

    private static Encoder resolveEncoder(EncoderConfig config, MotorControllerGroup motors) {
        if (config == null) {
            return null;
        }
        Encoder integrated = resolveIntegratedEncoder(config, motors);
        return integrated != null ? integrated : HardwareFactories.encoder(config);
    }

    private static boolean isEncoderOwnedByMotorGroup(Encoder encoder, MotorControllerGroup motors) {
        if (encoder == null || motors == null) {
            return false;
        }
        for (MotorController controller : motors.getControllers()) {
            if (controller == null) {
                continue;
            }
            Encoder motorEncoder = controller.getEncoder();
            if (motorEncoder == encoder) {
                return true;
            }
        }
        return false;
    }

    private static double sanitizeHardwareUpdatePeriod(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            return 0.02;
        }
        return periodSeconds;
    }

    private static int hardwareSlotsForPeriod(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= HARDWARE_REFRESH_QUANTUM_SECONDS) {
            return 1;
        }
        return Math.max(1, (int) Math.floor(periodSeconds / HARDWARE_REFRESH_QUANTUM_SECONDS));
    }

    private void recomputeHardwareRefreshPhase() {
        int slots = hardwareSlotsForPeriod(hardwareUpdatePeriodSeconds);
        int slotWithinPeriod = Math.floorMod(hardwareUpdateSlot, slots);
        hardwareUpdatePhaseSeconds = (hardwareUpdatePeriodSeconds / slots) * slotWithinPeriod;
        firstHardwareRefreshDueSeconds = Double.NaN;
    }

    private double resolveNextHardwareRefreshDueSeconds(double nowSeconds) {
        if (simulationModel != null) {
            return Double.NaN;
        }
        if (Double.isFinite(lastHardwareUpdateSeconds)) {
            return lastHardwareUpdateSeconds + hardwareUpdatePeriodSeconds;
        }
        if (Double.isFinite(firstHardwareRefreshDueSeconds)) {
            return firstHardwareRefreshDueSeconds;
        }
        if (Double.isFinite(nowSeconds)) {
            return nowSeconds + hardwareUpdatePhaseSeconds;
        }
        return Double.NaN;
    }

    private static Encoder resolveIntegratedEncoder(EncoderConfig config, MotorControllerGroup motors) {
        if (motors == null || config.type() == null) {
            return null;
        }
        String key = config.type().getKey();
        if (!isIntegratedEncoderKey(key)) {
            return null;
        }
        int encoderId = config.id();
        MotorController matchingMotor = null;
        for (MotorController motor : motors.getControllers()) {
            if (motor.getId() == encoderId) {
                matchingMotor = motor;
                break;
            }
        }
        if (matchingMotor == null) {
            throw new IllegalStateException(
                    "Integrated encoder requested for motor ID " + encoderId + ", but no matching motor was configured.");
        }
        Encoder motorEncoder = matchingMotor.getEncoder();
        if (motorEncoder == null) {
            throw new IllegalStateException(
                    "Motor controller '" + matchingMotor.getName() + "' has no integrated encoder.");
        }
        return motorEncoder;
    }

    private static boolean isIntegratedEncoderKey(String key) {
        return "ctre:talonfx-integrated".equals(key)
                || "rev:sparkmax".equals(key)
                || "rev:sparkflex".equals(key);
    }

    public Mechanism(MotorControllerGroup motors, Encoder encoder, PIDController pidController,
            ProfiledPIDController profiledPIDController, boolean useAbsolute, boolean useVoltage,
            GenericLimitSwitch[] limitSwitches, boolean useSetpointAsOutput, double pidPeriod) {
        this(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, limitSwitches, useSetpointAsOutput, pidPeriod, null, null, null);
    }

    @Override
    public java.util.List<Mechanism> flattenForRegistration() {
        return java.util.List.of(this);
    }

    public Mechanism(MotorControllerGroup motors, Encoder encoder, PIDController pidController,
            ProfiledPIDController profiledPIDController, boolean useAbsolute, boolean useVoltage,
            GenericLimitSwitch[] limitSwitches, boolean useSetpointAsOutput, double pidPeriod,
            MechanismSimulationConfig simulationConfig,
            MechanismVisualizationConfig visualizationConfig,
            MechanismSensorSimulationConfig sensorSimulationConfig) {
        this.sourceConfig = null;
        this.motors = motors;
        this.encoder = encoder;
        this.pidController = pidController;
        this.profiledPIDController = profiledPIDController;
        this.useAbsolute = useAbsolute;
        this.outputType = useVoltage ? OutputType.VOLTAGE : OutputType.PERCENT;
        this.override = false;
        this.limitSwitches = limitSwitches;
        this.setpointIsOutput = useSetpointAsOutput;
        this.pidPeriod = pidPeriod;
        this.motionLimits = new MotionLimits();
        this.hardwareUpdatePeriodSeconds = 0.02;
        this.hardwareUpdateSlot = NEXT_HARDWARE_UPDATE_SLOT.getAndIncrement();
        recomputeHardwareRefreshPhase();
        this.encoderOwnedByMotorGroup = isEncoderOwnedByMotorGroup(this.encoder, this.motors);

        if(profiledPIDController != null){
            profiledPIDController.reset(getPosition(), getVelocity());
        }
        this.baseProfiledConstraints =
                profiledPIDController != null ? profiledPIDController.getConstraints() : null;

        pidEnabled = pidController != null || profiledPIDController != null;

        MechanismSimulationModel model = null;
        double updatePeriod = 0.02;
        if (simulationConfig != null && RobotBase.isSimulation()) {
            model = simulationConfig.createSimulation(this);
            updatePeriod = simulationConfig.updatePeriodSeconds();
        }
        this.simulationModel = model;
        this.simulationUpdatePeriodSeconds = updatePeriod;
        if (simulationModel != null) {
            simulationModel.reset();
            lastSimulationTimestampSeconds = Timer.getFPGATimestamp();
        }
        MechanismVisualizationConfig resolvedVisualizationConfig =
                visualizationConfig != null ? visualizationConfig : MechanismVisualizationDefaults.forMechanism(this);
        this.visualization = resolvedVisualizationConfig != null ? new MechanismVisualization(resolvedVisualizationConfig) : null;
        if (RobotBase.isSimulation()) {
            if (sensorSimulationConfig != null) {
                this.sensorSimulation = MechanismSensorSimulation.fromConfig(this, sensorSimulationConfig);
            } else {
                this.sensorSimulation = MechanismSensorSimulation.forLimitSwitches(this);
            }
        } else {
            this.sensorSimulation = MechanismSensorSimulation.empty();
        }
        this.periodicHooks = new ArrayList<>();
        this.timedPeriodicHooks = new ArrayList<>();
        this.controlLoops = new ArrayList<>();
        this.controlLoopsByName = new HashMap<>();
        this.disabledControlLoops = new HashSet<>();
        this.controlLoopInputs = new HashMap<>();
        this.controlLoopDoubleInputs = new HashMap<>();
        this.controlLoopIntInputs = new HashMap<>();
        this.controlLoopStringInputs = new HashMap<>();
        this.controlLoopPose2dInputs = new HashMap<>();
        this.controlLoopPose3dInputs = new HashMap<>();
        this.controlLoopObjectInputs = new HashMap<>();
        this.mutableBoolInputDefaults = new HashMap<>();
        this.mutableBoolInputs = new HashMap<>();
        this.mutableDoubleInputDefaults = new HashMap<>();
        this.mutableDoubleInputs = new HashMap<>();
        this.mutableIntInputDefaults = new HashMap<>();
        this.mutableIntInputs = new HashMap<>();
        this.mutableStringInputDefaults = new HashMap<>();
        this.mutableStringInputs = new HashMap<>();
        this.mutablePose2dInputDefaults = new HashMap<>();
        this.mutablePose2dInputs = new HashMap<>();
        this.mutablePose3dInputDefaults = new HashMap<>();
        this.mutablePose3dInputs = new HashMap<>();
        this.controlLoopPids = new HashMap<>();
        this.controlLoopProfiledPids = new HashMap<>();
        this.controlLoopPidOutputTypes = new HashMap<>();
        this.controlLoopBangBangs = new HashMap<>();
        this.controlLoopFeedforwards = new HashMap<>();
        this.controlLoopFeedforwardOutputTypes = new HashMap<>();
    }

    // Package-private helpers for state-hook contexts (StatefulMechanismCore).
    boolean hasMutableBoolValKey(String key) { return key != null && mutableBoolInputs.containsKey(key); }
    boolean mutableBoolVal(String key) { return mutableBoolInputs.get(key); }
    boolean hasMutableDblValKey(String key) { return key != null && mutableDoubleInputs.containsKey(key); }
    double mutableDblVal(String key) { return mutableDoubleInputs.get(key); }
    boolean hasMutableIntValKey(String key) { return key != null && mutableIntInputs.containsKey(key); }
    int mutableIntVal(String key) { return mutableIntInputs.get(key); }
    boolean hasMutableStrValKey(String key) { return key != null && mutableStringInputs.containsKey(key); }
    String mutableStrVal(String key) { return mutableStringInputs.get(key); }
    boolean hasMutablePose2dValKey(String key) { return key != null && mutablePose2dInputs.containsKey(key); }
    edu.wpi.first.math.geometry.Pose2d mutablePose2dVal(String key) { return mutablePose2dInputs.get(key); }
    boolean hasMutablePose3dValKey(String key) { return key != null && mutablePose3dInputs.containsKey(key); }
    edu.wpi.first.math.geometry.Pose3d mutablePose3dVal(String key) { return mutablePose3dInputs.get(key); }

    private void requireMutableKey(String key, Map<String, ?> defaults) {
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("input key cannot be null/blank");
        }
        if (!defaults.containsKey(key)) {
            throw new IllegalArgumentException("Mutable input key not declared in config: " + key);
        }
    }

    public Mechanism input(Consumer<InputSection> section) {
        if (section != null) {
            section.accept(new InputSection(this));
        }
        return this;
    }

    public InputSection input() {
        return new InputSection(this);
    }

    public Mechanism motors(Consumer<MotorsSection> section) {
        if (section != null) {
            section.accept(new MotorsSection(this));
        }
        return this;
    }

    public MotorsSection motors() {
        return new MotorsSection(this);
    }

    public Mechanism encoder(Consumer<EncoderSection> section) {
        if (section != null) {
            section.accept(new EncoderSection(this));
        }
        return this;
    }

    public EncoderSection encoder() {
        return new EncoderSection(this);
    }

    public Mechanism control(Consumer<ControlSection> section) {
        if (section != null) {
            section.accept(new ControlSection(this));
        }
        return this;
    }

    public ControlSection control() {
        return new ControlSection(this);
    }

    public Mechanism loops(Consumer<LoopsSection> section) {
        if (section != null) {
            section.accept(new LoopsSection(this));
        }
        return this;
    }

    public LoopsSection loops() {
        return new LoopsSection(this);
    }

    public Mechanism sysId(Consumer<SysIdSection> section) {
        if (section != null) {
            section.accept(new SysIdSection(this));
        }
        return this;
    }

    public SysIdSection sysId() {
        return new SysIdSection(this);
    }

    public Mechanism sim(Consumer<SimulationSection> section) {
        if (section != null) {
            section.accept(new SimulationSection(this));
        }
        return this;
    }

    public SimulationSection sim() {
        return new SimulationSection(this);
    }

    public Mechanism networkTables(Consumer<NetworkTablesSection> section) {
        if (section != null) {
            section.accept(new NetworkTablesSection(this));
        }
        return this;
    }

    public NetworkTablesSection networkTables() {
        return new NetworkTablesSection(this);
    }

    public Mechanism visualization(Consumer<VisualizationSection> section) {
        if (section != null) {
            section.accept(new VisualizationSection(this));
        }
        return this;
    }

    public VisualizationSection visualization() {
        return new VisualizationSection(this);
    }

    public Mechanism setpoint(double value) {
        control().setpoint(value);
        return this;
    }

    public Mechanism output(double value) {
        control().output(value);
        return this;
    }

    public Mechanism speed(double value) {
        control().speed(value);
        return this;
    }

    public Mechanism voltage(double value) {
        control().voltage(value);
        return this;
    }

    public Mechanism manualOverride(boolean enabled) {
        control().manualOverride(enabled);
        return this;
    }

    public boolean manualOverride() {
        return override;
    }

    public double position() {
        return readPosition(false);
    }

    public Rotation2d rotation2d() {
        return readRotation2d(false);
    }

    public double velocity() {
        return readVelocity(false);
    }

    public double setpoint() {
        return setpoint;
    }

    public double output() {
        return readOutput();
    }

    public double controllerSetpointPosition() {
        return getControllerSetpointPosition();
    }

    public double controllerSetpointVelocity() {
        return getControllerSetpointVelocity();
    }

    public double pidOutput() {
        return getPidOutput();
    }

    public double feedforwardOutput() {
        return getFeedforwardOutput();
    }

    public OutputType outputType() {
        return outputType;
    }

    public boolean emergencyStopped() {
        return emergencyStopped;
    }

    public boolean useAbsolute() {
        return useAbsolute;
    }

    public boolean useVoltage() {
        return outputType == OutputType.VOLTAGE;
    }

    public boolean pidEnabled() {
        return pidEnabled;
    }

    public boolean feedforwardEnabled() {
        return feedforwardEnabled;
    }

    public double nudge() {
        return nudge;
    }

    public double pidPeriodSeconds() {
        return pidPeriod;
    }

    public double networkTablesPeriodSeconds() {
        return readNetworkTablesPeriodSeconds();
    }

    public double simulationUpdatePeriodSeconds() {
        return getSimulationUpdatePeriodSeconds();
    }

    public double hardwareUpdatePeriodSeconds() {
        return hardwareUpdatePeriodSeconds;
    }

    public boolean setpointAsOutput() {
        return setpointIsOutput;
    }

    public boolean customPidCycle() {
        return customPIDCycle;
    }

    public boolean outputIsVoltage() {
        return isOutputVoltage();
    }

    public double minBound() {
        return minBound;
    }

    public double maxBound() {
        return maxBound;
    }

    public Enum<?> activeState() {
        return getActiveState();
    }

    public GenericLimitSwitch[] limitSwitches() {
        return limitSwitches.clone();
    }

    public static final class InputSection {
        private final Mechanism owner;

        private InputSection(Mechanism owner) {
            this.owner = owner;
        }

        public InputSection bool(String key, boolean value) {
            owner.setBoolVal(key, value);
            return this;
        }

        public InputSection resetBool(String key) {
            owner.resetBoolVal(key);
            return this;
        }

        public InputSection dbl(String key, double value) {
            owner.setDoubleVal(key, value);
            return this;
        }

        public InputSection resetDouble(String key) {
            owner.resetDoubleVal(key);
            return this;
        }

        public InputSection integer(String key, int value) {
            owner.setIntVal(key, value);
            return this;
        }

        public InputSection resetInt(String key) {
            owner.resetIntVal(key);
            return this;
        }

        public InputSection string(String key, String value) {
            owner.setStringVal(key, value);
            return this;
        }

        public InputSection resetString(String key) {
            owner.resetStringVal(key);
            return this;
        }

        public InputSection pose2d(String key, edu.wpi.first.math.geometry.Pose2d value) {
            owner.setPose2dVal(key, value);
            return this;
        }

        public InputSection resetPose2d(String key) {
            owner.resetPose2dVal(key);
            return this;
        }

        public InputSection pose3d(String key, edu.wpi.first.math.geometry.Pose3d value) {
            owner.setPose3dVal(key, value);
            return this;
        }

        public InputSection resetPose3d(String key) {
            owner.resetPose3dVal(key);
            return this;
        }

        public boolean hasBool(String key) {
            return owner.hasMutableBoolValKey(key);
        }

        public boolean hasDouble(String key) {
            return owner.hasMutableDblValKey(key);
        }

        public boolean hasInt(String key) {
            return owner.hasMutableIntValKey(key);
        }

        public boolean hasString(String key) {
            return owner.hasMutableStrValKey(key);
        }

        public boolean hasPose2d(String key) {
            return owner.hasMutablePose2dValKey(key);
        }

        public boolean hasPose3d(String key) {
            return owner.hasMutablePose3dValKey(key);
        }

        public boolean bool(String key) {
            if (!hasBool(key)) {
                throw new IllegalArgumentException("Mutable bool input key not declared in config: " + key);
            }
            return owner.mutableBoolVal(key);
        }

        public double dbl(String key) {
            if (!hasDouble(key)) {
                throw new IllegalArgumentException("Mutable double input key not declared in config: " + key);
            }
            return owner.mutableDblVal(key);
        }

        public int integer(String key) {
            if (!hasInt(key)) {
                throw new IllegalArgumentException("Mutable int input key not declared in config: " + key);
            }
            return owner.mutableIntVal(key);
        }

        public String string(String key) {
            if (!hasString(key)) {
                throw new IllegalArgumentException("Mutable string input key not declared in config: " + key);
            }
            return owner.mutableStrVal(key);
        }

        public edu.wpi.first.math.geometry.Pose2d pose2d(String key) {
            if (!hasPose2d(key)) {
                throw new IllegalArgumentException("Mutable Pose2d input key not declared in config: " + key);
            }
            return owner.mutablePose2dVal(key);
        }

        public edu.wpi.first.math.geometry.Pose3d pose3d(String key) {
            if (!hasPose3d(key)) {
                throw new IllegalArgumentException("Mutable Pose3d input key not declared in config: " + key);
            }
            return owner.mutablePose3dVal(key);
        }
    }

    public static final class MotorsSection {
        private final Mechanism owner;

        private MotorsSection(Mechanism owner) {
            this.owner = owner;
        }

        public MotorsSection speed(double value) {
            owner.motors.setSpeed(value);
            return this;
        }

        public MotorsSection voltage(double value) {
            owner.motors.setVoltage(value);
            return this;
        }

        public MotorsSection position(double value) {
            owner.motors.setPosition(value);
            return this;
        }

        public MotorsSection velocity(double value) {
            owner.motors.setVelocity(value);
            return this;
        }

        public MotorsSection stop() {
            owner.motors.stopMotors();
            return this;
        }

        public MotorsSection currentLimit(double amps) {
            owner.motors.setCurrentLimit(amps);
            return this;
        }

        public MotorsSection neutralMode(MotorNeutralMode mode) {
            owner.motors.setNeutralMode(mode);
            return this;
        }

        public MotorsSection output(Consumer<MotorControllerGroup.OutputSection> section) {
            if (section != null) {
                owner.motors.output(section);
            }
            return this;
        }

        public MotorsSection config(Consumer<MotorControllerGroup.ConfigSection> section) {
            if (section != null) {
                owner.motors.config(section);
            }
            return this;
        }

        public MotorControllerGroup.OutputSection output() {
            return owner.motors.output();
        }

        public MotorControllerGroup.ConfigSection config() {
            return owner.motors.config();
        }

        public MotorsSection motionLimits(MotionLimits.AxisLimits limits) {
            owner.setMotionLimits(limits);
            return this;
        }

        public MotorsSection motionLimits(double maxVelocity, double maxAcceleration) {
            owner.setMotionLimits(maxVelocity, maxAcceleration);
            return this;
        }

        public MotorsSection motionLimitsProvider(MotionLimits.AxisLimitsProvider provider) {
            owner.registerMotionLimitsProvider(provider);
            return this;
        }

        public MotorsSection hardwareUpdatePeriodSeconds(double seconds) {
            owner.setHardwareUpdatePeriodSeconds(seconds);
            return this;
        }

        public MotorsSection hardwareUpdatePeriodMs(double ms) {
            owner.setHardwareUpdatePeriodMs(ms);
            return this;
        }

        public MotorControllerGroup device() {
            return owner.motors;
        }

        public MotorController[] controllers() {
            return owner.motors.getControllers();
        }

        public ca.frc6390.athena.hardware.encoder.EncoderGroup encoderGroup() {
            return owner.motors.getEncoderGroup();
        }

        public boolean allConnected() {
            return owner.motors.allMotorsConnected();
        }

        public double averageTemperatureCelsius() {
            return owner.motors.getAverageTemperatureCelsius();
        }

        public MotorsStateView state() {
            return new MotorsStateView(owner, false);
        }
    }

    public static final class MotorsStateView {
        private final Mechanism owner;
        private final boolean poll;

        private MotorsStateView(Mechanism owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public MotorsStateView poll() {
            return new MotorsStateView(owner, true);
        }

        public boolean allConnected() {
            return owner.motors.allMotorsConnected();
        }

        public double averageTemperatureCelsius() {
            return owner.motors.getAverageTemperatureCelsius();
        }

        public MotionLimits.AxisLimits motionLimits() {
            return owner.resolveMotionLimits();
        }

        public double hardwareUpdatePeriodSeconds() {
            return owner.hardwareUpdatePeriodSeconds;
        }

        public double maxFreeSpeedMetersPerSecond() {
            return owner.calculateMaxFreeSpeed();
        }

        public GenericLimitSwitch[] limitSwitches() {
            return owner.limitSwitches.clone();
        }
    }

    public static final class EncoderSection {
        private final Mechanism owner;

        private EncoderSection(Mechanism owner) {
            this.owner = owner;
        }

        public EncoderSection useAbsolute(boolean enabled) {
            owner.useAbsolute = enabled;
            return this;
        }

        public boolean useAbsolute() {
            return owner.useAbsolute;
        }

        public EncoderSection position(double value) {
            owner.writeEncoderPosition(value);
            return this;
        }

        public EncoderSection simState(double position, double velocity) {
            owner.setSimulatedEncoderState(position, velocity);
            return this;
        }

        public EncoderSection clearSimState() {
            owner.clearSimulatedEncoderState();
            return this;
        }

        public EncoderSection config(Consumer<Encoder.RuntimeSection> section) {
            if (owner.encoder != null && section != null) {
                owner.encoder.config(section);
            }
            return this;
        }

        public EncoderSection sim(Consumer<Encoder.SimulationSection> section) {
            if (owner.encoder != null && section != null) {
                owner.encoder.sim(section);
            }
            return this;
        }

        public double position() {
            return owner.readPosition(false);
        }

        public double position(boolean poll) {
            return owner.readPosition(poll);
        }

        public double velocity() {
            return owner.readVelocity(false);
        }

        public double velocity(boolean poll) {
            return owner.readVelocity(poll);
        }

        public Rotation2d rotation2d() {
            return owner.readRotation2d(false);
        }

        public Rotation2d rotation2d(boolean poll) {
            return owner.readRotation2d(poll);
        }

        public double positionModulus(double min, double max) {
            return owner.readPositionModulus(min, max);
        }

        public Encoder.MeasurementsView measurements() {
            return owner.encoder != null ? owner.encoder.measurements() : null;
        }

        public Encoder.StatusView status() {
            return owner.encoder != null ? owner.encoder.status() : null;
        }

        public Encoder device() {
            return owner.encoder;
        }
    }

    public static final class ControlSection {
        private final Mechanism owner;

        private ControlSection(Mechanism owner) {
            this.owner = owner;
        }

        public ControlSection setpoint(double setpoint) {
            owner.setSetpoint(setpoint);
            return this;
        }

        public ControlSection output(double output) {
            owner.setOutput(output);
            return this;
        }

        public ControlSection speed(double speed) {
            owner.setSpeed(speed);
            return this;
        }

        public ControlSection voltage(double voltage) {
            owner.setVoltage(voltage);
            return this;
        }

        public ControlSection manualOverride(boolean override) {
            owner.setOverride(override);
            return this;
        }

        public ControlSection useAbsolute(boolean useAbsolute) {
            owner.setUseAbsolute(useAbsolute);
            return this;
        }

        public ControlSection outputType(OutputType outputType) {
            owner.setOutputType(outputType);
            return this;
        }

        public ControlSection setpointAsOutput(boolean setpointAsOutput) {
            owner.setSetpointAsOutput(setpointAsOutput);
            return this;
        }

        public ControlSection pidEnabled(boolean enabled) {
            owner.setPidEnabled(enabled);
            return this;
        }

        public ControlSection feedforwardEnabled(boolean enabled) {
            owner.setFeedforwardEnabled(enabled);
            return this;
        }

        public ControlSection emergencyStop(boolean stopped) {
            owner.setEmergencyStopped(stopped);
            return this;
        }

        public ControlSection nudge(double nudge) {
            owner.setNudge(nudge);
            return this;
        }

        public ControlSection clearNudge() {
            owner.setNudge(0.0);
            return this;
        }

        public ControlSection bounds(double min, double max) {
            owner.setBounds(min, max);
            return this;
        }

        public ControlSection clearBounds() {
            owner.clearBounds();
            return this;
        }

        public ControlSection encoderPosition(double position) {
            owner.setEncoderPosition(position);
            return this;
        }

        public ControlSection resetPid() {
            owner.resetPID();
            return this;
        }

        public ControlSection pidPeriodSeconds(double periodSeconds) {
            owner.setPidPeriod(periodSeconds);
            return this;
        }

        public ControlSection customPidCycle(boolean enabled) {
            owner.setCustomPIDCycle(enabled);
            return this;
        }
    }

    public static final class LoopsSection {
        private final Mechanism owner;

        private LoopsSection(Mechanism owner) {
            this.owner = owner;
        }

        public LoopsSection enable(String name) {
            owner.enableControlLoop(name);
            return this;
        }

        public LoopsSection disable(String name) {
            owner.disableControlLoop(name);
            return this;
        }

        public LoopsSection enabled(String name, boolean enabled) {
            owner.setControlLoopEnabled(name, enabled);
            return this;
        }

        public boolean enabled(String name) {
            return owner.isControlLoopEnabled(name);
        }

        public Set<String> names() {
            return java.util.Collections.unmodifiableSet(new java.util.LinkedHashSet<>(owner.controlLoopsByName.keySet()));
        }

        public Map<String, Boolean> states() {
            Map<String, Boolean> states = new LinkedHashMap<>();
            for (String name : owner.controlLoopsByName.keySet()) {
                states.put(name, !owner.disabledControlLoops.contains(name));
            }
            return java.util.Collections.unmodifiableMap(states);
        }
    }

    public static final class SysIdSection {
        private final Mechanism owner;

        private SysIdSection(Mechanism owner) {
            this.owner = owner;
        }

        public SysIdSection rampRateVoltsPerSecond(double value) {
            owner.setSysIdRampRateVoltsPerSecond(value);
            return this;
        }

        public SysIdSection stepVoltage(double value) {
            owner.setSysIdStepVoltage(value);
            return this;
        }

        public SysIdSection timeoutSeconds(double value) {
            owner.setSysIdTimeoutSeconds(value);
            return this;
        }

        public double rampRateVoltsPerSecond() {
            return owner.sysIdRampRateVoltsPerSecond;
        }

        public double stepVoltage() {
            return owner.sysIdStepVoltage;
        }

        public double timeoutSeconds() {
            return owner.sysIdTimeoutSeconds;
        }

        public boolean active() {
            return owner.sysIdActive;
        }
    }

    public static final class SimulationSection {
        private final Mechanism owner;

        private SimulationSection(Mechanism owner) {
            this.owner = owner;
        }

        public SimulationSection reset() {
            owner.resetSimulation();
            return this;
        }

        public SimulationSection encoderState(double position, double velocity) {
            owner.setSimulatedEncoderState(position, velocity);
            return this;
        }

        public SimulationSection clearEncoderState() {
            owner.clearSimulatedEncoderState();
            return this;
        }

        private boolean hasSimulation() {
            return owner.simulationModel != null;
        }

        public double updatePeriodSeconds() {
            return owner.simulationUpdatePeriodSeconds;
        }
    }

    public static final class NetworkTablesSection {
        private final Mechanism owner;

        private NetworkTablesSection(Mechanism owner) {
            this.owner = owner;
        }

        public NetworkTablesSection periodSeconds(double periodSeconds) {
            owner.setNetworkTablesPeriodSeconds(periodSeconds);
            return this;
        }

        public double periodSeconds() {
            return owner.readNetworkTablesPeriodSeconds();
        }

        public NetworkTablesSection ownerPath(String ownerPath) {
            owner.setNetworkTablesOwnerPath(ownerPath);
            return this;
        }

        public String ownerPath() {
            return owner.networkTablesOwnerPath;
        }

        public NetworkTablesSection publishHint(String ownerHint) {
            owner.publishNetworkTablesHint(ownerHint);
            return this;
        }

        public RobotNetworkTables.MechanismToggles toggles() {
            return owner.networkTablesConfig();
        }

        public String typeName() {
            return owner.getNetworkTablesTypeName();
        }

        public RobotNetworkTables.Node resolveNode(RobotNetworkTables.Node mechanismsRoot) {
            return owner.resolveDefaultMechanismNode(mechanismsRoot);
        }
    }

    public static final class VisualizationSection {
        private final Mechanism owner;

        private VisualizationSection(Mechanism owner) {
            this.owner = owner;
        }

        public VisualizationSection rootOverride(Pose3d pose) {
            owner.setVisualizationRootOverride(pose);
            return this;
        }

        public Mechanism2d mechanism2d() {
            return owner.getMechanism2d();
        }

        public Map<String, Pose3d> mechanism3dPoses() {
            return owner.getMechanism3dPoses();
        }
    }

    /**
     */
    private void setBoolVal(String key, boolean value) {
        requireMutableKey(key, mutableBoolInputDefaults);
        mutableBoolInputs.put(key, value);
    }

    /**
     */
    private void resetBoolVal(String key) {
        requireMutableKey(key, mutableBoolInputDefaults);
        mutableBoolInputs.put(key, mutableBoolInputDefaults.get(key));
    }

    /**
     */
    private void setDoubleVal(String key, double value) {
        requireMutableKey(key, mutableDoubleInputDefaults);
        mutableDoubleInputs.put(key, value);
    }

    /**
     */
    private void resetDoubleVal(String key) {
        requireMutableKey(key, mutableDoubleInputDefaults);
        mutableDoubleInputs.put(key, mutableDoubleInputDefaults.get(key));
    }

    /**
     */
    private void setIntVal(String key, int value) {
        requireMutableKey(key, mutableIntInputDefaults);
        mutableIntInputs.put(key, value);
    }

    /**
     */
    private void resetIntVal(String key) {
        requireMutableKey(key, mutableIntInputDefaults);
        mutableIntInputs.put(key, mutableIntInputDefaults.get(key));
    }

    /**
     */
    private void setStringVal(String key, String value) {
        requireMutableKey(key, mutableStringInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("string input value cannot be null");
        }
        mutableStringInputs.put(key, value);
    }

    /**
     */
    private void resetStringVal(String key) {
        requireMutableKey(key, mutableStringInputDefaults);
        mutableStringInputs.put(key, mutableStringInputDefaults.get(key));
    }

    /**
     */
    private void setPose2dVal(String key, edu.wpi.first.math.geometry.Pose2d value) {
        requireMutableKey(key, mutablePose2dInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("Pose2d input value cannot be null");
        }
        mutablePose2dInputs.put(key, value);
    }

    /**
     */
    private void resetPose2dVal(String key) {
        requireMutableKey(key, mutablePose2dInputDefaults);
        mutablePose2dInputs.put(key, mutablePose2dInputDefaults.get(key));
    }

    /**
     */
    private void setPose3dVal(String key, edu.wpi.first.math.geometry.Pose3d value) {
        requireMutableKey(key, mutablePose3dInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("Pose3d input value cannot be null");
        }
        mutablePose3dInputs.put(key, value);
    }

    /**
     */
    private void resetPose3dVal(String key) {
        requireMutableKey(key, mutablePose3dInputDefaults);
        mutablePose3dInputs.put(key, mutablePose3dInputDefaults.get(key));
    }

    private MotionLimits getMotionLimits() {
        return motionLimits;
    }

    private Mechanism setMotionLimits(MotionLimits.AxisLimits limits) {
        motionLimits.setBaseAxisLimits(MOTION_AXIS_ID, limits);
        return this;
    }

    private Mechanism setMotionLimits(double maxVelocity, double maxAcceleration) {
        return setMotionLimits(new MotionLimits.AxisLimits(maxVelocity, maxAcceleration));
    }

    private Mechanism registerMotionLimitsProvider(MotionLimits.AxisLimitsProvider provider) {
        if (provider != null) {
            motionLimits.registerAxisProvider(MOTION_AXIS_ID, provider);
        }
        return this;
    }

    private MotionLimits.AxisLimits resolveMotionLimits() {
        return motionLimits.resolveAxisLimits(MOTION_AXIS_ID);
    }

    private double getHardwareUpdatePeriodSeconds() {
        return hardwareUpdatePeriodSeconds;
    }

    private void setHardwareUpdatePeriodSeconds(double periodSeconds) {
        this.hardwareUpdatePeriodSeconds = sanitizeHardwareUpdatePeriod(periodSeconds);
        recomputeHardwareRefreshPhase();
    }

    private void setHardwareUpdatePeriodMs(double periodMs) {
        setHardwareUpdatePeriodSeconds(periodMs / 1000.0);
    }

    public void setRobotCore(RobotCore<?> robotCore) {
        this.robotCore = robotCore;
    }

    public RobotCore<?> getRobotCore() {
        return robotCore;
    }

    private double getSysIdRampRateVoltsPerSecond() {
        return sysIdRampRateVoltsPerSecond;
    }

    private void setSysIdRampRateVoltsPerSecond(double rampRate) {
        if (!Double.isFinite(rampRate) || rampRate <= 0.0) {
            return;
        }
        sysIdRampRateVoltsPerSecond = rampRate;
        invalidateSysIdRoutine();
    }

    private double getSysIdStepVoltage() {
        return sysIdStepVoltage;
    }

    private void setSysIdStepVoltage(double stepVoltage) {
        if (!Double.isFinite(stepVoltage) || stepVoltage <= 0.0) {
            return;
        }
        sysIdStepVoltage = stepVoltage;
        invalidateSysIdRoutine();
    }

    private double getSysIdTimeoutSeconds() {
        return sysIdTimeoutSeconds;
    }

    private void setSysIdTimeoutSeconds(double timeoutSeconds) {
        if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
            return;
        }
        sysIdTimeoutSeconds = timeoutSeconds;
        invalidateSysIdRoutine();
    }

    private boolean isSysIdActive() {
        return sysIdActive;
    }

    private double calculateMaxFreeSpeed(){
        if (encoder == null) {
            return 0;
        }
        double wheelCircumferenceMeters = Math.PI * encoder.getConversion();
        double motorRPM = 6000; //motors.getControllers()[0].getFreeSpeedRPM();
        double wheelRPM = motorRPM * encoder.getGearRatio();
        return wheelCircumferenceMeters * (wheelRPM / 60.0);
    }

    private Rotation2d readRotation2d(boolean poll) {
        if (encoder == null) {
            return Rotation2d.kZero;
        }
        return useAbsolute
                ? encoder.getAbsoluteRotation2d(poll)
                : encoder.getRotation2d(poll);
    }

    private double readPosition(boolean poll) {
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderPosition;
        }
        if (encoder == null) {
            return 0.0;
        }
        return useAbsolute
                ? encoder.getAbsolutePosition(poll)
                : encoder.getPosition(poll);
    }

    private double readPositionModulus(double min, double max) {
        return MathUtil.inputModulus(readPosition(false), min, max);
    }

    private double readVelocity(boolean poll) {
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderVelocity;
        }
        if (encoder == null) {
            return 0.0;
        }
        return encoder.getVelocity(poll);
    }

    private double readOutput() {
        if (override) {
            return manualOutputActive ? manualOutput : 0.0;
        }
        if (RobotBase.isSimulation() && lastOutputValid) {
            return lastOutput;
        }
        return output;
    }

    private void writeEncoderPosition(double position) {
        if (encoder != null) {
            encoder.setPosition(position);
        }
    }

    private double readNetworkTablesPeriodSeconds() {
        RobotNetworkTables nt = lastRobotNetworkTables;
        if (nt == null && robotCore != null) {
            nt = robotCore.networkTables();
        }
        double defaultPeriod = nt != null ? nt.getDefaultPeriodSeconds() : 1.0;
        return Double.isFinite(networkTablesPeriodSecondsOverride) && networkTablesPeriodSecondsOverride > 0.0
                ? networkTablesPeriodSecondsOverride
                : defaultPeriod;
    }

    /**
     */
    private void setVoltage(double voltage){
        recordOutput(voltage, true);
        if (emergencyStopped) {
            motors.stopMotors();
            return;
        }
        motors.setVoltage(voltage);
    }

    /**
     */
    private void setSpeed(double speed){
        recordOutput(speed, false);
        if (emergencyStopped) {
            motors.stopMotors();
            return;
        }
        motors.setSpeed(speed);
    }

    /**
     * Sends a manual output in the currently configured output space.
     */
    /**
     */
    private void setOutput(double output) {
        setOverride(true);
        switch (outputType) {
            case VOLTAGE -> setVoltage(output);
            case POSITION -> {
                recordOutput(output, false);
                if (emergencyStopped) {
                    motors.stopMotors();
                    return;
                }
                motors.setPosition(output);
            }
            case VELOCITY -> {
                recordOutput(output, false);
                if (emergencyStopped) {
                    motors.stopMotors();
                    return;
                }
                motors.setVelocity(output);
            }
            case PERCENT -> setSpeed(output);
        }
    }
    /**
     * Will enable manual override (should not be used unless manual control is wanted, override will not be disabled until done explicitly)
     * @param speed
     */
    /**
     */
    private void setMotors(double speed){
        setOverride(true);
        setSpeed(speed);
    }

    private MotorControllerGroup getMotorGroup(){
        return motors;
    }

    private Encoder getEncoder(){
       return encoder;
    }

    private Rotation2d getRotation2d(){
        if (encoder == null) return Rotation2d.kZero;
        return useAbsolute ? getEncoder().getAbsoluteRotation2d() : getEncoder().getRotation2d();
    }

    private Rotation2d getRotation2d(boolean poll){
        if (encoder == null) return Rotation2d.kZero;
        return useAbsolute ? getEncoder().getAbsoluteRotation2d(poll) : getEncoder().getRotation2d(poll);
    }

    private double getPosition(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderPosition;
        }
        if (encoder == null) return 0;
        return useAbsolute ? getEncoder().getAbsolutePosition() : getEncoder().getPosition();
    }

    private double getPosition(boolean poll){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderPosition;
        }
        if (encoder == null) return 0;
        return useAbsolute ? getEncoder().getAbsolutePosition(poll) : getEncoder().getPosition(poll);
    }

    /**
     * Returns the current mechanism position wrapped into the provided range.
     *
     * This does not change the underlying sensor; it is just a view for dashboards/logging and
     * angle math. Units match {@link #getPosition()}.
     */
    private double getPositionModulus(double min, double max) {
        return MathUtil.inputModulus(getPosition(), min, max);
    }

    private double getVelocity(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderVelocity;
        }
        if (encoder == null) return 0;
        return getEncoder().getVelocity();
    }

    private double getVelocity(boolean poll){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderVelocity;
        }
        if (encoder == null) return 0;
        return getEncoder().getVelocity(poll);
    }

    public boolean atSetpoint(){
        return (pidController == null || pidController.atSetpoint()) &&
           (profiledPIDController == null || profiledPIDController.atGoal());
    }

    /**
     */
    private void setSetpoint(double setpoint){
        this.setpoint = applyBounds(setpoint);
    }

    private double getSetpoint(){
        return setpoint;
    }

    protected Enum<?> getActiveState() {
        return null;
    }

    /**
     * Clamps all setpoints to the provided bounds.
     */
    /**
     */
    protected Mechanism setBounds(double min, double max) {
        if (Double.isFinite(min) && Double.isFinite(max) && max > min) {
            this.minBound = min;
            this.maxBound = max;
            this.boundsEnabled = true;
            MechanismTravelRange.registerKnownRange(this, min, max);
        } else {
            clearBounds();
        }
        return this;
    }

    /**
     * Clears any configured setpoint bounds.
     */
    /**
     */
    protected Mechanism clearBounds() {
        this.boundsEnabled = false;
        this.minBound = Double.NaN;
        this.maxBound = Double.NaN;
        return this;
    }

    private double applyBounds(double value) {
        if (!boundsEnabled) {
            return value;
        }
        return MathUtil.clamp(value, minBound, maxBound);
    }

    protected boolean hasBounds() {
        return boundsEnabled;
    }

    protected double getMinBound() {
        return minBound;
    }

    protected double getMaxBound() {
        return maxBound;
    }

    private double getControllerSetpointVelocity(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().velocity : pidController != null ? pidController.getSetpoint() : 0;
    }

    private double getControllerSetpointPosition(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().position : pidController != null ? pidController.getSetpoint() : 0;
    }

    /**
     * Measurement used by PID loops. Defaults to position; mechanisms may override.
     */
    protected double getPidMeasurement() {
        
        return shouldCustomEncoder ? customEncoderPos.getAsDouble() : getPosition();

    }



    private boolean isOverride() {
        return override;
    }

    private boolean isEmergencyStopped() {
        return emergencyStopped;
    }

    /**
     */
    private void setOverride(boolean override) {
        if (this.override != override) {
            appendDiagnosticLog("INFO", "control", override ? "manual override enabled" : "manual override disabled");
        }
        this.override = override;
        if (override) {
            manualOutputActive = false;
            manualOutput = 0.0;
            manualOutputIsVoltage = false;
        }
    }

    private boolean isUseAbsolute() {
        return useAbsolute;
    }

    /**
     */
    private void setUseAbsolute(boolean useAbsolute) {
        this.useAbsolute = useAbsolute;
    }

    private boolean isUseVoltage() {
        return outputType == OutputType.VOLTAGE;
    }

    private OutputType getOutputType() {
        return outputType;
    }

    /**
     */
    private void setOutputType(OutputType outputType) {
        if (outputType == null) {
            return;
        }
        this.outputType = outputType;
    }

    protected double percentToOutput(double percent) {
        return OutputConversions.toMechanismOutput(
                outputType,
                OutputType.PERCENT,
                percent,
                RobotController.getBatteryVoltage());
    }

    protected double voltsToOutput(double volts) {
        return OutputConversions.toMechanismOutput(
                outputType,
                OutputType.VOLTAGE,
                volts,
                RobotController.getBatteryVoltage());
    }

    /**
     * Converts a value from the given output type into this mechanism's output space.
     */
    protected double toOutput(OutputType type, double value) {
        return OutputConversions.toMechanismOutput(
                outputType,
                type,
                value,
                RobotController.getBatteryVoltage());
    }

    OutputType getControlLoopPidOutputType(String name) {
        if (name == null) {
            return OutputType.PERCENT;
        }
        return controlLoopPidOutputTypes.getOrDefault(name, OutputType.PERCENT);
    }

    MechanismConfig.BangBangProfile getControlLoopBangBangProfile(String name) {
        if (name == null) {
            return null;
        }
        return controlLoopBangBangs.get(name);
    }

    OutputType getControlLoopFeedforwardOutputType(String name) {
        if (name == null) {
            return OutputType.VOLTAGE;
        }
        return controlLoopFeedforwardOutputTypes.getOrDefault(name, OutputType.VOLTAGE);
    }

    static double sanitizeBangBangLevel(double output) {
        return Double.isFinite(output) ? output : 0.0;
    }

    static double sanitizeBangBangTolerance(double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            return 0.0;
        }
        return tolerance;
    }

    static boolean isBangBangWithinTolerance(
            MechanismConfig.BangBangProfile profile,
            double measurement,
            double setpoint) {
        if (profile == null || !Double.isFinite(measurement) || !Double.isFinite(setpoint)) {
            return false;
        }
        double error = setpoint - measurement;
        return Math.abs(error) <= sanitizeBangBangTolerance(profile.tolerance());
    }

    static double calculateBangBangRaw(
            MechanismConfig.BangBangProfile profile,
            double measurement,
            double setpoint) {
        if (profile == null || !Double.isFinite(measurement) || !Double.isFinite(setpoint)) {
            return 0.0;
        }
        if (isBangBangWithinTolerance(profile, measurement, setpoint)) {
            return 0.0;
        }
        double error = setpoint - measurement;
        return error > 0.0
                ? sanitizeBangBangLevel(profile.highOutput())
                : sanitizeBangBangLevel(profile.lowOutput());
    }

    private boolean isSetpointAsOutput() {
        return setpointIsOutput;
    }

    /**
     */
    private void setSetpointAsOutput(boolean setpointAsOutput) {
        this.setpointIsOutput = setpointAsOutput;
    }

    private void setPidPeriod(double pidPeriod) {
        if (!Double.isFinite(pidPeriod)) {
            return;
        }
        this.pidPeriod = pidPeriod;
    }
    private boolean isOutputVoltage() {
        if (override && manualOutputActive) {
            return manualOutputIsVoltage;
        }
        if (RobotBase.isSimulation() && lastOutputValid) {
            return lastOutputIsVoltage;
        }
        return isUseVoltage();
    }

    private boolean isCustomPIDCycle() {
        return customPIDCycle;
    }

    private void setCustomPIDCycle(boolean customPIDCycle) {
        this.customPIDCycle = customPIDCycle;
    }

    public double calculateFeedForward(){
        return 0;
    }

    /**
     */
    private void setEncoderPosition(double position){
        if(encoder != null){
            encoder.setPosition(position);
        }
    }

    /**
     */
    private void resetPID(){
        if(pidController != null) pidController.reset();
        if(profiledPIDController != null) profiledPIDController.reset(getPosition(), getVelocity());
        this.output = 0;
    }

    private PIDController getPIDController()
    {
        return pidController;
    }
    private double getMin()
    {
        return minBound;
    }
    private double getMax()
    {
        return maxBound;
    }

    private double calculatePID(){
        double output = 0;
        double encoderPos = getPidMeasurement();
        double setpoint = getSetpoint();
        double target = setpoint + getNudge();
        applyMotionLimits();

        if (pidController != null){
            output += pidController.calculate(encoderPos, target);
        }

        if(profiledPIDController != null){
            if(prevSetpoint != getSetpoint()) resetPID();
            // Profiled PID is assumed to already be expressed in the mechanism output space.
            output += profiledPIDController.calculate(encoderPos, getSetpoint() + getNudge());
        }
        
        return output;
    }

    private void applyMotionLimits() {
        if (profiledPIDController == null || baseProfiledConstraints == null) {
            return;
        }
        MotionLimits.AxisLimits limits = resolveMotionLimits();
        double maxVel = baseProfiledConstraints.maxVelocity;
        double maxAccel = baseProfiledConstraints.maxAcceleration;
        if (limits != null) {
            if (limits.maxVelocity() > 0.0) {
                if (Double.isFinite(maxVel) && maxVel > 0.0) {
                    maxVel = Math.min(maxVel, limits.maxVelocity());
                } else {
                    maxVel = limits.maxVelocity();
                }
            }
            if (limits.maxAcceleration() > 0.0) {
                if (Double.isFinite(maxAccel) && maxAccel > 0.0) {
                    maxAccel = Math.min(maxAccel, limits.maxAcceleration());
                } else {
                    maxAccel = limits.maxAcceleration();
                }
            }
        }
        if (!Double.isFinite(maxVel) || maxVel <= 0.0) {
            maxVel = baseProfiledConstraints.maxVelocity;
        }
        if (!Double.isFinite(maxAccel) || maxAccel <= 0.0) {
            maxAccel = baseProfiledConstraints.maxAcceleration;
        }
        profiledPIDController.setConstraints(new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    private void outputMotor(double output) {
        switch (outputType) {
            case VOLTAGE -> setVoltage(output);
            case POSITION -> motors.setPosition(output);
            case VELOCITY -> motors.setVelocity(output);
            case PERCENT -> setSpeed(output);
        }
    }

    public void updatePID(){
        pidOutput = calculatePID();
        feedforwardOutput = calculateFeedForward();
    }

    public void update(){

        double output = 0;

        boolean encoderDisconnected = encoder != null && !encoder.isConnected();
        boolean motorsDisconnected = !motors.allMotorsConnected();
        updateConnectivityEmergencyStop(encoderDisconnected, motorsDisconnected);

        if(!customPIDCycle) updatePID();

        output = isPidEnabled() ? pidOutput : 0;
        output += isFeedforwardEnabled() ? feedforwardOutput : 0;

        if (setpointIsOutput){
            output = getSetpoint() + getNudge();
        }

        output += calculateControlLoopOutput();

        this.output = output;

        this.prevSetpoint = getSetpoint();

        // Apply limit switch encoder zeroing regardless of control mode. Previously, override
        // short-circuited the update loop and would skip zeroing entirely.
        applyLimitSwitchZeroing();

        if(emergencyStopped){
            motors.stopMotors();
            return;
        }

        if (override){
            return;
        }

        output = applyHardstopSuppression(output);

        outputMotor(output);
    }

    private void applyLimitSwitchZeroing() {
        for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
            if (!genericLimitSwitch.getAsBoolean()) {
                continue;
            }
            double position = genericLimitSwitch.getPosition();
            if (Double.isFinite(position)) {
                setEncoderPosition(position);
            }
        }
    }

    private double applyHardstopSuppression(double output) {
        double suppressed = output;
        for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
            if (!genericLimitSwitch.getAsBoolean()) {
                continue;
            }
            if (!genericLimitSwitch.isHardstop()) {
                continue;
            }
            int direction = genericLimitSwitch.getBlockDirectionMultiplier();
            if (Math.signum(direction) == Math.signum(suppressed)) {
                suppressed = 0;
            }
        }
        return suppressed;
    }

    private void logEmergencyStopReason(boolean encoderDisconnected, boolean motorsDisconnected) {
        String reasonText = connectivityReasonText(encoderDisconnected, motorsDisconnected);
        double now = Timer.getFPGATimestamp();
        boolean reasonChanged = !reasonText.equals(lastEmergencyStopReason);
        boolean shouldLog = Double.isNaN(lastEmergencyStopLogSeconds)
                || (now - lastEmergencyStopLogSeconds) >= EMERGENCY_STOP_LOG_PERIOD_SECONDS
                || reasonChanged;
        if (shouldLog) {
            System.out.println("[Athena][EmergencyStop] " + getName() + ": " + reasonText);
            lastEmergencyStopLogSeconds = now;
            lastEmergencyStopReason = reasonText;
            if (reasonChanged) {
                appendDiagnosticLog("ERROR", "fault", reasonText);
            }
        }
    }

    private void updateConnectivityEmergencyStop(boolean encoderDisconnected, boolean motorsDisconnected) {
        boolean disconnected = encoderDisconnected || motorsDisconnected;
        if (disconnected) {
            connectivityFaultReason = connectivityReasonText(encoderDisconnected, motorsDisconnected);
            lastFaultReason = connectivityFaultReason;
            connectivityFaultEmergencyStopped = true;
            refreshEmergencyStopState();
            logEmergencyStopReason(encoderDisconnected, motorsDisconnected);
            return;
        }
        if (!connectivityFaultEmergencyStopped) {
            return;
        }
        connectivityFaultEmergencyStopped = false;
        connectivityFaultReason = "";
        refreshEmergencyStopState();
        logEmergencyStopRecovery();
    }

    private void logEmergencyStopRecovery() {
        String reasonText = "connectivity restored";
        double now = Timer.getFPGATimestamp();
        boolean shouldLog = Double.isNaN(lastEmergencyStopLogSeconds)
                || (now - lastEmergencyStopLogSeconds) >= EMERGENCY_STOP_LOG_PERIOD_SECONDS
                || !reasonText.equals(lastEmergencyStopReason);
        if (shouldLog) {
            System.out.println("[Athena][EmergencyStop] " + getName() + ": " + reasonText);
            lastEmergencyStopLogSeconds = now;
            lastEmergencyStopReason = reasonText;
            appendDiagnosticLog("INFO", "fault", reasonText);
        }
    }

    private void refreshEmergencyStopState() {
        emergencyStopped = manualEmergencyStopped || connectivityFaultEmergencyStopped;
    }

    private String connectivityReasonText(boolean encoderDisconnected, boolean motorsDisconnected) {
        StringBuilder reason = new StringBuilder();
        if (encoderDisconnected) {
            reason.append("encoder disconnected");
        }
        if (motorsDisconnected) {
            if (reason.length() > 0) {
                reason.append(", ");
            }
            reason.append("motor controller disconnected");
        }
        return reason.toString();
    }

    private String resolveActiveFaultReason() {
        if (manualEmergencyStopped && connectivityFaultEmergencyStopped) {
            return "manual emergency stop, " + connectivityFaultReason;
        }
        if (manualEmergencyStopped) {
            return "manual emergency stop";
        }
        if (connectivityFaultEmergencyStopped) {
            return connectivityFaultReason;
        }
        return "";
    }

    private void appendDiagnosticLog(String level, String category, String message) {
        String text = message != null ? message.trim() : "";
        if (text.isEmpty()) {
            return;
        }
        String lvl = level != null && !level.isBlank() ? level : "INFO";
        String cat = category != null && !category.isBlank() ? category : "general";
        String systemKey = diagnosticsSystemKey();
        String mechanismName = systemKey.lastIndexOf('/') >= 0
                ? systemKey.substring(systemKey.lastIndexOf('/') + 1)
                : systemKey;
        String line = "[" + mechanismName + "] "
                + lvl.toLowerCase(java.util.Locale.ROOT)
                + " " + cat + ": " + text;
        diagnosticsLog.append((sequence, timestampSeconds) ->
                new DiagnosticsChannel.Event(sequence, timestampSeconds, systemKey, lvl, cat, text, line));
    }

    private static int sanitizeDiagnosticLogLimit(int requestedLimit) {
        if (requestedLimit <= 0) {
            return 0;
        }
        return Math.min(requestedLimit, DIAGNOSTIC_LOG_CAPACITY);
    }

    private List<MechanismLogEvent> getDiagnosticLog(int requestedLimit) {
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        List<DiagnosticsChannel.Event> events = diagnosticsLog.snapshot(limit);
        List<MechanismLogEvent> logs = new ArrayList<>(events.size());
        for (DiagnosticsChannel.Event event : events) {
            if (event == null) {
                continue;
            }
            logs.add(new MechanismLogEvent(
                    event.sequence(),
                    event.timestampSeconds(),
                    event.systemKey(),
                    event.level(),
                    event.category(),
                    event.message(),
                    event.line()));
        }
        return logs;
    }

    private List<DiagnosticsChannel.Event> getDiagnosticEvents(int requestedLimit) {
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        return diagnosticsLog.snapshot(limit);
    }

    private int getDiagnosticLogCount() {
        return diagnosticsLog.count();
    }

    private void clearDiagnosticLog() {
        diagnosticsLog.clear();
    }

    private Map<String, Object> getDiagnosticsSummary() {
        return buildDiagnosticsSnapshot(0, false);
    }

    private Map<String, Object> getDiagnosticsSnapshot(int logLimit) {
        return buildDiagnosticsSnapshot(logLimit, true);
    }

    public DiagnosticsView diagnostics() {
        return diagnosticsView;
    }

    public final class DiagnosticsView {
        public DiagnosticsView log(String level, String category, String message) {
            appendDiagnosticLog(level, category, message);
            return this;
        }

        public DiagnosticsView info(String category, String message) {
            appendDiagnosticLog("INFO", category, message);
            return this;
        }

        public DiagnosticsView warn(String category, String message) {
            appendDiagnosticLog("WARN", category, message);
            return this;
        }

        public DiagnosticsView error(String category, String message) {
            appendDiagnosticLog("ERROR", category, message);
            return this;
        }

        public List<DiagnosticsChannel.Event> events(int limit) {
            return getDiagnosticEvents(limit);
        }

        public int count() {
            return getDiagnosticLogCount();
        }

        public DiagnosticsView clear() {
            clearDiagnosticLog();
            return this;
        }

        public Map<String, Object> summary() {
            return getDiagnosticsSummary();
        }

        public Map<String, Object> snapshot(int limit) {
            return getDiagnosticsSnapshot(limit);
        }
    }

    private Map<String, Object> buildDiagnosticsSnapshot(int logLimit, boolean includeLogs) {
        double nowSeconds = Timer.getFPGATimestamp();
        double position = getPosition();
        double velocity = getVelocity();
        double setpointValue = getSetpoint();
        double setpointWithNudge = setpointValue + getNudge();
        boolean encoderPresent = encoder != null;
        boolean encoderConnected = !encoderPresent || encoder.isConnected();
        MotorController[] controllers = motors != null ? motors.getControllers() : new MotorController[0];
        List<Map<String, Object>> motorDetails = new ArrayList<>(controllers.length);
        int connectedMotors = 0;
        for (MotorController controller : controllers) {
            if (controller == null) {
                continue;
            }
            boolean connected = controller.isConnected();
            if (connected) {
                connectedMotors++;
            }
            Map<String, Object> motor = new LinkedHashMap<>();
            motor.put("id", controller.getId());
            motor.put("canbus", controller.getCanbus());
            motor.put("type", controller.getType() != null ? controller.getType().getKey() : "unknown");
            motor.put("connected", connected);
            motor.put("temperatureC", controller.getTemperatureCelsius());
            motorDetails.add(motor);
        }
        List<Map<String, Object>> limitSwitchDetails = new ArrayList<>(limitSwitches.length);
        for (GenericLimitSwitch sw : limitSwitches) {
            if (sw == null) {
                continue;
            }
            Map<String, Object> status = new LinkedHashMap<>();
            status.put("port", sw.getPort());
            status.put("active", sw.getAsBoolean());
            status.put("hardstop", sw.isHardstop());
            status.put("position", sw.getPosition());
            status.put("blockDirection", String.valueOf(sw.getBlockDirection()));
            limitSwitchDetails.add(status);
        }
        double lastHardwareAgeMs =
                (Double.isFinite(lastHardwareUpdateSeconds) && Double.isFinite(nowSeconds) && nowSeconds >= lastHardwareUpdateSeconds)
                        ? (nowSeconds - lastHardwareUpdateSeconds) * 1000.0
                        : -1.0;
        double nextHardwareMs = -1.0;
        double nextDue = resolveNextHardwareRefreshDueSeconds(nowSeconds);
        if (Double.isFinite(nextDue) && Double.isFinite(nowSeconds)) {
            nextHardwareMs = Math.max(0.0, nextDue - nowSeconds) * 1000.0;
        }

        Map<String, Object> fault = new LinkedHashMap<>();
        fault.put("emergencyStopped", emergencyStopped);
        fault.put("manualEmergencyStop", manualEmergencyStopped);
        fault.put("connectivityEmergencyStop", connectivityFaultEmergencyStopped);
        fault.put("activeReason", resolveActiveFaultReason());
        fault.put("lastReason", lastFaultReason);

        Map<String, Object> control = new LinkedHashMap<>();
        control.put("setpoint", setpointValue);
        control.put("setpointWithNudge", setpointWithNudge);
        control.put("nudge", nudge);
        control.put("position", position);
        control.put("velocity", velocity);
        control.put("error", setpointWithNudge - position);
        control.put("output", getOutput());
        control.put("pidOutput", pidOutput);
        control.put("feedforwardOutput", feedforwardOutput);
        control.put("outputType", String.valueOf(outputType));
        control.put("atSetpoint", atSetpoint());
        control.put("override", override);
        control.put("pidEnabled", pidEnabled);
        control.put("feedforwardEnabled", feedforwardEnabled);
        control.put("setpointAsOutput", setpointIsOutput);

        Map<String, Object> hardware = new LinkedHashMap<>();
        hardware.put("encoderPresent", encoderPresent);
        hardware.put("encoderConnected", encoderConnected);
        hardware.put("motorsConnected", connectedMotors == motorDetails.size());
        hardware.put("motorCount", motorDetails.size());
        hardware.put("connectedMotorCount", connectedMotors);
        hardware.put("disconnectedMotorCount", Math.max(0, motorDetails.size() - connectedMotors));
        hardware.put("averageMotorTempC", motors != null ? motors.getAverageTemperatureCelsius() : 0.0);
        hardware.put("motors", motorDetails);
        hardware.put("limitSwitches", limitSwitchDetails);

        Map<String, Object> timing = new LinkedHashMap<>();
        timing.put("pidPeriodMs", pidPeriod * 1000.0);
        timing.put("hardwareUpdatePeriodMs", hardwareUpdatePeriodSeconds * 1000.0);
        timing.put("hardwareLastRefreshAgeMs", lastHardwareAgeMs);
        timing.put("hardwareNextRefreshInMs", nextHardwareMs);
        timing.put("networkTablesPeriodSec", getNetworkTablesPeriodSeconds());
        timing.put("simulation", hasSimulation());
        timing.put("simulationUpdatePeriodSec", simulationUpdatePeriodSeconds);
        timing.put("sysIdActive", sysIdActive);

        Map<String, Object> snapshot = new LinkedHashMap<>();
        snapshot.put("name", getName());
        snapshot.put("systemKey", diagnosticsSystemKey());
        snapshot.put("type", getNetworkTablesTypeName());
        snapshot.put("ownerPath", getNetworkTablesOwnerPath());
        snapshot.put("timestampSeconds", nowSeconds);
        snapshot.put("fault", fault);
        snapshot.put("control", control);
        snapshot.put("hardware", hardware);
        snapshot.put("timing", timing);
        snapshot.put("logCount", getDiagnosticLogCount());
        if (includeLogs) {
            snapshot.put("events", getDiagnosticEvents(logLimit));
            snapshot.put("logs", getDiagnosticLog(logLimit));
        }
        return snapshot;
    }

    private GenericLimitSwitch[] getLimitSwitches() {
        return limitSwitches;
    }

    /**
     */
    private void setCurrentLimit(double currentLimit){
        motors.setCurrentLimit(currentLimit);
    }

    /**
     */
    private void setMotorNeutralMode(MotorNeutralMode mode){
        motors.setNeutralMode(mode);
    }

    private boolean isFeedforwardEnabled() {
        return feedforwardEnabled;
    }

    private boolean isPidEnabled() {
        return pidEnabled;
    }

    private double getPidOutput() {
        return pidOutput;
    }

    private double getFeedforwardOutput() {
        return feedforwardOutput;
    }

    /**
     */
    private void setFeedforwardEnabled(boolean feedforwardEnabled) {
        if (this.feedforwardEnabled != feedforwardEnabled) {
            appendDiagnosticLog(
                    "INFO",
                    "control",
                    feedforwardEnabled ? "feedforward enabled" : "feedforward disabled");
        }
        this.feedforwardEnabled = feedforwardEnabled;
    }

    /**
     */
    private void setPidEnabled(boolean pidEnabled) {
        if (this.pidEnabled != pidEnabled) {
            appendDiagnosticLog("INFO", "control", pidEnabled ? "pid enabled" : "pid disabled");
        }
        this.pidEnabled = pidEnabled;
    }

    /**
     */
    private void setEmergencyStopped(boolean emergancyStopped) {
        if (this.manualEmergencyStopped != emergancyStopped) {
            appendDiagnosticLog(
                    emergancyStopped ? "ERROR" : "INFO",
                    "fault",
                    emergancyStopped ? "manual emergency stop enabled" : "manual emergency stop cleared");
        }
        this.manualEmergencyStopped = emergancyStopped;
        if (emergancyStopped) {
            lastFaultReason = "manual emergency stop";
        }
        refreshEmergencyStopState();
    }

    private double getOutput() {
        if (override) {
            return manualOutputActive ? manualOutput : 0.0;
        }
        if (RobotBase.isSimulation() && lastOutputValid) {
            return lastOutput;
        }
        return output;
    }

    /**
     */
    private void setNudge(double nudge){
        this.nudge = nudge;
    }

    private void recordOutput(double output, boolean outputIsVoltage) {
        lastOutputValid = true;
        lastOutput = output;
        lastOutputIsVoltage = outputIsVoltage;
        if (!override) {
            return;
        }
        manualOutputActive = true;
        manualOutput = output;
        manualOutputIsVoltage = outputIsVoltage;
    }

    private double getNudge() {
        return nudge;
    }

    private double getPidPeriod() {
        return pidPeriod;
    }

    private double getNetworkTablesPeriodSeconds() {
        RobotNetworkTables nt = lastRobotNetworkTables;
        if (nt == null && robotCore != null) {
            nt = robotCore.networkTables();
        }
        double defaultPeriod = nt != null ? nt.getDefaultPeriodSeconds() : 1.0;
        return Double.isFinite(networkTablesPeriodSecondsOverride) && networkTablesPeriodSecondsOverride > 0.0
                ? networkTablesPeriodSecondsOverride
                : defaultPeriod;
    }

    private void setNetworkTablesPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            // Reset to global default.
            this.networkTablesPeriodSecondsOverride = Double.NaN;
            return;
        }
        this.networkTablesPeriodSecondsOverride = periodSeconds;
    }

    /**
     */
    private void disableControlLoop(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return;
        }
        disabledControlLoops.add(name);
        appendDiagnosticLog("WARN", "control-loop", "disabled control loop '" + name + "'");
    }

    /**
     */
    private void enableControlLoop(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return;
        }
        disabledControlLoops.remove(name);
        appendDiagnosticLog("INFO", "control-loop", "enabled control loop '" + name + "'");
    }

    /**
     */
    private void setControlLoopEnabled(String name, boolean enabled) {
        if (enabled) {
            enableControlLoop(name);
        } else {
            disableControlLoop(name);
        }
    }

    private boolean isControlLoopEnabled(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return false;
        }
        return !disabledControlLoops.contains(name);
    }

    private boolean hasSimulation() {
        return simulationModel != null;
    }

    private double getSimulationUpdatePeriodSeconds() {
        return simulationUpdatePeriodSeconds;
    }

    public void loadConfig(Path path) {
        MechanismConfigIO.apply(this, MechanismConfigIO.load(path));
    }

    public void saveConfig(Path path) {
        MechanismConfigIO.save(path, MechanismConfigIO.snapshot(this));
    }

    private void resetSimulation() {
        if (simulationModel == null) {
            return;
        }
        simulationModel.reset();
        lastSimulationTimestampSeconds = Timer.getFPGATimestamp();
    }

    private void advanceSimulation(double nowSeconds) {
        if (simulationModel == null) {
            return;
        }

        if (!Double.isFinite(nowSeconds)) {
            return;
        }

        if (Double.isNaN(lastSimulationTimestampSeconds)) {
            simulationModel.reset();
            lastSimulationTimestampSeconds = nowSeconds;
            return;
        }

        double elapsed = nowSeconds - lastSimulationTimestampSeconds;
        if (!(elapsed > 0.0)) {
            elapsed = simulationUpdatePeriodSeconds;
        } else if (elapsed > 0.2) {
            elapsed = 0.2;
        }

        double remaining = elapsed;
        while (remaining > 1e-9) {
            double step = Math.min(simulationUpdatePeriodSeconds, remaining);
            simulationModel.update(step);
            remaining -= step;
        }

        lastSimulationTimestampSeconds = nowSeconds;
    }

    @SuppressWarnings("unchecked")
    private void applyPeriodicHooks() {
        for (Consumer<?> hook : periodicHooks) {
            ((Consumer<Mechanism>) hook).accept(this);
        }
        if (!timedPeriodicHooks.isEmpty()) {
            double nowSeconds = Timer.getFPGATimestamp();
            for (TimedRunner<Consumer<Mechanism>> runner : timedPeriodicHooks) {
                if (runner.shouldRunSeconds(nowSeconds)) {
                    runner.run(hook -> hook.accept(this), nowSeconds);
                }
            }
        }
    }

    private void updateNetworkTablesCache() {
        if (!networkTablesEnabled) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        double periodSeconds = getNetworkTablesPeriodSeconds();
        if (Double.isFinite(periodSeconds) && periodSeconds > 0.0
                && Double.isFinite(nowSeconds)
                && !Double.isNaN(lastNetworkTablesCacheUpdateSeconds)
                && (nowSeconds - lastNetworkTablesCacheUpdateSeconds) < periodSeconds) {
            return;
        }
        lastNetworkTablesCacheUpdateSeconds = nowSeconds;
        cachedEmergencyStopped = emergencyStopped;
        cachedOverride = override;
        cachedAtSetpoint = atSetpoint();
        cachedSetpoint = setpoint;
        cachedNudge = nudge;
        cachedOutput = output;
        cachedPidOutput = pidOutput;
        cachedFeedforwardOutput = feedforwardOutput;
        cachedHasSimulation = hasSimulation();
        cachedSimulationUpdatePeriodSeconds = simulationUpdatePeriodSeconds;
        cachedUseVoltage = isUseVoltage();
        cachedUseAbsolute = useAbsolute;
        cachedSetpointAsOutput = setpointIsOutput;
        cachedPidPeriod = pidPeriod;
        cachedFeedforwardEnabled = feedforwardEnabled;
        cachedPidEnabled = pidEnabled;
        cachedSysIdRampRate = sysIdRampRateVoltsPerSecond;
        cachedSysIdStepVoltage = sysIdStepVoltage;
        cachedSysIdTimeoutSeconds = sysIdTimeoutSeconds;
        cachedSysIdActive = sysIdActive;
        cachedHardwareUpdatePeriodSeconds = hardwareUpdatePeriodSeconds;
        cachedHardwareUpdatePhaseSeconds = hardwareUpdatePhaseSeconds;
        cachedHardwareUpdateSlot = hardwareUpdateSlot;
        cachedHardwareLastRefreshAgeSeconds =
                (Double.isFinite(lastHardwareUpdateSeconds) && Double.isFinite(nowSeconds) && nowSeconds >= lastHardwareUpdateSeconds)
                        ? (nowSeconds - lastHardwareUpdateSeconds)
                        : Double.NaN;
        double nextDueSeconds = resolveNextHardwareRefreshDueSeconds(nowSeconds);
        cachedHardwareNextRefreshInSeconds =
                (Double.isFinite(nextDueSeconds) && Double.isFinite(nowSeconds))
                        ? Math.max(0.0, nextDueSeconds - nowSeconds)
                        : Double.NaN;
        cachedFaultReason = resolveActiveFaultReason();
        cachedLastFaultReason = lastFaultReason;
    }

    private double calculateControlLoopOutput() {
        if (controlLoops.isEmpty()) {
            return 0.0;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        double total = 0.0;
        for (TimedRunner<MechanismConfig.MechanismControlLoop<Mechanism>> runner : controlLoops) {
            if (!isControlLoopEnabled(runner.name())) {
                runner.setLastOutput(0.0);
                continue;
            }
            if (runner.shouldRunSeconds(nowSeconds)) {
                double dtSeconds;
                if (Double.isNaN(runner.lastRunSeconds())) {
                    dtSeconds = runner.periodSeconds();
                } else {
                    dtSeconds = nowSeconds - runner.lastRunSeconds();
                }
                if (!Double.isFinite(dtSeconds) || dtSeconds < 0.0) {
                    dtSeconds = Double.NaN;
                }
                controlContext.setControlLoopDtSeconds(dtSeconds);
                runner.setLastOutput(runner.task().calculate(controlContext));
                runner.setLastRunSeconds(nowSeconds);
            }
            total += runner.lastOutput();
        }
        return total;
    }

    private boolean shouldRefreshHardwareSignals(double nowSeconds) {
        if (simulationModel != null) {
            return false;
        }
        if (!Double.isFinite(nowSeconds)) {
            return true;
        }
        if (Double.isNaN(lastHardwareUpdateSeconds) || !Double.isFinite(lastHardwareUpdateSeconds)) {
            if (!Double.isFinite(firstHardwareRefreshDueSeconds)) {
                firstHardwareRefreshDueSeconds = nowSeconds + hardwareUpdatePhaseSeconds;
            }
            return nowSeconds >= firstHardwareRefreshDueSeconds;
        }
        if (nowSeconds < lastHardwareUpdateSeconds) {
            firstHardwareRefreshDueSeconds = Double.NaN;
            lastHardwareUpdateSeconds = Double.NaN;
            return false;
        }
        return (nowSeconds - lastHardwareUpdateSeconds) >= hardwareUpdatePeriodSeconds;
    }

    private void refreshHardwareSignals(boolean force) {
        if (simulationModel != null) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        if (!force && !shouldRefreshHardwareSignals(nowSeconds)) {
            return;
        }
        if (encoder != null && !encoderOwnedByMotorGroup) {
            encoder.update();
        }
        motors.update();
        lastHardwareUpdateSeconds = nowSeconds;
    }

    @Override
    public void periodic() {
        if (LoopTiming.shouldSampleMechanisms()) {
            double startSeconds = Timer.getFPGATimestamp();
            periodicImpl();
            double durationMs = (Timer.getFPGATimestamp() - startSeconds) * 1000.0;
            LoopTiming.recordMechanism(getName(), durationMs);
            return;
        }
        periodicImpl();
    }

    private void periodicImpl() {
        if(DriverStation.isAutonomousEnabled()){
            robotMode = RobotMode.AUTO;
        } else if(DriverStation.isTeleopEnabled()){
            robotMode = RobotMode.TELE;
        } else if(DriverStation.isDisabled()){
            robotMode = RobotMode.DISABLE;
        } else if(DriverStation.isTestEnabled()){
            robotMode = RobotMode.TEST;
        }

        if(!prevRobotMode.equals(robotMode)){
            resetPID();
        }

        if (simulationModel != null) {
            advanceSimulation(Timer.getFPGATimestamp());
        } else {
            refreshHardwareSignals(false);
        }

        sensorSimulation.update(this);

        applyPeriodicHooks();

        update();

        updateNetworkTablesCache();
        refreshNetworkTablesOnConfigChange();
        attemptNetworkTablesPublishIfNeeded();

        if (visualization != null) {
            visualization.setExternalRootPose(visualizationRootOverride);
            visualization.update(this);
        }

        prevRobotMode = robotMode;
    }

    private void refreshNetworkTablesOnConfigChange() {
        RobotNetworkTables nt = lastRobotNetworkTables;
        if (!shouldPublishMechanismNetworkTables(nt)) {
            return;
        }
        if (!networkTablesEnabled) {
            return;
        }
        RobotNetworkTables.Node node = lastNetworkTablesNode;
        if (node == null) {
            return;
        }
        long revision = nt.revision();
        if (revision == lastNetworkTablesConfigRevision) {
            return;
        }
        networkTablesInternal(nt, node);
    }

    @SuppressWarnings("unchecked")
    private void registerControlLoop(MechanismConfig.ControlLoopBinding<?> binding) {
        if (binding == null) {
            return;
        }
        String name = binding.name();
        if (name == null || name.isBlank()) {
            return;
        }
        if (controlLoopsByName.containsKey(name)) {
            throw new IllegalArgumentException("control loop name already registered: " + name);
        }
        TimedRunner<MechanismConfig.MechanismControlLoop<Mechanism>> runner = new TimedRunner<>(
                name,
                binding.periodSeconds(),
                (MechanismConfig.MechanismControlLoop<Mechanism>) binding.loop());
        controlLoops.add(runner);
        controlLoopsByName.put(name, runner);
    }

    @SuppressWarnings("unchecked")
    private void registerPeriodicHook(MechanismConfig.PeriodicHookBinding<?> binding) {
        if (binding == null) {
            return;
        }
        timedPeriodicHooks.add(TimedRunner.periodicMs((Consumer<Mechanism>) binding.hook(), binding.periodMs()));
    }

    private final class MechanismControlContextImpl implements MechanismControlContext<Mechanism> {
        private double controlLoopDtSeconds = Double.NaN;
        private final TypedInputResolver inputs = new TypedInputResolver(
                "MechanismControlContext",
                TypedInputResolver.ValueMode.STRICT,
                new TypedInputResolver.MutableInputs() {
                    @Override
                    public boolean hasBool(String key) {
                        return mutableBoolInputs.containsKey(key);
                    }

                    @Override
                    public boolean bool(String key) {
                        return mutableBoolInputs.get(key);
                    }

                    @Override
                    public boolean hasDouble(String key) {
                        return mutableDoubleInputs.containsKey(key);
                    }

                    @Override
                    public double dbl(String key) {
                        return mutableDoubleInputs.get(key);
                    }

                    @Override
                    public boolean hasInt(String key) {
                        return mutableIntInputs.containsKey(key);
                    }

                    @Override
                    public int intVal(String key) {
                        return mutableIntInputs.get(key);
                    }

                    @Override
                    public boolean hasString(String key) {
                        return mutableStringInputs.containsKey(key);
                    }

                    @Override
                    public String str(String key) {
                        return mutableStringInputs.get(key);
                    }

                    @Override
                    public boolean hasPose2d(String key) {
                        return mutablePose2dInputs.containsKey(key);
                    }

                    @Override
                    public edu.wpi.first.math.geometry.Pose2d pose2d(String key) {
                        return mutablePose2dInputs.get(key);
                    }

                    @Override
                    public boolean hasPose3d(String key) {
                        return mutablePose3dInputs.containsKey(key);
                    }

                    @Override
                    public edu.wpi.first.math.geometry.Pose3d pose3d(String key) {
                        return mutablePose3dInputs.get(key);
                    }
                },
                controlLoopInputs,
                controlLoopDoubleInputs,
                controlLoopIntInputs,
                controlLoopStringInputs,
                controlLoopPose2dInputs,
                controlLoopPose3dInputs,
                controlLoopObjectInputs);

        @Override
        public Mechanism mechanism() {
            return Mechanism.this;
        }

        @Override
        public double controlLoopDtSeconds() {
            return controlLoopDtSeconds;
        }

        @Override
        public double setpoint() {
            return Mechanism.this.setpoint();
        }

        @Override
        public Enum<?> state() {
            return getActiveState();
        }

        @Override
        public boolean input(String key) {
            return inputs.boolVal(key);
        }

        @Override
        public BooleanSupplier inputSupplier(String key) {
            return inputs.boolSupplier(key);
        }

        @Override
        public double doubleInput(String key) {
            return inputs.doubleVal(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return inputs.doubleSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return inputs.intVal(key);
        }

        @Override
        public java.util.function.IntSupplier intValSupplier(String key) {
            return inputs.intSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return inputs.stringVal(key);
        }

        @Override
        public java.util.function.Supplier<String> stringValSupplier(String key) {
            return inputs.stringSupplier(key);
        }

        @Override
        public edu.wpi.first.math.geometry.Pose2d pose2dVal(String key) {
            return inputs.pose2dVal(key);
        }

        @Override
        public java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> pose2dValSupplier(String key) {
            return inputs.pose2dSupplier(key);
        }

        @Override
        public edu.wpi.first.math.geometry.Pose3d pose3dVal(String key) {
            return inputs.pose3dVal(key);
        }

        @Override
        public java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d> pose3dValSupplier(String key) {
            return inputs.pose3dSupplier(key);
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            return inputs.objectVal(key, type);
        }

        @Override
        public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            return inputs.objectSupplier(key, type);
        }

        @Override
        public PIDController pid(String name) {
            PIDController pid = controlLoopPids.get(name);
            if (pid == null) {
                if (controlLoopProfiledPids.containsKey(name)) {
                    throw new IllegalArgumentException(
                            "PID profile '" + name + "' is profiled; use profiledPid(name) or pidOut(name, measurement, setpoint)");
                }
                throw new IllegalArgumentException("No PID profile found for name " + name);
            }
            return pid;
        }

        @Override
        public ProfiledPIDController profiledPid(String name) {
            return controlLoopProfiledPids.get(name);
        }

        @Override
        public MechanismConfig.BangBangProfile bangBang(String name) {
            MechanismConfig.BangBangProfile profile = controlLoopBangBangs.get(name);
            if (profile == null) {
                throw new IllegalArgumentException("No bang-bang profile found for name " + name);
            }
            return profile;
        }

        @Override
        public SimpleMotorFeedforward feedforward(String name) {
            SimpleMotorFeedforward ff = controlLoopFeedforwards.get(name);
            if (ff == null) {
                throw new IllegalArgumentException("No feedforward profile found for name " + name);
            }
            return ff;
        }

        private void setControlLoopDtSeconds(double dtSeconds) {
            this.controlLoopDtSeconds = dtSeconds;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Simulation updates are executed within periodic() to keep sensor data fresh before control.
    }

    private Mechanism publishNetworkTablesHint(String ownerHint) {
        recordNetworkTablesRequest(ownerHint);
        publishToDefaultMechanismsNode();
        return this;
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return null;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }
        networkTablesEnabled = true;
        lastNetworkTablesNode = node;
        lastRobotNetworkTables = nt;
        networkTablesInternal(nt, node);
        return node;
    }

    private void recordNetworkTablesRequest(String ownerHint) {
        // Explicit publish request from user code: also flip the per-mech toggles on so the request
        // takes effect without hunting for another knob.
        if (robotCore != null) {
            RobotNetworkTables nt = robotCore.networkTables();
            RobotNetworkTables.Node mechNode = resolveDefaultMechanismNode(nt.root().child("Mechanisms"));
            nt.mechanismConfig(mechNode).details(true);
        }
        networkTablesPublishRequested = true;

        // For standalone mechanisms, allow the hint to act as an owner/group path.
        if (networkTablesOwnerPath == null || networkTablesOwnerPath.isBlank()) {
            setNetworkTablesOwnerPath(ownerHint);
        }
    }

    private void attemptNetworkTablesPublishIfNeeded() {
        RobotNetworkTables nt = lastRobotNetworkTables;
        if (!networkTablesEnabled || !shouldPublishMechanismNetworkTables(nt)) {
            return;
        }
        RobotNetworkTables.Node node = lastNetworkTablesNode;
        if (node == null) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        double period = getNetworkTablesPeriodSeconds();
        if (Double.isFinite(period) && period > 0.0
                && Double.isFinite(now)
                && Double.isFinite(lastNetworkTablesPublishAttemptSeconds)
                && (now - lastNetworkTablesPublishAttemptSeconds) < period
                && nt.revision() == lastNetworkTablesConfigRevision) {
            return;
        }

        lastNetworkTablesPublishAttemptSeconds = now;
        networkTablesInternal(nt, node);
    }

    private boolean shouldPublishMechanismNetworkTables(RobotNetworkTables nt) {
        if (nt == null || !nt.isPublishingEnabled()) {
            return false;
        }
        if (networkTablesPublishRequested) {
            return true;
        }
        return nt.enabled(RobotNetworkTables.Flag.AUTO_PUBLISH_MECHANISMS);
    }

    private void networkTablesInternal(RobotNetworkTables nt, RobotNetworkTables.Node node) {
        if (nt == null || node == null) {
            return;
        }
        long revision = nt.revision();
        boolean nodeChanged = (node != lastNetworkTablesNode);
        if (nodeChanged) {
            lastNetworkTablesNode = node;
            lastRobotNetworkTables = nt;
            lastNetworkTablesConfigRevision = -1;
        }
        // Always publish when we're due (handled by attemptNetworkTablesPublishIfNeeded) or when config changes.
        if (!nodeChanged && revision == lastNetworkTablesConfigRevision) {
            publishNetworkTables(node);
            return;
        }
        lastNetworkTablesConfigRevision = revision;
        publishNetworkTables(node);
    }

    private void publishToDefaultMechanismsNode() {
        if (robotCore == null) {
            return;
        }
        RobotNetworkTables nt = robotCore.networkTables();
        RobotNetworkTables.Node mechanisms = nt.root().child("Mechanisms");
        RobotNetworkTables.Node node = resolveDefaultMechanismNode(mechanisms);
        networkTables(node);
    }

    private String getNetworkTablesOwnerPath() {
        return networkTablesOwnerPath;
    }

    private void setNetworkTablesOwnerPath(String ownerPath) {
        if (ownerPath == null) {
            this.networkTablesOwnerPath = null;
            return;
        }
        String normalized = ownerPath.trim();
        if (normalized.isEmpty()) {
            this.networkTablesOwnerPath = null;
            return;
        }
        normalized = normalized.replace('\\', '/');
        normalized = normalized.replaceAll("/+", "/");
        while (normalized.startsWith("/")) {
            normalized = normalized.substring(1);
        }
        while (normalized.endsWith("/")) {
            normalized = normalized.substring(0, normalized.length() - 1);
        }
        this.networkTablesOwnerPath = normalized.isEmpty() ? null : normalized;
    }

    private String getNetworkTablesTypeName() {
        if (this instanceof TurretMechanism) {
            return "Turret";
        }
        if (this instanceof ElevatorMechanism) {
            return "Elevator";
        }
        if (this instanceof ArmMechanism) {
            return "Arm";
        }
        if (this instanceof FlywheelMechanism) {
            return "Flywheel";
        }
        if (this instanceof SimpleMotorMechanism) {
            return "Motor";
        }
        return "Mechanism";
    }

    private RobotNetworkTables.Node resolveDefaultMechanismNode(RobotNetworkTables.Node mechanismsRoot) {
        RobotNetworkTables.Node root = mechanismsRoot;
        String owner = getNetworkTablesOwnerPath();
        if (owner != null && !owner.isBlank()) {
            for (String part : owner.split("/")) {
                String seg = part != null ? part.trim() : "";
                if (!seg.isEmpty()) {
                    root = root.child(seg);
                }
            }
        }
        String name = getName();
        if (name == null || name.isBlank()) {
            name = "Mechanism";
        }
        return root.child(name);
    }

    /**
     * IntelliSense-friendly per-mechanism publishing toggles published under:
     * {@code Athena/Mechanisms/.../<MechName>/NetworkTableConfig/...}.
     */
    private final RobotNetworkTables.MechanismToggles networkTablesConfig() {
        if (robotCore == null) {
            throw new IllegalStateException("Mechanism is not registered with a RobotCore; call robot.registerMechanism(...) before using networkTablesConfig() toggles.");
        }
        RobotNetworkTables nt = robotCore.networkTables();
        RobotNetworkTables.Node mechNode = resolveDefaultMechanismNode(nt.root().child("Mechanisms"));
        return nt.mechanismConfig(mechNode);
    }

    private void publishNetworkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return;
        }

        String name = getName();
        RobotNetworkTables.Node meta = node.child("Meta");
        meta.putString("name", name != null ? name : "");
        meta.putString("type", getNetworkTablesTypeName());
        meta.putString("owner", getNetworkTablesOwnerPath() != null ? getNetworkTablesOwnerPath() : "");
        meta.putString("hint",
                "Enable " + node.path() + "/NetworkTableConfig/Details and section flags to publish more topics.");
        String jsonUrl = resolveConfigDownloadUrl("json");
        String tomlUrl = resolveConfigDownloadUrl("toml");
        String diagnosticsUrl = resolveDiagnosticsLogUrl();
        meta.putString("configUrlJson", jsonUrl != null ? jsonUrl : "");
        meta.putString("configUrlToml", tomlUrl != null ? tomlUrl : "");
        meta.putString("diagnosticsUrl", diagnosticsUrl != null ? diagnosticsUrl : "");

        RobotNetworkTables.MechanismToggles toggles = nt.mechanismConfig(node);
        boolean details = toggles.detailsEnabled();
        if (!details) {
            return;
        }

        if (toggles.motorsEnabled()) {
            motors.networkTables(node.child("Motors"));
        }

        if (encoder != null && toggles.encoderEnabled()) {
            encoder.networkTables(node.child("Encoder"));
        }

        if (toggles.constraintsEnabled()) {
            RobotNetworkTables.Node constraints = node.child("Constraints");
            double min = getMin();
            double max = getMax();
            boolean hasBounds = Double.isFinite(min) && Double.isFinite(max);
            constraints.putBoolean("hasBounds", hasBounds);
            if (hasBounds) {
                constraints.putDouble("min", min);
                constraints.putDouble("max", max);
            }
            MotionLimits.AxisLimits limits = resolveMotionLimits();
            if (limits != null) {
                constraints.putDouble("maxVelocity", limits.maxVelocity());
                constraints.putDouble("maxAcceleration", limits.maxAcceleration());
            }
        }

        if (toggles.sensorsEnabled()) {
            RobotNetworkTables.Node sensors = node.child("Sensors");
            GenericLimitSwitch[] switches = getLimitSwitches();
            if (switches != null && switches.length > 0) {
                RobotNetworkTables.Node ls = sensors.child("LimitSwitches");
                for (int i = 0; i < switches.length; i++) {
                    GenericLimitSwitch sw = switches[i];
                    if (sw == null) {
                        continue;
                    }
                    String key = "dio-" + sw.getPort();
                    RobotNetworkTables.Node swNode = ls.child(key);
                    swNode.putBoolean("active", sw.getAsBoolean());
                    swNode.putBoolean("hardstop", sw.isHardstop());
                    swNode.putDouble("position", sw.getPosition());
                    swNode.putString("blockDirection", String.valueOf(sw.getBlockDirection()));
                }
            }
        }

        if (toggles.controlEnabled()) {
            RobotNetworkTables.Node control = node.child("Control");
            RobotNetworkTables.Node status = control.child("Status");
            status.putBoolean("emergencyStopped", cachedEmergencyStopped);
            status.putBoolean("override", cachedOverride);
            status.putBoolean("atSetpoint", cachedAtSetpoint);
            status.putBoolean("pidEnabled", cachedPidEnabled);
            status.putBoolean("feedforwardEnabled", cachedFeedforwardEnabled);
            status.putString("outputType", String.valueOf(getOutputType()));
            status.putString("faultReason", cachedFaultReason != null ? cachedFaultReason : "");
            status.putString("lastFaultReason", cachedLastFaultReason != null ? cachedLastFaultReason : "");
            status.putDouble("diagnosticLogCount", getDiagnosticLogCount());
            status.putDouble("hwRefreshPeriodMs", cachedHardwareUpdatePeriodSeconds * 1000.0);
            status.putDouble("hwRefreshPhaseMs", cachedHardwareUpdatePhaseSeconds * 1000.0);
            status.putDouble("hwRefreshSlot", cachedHardwareUpdateSlot);
            boolean lastRefreshValid = Double.isFinite(cachedHardwareLastRefreshAgeSeconds);
            status.putBoolean("hwLastRefreshValid", lastRefreshValid);
            status.putDouble("hwLastRefreshAgeMs", lastRefreshValid ? cachedHardwareLastRefreshAgeSeconds * 1000.0 : -1.0);
            boolean nextRefreshValid = Double.isFinite(cachedHardwareNextRefreshInSeconds);
            status.putBoolean("hwNextRefreshValid", nextRefreshValid);
            status.putDouble("hwNextRefreshInMs", nextRefreshValid ? cachedHardwareNextRefreshInSeconds * 1000.0 : -1.0);

            RobotNetworkTables.Node setpoint = control.child("Setpoint");
            setpoint.putDouble("value", cachedSetpoint);
            setpoint.putDouble("nudge", cachedNudge);

            RobotNetworkTables.Node output = control.child("Output");
            output.putDouble("value", cachedOutput);
            output.putDouble("pid", cachedPidOutput);
            output.putDouble("feedforward", cachedFeedforwardOutput);
        }

        if (toggles.inputsEnabled()) {
            RobotNetworkTables.Node inputs = node.child("Inputs");
            RobotNetworkTables.Node bools = inputs.child("Bool");
            for (Map.Entry<String, Boolean> e : mutableBoolInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                bools.putBoolean(sanitizeTopicKey(e.getKey()), e.getValue());
            }
            for (Map.Entry<String, BooleanSupplier> e : controlLoopInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                bools.putBoolean(sanitizeTopicKey(e.getKey()), e.getValue().getAsBoolean());
            }
            RobotNetworkTables.Node dbls = inputs.child("Double");
            for (Map.Entry<String, Double> e : mutableDoubleInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                dbls.putDouble(sanitizeTopicKey(e.getKey()), e.getValue());
            }
            for (Map.Entry<String, DoubleSupplier> e : controlLoopDoubleInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                dbls.putDouble(sanitizeTopicKey(e.getKey()), e.getValue().getAsDouble());
            }
            RobotNetworkTables.Node ints = inputs.child("Int");
            for (Map.Entry<String, Integer> e : mutableIntInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                ints.putDouble(sanitizeTopicKey(e.getKey()), e.getValue());
            }
            for (Map.Entry<String, java.util.function.IntSupplier> e : controlLoopIntInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                ints.putDouble(sanitizeTopicKey(e.getKey()), e.getValue().getAsInt());
            }
            RobotNetworkTables.Node strs = inputs.child("String");
            for (Map.Entry<String, String> e : mutableStringInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                strs.putString(sanitizeTopicKey(e.getKey()), e.getValue());
            }
            for (Map.Entry<String, java.util.function.Supplier<String>> e : controlLoopStringInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                strs.putString(sanitizeTopicKey(e.getKey()), e.getValue().get());
            }
            RobotNetworkTables.Node poses2d = inputs.child("Pose2d");
            for (Map.Entry<String, edu.wpi.first.math.geometry.Pose2d> e : mutablePose2dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                edu.wpi.first.math.geometry.Pose2d pose = e.getValue();
                RobotNetworkTables.Node p = poses2d.child(sanitizeTopicKey(e.getKey()));
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("deg", pose.getRotation().getDegrees());
            }
            for (Map.Entry<String, java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d>> e : controlLoopPose2dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                edu.wpi.first.math.geometry.Pose2d pose = e.getValue().get();
                if (pose == null) {
                    continue;
                }
                RobotNetworkTables.Node p = poses2d.child(sanitizeTopicKey(e.getKey()));
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("deg", pose.getRotation().getDegrees());
            }
            RobotNetworkTables.Node poses3d = inputs.child("Pose3d");
            for (Map.Entry<String, edu.wpi.first.math.geometry.Pose3d> e : mutablePose3dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                edu.wpi.first.math.geometry.Pose3d pose = e.getValue();
                RobotNetworkTables.Node p = poses3d.child(sanitizeTopicKey(e.getKey()));
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("z", pose.getZ());
                p.putDouble("rxDeg", pose.getRotation().getX() * 180.0 / Math.PI);
                p.putDouble("ryDeg", pose.getRotation().getY() * 180.0 / Math.PI);
                p.putDouble("rzDeg", pose.getRotation().getZ() * 180.0 / Math.PI);
            }
            for (Map.Entry<String, java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d>> e : controlLoopPose3dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                edu.wpi.first.math.geometry.Pose3d pose = e.getValue().get();
                if (pose == null) {
                    continue;
                }
                RobotNetworkTables.Node p = poses3d.child(sanitizeTopicKey(e.getKey()));
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("z", pose.getZ());
                p.putDouble("rxDeg", pose.getRotation().getX() * 180.0 / Math.PI);
                p.putDouble("ryDeg", pose.getRotation().getY() * 180.0 / Math.PI);
                p.putDouble("rzDeg", pose.getRotation().getZ() * 180.0 / Math.PI);
            }
        }

        if (RobotBase.isSimulation() && toggles.simulationEnabled()) {
            RobotNetworkTables.Node simNode = node.child("Sim");
            simNode.putBoolean("enabled", cachedHasSimulation);
            simNode.putDouble("dtSec", cachedSimulationUpdatePeriodSeconds);
        }

        if (toggles.sysIdEnabled()) {
            RobotNetworkTables.Node sysid = node.child("SysId");
            sysid.putDouble("rampRateVPerSec", cachedSysIdRampRate);
            sysid.putDouble("stepVoltageV", cachedSysIdStepVoltage);
            sysid.putDouble("timeoutSec", cachedSysIdTimeoutSeconds);
            sysid.putBoolean("active", cachedSysIdActive);
        }
    }

    private static String sanitizeTopicKey(String key) {
        if (key == null) {
            return "";
        }
        String trimmed = key.trim();
        if (trimmed.isEmpty()) {
            return "";
        }
        return trimmed.replaceAll("[^A-Za-z0-9_\\-]", "_");
    }

    private String resolveConfigDownloadUrl(String ext) {
        RobotCore<?> core = robotCore;
        if (core == null) {
            return null;
        }
        String base = core.configServer().baseUrl();
        if (base == null || base.isBlank()) {
            return null;
        }
        String n = getName();
        if (n == null || n.isBlank()) {
            return null;
        }
        String encoded = java.net.URLEncoder.encode(n, java.nio.charset.StandardCharsets.UTF_8);
        // Prefer the canonical /Athena path (server also exposes /athena aliases).
        return base + "/Athena/config/mechanisms/" + encoded + "." + ext;
    }

    private String resolveDiagnosticsLogUrl() {
        RobotCore<?> core = robotCore;
        if (core == null) {
            return null;
        }
        String base = core.configServer().baseUrl();
        if (base == null || base.isBlank()) {
            return null;
        }
        String n = getName();
        if (n == null || n.isBlank()) {
            return null;
        }
        String key = "mechanisms/" + n;
        String encodedKey = java.net.URLEncoder.encode(key, java.nio.charset.StandardCharsets.UTF_8);
        return base + "/Athena/diagnostics/" + encodedKey + ".json";
    }

    private Mechanism2d getMechanism2d() {
        return visualization != null ? visualization.mechanism2d() : null;
    }

    private Map<String, Pose3d> getMechanism3dPoses() {
        return visualization != null ? visualization.poses() : Map.of();
    }


    /**
     * Overrides the root pose used for visualization. Pass {@code null} to clear and fall back
     * to the configured root pose supplier.
     */
    private void setVisualizationRootOverride(Pose3d pose) {
        this.visualizationRootOverride = pose;
    }

    private void setSimulatedEncoderState(double position, double velocity) {
        this.simEncoderOverride = true;
        this.simEncoderPosition = position;
        this.simEncoderVelocity = velocity;
    }

    private void clearSimulatedEncoderState() {
        this.simEncoderOverride = false;
    }

    private SysIdRoutine getSysIdRoutine() {
        if (sysIdRoutine == null) {
            sysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            edu.wpi.first.units.Units.Volts.of(sysIdRampRateVoltsPerSecond).per(edu.wpi.first.units.Units.Second),
                            edu.wpi.first.units.Units.Volts.of(sysIdStepVoltage),
                            edu.wpi.first.units.Units.Seconds.of(sysIdTimeoutSeconds)),
                    new SysIdRoutine.Mechanism(this::applySysIdVoltage, this::logSysIdData, this, getName()));
        }
        return sysIdRoutine;
    }

    private void invalidateSysIdRoutine() {
        sysIdRoutine = null;
    }

    private void applySysIdVoltage(Voltage voltage) {
        sysIdLastVoltage = voltage.in(edu.wpi.first.units.Units.Volts);
        setVoltage(sysIdLastVoltage);
    }

    private void logSysIdData(SysIdRoutineLog log) {
        double position = encoder != null ? getPosition() : 0.0;
        double velocity = encoder != null ? getVelocity() : 0.0;
        log.motor(getName())
                .voltage(edu.wpi.first.units.Units.Volts.of(sysIdLastVoltage))
                .value("position", position, "units")
                .value("velocity", velocity, "units/s");
    }

    private void startSysId() {
        sysIdActive = true;
        sysIdPreviousOverride = override;
        setOverride(true);
    }

    private void stopSysId() {
        sysIdActive = false;
        setOverride(sysIdPreviousOverride);
        motors.stopMotors();
    }

    private edu.wpi.first.wpilibj2.command.Command sysIdCommand(java.util.function.Supplier<edu.wpi.first.wpilibj2.command.Command> supplier) {
        return Commands.defer(
                () -> Commands.either(
                        wrapSysIdCommand(supplier.get()),
                        Commands.runOnce(() -> DriverStation.reportWarning("SysId commands require Test mode.", false)),
                        DriverStation::isTest),
                Set.of(this));
    }

    private edu.wpi.first.wpilibj2.command.Command wrapSysIdCommand(edu.wpi.first.wpilibj2.command.Command base) {
        return Commands.runOnce(this::startSysId)
                .andThen(base)
                .finallyDo(this::stopSysId);
    }
}
