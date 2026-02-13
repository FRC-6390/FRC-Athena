package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.LoopTiming;
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
    private final OutputType mainPidOutputType;
    private boolean useAbsolute;
    private OutputType outputType = OutputType.PERCENT;
    private final GenericLimitSwitch[] limitSwitches;
    private static final String MOTION_AXIS_ID = "axis";
    private final MotionLimits motionLimits;
    private final TrapezoidProfile.Constraints baseProfiledConstraints;
    private boolean override, emergencyStopped, pidEnabled, feedforwardEnabled, setpointIsOutput, suppressMotorOutput, customPIDCycle;
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
    private RobotCore<?> robotCore;
    private SysIdRoutine sysIdRoutine;
    private double sysIdRampRateVoltsPerSecond = 1.0;
    private double sysIdStepVoltage = 7.0;
    private double sysIdTimeoutSeconds = 10.0;
    private double sysIdLastVoltage = 0.0;
    private boolean sysIdActive = false;
    private boolean sysIdPreviousOverride = false;
    private final List<Consumer<?>> periodicHooks;
    private final List<PeriodicHookRunner<Mechanism>> timedPeriodicHooks;
    private final List<ControlLoopRunner<Mechanism>> controlLoops;
    private final Map<String, ControlLoopRunner<Mechanism>> controlLoopsByName;
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
    private final Map<String, OutputType> controlLoopPidOutputTypes;
    private final Map<String, SimpleMotorFeedforward> controlLoopFeedforwards;
    private final Map<String, OutputType> controlLoopFeedforwardOutputTypes;
    private final MechanismControlContextImpl controlContext = new MechanismControlContextImpl();
    private  boolean shouldCustomEncoder = false;
    private  DoubleSupplier customEncoderPos;
    private boolean shouldSetpointOverride = false;
    private double setPointOverride = 0;
    private boolean networkTablesEnabled;
    private double lastNetworkTablesCacheUpdateSeconds = Double.NaN;
    private double lastEmergencyStopLogSeconds = Double.NaN;
    private String lastEmergencyStopReason = "";
    private static final double EMERGENCY_STOP_LOG_PERIOD_SECONDS = 1.0;

    private enum RobotMode {
        TELE,
        AUTO,
        DISABLE,
        TEST,
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
        PIDController basePid = config.data().pidController();
        OutputType resolvedMainPidOutputType = this.outputType;
        if (basePid == null && config != null && config.mainPidProfileName() != null) {
            MechanismConfig.PidProfile profile = config.resolveMainPidProfile();
            if (profile == null) {
                throw new IllegalStateException("Main PID profile selected but not registered: " + config.mainPidProfileName());
            }
            OutputType profileOutput = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
            if (profileOutput != OutputType.PERCENT && profileOutput != OutputType.VOLTAGE) {
                throw new IllegalStateException("Main PID profile output type must be PERCENT or VOLTAGE");
            }
            PIDController pid = new PIDController(profile.kP(), profile.kI(), profile.kD());
            if (Double.isFinite(profile.iZone()) && profile.iZone() > 0.0) {
                pid.setIZone(profile.iZone());
            }
            if (Double.isFinite(profile.tolerance()) && profile.tolerance() > 0.0) {
                pid.setTolerance(profile.tolerance());
            }
            basePid = pid;
            resolvedMainPidOutputType = profileOutput;
        }
        this.pidController = basePid;
        this.mainPidOutputType = resolvedMainPidOutputType != null ? resolvedMainPidOutputType : this.outputType;
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
        if (config.simulationConfig != null && RobotBase.isSimulation()) {
            model = config.simulationConfig.createSimulation(this);
            updatePeriod = config.simulationConfig.updatePeriodSeconds();
        }
        this.simulationModel = model;
        this.simulationUpdatePeriodSeconds = updatePeriod;
        if (simulationModel != null) {
            simulationModel.reset();
            lastSimulationTimestampSeconds = Timer.getFPGATimestamp();
        }
        MechanismVisualizationConfig resolvedVisualizationConfig =
                config.visualizationConfig != null ? config.visualizationConfig : MechanismVisualizationDefaults.forMechanism(this);
        this.visualization = resolvedVisualizationConfig != null ? new MechanismVisualization(resolvedVisualizationConfig) : null;
        if (RobotBase.isSimulation()) {
            if (config.sensorSimulationConfig != null) {
                this.sensorSimulation = MechanismSensorSimulation.fromConfig(this, config.sensorSimulationConfig);
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
        this.periodicHooks.addAll(config.periodicHooks);
        if (config.periodicHookBindings != null) {
            for (MechanismConfig.PeriodicHookBinding<?> binding : config.periodicHookBindings) {
                registerPeriodicHook(binding);
            }
        }
        this.shouldCustomEncoder = config.shouldCustomEncoder;
        this.customEncoderPos = config.customEncoderPos;
        this.controlLoopInputs = new HashMap<>(config.inputs);
        this.controlLoopDoubleInputs = new HashMap<>(config.doubleInputs);
        this.controlLoopIntInputs = new HashMap<>(config.intInputs);
        this.controlLoopStringInputs = new HashMap<>(config.stringInputs);
        this.controlLoopPose2dInputs = new HashMap<>(config.pose2dInputs);
        this.controlLoopPose3dInputs = new HashMap<>(config.pose3dInputs);
        this.controlLoopObjectInputs = new HashMap<>(config.objectInputs);
        this.mutableBoolInputDefaults = new HashMap<>(config.mutableBoolInputDefaults);
        this.mutableBoolInputs = new HashMap<>(this.mutableBoolInputDefaults);
        this.mutableDoubleInputDefaults = new HashMap<>(config.mutableDoubleInputDefaults);
        this.mutableDoubleInputs = new HashMap<>(this.mutableDoubleInputDefaults);
        this.mutableIntInputDefaults = new HashMap<>(config.mutableIntInputDefaults);
        this.mutableIntInputs = new HashMap<>(this.mutableIntInputDefaults);
        this.mutableStringInputDefaults = new HashMap<>(config.mutableStringInputDefaults);
        this.mutableStringInputs = new HashMap<>(this.mutableStringInputDefaults);
        this.mutablePose2dInputDefaults = new HashMap<>(config.mutablePose2dInputDefaults);
        this.mutablePose2dInputs = new HashMap<>(this.mutablePose2dInputDefaults);
        this.mutablePose3dInputDefaults = new HashMap<>(config.mutablePose3dInputDefaults);
        this.mutablePose3dInputs = new HashMap<>(this.mutablePose3dInputDefaults);
        this.controlLoopPids = new HashMap<>();
        this.controlLoopPidOutputTypes = new HashMap<>();
        this.controlLoopFeedforwards = new HashMap<>();
        this.controlLoopFeedforwardOutputTypes = new HashMap<>();
        this.controlLoops = new ArrayList<>();
        this.controlLoopsByName = new HashMap<>();
        this.disabledControlLoops = new HashSet<>();
        if (config.controlLoopPidProfiles != null) {
            for (Map.Entry<String, MechanismConfig.PidProfile> entry : config.controlLoopPidProfiles.entrySet()) {
                String name = entry.getKey();
                MechanismConfig.PidProfile profile = entry.getValue();
                if (name == null || name.isBlank() || profile == null) {
                    continue;
                }
                OutputType profileOutput = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
                if (profileOutput != OutputType.PERCENT && profileOutput != OutputType.VOLTAGE) {
                    throw new IllegalStateException("PID profile '" + name + "' output type must be PERCENT or VOLTAGE");
                }
                PIDController pid = new PIDController(profile.kP(), profile.kI(), profile.kD());
                if (Double.isFinite(profile.iZone()) && profile.iZone() > 0.0) {
                    pid.setIZone(profile.iZone());
                }
                if (Double.isFinite(profile.tolerance()) && profile.tolerance() > 0.0) {
                    pid.setTolerance(profile.tolerance());
                }
                controlLoopPids.put(name, pid);
                controlLoopPidOutputTypes.put(name, profileOutput);
            }
        }
        if (config.controlLoopFeedforwardProfiles != null) {
            for (Map.Entry<String, MechanismConfig.FeedforwardProfile> entry : config.controlLoopFeedforwardProfiles.entrySet()) {
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
        for (MechanismConfig.ControlLoopBinding<?> binding : config.controlLoops) {
            registerControlLoop(binding);
        }

    }

    public MechanismConfig<? extends Mechanism> getSourceConfig() {
        return sourceConfig;
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
        if (motors == null || config.type == null) {
            return null;
        }
        String key = config.type.getKey();
        if (!isIntegratedEncoderKey(key)) {
            return null;
        }
        int encoderId = config.id;
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
        this.mainPidOutputType = this.outputType;
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
        this.controlLoopPidOutputTypes = new HashMap<>();
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

    public void setBoolVal(String key, boolean value) {
        requireMutableKey(key, mutableBoolInputDefaults);
        mutableBoolInputs.put(key, value);
    }

    public void resetBoolVal(String key) {
        requireMutableKey(key, mutableBoolInputDefaults);
        mutableBoolInputs.put(key, mutableBoolInputDefaults.get(key));
    }

    public void setDoubleVal(String key, double value) {
        requireMutableKey(key, mutableDoubleInputDefaults);
        mutableDoubleInputs.put(key, value);
    }

    public void resetDoubleVal(String key) {
        requireMutableKey(key, mutableDoubleInputDefaults);
        mutableDoubleInputs.put(key, mutableDoubleInputDefaults.get(key));
    }

    public void setIntVal(String key, int value) {
        requireMutableKey(key, mutableIntInputDefaults);
        mutableIntInputs.put(key, value);
    }

    public void resetIntVal(String key) {
        requireMutableKey(key, mutableIntInputDefaults);
        mutableIntInputs.put(key, mutableIntInputDefaults.get(key));
    }

    public void setStringVal(String key, String value) {
        requireMutableKey(key, mutableStringInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("string input value cannot be null");
        }
        mutableStringInputs.put(key, value);
    }

    public void resetStringVal(String key) {
        requireMutableKey(key, mutableStringInputDefaults);
        mutableStringInputs.put(key, mutableStringInputDefaults.get(key));
    }

    public void setPose2dVal(String key, edu.wpi.first.math.geometry.Pose2d value) {
        requireMutableKey(key, mutablePose2dInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("Pose2d input value cannot be null");
        }
        mutablePose2dInputs.put(key, value);
    }

    public void resetPose2dVal(String key) {
        requireMutableKey(key, mutablePose2dInputDefaults);
        mutablePose2dInputs.put(key, mutablePose2dInputDefaults.get(key));
    }

    public void setPose3dVal(String key, edu.wpi.first.math.geometry.Pose3d value) {
        requireMutableKey(key, mutablePose3dInputDefaults);
        if (value == null) {
            throw new IllegalArgumentException("Pose3d input value cannot be null");
        }
        mutablePose3dInputs.put(key, value);
    }

    public void resetPose3dVal(String key) {
        requireMutableKey(key, mutablePose3dInputDefaults);
        mutablePose3dInputs.put(key, mutablePose3dInputDefaults.get(key));
    }

    public MotionLimits getMotionLimits() {
        return motionLimits;
    }

    public Mechanism setMotionLimits(MotionLimits.AxisLimits limits) {
        motionLimits.setBaseAxisLimits(MOTION_AXIS_ID, limits);
        return this;
    }

    public Mechanism setMotionLimits(double maxVelocity, double maxAcceleration) {
        return setMotionLimits(new MotionLimits.AxisLimits(maxVelocity, maxAcceleration));
    }

    public Mechanism registerMotionLimitsProvider(MotionLimits.AxisLimitsProvider provider) {
        if (provider != null) {
            motionLimits.registerAxisProvider(MOTION_AXIS_ID, provider);
        }
        return this;
    }

    public MotionLimits.AxisLimits resolveMotionLimits() {
        return motionLimits.resolveAxisLimits(MOTION_AXIS_ID);
    }

    public double getHardwareUpdatePeriodSeconds() {
        return hardwareUpdatePeriodSeconds;
    }

    public void setHardwareUpdatePeriodSeconds(double periodSeconds) {
        this.hardwareUpdatePeriodSeconds = sanitizeHardwareUpdatePeriod(periodSeconds);
        recomputeHardwareRefreshPhase();
    }

    public void setHardwareUpdatePeriodMs(double periodMs) {
        setHardwareUpdatePeriodSeconds(periodMs / 1000.0);
    }

    public void setRobotCore(RobotCore<?> robotCore) {
        this.robotCore = robotCore;
    }

    public RobotCore<?> getRobotCore() {
        return robotCore;
    }

    public double getSysIdRampRateVoltsPerSecond() {
        return sysIdRampRateVoltsPerSecond;
    }

    public void setSysIdRampRateVoltsPerSecond(double rampRate) {
        if (!Double.isFinite(rampRate) || rampRate <= 0.0) {
            return;
        }
        sysIdRampRateVoltsPerSecond = rampRate;
        invalidateSysIdRoutine();
    }

    public double getSysIdStepVoltage() {
        return sysIdStepVoltage;
    }

    public void setSysIdStepVoltage(double stepVoltage) {
        if (!Double.isFinite(stepVoltage) || stepVoltage <= 0.0) {
            return;
        }
        sysIdStepVoltage = stepVoltage;
        invalidateSysIdRoutine();
    }

    public double getSysIdTimeoutSeconds() {
        return sysIdTimeoutSeconds;
    }

    public void setSysIdTimeoutSeconds(double timeoutSeconds) {
        if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
            return;
        }
        sysIdTimeoutSeconds = timeoutSeconds;
        invalidateSysIdRoutine();
    }

    public boolean isSysIdActive() {
        return sysIdActive;
    }

    public double calculateMaxFreeSpeed(){
        if (encoder == null) {
            return 0;
        }
        double wheelCircumferenceMeters = Math.PI * encoder.getConversion();
        double motorRPM = 6000; //motors.getControllers()[0].getFreeSpeedRPM();
        double wheelRPM = motorRPM * encoder.getGearRatio();
        return wheelCircumferenceMeters * (wheelRPM / 60.0);
    }

    public void setVoltage(double voltage){
        recordOutput(voltage, true);
        if (emergencyStopped || suppressMotorOutput) {
            motors.stopMotors();
            return;
        }
        motors.setVoltage(voltage);
    }

    public void setSpeed(double speed){
        recordOutput(speed, false);
        if (emergencyStopped || suppressMotorOutput) {
            motors.stopMotors();
            return;
        }
        motors.setSpeed(speed);
    }

    /**
     * Sends a manual output in the currently configured output space.
     */
    public void setOutput(double output) {
        setOverride(true);
        switch (outputType) {
            case VOLTAGE -> setVoltage(output);
            case POSITION -> {
                recordOutput(output, false);
                if (emergencyStopped || suppressMotorOutput) {
                    motors.stopMotors();
                    return;
                }
                motors.setPosition(output);
            }
            case VELOCITY -> {
                recordOutput(output, false);
                if (emergencyStopped || suppressMotorOutput) {
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
    public void setMotors(double speed){
        setOverride(true);
        setSpeed(speed);
    }

    public MotorControllerGroup getMotorGroup(){
        return motors;
    }

    public Encoder getEncoder(){
       return encoder;
    }

    public Rotation2d getRotation2d(){
        if (encoder == null) return Rotation2d.kZero;
        return useAbsolute ? getEncoder().getAbsoluteRotation2d() : getEncoder().getRotation2d();
    }

    public Rotation2d getRotation2d(boolean poll){
        if (encoder == null) return Rotation2d.kZero;
        return useAbsolute ? getEncoder().getAbsoluteRotation2d(poll) : getEncoder().getRotation2d(poll);
    }

    public double getPosition(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderPosition;
        }
        if (encoder == null) return 0;
        return useAbsolute ? getEncoder().getAbsolutePosition() : getEncoder().getPosition();
    }

    public double getPosition(boolean poll){
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
    public double getPositionModulus(double min, double max) {
        return MathUtil.inputModulus(getPosition(), min, max);
    }

    public double getVelocity(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderVelocity;
        }
        if (encoder == null) return 0;
        return getEncoder().getVelocity();
    }

    public double getVelocity(boolean poll){
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

    public void setSetpoint(double setpoint){
        this.setpoint = applyBounds(setpoint);
    }

    public double getSetpoint(){
        return setpoint;
    }

    protected double getBaseSetpoint() {
        return getSetpoint();
    }

    protected Enum<?> getActiveState() {
        return null;
    }

    /**
     * Clamps all setpoints to the provided bounds.
     */
    public Mechanism setBounds(double min, double max) {
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
    public Mechanism clearBounds() {
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

    public double getControllerSetpointVelocity(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().velocity : pidController != null ? pidController.getSetpoint() : 0;
    }

    public double getControllerSetpointPosition(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().position : pidController != null ? pidController.getSetpoint() : 0;
    }

    /**
     * Measurement used by PID loops. Defaults to position; mechanisms may override.
     */
    protected double getPidMeasurement() {
        
        return shouldCustomEncoder ? customEncoderPos.getAsDouble() : getPosition();

    }



    public boolean isOverride() {
        return override;
    }

    public boolean isEmergencyStopped() {
        return emergencyStopped;
    }

    public void setOverride(boolean override) {
        this.override = override;
        if (override) {
            manualOutputActive = false;
            manualOutput = 0.0;
            manualOutputIsVoltage = false;
        }
    }

    public boolean isUseAbsolute() {
        return useAbsolute;
    }

    public void setUseAbsolute(boolean useAbsolute) {
        this.useAbsolute = useAbsolute;
    }

    public boolean isUseVoltage() {
        return outputType == OutputType.VOLTAGE;
    }

    public OutputType getOutputType() {
        return outputType;
    }

    public void setOutputType(OutputType outputType) {
        if (outputType == null) {
            return;
        }
        this.outputType = outputType;
    }

    protected double percentToOutput(double percent) {
        if (outputType == OutputType.VOLTAGE) {
            return percent * RobotController.getBatteryVoltage();
        }
        return percent;
    }

    protected double voltsToOutput(double volts) {
        if (outputType == OutputType.VOLTAGE) {
            return volts;
        }
        double vbat = RobotController.getBatteryVoltage();
        if (!Double.isFinite(vbat) || vbat == 0.0) {
            return 0.0;
        }
        return volts / vbat;
    }

    /**
     * Converts a value from the given output type into this mechanism's output space.
     */
    protected double toOutput(OutputType type, double value) {
        if (type == null || type == outputType) {
            return value;
        }
        return switch (type) {
            case VOLTAGE -> voltsToOutput(value);
            case PERCENT -> percentToOutput(value);
            case POSITION, VELOCITY -> value;
        };
    }

    public OutputType getControlLoopPidOutputType(String name) {
        if (name == null) {
            return OutputType.PERCENT;
        }
        return controlLoopPidOutputTypes.getOrDefault(name, OutputType.PERCENT);
    }

    public OutputType getControlLoopFeedforwardOutputType(String name) {
        if (name == null) {
            return OutputType.VOLTAGE;
        }
        return controlLoopFeedforwardOutputTypes.getOrDefault(name, OutputType.VOLTAGE);
    }

    public boolean isSetpointAsOutput() {
        return setpointIsOutput;
    }

    public void setSetpointAsOutput(boolean setpointAsOutput) {
        this.setpointIsOutput = setpointAsOutput;
    }

    public void setPidPeriod(double pidPeriod) {
        if (!Double.isFinite(pidPeriod)) {
            return;
        }
        this.pidPeriod = pidPeriod;
    }
    public boolean isOutputVoltage() {
        if (override && manualOutputActive) {
            return manualOutputIsVoltage;
        }
        if (RobotBase.isSimulation() && lastOutputValid) {
            return lastOutputIsVoltage;
        }
        return isUseVoltage();
    }

    public boolean isCustomPIDCycle() {
        return customPIDCycle;
    }

    public void setCustomPIDCycle(boolean customPIDCycle) {
        this.customPIDCycle = customPIDCycle;
    }

    public double calculateFeedForward(){
        return 0;
    }

    public void setEncoderPosition(double position){
        if(encoder != null){
            encoder.setPosition(position);
        }
    }

    public void resetPID(){
        if(pidController != null) pidController.reset();
        if(profiledPIDController != null) profiledPIDController.reset(getPosition(), getVelocity());
        this.output = 0;
    }

    public PIDController getPIDController()
    {
        return pidController;
    }


    public void setSetpointOverride(boolean should)
    {
        shouldSetpointOverride = should;
    }

    public void setPointOverride(double setpoint)
    {
        setPointOverride = applyBounds(setpoint);
    }

    public double getMin()
    {
        return minBound;
    }
    public double getMax()
    {
        return maxBound;
    }

    public double calculatePID(){
        double output = 0;
        double encoderPos = getPidMeasurement();
        applyMotionLimits();

        if (pidController != null){
            double setpoint = shouldSetpointOverride ? setPointOverride : getSetpoint();
            output += toOutput(mainPidOutputType, pidController.calculate(encoderPos, setpoint + getNudge()));
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
        if (!suppressMotorOutput){
            switch (outputType) {
                case VOLTAGE -> setVoltage(output);
                case POSITION -> motors.setPosition(output);
                case VELOCITY -> motors.setVelocity(output);
                case PERCENT -> setSpeed(output);
            }
        }
    }

    public void setSuppressMotorOutput(boolean suppressMotorOutput) {
        this.suppressMotorOutput = suppressMotorOutput;
    }

    public void updatePID(){
        pidOutput = calculatePID();
        feedforwardOutput = calculateFeedForward();
    }

    public void update(){

        double output = 0;

        boolean encoderDisconnected = encoder != null && !encoder.isConnected();
        boolean motorsDisconnected = !motors.allMotorsConnected();
        if (encoderDisconnected || motorsDisconnected) {
            emergencyStopped = true;
            logEmergencyStopReason(encoderDisconnected, motorsDisconnected);
        }

        if(!customPIDCycle) updatePID();

        output = isPidEnabled() ? pidOutput : 0;
        output += isFeedforwardEnabled() ? feedforwardOutput : 0;

        if (setpointIsOutput){
            output = getSetpoint() + getNudge();
        }

        output += calculateControlLoopOutput();

        this.output = output;

        this.prevSetpoint = shouldSetpointOverride ? setPointOverride : getSetpoint();

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
        String reasonText = reason.toString();
        double now = Timer.getFPGATimestamp();
        boolean shouldLog = Double.isNaN(lastEmergencyStopLogSeconds)
                || (now - lastEmergencyStopLogSeconds) >= EMERGENCY_STOP_LOG_PERIOD_SECONDS
                || !reasonText.equals(lastEmergencyStopReason);
        if (shouldLog) {
            System.out.println("[Athena][EmergencyStop] " + getName() + ": " + reasonText);
            lastEmergencyStopLogSeconds = now;
            lastEmergencyStopReason = reasonText;
        }
    }

    public GenericLimitSwitch[] getLimitSwitches() {
        return limitSwitches;
    }

    public void setCurrentLimit(double currentLimit){
        motors.setCurrentLimit(currentLimit);
    }

    public void setMotorNeutralMode(MotorNeutralMode mode){
        motors.setNeutralMode(mode);
    }

    public boolean isFeedforwardEnabled() {
        return feedforwardEnabled;
    }

    public boolean isPidEnabled() {
        return pidEnabled;
    }

    public double getPidOutput() {
        return pidOutput;
    }

    public double getFeedforwardOutput() {
        return feedforwardOutput;
    }

    public void setFeedforwardEnabled(boolean feedforwardEnabled) {
        this.feedforwardEnabled = feedforwardEnabled;
    }

    public void setPidEnabled(boolean pidEnabled) {
        this.pidEnabled = pidEnabled;
    }

    public void setEmergencyStopped(boolean emergancyStopped) {
        this.emergencyStopped = emergancyStopped;
    }

    public double getOutput() {
        if (override) {
            return manualOutputActive ? manualOutput : 0.0;
        }
        if (RobotBase.isSimulation() && lastOutputValid) {
            return lastOutput;
        }
        return output;
    }

    public void setNudge(double nudge){
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

    public double getNudge() {
        return nudge;
    }

    public double getPidPeriod() {
        return pidPeriod;
    }

    public double getNetworkTablesPeriodSeconds() {
        RobotNetworkTables nt = lastRobotNetworkTables;
        if (nt == null && robotCore != null) {
            nt = robotCore.networkTables();
        }
        double defaultPeriod = nt != null ? nt.getDefaultPeriodSeconds() : 1.0;
        return Double.isFinite(networkTablesPeriodSecondsOverride) && networkTablesPeriodSecondsOverride > 0.0
                ? networkTablesPeriodSecondsOverride
                : defaultPeriod;
    }

    public void setNetworkTablesPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            // Reset to global default.
            this.networkTablesPeriodSecondsOverride = Double.NaN;
            return;
        }
        this.networkTablesPeriodSecondsOverride = periodSeconds;
    }

    public void disableControlLoop(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return;
        }
        disabledControlLoops.add(name);
    }

    public void enableControlLoop(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return;
        }
        disabledControlLoops.remove(name);
    }

    public void setControlLoopEnabled(String name, boolean enabled) {
        if (enabled) {
            enableControlLoop(name);
        } else {
            disableControlLoop(name);
        }
    }

    public boolean isControlLoopEnabled(String name) {
        if (name == null || !controlLoopsByName.containsKey(name)) {
            return false;
        }
        return !disabledControlLoops.contains(name);
    }

    public boolean hasSimulation() {
        return simulationModel != null;
    }

    public double getSimulationUpdatePeriodSeconds() {
        return simulationUpdatePeriodSeconds;
    }

    public void loadConfig(Path path) {
        MechanismConfigIO.apply(this, MechanismConfigIO.load(path));
    }

    public void saveConfig(Path path) {
        MechanismConfigIO.save(path, MechanismConfigIO.snapshot(this));
    }

    public void resetSimulation() {
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
            double nowMs = nowSeconds * 1000.0;
            for (PeriodicHookRunner<Mechanism> runner : timedPeriodicHooks) {
                if (runner.shouldRun(nowMs)) {
                    runner.run(this, nowMs);
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
    }

    private double calculateControlLoopOutput() {
        if (controlLoops.isEmpty()) {
            return 0.0;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        double total = 0.0;
        for (ControlLoopRunner<Mechanism> runner : controlLoops) {
            if (!isControlLoopEnabled(runner.name())) {
                runner.setLastOutput(0.0);
                continue;
            }
            if (runner.shouldRun(nowSeconds)) {
                double dtSeconds;
                if (Double.isNaN(runner.lastRunSeconds)) {
                    dtSeconds = runner.periodSeconds();
                } else {
                    dtSeconds = nowSeconds - runner.lastRunSeconds;
                }
                if (!Double.isFinite(dtSeconds) || dtSeconds < 0.0) {
                    dtSeconds = Double.NaN;
                }
                controlContext.setControlLoopDtSeconds(dtSeconds);
                runner.setLastOutput(runner.loop().calculate(controlContext));
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
        if (nt == null || !nt.isPublishingEnabled()) {
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
        ControlLoopRunner<Mechanism> runner = new ControlLoopRunner<>(
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
        timedPeriodicHooks.add(new PeriodicHookRunner<>((Consumer<Mechanism>) binding.hook(), binding.periodMs()));
    }

    private final class MechanismControlContextImpl implements MechanismControlContext<Mechanism> {
        private double controlLoopDtSeconds = Double.NaN;

        @Override
        public Mechanism mechanism() {
            return Mechanism.this;
        }

        @Override
        public double controlLoopDtSeconds() {
            return controlLoopDtSeconds;
        }

        @Override
        public double baseSetpoint() {
            return getBaseSetpoint();
        }

        @Override
        public Enum<?> state() {
            return getActiveState();
        }

        @Override
        public boolean input(String key) {
            if (mutableBoolInputs.containsKey(key)) {
                return mutableBoolInputs.get(key);
            }
            BooleanSupplier supplier = controlLoopInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No bool input found for key " + key);
            }
            return supplier.getAsBoolean();
        }

        @Override
        public BooleanSupplier inputSupplier(String key) {
            if (mutableBoolInputs.containsKey(key)) {
                return () -> mutableBoolInputs.get(key);
            }
            BooleanSupplier supplier = controlLoopInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No input found for key " + key);
            }
            return supplier;
        }

        @Override
        public double doubleInput(String key) {
            if (mutableDoubleInputs.containsKey(key)) {
                return mutableDoubleInputs.get(key);
            }
            DoubleSupplier supplier = controlLoopDoubleInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No double input found for key " + key);
            }
            return supplier.getAsDouble();
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            if (mutableDoubleInputs.containsKey(key)) {
                return () -> mutableDoubleInputs.get(key);
            }
            DoubleSupplier supplier = controlLoopDoubleInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No double input found for key " + key);
            }
            return supplier;
        }

        @Override
        public int intVal(String key) {
            if (mutableIntInputs.containsKey(key)) {
                return mutableIntInputs.get(key);
            }
            java.util.function.IntSupplier supplier = controlLoopIntInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No int input found for key " + key);
            }
            return supplier.getAsInt();
        }

        @Override
        public java.util.function.IntSupplier intValSupplier(String key) {
            if (mutableIntInputs.containsKey(key)) {
                return () -> mutableIntInputs.get(key);
            }
            java.util.function.IntSupplier supplier = controlLoopIntInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No int input found for key " + key);
            }
            return supplier;
        }

        @Override
        public String stringVal(String key) {
            if (mutableStringInputs.containsKey(key)) {
                return mutableStringInputs.get(key);
            }
            java.util.function.Supplier<String> supplier = controlLoopStringInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No string input found for key " + key);
            }
            return supplier.get();
        }

        @Override
        public java.util.function.Supplier<String> stringValSupplier(String key) {
            if (mutableStringInputs.containsKey(key)) {
                return () -> mutableStringInputs.get(key);
            }
            java.util.function.Supplier<String> supplier = controlLoopStringInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No string input found for key " + key);
            }
            return supplier;
        }

        @Override
        public edu.wpi.first.math.geometry.Pose2d pose2dVal(String key) {
            if (mutablePose2dInputs.containsKey(key)) {
                return mutablePose2dInputs.get(key);
            }
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> supplier = controlLoopPose2dInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No Pose2d input found for key " + key);
            }
            return supplier.get();
        }

        @Override
        public java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> pose2dValSupplier(String key) {
            if (mutablePose2dInputs.containsKey(key)) {
                return () -> mutablePose2dInputs.get(key);
            }
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> supplier = controlLoopPose2dInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No Pose2d input found for key " + key);
            }
            return supplier;
        }

        @Override
        public edu.wpi.first.math.geometry.Pose3d pose3dVal(String key) {
            if (mutablePose3dInputs.containsKey(key)) {
                return mutablePose3dInputs.get(key);
            }
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d> supplier = controlLoopPose3dInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No Pose3d input found for key " + key);
            }
            return supplier.get();
        }

        @Override
        public java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d> pose3dValSupplier(String key) {
            if (mutablePose3dInputs.containsKey(key)) {
                return () -> mutablePose3dInputs.get(key);
            }
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d> supplier = controlLoopPose3dInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No Pose3d input found for key " + key);
            }
            return supplier;
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            Supplier<?> supplier = controlLoopObjectInputs.get(key);
            if (supplier == null) {
                return null;
            }
            Object value = supplier.get();
            if (value == null) {
                return null;
            }
            if (!type.isInstance(value)) {
                throw new IllegalArgumentException("Input '" + key + "' is not of type " + type.getSimpleName());
            }
            return type.cast(value);
        }

        @Override
        public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            Supplier<?> supplier = controlLoopObjectInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No object input found for key " + key);
            }
            return () -> objectInput(key, type);
        }

        @Override
        public PIDController pid(String name) {
            PIDController pid = controlLoopPids.get(name);
            if (pid == null) {
                throw new IllegalArgumentException("No PID profile found for name " + name);
            }
            return pid;
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

    private static final class ControlLoopRunner<M extends Mechanism> {
        private final String name;
        private final double periodSeconds;
        private final MechanismConfig.MechanismControlLoop<M> loop;
        private double lastRunSeconds = Double.NaN;
        private double lastOutput;

        private ControlLoopRunner(String name, double periodSeconds, MechanismConfig.MechanismControlLoop<M> loop) {
            this.name = name;
            this.periodSeconds = Math.max(0.0, periodSeconds);
            this.loop = loop;
        }

        private String name() {
            return name;
        }

        private MechanismConfig.MechanismControlLoop<M> loop() {
            return loop;
        }

        private double lastOutput() {
            return lastOutput;
        }

        private void setLastOutput(double output) {
            this.lastOutput = output;
        }

        private void setLastRunSeconds(double nowSeconds) {
            this.lastRunSeconds = nowSeconds;
        }

        private double periodSeconds() {
            return periodSeconds;
        }

        private boolean shouldRun(double nowSeconds) {
            if (!Double.isFinite(nowSeconds)) {
                return true;
            }
            if (Double.isNaN(lastRunSeconds)) {
                return true;
            }
            if (periodSeconds <= 0.0) {
                return true;
            }
            return (nowSeconds - lastRunSeconds) >= periodSeconds;
        }
    }

    private static final class PeriodicHookRunner<M extends Mechanism> {
        private final Consumer<M> hook;
        private final double periodMs;
        private double lastRunMs = Double.NaN;

        private PeriodicHookRunner(Consumer<M> hook, double periodMs) {
            this.hook = hook;
            this.periodMs = Math.max(0.0, periodMs);
        }

        private boolean shouldRun(double nowMs) {
            if (!Double.isFinite(nowMs)) {
                return true;
            }
            if (Double.isNaN(lastRunMs)) {
                return true;
            }
            if (periodMs <= 0.0) {
                return true;
            }
            return (nowMs - lastRunMs) >= periodMs;
        }

        private void run(M mechanism, double nowMs) {
            hook.accept(mechanism);
            lastRunMs = nowMs;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Simulation updates are executed within periodic() to keep sensor data fresh before control.
    }

    public Mechanism publishNetworkTables(String ownerHint) {
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
        if (nt == null || !nt.isPublishingEnabled() || !networkTablesEnabled) {
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

    public String getNetworkTablesOwnerPath() {
        return networkTablesOwnerPath;
    }

    public void setNetworkTablesOwnerPath(String ownerPath) {
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

    public String getNetworkTablesTypeName() {
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

    public RobotNetworkTables.Node resolveDefaultMechanismNode(RobotNetworkTables.Node mechanismsRoot) {
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
    public final RobotNetworkTables.MechanismToggles networkTablesConfig() {
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
                "Enable " + node.path() + "/NetworkTableConfig/Details (and .../Advanced) to publish more topics.");
        String jsonUrl = resolveConfigDownloadUrl("json");
        String tomlUrl = resolveConfigDownloadUrl("toml");
        meta.putString("configUrlJson", jsonUrl != null ? jsonUrl : "");
        meta.putString("configUrlToml", tomlUrl != null ? tomlUrl : "");

        RobotNetworkTables.MechanismToggles toggles = nt.mechanismConfig(node);
        boolean details = toggles.detailsEnabled();
        boolean advanced = toggles.advancedEnabled();
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
                    if (advanced) {
                        swNode.putBoolean("hardstop", sw.isHardstop());
                        swNode.putDouble("position", sw.getPosition());
                        swNode.putString("blockDirection", String.valueOf(sw.getBlockDirection()));
                    }
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
            if (advanced) {
                setpoint.putDouble("nudge", cachedNudge);
            }

            RobotNetworkTables.Node output = control.child("Output");
            output.putDouble("value", cachedOutput);
            if (advanced) {
                output.putDouble("pid", cachedPidOutput);
                output.putDouble("feedforward", cachedFeedforwardOutput);
            }
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
            if (advanced) {
                simNode.putDouble("dtSec", cachedSimulationUpdatePeriodSeconds);
            }
        }

        if (advanced && toggles.sysIdEnabled()) {
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
        String base = core.getConfigServerBaseUrl();
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

    public Mechanism2d getMechanism2d() {
        return visualization != null ? visualization.mechanism2d() : null;
    }

    public Map<String, Pose3d> getMechanism3dPoses() {
        return visualization != null ? visualization.poses() : Map.of();
    }


    /**
     * Overrides the root pose used for visualization. Pass {@code null} to clear and fall back
     * to the configured root pose supplier.
     */
    public void setVisualizationRootOverride(Pose3d pose) {
        this.visualizationRootOverride = pose;
    }

    public void setSimulatedEncoderState(double position, double velocity) {
        this.simEncoderOverride = true;
        this.simEncoderPosition = position;
        this.simEncoderVelocity = velocity;
    }

    public void clearSimulatedEncoderState() {
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
