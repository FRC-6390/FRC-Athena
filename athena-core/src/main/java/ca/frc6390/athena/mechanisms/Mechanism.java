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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Mechanism extends SubsystemBase implements RobotSendableSystem, RegisterableMechanism{

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
    private boolean override, emergencyStopped, pidEnabled, feedforwardEnabled, setpointIsOutput, suppressMotorOutput, customPIDCycle;
    private double setpoint, prevSetpoint, pidOutput, feedforwardOutput, output, nudge, pidPeriod; 
    private RobotMode prevRobotMode = RobotMode.DISABLE, robotMode = RobotMode.DISABLE;
    private final MechanismSimulationModel simulationModel;
    private final double simulationUpdatePeriodSeconds;
    private double lastSimulationTimestampSeconds = Double.NaN;
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
    private double shuffleboardPeriodSeconds = RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
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
    private final Map<String, Supplier<?>> controlLoopObjectInputs;
    private final MechanismControlContextImpl controlContext = new MechanismControlContextImpl();
    private  boolean shouldCustomEncoder = false;
    private  DoubleSupplier customEncoderPos;
    private boolean shouldSetpointOverride = false;
    private double setPointOverride = 0;
    private boolean shuffleboardEnabled;
    private double lastShuffleboardCacheUpdateSeconds = Double.NaN;
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
        this.motors =
                MotorControllerGroup.fromConfigs(config.data().motors().toArray(MotorControllerConfig[]::new));
        this.encoder = resolveEncoder(config.data().encoder(), this.motors);
        this.pidController = config.data().pidController();
        this.profiledPIDController = config.data().profiledPIDController();
        this.useAbsolute = config.data().useAbsolute();
        OutputType configOutputType = config.data().outputType();
        if (configOutputType == null) {
            this.outputType = config.data().useVoltage() ? OutputType.VOLTAGE : OutputType.PERCENT;
        } else {
            this.outputType = configOutputType;
        }
        this.override = false;
        this.limitSwitches =
                config.data().limitSwitches().stream().map(GenericLimitSwitch::fromConfig).toArray(GenericLimitSwitch[]::new);
        this.setpointIsOutput = config.data().useSetpointAsOutput();
        this.pidPeriod = config.data().pidPeriod();
        this.motionLimits = new MotionLimits();

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
        this.controlLoopObjectInputs = new HashMap<>(config.objectInputs);
        this.controlLoops = new ArrayList<>();
        this.controlLoopsByName = new HashMap<>();
        this.disabledControlLoops = new HashSet<>();
        for (MechanismConfig.ControlLoopBinding<?> binding : config.controlLoops) {
            registerControlLoop(binding);
        }

    }

    private static Encoder resolveEncoder(EncoderConfig config, MotorControllerGroup motors) {
        if (config == null) {
            return null;
        }
        Encoder integrated = resolveIntegratedEncoder(config, motors);
        return integrated != null ? integrated : HardwareFactories.encoder(config);
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
        this.controlLoopObjectInputs = new HashMap<>();
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

    public double getPosition(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderPosition;
        }
        if (encoder == null) return 0;
        return useAbsolute ? getEncoder().getAbsolutePosition() : getEncoder().getPosition();
    }

    public double getVelocity(){
        if (RobotBase.isSimulation() && simEncoderOverride && (encoder == null || !encoder.supportsSimulation())) {
            return simEncoderVelocity;
        }
        if (encoder == null) return 0;
        return getEncoder().getVelocity();
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

    public void setSetpointOverride(boolean should)
    {
        shouldSetpointOverride = should;
    }

    public void setPointOverride(double setpoint)
    {
        setPointOverride = setpoint;
    }

    public double calculatePID(){
        double output = 0;
        double encoderPos = getPidMeasurement();
        applyMotionLimits();

        if (pidController != null){
            double setpoint = shouldSetpointOverride ? setPointOverride : getSetpoint();
            output += pidController.calculate(encoderPos, setpoint + getNudge());
        }

        if(profiledPIDController != null){
            if(prevSetpoint != getSetpoint()) resetPID();
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

        boolean encoderDisconnected = encoder != null && !encoder.isCachedConnected();
        boolean motorsDisconnected = !motors.allMotorsCachedConnected();
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

        if(emergencyStopped){
            motors.stopMotors();
            return;
        }

        if (override){
            return;
        }

        for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
            if(genericLimitSwitch.getAsBoolean()){

                if (genericLimitSwitch.isHardstop()){
                    if(Math.signum(genericLimitSwitch.getBlockDirection()) == Math.signum(output)){
                        output = 0;
                    }
                }

                if (!Double.isNaN(genericLimitSwitch.getPosition())) {
                    setEncoderPosition(genericLimitSwitch.getPosition());
                }
            }
        }

        outputMotor(output);
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

    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            return;
        }
        this.shuffleboardPeriodSeconds = periodSeconds;
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

    private void updateShuffleboardCache() {
        if (!shuffleboardEnabled) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        if (Double.isFinite(shuffleboardPeriodSeconds) && shuffleboardPeriodSeconds > 0.0
                && Double.isFinite(nowSeconds)
                && !Double.isNaN(lastShuffleboardCacheUpdateSeconds)
                && (nowSeconds - lastShuffleboardCacheUpdateSeconds) < shuffleboardPeriodSeconds) {
            return;
        }
        lastShuffleboardCacheUpdateSeconds = nowSeconds;
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
                runner.setLastOutput(runner.loop().calculate(controlContext));
                runner.setLastRunSeconds(nowSeconds);
            }
            total += runner.lastOutput();
        }
        return total;
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
            if (encoder != null) {
                encoder.update();
            }
            motors.update();
        }

        sensorSimulation.update(this);

        applyPeriodicHooks();

        update();

        updateShuffleboardCache();

        if (visualization != null) {
            visualization.setExternalRootPose(visualizationRootOverride);
            visualization.update(this);
        }

        prevRobotMode = robotMode;
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
        @Override
        public Mechanism mechanism() {
            return Mechanism.this;
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
            BooleanSupplier supplier = controlLoopInputs.get(key);
            return supplier != null && supplier.getAsBoolean();
        }

        @Override
        public BooleanSupplier inputSupplier(String key) {
            BooleanSupplier supplier = controlLoopInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No input found for key " + key);
            }
            return supplier;
        }

        @Override
        public double doubleInput(String key) {
            DoubleSupplier supplier = controlLoopDoubleInputs.get(key);
            return supplier != null ? supplier.getAsDouble() : Double.NaN;
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            DoubleSupplier supplier = controlLoopDoubleInputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No double input found for key " + key);
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

    @Override
    public Mechanism shuffleboard(String tab, SendableLevel level) {
        return (Mechanism) RobotSendableSystem.super.shuffleboard(tab, level);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        shuffleboardEnabled = true;
        shuffleboardInternal(tab, level);
        return tab;
    }

    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        shuffleboardEnabled = true;
        shuffleboardInternal(layout, level);
        return layout;
    }

    private void shuffleboardInternal(ShuffleboardContainer container, SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        ShuffleboardLayout motorsLayout = container.getLayout("Motors", BuiltInLayouts.kList);
        motors.shuffleboard(motorsLayout, level);

        if (encoder != null) {
            ShuffleboardLayout encoderLayout = container.getLayout("Encoders", BuiltInLayouts.kList);
            encoder.shuffleboard(encoderLayout, level);
        }

        ShuffleboardLayout statusLayout = container.getLayout("Status", BuiltInLayouts.kList);
        statusLayout.addBoolean("Emergency Stopped", RobotSendableSystem.rateLimit(() -> cachedEmergencyStopped, period));
        statusLayout.addBoolean("Override", RobotSendableSystem.rateLimit(() -> cachedOverride, period));
        statusLayout.addBoolean("At Setpoint", RobotSendableSystem.rateLimit(() -> cachedAtSetpoint, period));

        ShuffleboardLayout setpointLayout = container.getLayout("Setpoints", BuiltInLayouts.kList);
        setpointLayout.addDouble("Setpoint", RobotSendableSystem.rateLimit(() -> cachedSetpoint, period));
        if (level.equals(SendableLevel.DEBUG)) {
            setpointLayout.addDouble("Nudge", RobotSendableSystem.rateLimit(() -> cachedNudge, period));
        }

        ShuffleboardLayout outputLayout = container.getLayout("Outputs", BuiltInLayouts.kList);
        outputLayout.addDouble("Output", RobotSendableSystem.rateLimit(() -> cachedOutput, period));
        if (level.equals(SendableLevel.DEBUG)) {
            outputLayout.addDouble("PID Output", RobotSendableSystem.rateLimit(() -> cachedPidOutput, period));
            outputLayout.addDouble("Feedforward Output", RobotSendableSystem.rateLimit(() -> cachedFeedforwardOutput, period));
        }

        if (visualization != null) {
            ShuffleboardLayout visualizationLayout = container.getLayout("Visualization", BuiltInLayouts.kList);
            visualizationLayout.add("Mechanism2d", visualization.mechanism2d());
        }

        if (RobotBase.isSimulation()) {
            ShuffleboardLayout simulationLayout = container.getLayout("Simulation", BuiltInLayouts.kList);
            simulationLayout.addBoolean("Simulation Enabled", RobotSendableSystem.rateLimit(() -> cachedHasSimulation, period));
            if (level.equals(SendableLevel.DEBUG)) {
                simulationLayout.addDouble("Simulation dt", RobotSendableSystem.rateLimit(() -> cachedSimulationUpdatePeriodSeconds, period));
            }
        }

        if (level.equals(SendableLevel.DEBUG)) {
            ShuffleboardLayout configLayout = container.getLayout("Config", BuiltInLayouts.kList);
            configLayout.addString("Output Type", RobotSendableSystem.rateLimit(() -> outputType.name(), period));
            configLayout.add("Use Absolute", builder ->
                    builder.addBooleanProperty(
                            "Use Absolute",
                            RobotSendableSystem.rateLimit(() -> cachedUseAbsolute, period),
                            this::setUseAbsolute));
            configLayout.add("Setpoint As Output", builder ->
                    builder.addBooleanProperty(
                            "Setpoint As Output",
                            RobotSendableSystem.rateLimit(() -> cachedSetpointAsOutput, period),
                            this::setSetpointAsOutput));
            configLayout.add("PID Period (s)", builder ->
                    builder.addDoubleProperty(
                            "PID Period (s)",
                            RobotSendableSystem.rateLimit(() -> cachedPidPeriod, period),
                            this::setPidPeriod));
            configLayout.addBoolean("Feedforward Enabled", RobotSendableSystem.rateLimit(() -> cachedFeedforwardEnabled, period));
            configLayout.addBoolean("PID Enabled", RobotSendableSystem.rateLimit(() -> cachedPidEnabled, period));
        }

        if (level.equals(SendableLevel.DEBUG)) {
            ShuffleboardLayout limitsLayout = container.getLayout("Limit Switches", BuiltInLayouts.kList);
            for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
                ShuffleboardLayout limitLayout =
                        limitsLayout.getLayout("DIO " + genericLimitSwitch.getPort(), BuiltInLayouts.kList);
                genericLimitSwitch.shuffleboard(limitLayout);
                sensorSimulation.decorateSensorLayout(genericLimitSwitch, limitLayout, level);
            }
        }

        ShuffleboardLayout commandsLayout = container.getLayout("Commands", BuiltInLayouts.kList);
        commandsLayout.add("Brake Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Brake)))
                .withWidget(BuiltInWidgets.kCommand);
        commandsLayout.add("Coast Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Coast)))
                .withWidget(BuiltInWidgets.kCommand);
        if (level.equals(SendableLevel.DEBUG)) {
            commandsLayout.add("Enable/Disable PID", new InstantCommand(() -> setPidEnabled(!pidEnabled)))
                    .withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable/Disable FeedForward", new InstantCommand(() -> setFeedforwardEnabled(!feedforwardEnabled)))
                    .withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable/Disable Override", new InstantCommand(() -> setOverride(!override)))
                    .withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable/Disable Emergency Stop", new InstantCommand(() -> setEmergencyStopped(!emergencyStopped)))
                    .withWidget(BuiltInWidgets.kCommand);
        }

        ShuffleboardLayout sysIdLayout = container.getLayout("SysId", BuiltInLayouts.kList);
        sysIdLayout.add("Quasistatic Forward", sysIdCommand(() -> getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward)))
                .withWidget(BuiltInWidgets.kCommand);
        sysIdLayout.add("Quasistatic Reverse", sysIdCommand(() -> getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse)))
                .withWidget(BuiltInWidgets.kCommand);
        sysIdLayout.add("Dynamic Forward", sysIdCommand(() -> getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward)))
                .withWidget(BuiltInWidgets.kCommand);
        sysIdLayout.add("Dynamic Reverse", sysIdCommand(() -> getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse)))
                .withWidget(BuiltInWidgets.kCommand);
        sysIdLayout.add("Ramp Rate (V/s)",
                builder -> builder.addDoubleProperty(
                        "Ramp Rate (V/s)",
                        RobotSendableSystem.rateLimit(() -> cachedSysIdRampRate, period),
                        this::setSysIdRampRateVoltsPerSecond));
        sysIdLayout.add("Step Voltage (V)",
                builder -> builder.addDoubleProperty(
                        "Step Voltage (V)",
                        RobotSendableSystem.rateLimit(() -> cachedSysIdStepVoltage, period),
                        this::setSysIdStepVoltage));
        sysIdLayout.add("Timeout (s)",
                builder -> builder.addDoubleProperty(
                        "Timeout (s)",
                        RobotSendableSystem.rateLimit(() -> cachedSysIdTimeoutSeconds, period),
                        this::setSysIdTimeoutSeconds));
        sysIdLayout.addBoolean("Active", RobotSendableSystem.rateLimit(() -> cachedSysIdActive, period))
                .withWidget(BuiltInWidgets.kBooleanBox);

        if (level.equals(SendableLevel.DEBUG)) {
            ShuffleboardLayout controllersLayout = container.getLayout("Controllers", BuiltInLayouts.kList);
            if (pidController != null) {
                controllersLayout.add("PID Controller", pidController);
            }
            if (profiledPIDController != null) {
                controllersLayout.add("Profiled PID Controller", profiledPIDController);
            }
        }
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
