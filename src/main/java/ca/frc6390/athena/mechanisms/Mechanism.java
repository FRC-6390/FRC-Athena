package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulation;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationModel;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualization;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Map;

public class Mechanism extends SubsystemBase implements RobotSendableSystem{

    private final MotorControllerGroup motors;
    private final Encoder encoder;
    private final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;
    private final boolean useAbsolute, useVoltage;
    private final GenericLimitSwitch[] limitSwitches;
    private boolean override, emergencyStopped, pidEnabled, feedforwardEnabled, setpointIsOutput, suppressMotorOutput, customPIDCycle;
    private double setpoint, prevSetpoint, pidOutput, feedforwardOutput, output, nudge, pidPeriod; 
    private RobotMode prevRobotMode = RobotMode.DISABLE, robotMode = RobotMode.DISABLE;
    private final MechanismSimulationModel simulationModel;
    private final double simulationUpdatePeriodSeconds;
    private double lastSimulationTimestampSeconds = Double.NaN;
    private final MechanismVisualization visualization;
    private final MechanismSensorSimulation sensorSimulation;

    private enum RobotMode {
        TELE,
        AUTO,
        DISABLE,
        TEST,
    }

    public Mechanism(MechanismConfig<? extends Mechanism> config){
        this(
            MotorControllerGroup.fromConfigs(config.motors.toArray(MotorControllerConfig[]::new)),
            Encoder.fromConfig(config.encoder),
            config.pidController,
            config.profiledPIDController,
            config.useAbsolute,
            config.useVoltage,
            config.limitSwitches.stream().map(GenericLimitSwitch::fromConfig).toArray(GenericLimitSwitch[]::new),
            config.useSetpointAsOutput,
            config.pidPeriod,
            config.simulationConfig,
            config.visualizationConfig,
            config.sensorSimulationConfig
        );
    }

    public Mechanism(MotorControllerGroup motors, Encoder encoder, PIDController pidController,
            ProfiledPIDController profiledPIDController, boolean useAbsolute, boolean useVoltage,
            GenericLimitSwitch[] limitSwitches, boolean useSetpointAsOutput, double pidPeriod) {
        this(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, limitSwitches, useSetpointAsOutput, pidPeriod, null, null, null);
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
        this.useVoltage = useVoltage;
        this.override = false;
        this.limitSwitches = limitSwitches;
        this.setpointIsOutput = useSetpointAsOutput;
        this.pidPeriod = pidPeriod;

        if(profiledPIDController != null){
            profiledPIDController.reset(getPosition(), getVelocity());
        }

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
        this.visualization = visualizationConfig != null ? new MechanismVisualization(visualizationConfig) : null;
        if (RobotBase.isSimulation()) {
            if (sensorSimulationConfig != null) {
                this.sensorSimulation = MechanismSensorSimulation.fromConfig(this, sensorSimulationConfig);
            } else {
                this.sensorSimulation = MechanismSensorSimulation.forLimitSwitches(this);
            }
        } else {
            this.sensorSimulation = MechanismSensorSimulation.empty();
        }
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
        motors.setVoltage(voltage);
    }

    public void setSpeed(double speed){
        motors.setSpeed(speed);
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
        if (encoder == null) return 0;
        return useAbsolute ? getEncoder().getAbsolutePosition() : getEncoder().getPosition();
    }

    public double getVelocity(){
        if (encoder == null) return 0;
        return getEncoder().getVelocity();
    }

    public boolean atSetpoint(){
        return (pidController == null || pidController.atSetpoint()) &&
           (profiledPIDController == null || profiledPIDController.atGoal());
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public double getControllerSetpointVelocity(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().velocity : pidController != null ? pidController.getSetpoint() : 0;
    }

    public double getControllerSetpointPosition(){
        return profiledPIDController != null ? profiledPIDController.getSetpoint().position : pidController != null ? pidController.getSetpoint() : 0;
    }


    public boolean isOverride() {
        return override;
    }

    public boolean isEmergencyStopped() {
        return emergencyStopped;
    }

    public void setOverride(boolean override) {
        this.override = override;
    }

    public boolean isUseAbsolute() {
        return useAbsolute;
    }

    public boolean isUseVoltage() {
        return useVoltage;
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

    public double calculatePID(){
        double output = 0;
        double encoderPos = getPosition();

        if (pidController != null){
            output += pidController.calculate(encoderPos, getSetpoint() + getNudge());
        }

        if(profiledPIDController != null){
            if(prevSetpoint != getSetpoint()) resetPID();
            output += profiledPIDController.calculate(encoderPos, getSetpoint() + getNudge());
        }
        
        return output;
    }

    private void outputMotor(double output) {
        if (!suppressMotorOutput){
            if (useVoltage) {
                setVoltage(output);
            } else {
                setSpeed(output);
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

        if ( (encoder != null && !encoder.isConnected()) || !motors.allMotorsConnected()) {
            emergencyStopped = true;
        }

        if(!customPIDCycle) updatePID();

        output = isPidEnabled() ? pidOutput : 0;
        output += isFeedforwardEnabled() ? feedforwardOutput : 0;

        if (setpointIsOutput){
            output = getSetpoint() + getNudge();
        }

        this.output = output;
        this.prevSetpoint = getSetpoint();

        if (override){
            return;
        }
        
        if(emergencyStopped){
            motors.stopMotors();
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
        return output;
    }

    public void setNudge(double nudge){
        this.nudge = nudge;
    }

    public double getNudge() {
        return nudge;
    }

    public double getPidPeriod() {
        return pidPeriod;
    }

    public boolean hasSimulation() {
        return simulationModel != null;
    }

    public double getSimulationUpdatePeriodSeconds() {
        return simulationUpdatePeriodSeconds;
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

    @Override
    public void periodic() {

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

        update();

        if (visualization != null) {
            visualization.update(this);
        }

        prevRobotMode = robotMode;
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
        
        motors.shuffleboard(tab.getLayout("Motors", BuiltInLayouts.kList), level);
        if (encoder != null) encoder.shuffleboard(tab.getLayout("Encoder", BuiltInLayouts.kList), level);

        
        tab.addBoolean("Emergency Stopped", this::isEmergencyStopped);
        tab.addBoolean("Override", this::isOverride);
        if (visualization != null) {
            tab.add("Mechanism2d", visualization.mechanism2d());
        }
        if (RobotBase.isSimulation()) {
            tab.addBoolean("Simulation Enabled", this::hasSimulation);
            if (level.equals(SendableLevel.DEBUG)) {
                tab.addDouble("Simulation dt", this::getSimulationUpdatePeriodSeconds);
            }
        }
        if(level.equals(SendableLevel.DEBUG)){
            tab.addBoolean("Use Voltage", this::isUseVoltage);
            tab.addBoolean("Use Absolute", this::isUseAbsolute);
            tab.addBoolean("Feedforward Enabled", this::isFeedforwardEnabled);
            tab.addBoolean("PID Enabled", this::isPidEnabled);
            tab.addDouble("Nudge", this::getNudge);
            tab.addDouble("PID Output", this::getPidOutput);
            tab.addDouble("Feedforward Output", this::getFeedforwardOutput);
        }
       
        tab.addBoolean("At Setpoint", this::atSetpoint);
        tab.addDouble("Setpoint", this::getSetpoint);
        tab.addDouble("Output", this::getOutput);
        
        if(level.equals(SendableLevel.DEBUG)){
            for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
                ShuffleboardLayout limitLayout =
                        tab.getLayout(genericLimitSwitch.getPort()+"\\Limitswitch",BuiltInLayouts.kList);
                genericLimitSwitch.shuffleboard(limitLayout);  
                sensorSimulation.decorateSensorLayout(genericLimitSwitch, limitLayout, level);
            }
        }

        ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kList);
        {
            commandsLayout.add("Brake Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Coast Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
            if(level.equals(SendableLevel.DEBUG)){
                commandsLayout.add("Enable\\Disable PID", new InstantCommand(() -> setPidEnabled(!pidEnabled))).withWidget(BuiltInWidgets.kCommand);
                commandsLayout.add("Enable\\Disable FeedForward", new InstantCommand(() -> setFeedforwardEnabled(!feedforwardEnabled))).withWidget(BuiltInWidgets.kCommand);
                commandsLayout.add("Enable\\Disable Override", new InstantCommand(() -> setOverride(!override))).withWidget(BuiltInWidgets.kCommand);
                commandsLayout.add("Enable\\Disable Emergency Stop", new InstantCommand(() -> setEmergencyStopped(!emergencyStopped))).withWidget(BuiltInWidgets.kCommand);
            }
        }
        if(level.equals(SendableLevel.DEBUG)){
            if(pidController != null) tab.add("PID Controller", pidController);
            if(profiledPIDController != null) tab.add("Profiled PID Controller", profiledPIDController);
        }

        return tab;
    }

    public Mechanism2d getMechanism2d() {
        return visualization != null ? visualization.mechanism2d() : null;
    }

    public Map<String, Pose3d> getMechanism3dPoses() {
        return visualization != null ? visualization.poses() : Map.of();
    }
}
