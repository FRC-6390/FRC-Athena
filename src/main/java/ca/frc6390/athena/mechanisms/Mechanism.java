package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism extends SubsystemBase implements RobotSendableSystem{

    private final MotorControllerGroup motors;
    private final Encoder encoder;
    private final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;
    private final boolean useAbsolute, useVoltage;
    private final GenericLimitSwitch[] limitSwitches;
    private boolean override, emergencyStopped, pidEnabled, feedforwardEnabled, setpointIsOutput;
    private double setpoint, pidOutput, feedforwardOutput, output, nudge; 

    public Mechanism(MechanismConfig<? extends Mechanism> config){
        this(MotorControllerGroup.fromConfigs(config.motors.toArray(MotorControllerConfig[]::new)), Encoder.fromConfig(config.encoder), config.pidController, config.profiledPIDController, config.useAbsolute, config.useVoltage,config.limitSwitches.stream().map(GenericLimitSwitch::fromConfig).toArray(GenericLimitSwitch[]::new), config.useSetpointAsOutput);
    }

    public Mechanism(MotorControllerGroup motors, Encoder encoder, PIDController pidController, ProfiledPIDController profiledPIDController, boolean useAbsolute, boolean useVoltage, GenericLimitSwitch[] limitSwitches, boolean useSetpointAsOutput){
        this.motors = motors;
        this.encoder = encoder;
        this.pidController = pidController;
        this.profiledPIDController = profiledPIDController;
        this.useAbsolute = useAbsolute;
        this.useVoltage = useVoltage;
        this.override = false;
        this.limitSwitches = limitSwitches;
        this.setpointIsOutput = useSetpointAsOutput;

        if(profiledPIDController != null){
            profiledPIDController.reset(getPosition(), getVelocity());
        }
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
            profiledPIDController.calculate(encoderPos, getSetpoint() + getNudge());
            output += profiledPIDController.getSetpoint().velocity;
        }

        return output;
    }

    public void update(){

        double output = 0;

        if ( (encoder != null && !encoder.isConnected()) || !motors.allMotorsConnected()) {
            emergencyStopped = true;
        }

        pidOutput = calculatePID();
        feedforwardOutput = calculateFeedForward();

        output = isPidEnabled() ? pidOutput : 0;
        output += isFeedforwardEnabled() ? feedforwardOutput : 0;

        if (setpointIsOutput){
            output = getSetpoint() + getNudge();
        }

        this.output = output;

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
                    if(Math.signum(genericLimitSwitch.getBlockDirection()) == output){
                        output = 0;
                    }
                }

                if (!Double.isNaN(genericLimitSwitch.getPosition())) {
                    setEncoderPosition(genericLimitSwitch.getPosition());
                }
            }
        }

        if (useVoltage) {
            setVoltage(output);
        }else {
            setSpeed(output);
        }
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

    @Override
    public void periodic() {
        if (encoder != null) encoder.update();
        motors.update();
        update();
    }

    @Override
    public Mechanism shuffleboard(String tab) {
        return (Mechanism) RobotSendableSystem.super.shuffleboard(tab);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        
        motors.shuffleboard(tab.getLayout("Motors", BuiltInLayouts.kList));
        if (encoder != null) encoder.shuffleboard(tab.getLayout(encoder.getName(), BuiltInLayouts.kList));
        tab.addBoolean("Emergency Stopped", this::isEmergencyStopped);
        tab.addBoolean("Override", this::isOverride);
        tab.addBoolean("Use Voltage", this::isUseVoltage);
        tab.addBoolean("Use Absolute", this::isUseAbsolute);
        tab.addBoolean("Feedforward Enabled", this::isFeedforwardEnabled);
        tab.addBoolean("PID Enabled", this::isPidEnabled);
        tab.addBoolean("At Setpoint", this::atSetpoint);
        tab.addDouble("Setpoint", this::getSetpoint);
        tab.addDouble("Nudge", this::getNudge);

        tab.addDouble("Nudge + Setpoint", () -> getNudge() + getSetpoint());

        tab.addDouble("PID Output", this::getPidOutput);
        tab.addDouble("Feedforward Output", this::getFeedforwardOutput);
        tab.addDouble("Output", this::getOutput);
        for (GenericLimitSwitch genericLimitSwitch : limitSwitches) {
           genericLimitSwitch.shuffleboard(tab.getLayout(genericLimitSwitch.getPort()+"\\Limitswitch",BuiltInLayouts.kList));  
        }

        ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kList);
        {
            commandsLayout.add("Brake Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Coast Mode", new InstantCommand(() -> setMotorNeutralMode(MotorNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable\\Disable PID", new InstantCommand(() -> setPidEnabled(!pidEnabled))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable\\Disable FeedForward", new InstantCommand(() -> setFeedforwardEnabled(!feedforwardEnabled))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable\\Disable Override", new InstantCommand(() -> setOverride(!override))).withWidget(BuiltInWidgets.kCommand);
            commandsLayout.add("Enable\\Disable Emergency Stop", new InstantCommand(() -> setEmergencyStopped(!emergencyStopped))).withWidget(BuiltInWidgets.kCommand);
        }

        if(pidController != null) tab.add("PID Controller", pidController);
        if(profiledPIDController != null) tab.add("Profiled PID Controller", pidController);

        return tab;
    }

    public static class StatefulMechanism<E extends Enum<E> & SetpointProvider<Double>> extends Mechanism {
        
        private final StateMachine<Double, E> stateMachine;

        public StatefulMechanism(MechanismConfig<StatefulMechanism<E>> config, E initialState) {
            super(config);
            this.stateMachine = new StateMachine<>(initialState, this::atSetpoint);
        }

        @Override
        public double getSetpoint() {
            return stateMachine.getGoalState().getSetpoint();
        }

        @Override
        public void update() {
            stateMachine.update();  
            super.update();
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateMachine;
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulMechanism<E> shuffleboard(String tab) {
            return (StatefulMechanism<E>) super.shuffleboard(tab);
        }

        @Override
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
            stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList));
            return super.shuffleboard(tab);
        }

    }
}