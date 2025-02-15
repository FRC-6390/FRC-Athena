package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import ca.frc6390.athena.devices.Encoder.EncoderConfig;
import ca.frc6390.athena.devices.Encoder.EncoderType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorController.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorController.MotorControllerType;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Mechanism implements Subsystem{
    public record MechanismConfig<T extends Mechanism>(
        ArrayList<MotorControllerConfig> motors, 
        EncoderConfig encoder, 
        PIDController pidController, 
        ProfiledPIDController profiledPIDController, 
        boolean useAbsolute, 
        boolean useVoltage,
        Function<MechanismConfig<T>, T> factory,
        ArrayList<GenericLimitSwitchConfig> limitSwitches
        ) {

        public static MechanismConfig<Mechanism> generic(){
            return custom(Mechanism::new);
        }

        public static <E extends Enum<E> & SetpointProvider, T extends StatefulMechanism<E>> MechanismConfig<T> stateful(BiFunction<MechanismConfig<T>, E, T> factory, E initialState) {
            return custom(config -> factory.apply(config, initialState));
        }

        public static MechanismConfig<ElevatorMechanism> elevator(ElevatorFeedforward feedforward) {
            return custom(config -> new ElevatorMechanism(config, feedforward));
        }

        public static <T extends ElevatorMechanism> MechanismConfig<T> elevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static <E extends Enum<E> & SetpointProvider> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(ElevatorFeedforward feedforward, E initialState) {
            return custom(config -> new StatefulElevatorMechanism<>(config, feedforward, initialState));
        }

        public static <E extends Enum<E> & SetpointProvider, T extends StatefulElevatorMechanism<E>> MechanismConfig<T> statefulElevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static <E extends Enum<E> & SetpointProvider> MechanismConfig<StatefulArmMechanism<E>> statefulArm(ArmFeedforward feedforward, E initialState) {
            return custom(config -> new StatefulArmMechanism<>(config, feedforward, initialState));
        }

        public static <E extends Enum<E> & SetpointProvider, T extends StatefulArmMechanism<E>> MechanismConfig<T> statefulArm(ArmFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static <E extends Enum<E> & SetpointProvider> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(SimpleMotorFeedforward feedforward, E initialState) {
            return custom(config -> new StatefulTurretMechanism<>(config, feedforward, initialState));
        }

        public static <E extends Enum<E> & SetpointProvider, T extends StatefulTurretMechanism<E>> MechanismConfig<T> statefulTurret(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static MechanismConfig<TurretMechanism> turret(SimpleMotorFeedforward feedforward) {
            return custom(config -> new TurretMechanism(config, feedforward));
        }

        public static <T extends TurretMechanism> MechanismConfig<T> turret(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static MechanismConfig<ArmMechanism> arm(ArmFeedforward feedforward) {
            return custom(config -> new ArmMechanism(config, feedforward));
        }

        public static <T extends ArmMechanism> MechanismConfig<T> arm(ArmFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
            return custom(factory);
        }

        public static <T extends Mechanism> MechanismConfig<T> custom(Function<MechanismConfig<T>, T> factory){
            return new MechanismConfig<>(new ArrayList<>(), null, null, null, false, false, factory, new ArrayList<>());
        }

        public MechanismConfig<T> addMotor(MotorControllerConfig config){
            motors.add(config);
            return this;
        }

        public MechanismConfig<T> addMotor(MotorControllerType type, int id){
            return addMotor(new MotorControllerConfig(type, id));
        }

        public MechanismConfig<T> addMotor(MotorControllerType type, int... ids){
            MechanismConfig<T> s = this;

            for (int i = 0; i < ids.length; i++) {
               s = s.addMotor(new MotorControllerConfig(type, ids[i]));
            }

            return s;
        }

        public MechanismConfig<T> addMotor(Motor type, int id){
            return addMotor(new MotorControllerConfig(type.getMotorControllerType(), id));
        }

        public MechanismConfig<T> addMotor(Motor type, int... ids){
            MechanismConfig<T> s = this;

            for (int i = 0; i < ids.length; i++) {
               s = s.addMotor(new MotorControllerConfig(type.getMotorControllerType(), ids[i]));
            }

            return s;
        }

        public MechanismConfig<T> setEncoder(EncoderConfig config){
            return new MechanismConfig<T>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setEncoder(EncoderType type, int id){
            return setEncoder(new EncoderConfig(type, id));
        }

        public MechanismConfig<T> setEncoderFromMotor(int id){
            return setEncoder(motors.stream().filter((motors) -> motors.id() == id).findFirst().get().encoderConfig());
        }

        public MechanismConfig<T> setPID(double p, double i, double d){
            return setPID(new PIDController(p, i, d));
        }

        public MechanismConfig<T> setPID(PIDController pidController){
            return new MechanismConfig<T>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setProfiledPID(double p, double i, double d, double maxVel, double maxAccel){
            return setProfiledPID(p, i, d, new TrapezoidProfile.Constraints(maxVel, maxAccel));
        }

        public MechanismConfig<T> setProfiledPID(double p, double i, double d, TrapezoidProfile.Constraints constraints){
            return setProfiledPID(new ProfiledPIDController(p, i, d, constraints));
        }

        public MechanismConfig<T> setProfiledPID(ProfiledPIDController profiledPIDController){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setCanbus(String canbus){
            motors.forEach((motor) -> motor.setCanbus(canbus));
            encoder.setCanbus(canbus);
            return this;
        }

        public MechanismConfig<T> setEncoderGearRatio(double ratio){
            encoder.setGearRatio(ratio);
            return this;
        }

        public MechanismConfig<T> setEncoderConversion(double conversion){
            encoder.setConversion(conversion);
            return this;
        }

        public MechanismConfig<T> setEncoderConversionOffset(double conversion){
            encoder.setConversionOffset(conversion);
            return this;
        }

        public MechanismConfig<T> setEncoderOffset(double offset){
            encoder.setOffset(offset);
            return this;
        }

        public MechanismConfig<T> setEncoderInverted(boolean inverted){
            encoder.setInverted(inverted);
            return this;
        }

        public MechanismConfig<T> setCurrentLimit(double limit){
            motors.forEach((motor) -> motor.setCurrentLimit(limit));
            return this;
        }

        public MechanismConfig<T> setEncoderConfig(Function<EncoderConfig, EncoderConfig> func){
            return new MechanismConfig<>(motors, func.apply(encoder), pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setUseEncoderAbsolute(boolean useAbsolute){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setUseVoltage(boolean useVoltage){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory, limitSwitches);
        }

        public MechanismConfig<T> setNeutralMode(MotorNeutralMode mode){
            motors.forEach((motor) -> motor.setNeutralMode(mode));
            return this;
        }

        public MechanismConfig<T> addLimitSwitch(GenericLimitSwitchConfig config) {
            limitSwitches.add(config);
            return this;
        }

        public MechanismConfig<T> addLimitSwitch(int id, double position){
           return addLimitSwitch(id, position, false);
        }

        public MechanismConfig<T> addLimitSwitch(int id, double position, boolean stopMotors){
            return addLimitSwitch(GenericLimitSwitchConfig.normal(id).setPosition(position).setHardstop(stopMotors));
        }

        public T create(){
            return factory.apply(this);
        }
    }

    private final MotorControllerGroup motors;
    private final Encoder encoder;
    private final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;
    private final boolean useAbsolute, useVoltage;
    private boolean override;
    private double setpoint;

    public Mechanism(MechanismConfig<? extends Mechanism> config){
        this(MotorControllerGroup.fromConfigs(config.motors.toArray(MotorControllerConfig[]::new)), Encoder.fromConfig(config.encoder), config.pidController, config.profiledPIDController, config.useAbsolute, config.useVoltage);
    }

    public Mechanism(MotorControllerGroup motors, Encoder encoder, PIDController pidController, ProfiledPIDController profiledPIDController, boolean useAbsolute, boolean useVoltage){
        this.motors = motors;
        this.encoder = encoder;
        this.pidController = pidController;
        this.profiledPIDController = profiledPIDController;
        this.useAbsolute = useAbsolute;
        this.useVoltage = useVoltage;
        this.override = false;

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

    public MotorControllerGroup getMotorGroup(){
        return motors;
    }

    public Encoder getEncoder(){
       return encoder;
    }

    public Rotation2d getRotation2d(){
        return useAbsolute ? getEncoder().getAbsoluteRotation2d() : getEncoder().getRotation2d();
    }

    public double getPosition(){
        return useAbsolute ? getEncoder().getAbsolutePosition() : getEncoder().getPosition();
    }

    public double getVelocity(){
        return getEncoder().getVelocity();
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

    public double calculatePID(){
        double output = 0;
        double encoderPos = getPosition();

        if (pidController != null){
            output += pidController.calculate(encoderPos, getSetpoint());
        }

        if(profiledPIDController != null){
            profiledPIDController.reset(getPosition(), getVelocity());
            output += profiledPIDController.calculate(encoderPos, getSetpoint());
        }

        return output;
    }

    public void update(){

        encoder.update();

        if (override){
            return;
        }

        double value = calculatePID() + calculateFeedForward(); 
        
        if (useVoltage) {
            setVoltage(value);
        }else {
            setSpeed(value);
        }
    }

    @Override
    public void periodic() {
        update();
    }

    public static class StatefulMechanism<E extends Enum<E> & SetpointProvider> extends Mechanism {
        
        private final StateMachine<E> stateMachine;

        public StatefulMechanism(MechanismConfig<StatefulMechanism<E>> config, E initialState) {
            super(config);
            this.stateMachine = new StateMachine<>(initialState, () -> true);
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

        public StateMachine<E> getStateMachine() {
            return stateMachine;
        }

    }
}
