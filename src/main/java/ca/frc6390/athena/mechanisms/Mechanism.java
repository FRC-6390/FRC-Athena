package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.function.Function;

import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.mechanisms.Mechanism.ArmMech;
import ca.frc6390.athena.mechanisms.Mechanism.ElevatorMech;
import ca.frc6390.athena.mechanisms.Mechanism.TurretMech;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
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

public class Mechanism {

    public static class StateMachineMech<E extends Enum<E> & SetpointProvider> extends Mechanism {

        private final StateMachine<E> machine;

        public StateMachineMech(MechanismConfig<StateMachineMech<E>> config, E intialState) {
            super(config);
            this.machine = new StateMachine<>(intialState, () -> true);
        }

        public StateMachine<E> getStateMachine(){
            return machine;
        }

        @Override
        public double getSetpoint() {
            return getStateMachine().getGoalState().getSetpoint();
        }
    }

    public static class ElevatorMech extends Mechanism {

        private final ElevatorFeedforward feedforward;

        public ElevatorMech(MechanismConfig<ElevatorMech> config, ElevatorFeedforward feedforward) {
            super(config);
            this.feedforward = feedforward;
        }

        @Override
        public double calculateFeedForward() {
            return feedforward.calculate(getVelocity());
        }
    }

    public static class ArmMech extends Mechanism {
        
        private final ArmFeedforward feedforward;

        public ArmMech(MechanismConfig<ArmMech> config, ArmFeedforward feedforward) {
            super(config);
            this.feedforward = feedforward;
        }

        @Override
        public double calculateFeedForward() {
            return feedforward.calculate(getRotation2d().getRadians(), getVelocity());
        }
    }


    public static class TurretMech extends Mechanism {

        private final SimpleMotorFeedforward feedforward;

        public TurretMech(MechanismConfig<TurretMech> config, SimpleMotorFeedforward feedforward) {
            super(config);
            this.feedforward = feedforward;
        }

        @Override
        public double calculateFeedForward() {
            return feedforward.calculate(getVelocity());
        }
    }


    public record MechanismConfig<T extends Mechanism>(
        ArrayList<MotorControllerConfig> motors, 
        EncoderConfig encoder, 
        PIDController pidController, 
        ProfiledPIDController profiledPIDController, 
        boolean useAbsolute, 
        boolean useVoltage,
        Function<MechanismConfig<T>, T> factory
        ) {

        public static MechanismConfig<Mechanism> generic(){
            return custom(Mechanism::new);
        }

        public static <E extends Enum<E> & SetpointProvider> MechanismConfig<StateMachineMech<E>> stateMachine(E initalSate){
            return custom(config -> new StateMachineMech<E>(config, initalSate));
        }

        public static MechanismConfig<ElevatorMech> elevator(ElevatorFeedforward feedforward) {
            return custom(config -> new ElevatorMech(config, feedforward));
        }

        public static MechanismConfig<TurretMech> turret(SimpleMotorFeedforward feedforward) {
            return custom(config -> new TurretMech(config, feedforward));
        }

        public static MechanismConfig<ArmMech> arm(ArmFeedforward feedforward) {
            return custom(config -> new ArmMech(config, feedforward));
        }

        public static <T extends Mechanism> MechanismConfig<T> custom(Function<MechanismConfig<T>, T> factory){
            return new MechanismConfig<>(new ArrayList<>(), null, null, null, false, false, factory);
        }

        public MechanismConfig<T> addMotor(MotorControllerConfig config){
            motors.add(config);
            return this;
        }

        public MechanismConfig<T> addMotor(MotorControllerType type, int id){
            return addMotor(new MotorControllerConfig(type, id));
        }

        public MechanismConfig<T> addMotor(Motor type, int id){
            return addMotor(new MotorControllerConfig(type.getMotorControllerType(), id));
        }

        public MechanismConfig<T> setEncoder(EncoderConfig config){
            return new MechanismConfig<T>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory);
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
            return new MechanismConfig<T>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory);
        }

        public MechanismConfig<T> setProfiledPID(double p, double i, double d, double maxVel, double maxAccel){
            return setProfiledPID(p, i, d, new TrapezoidProfile.Constraints(maxVel, maxAccel));
        }

        public MechanismConfig<T> setProfiledPID(double p, double i, double d, TrapezoidProfile.Constraints constraints){
            return setProfiledPID(new ProfiledPIDController(p, i, d, constraints));
        }

        public MechanismConfig<T> setProfiledPID(ProfiledPIDController profiledPIDController){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory);
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
            return new MechanismConfig<>(motors, func.apply(encoder), pidController, profiledPIDController, useAbsolute, useVoltage, factory);
        }

        public MechanismConfig<T> setUseEncoderAbsolute(boolean useAbsolute){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory);
        }

        public MechanismConfig<T> setUseVoltage(boolean useVoltage){
            return new MechanismConfig<>(motors, encoder, pidController, profiledPIDController, useAbsolute, useVoltage, factory);
        }

        public MechanismConfig<T> setNeutralMode(MotorNeutralMode mode){
            motors.forEach((motor) -> motor.setNeutralMode(mode));
            return this;
        }

        public T create(){
            return factory.apply(this);
        }
    }

    private final MotorControllerGroup motors;
    private final Encoder encoder;
    final PIDController pidController;
    private final ProfiledPIDController profiledPIDController;
    private final boolean useAbsolute, useVoltage;
    private boolean override;
    private double setpoint;

    public Mechanism(MechanismConfig<?> config){
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

        MechanismConfig.elevator(new ElevatorFeedforward(0, 0, 0))
                        .addMotor(Motor.KRAKEN_X60, 20)
                        .addMotor(Motor.KRAKEN_X60, 21)
                        .setEncoder(EncoderType.CTRECANcoder, 22)
                        .setEncoderConversion(3)
                        .setEncoderConversionOffset(20)
                        .setEncoderInverted(true)
                        .setProfiledPID(setpoint, setpoint, setpoint, 1,1);
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
}
