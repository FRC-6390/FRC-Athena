package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorControllerType;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MechanismConfig<T extends Mechanism> {

    public ArrayList<MotorControllerConfig> motors = new ArrayList<>();
    public EncoderConfig encoder = null;
    public PIDController pidController = null;
    public ProfiledPIDController profiledPIDController = null;
    public boolean useAbsolute = false;
    public boolean useVoltage = false;
    public boolean useSetpointAsOutput = false;
    public Function<MechanismConfig<T>, T> factory = null;
    public ArrayList<GenericLimitSwitchConfig> limitSwitches = new ArrayList<>();
    
    public String canbus = "rio";
    public double encoderGearRatio = 1;
    public double encoderConversion = 1;
    public double encoderConversionOffset = 0;
    public double encoderOffset = 0;
    public double motorCurrentLimit = 40;
    public double tolerance = 0;
    public MotorNeutralMode motorNeutralMode = MotorNeutralMode.Brake;


    public static MechanismConfig<Mechanism> generic(){
        return custom(Mechanism::new);
    }

    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulMechanism<E>> statefulGeneric(E initialState){
        return custom(config -> new StatefulMechanism<>(config, initialState));
    }

    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulMechanism<E>> MechanismConfig<T> stateful(BiFunction<MechanismConfig<T>, E, T> factory, E initialState) {
        return custom(config -> factory.apply(config, initialState));
    }

    public static MechanismConfig<ElevatorMechanism> elevator(ElevatorFeedforward feedforward) {
        return custom(config -> new ElevatorMechanism(config, feedforward));
    }

    public static <T extends ElevatorMechanism> MechanismConfig<T> elevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(ElevatorFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulElevatorMechanism<>(config, feedforward, initialState));
    }

    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulElevatorMechanism<E>> MechanismConfig<T> statefulElevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(ArmFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulArmMechanism<>(config, feedforward, initialState));
    }

    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulArmMechanism<E>> MechanismConfig<T> statefulArm(ArmFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(SimpleMotorFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulTurretMechanism<>(config, feedforward, initialState));
    }

    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulTurretMechanism<E>> MechanismConfig<T> statefulTurret(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
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
        MechanismConfig<T> cfg = new MechanismConfig<>();
        cfg.factory = factory;
        return cfg;
    }

    public MechanismConfig<T> addMotor(MotorControllerConfig config){
        motors.add(config);
        return this;
    }

    public MechanismConfig<T> addMotor(MotorControllerType type, int id){
        return addMotor(new MotorControllerConfig(type, id));
    }

    public MechanismConfig<T> addMotors(MotorControllerType type, int... ids){
        for (int i = 0; i < ids.length; i++) {
            addMotor(new MotorControllerConfig(type, ids[i]));
         }
         return this;
    }

    public MechanismConfig<T> addMotor(Motor type, int id){
        return addMotor(new MotorControllerConfig(type.getMotorControllerType(), id));
    }

    public MechanismConfig<T> addMotors(Motor type, int... ids){
        return addMotors(type.getMotorControllerType(), ids);
    }

    public MechanismConfig<T> setEncoder(EncoderConfig encoder){
        this.encoder  = encoder;
        return this;
    }

    public MechanismConfig<T> setEncoder(EncoderType type, int id){
        return setEncoder(EncoderConfig.type(type, id));
    }

    public MechanismConfig<T> setEncoderFromMotor(int id){
        return setEncoder(motors.stream().filter((motors) -> motors.id == Math.abs(id)).findFirst().get().encoderConfig.setInverted(id < 0));
    }

    public MechanismConfig<T> setPID(double p, double i, double d){
        return setPID(new PIDController(p, i, d));
    }

    public MechanismConfig<T> setPID(PIDController pidController){
        this.pidController = pidController;
        return this;
    }

    public MechanismConfig<T> setProfiledPID(double p, double i, double d, double maxVel, double maxAccel){
        return setProfiledPID(p, i, d, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    public MechanismConfig<T> setProfiledPID(double p, double i, double d, TrapezoidProfile.Constraints constraints){
        return setProfiledPID(new ProfiledPIDController(p, i, d, constraints));
    }

    public MechanismConfig<T> setProfiledPID(ProfiledPIDController profiledPIDController){
        this.profiledPIDController = profiledPIDController;
        return this;
    }

    public MechanismConfig<T> setTolerance(double tolerance){
        this.tolerance = tolerance;
        return this;
    }
    public MechanismConfig<T> setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    }

    public MechanismConfig<T> setEncoderGearRatio(double ratio){
        this.encoderGearRatio = ratio;
        return this;
    }

    public MechanismConfig<T> setEncoderConversion(double conversion){
        this.encoderConversion = conversion;
        return this;
    }

    public MechanismConfig<T> setEncoderConversionOffset(double conversionOffset){
        this.encoderConversionOffset = conversionOffset;
        return this;
    }

    public MechanismConfig<T> setEncoderOffset(double offset){
        this.encoderOffset = offset;
        return this;
    }

    public MechanismConfig<T> setCurrentLimit(double limit){
        this.motorCurrentLimit = limit;
        return this;
    }

    public MechanismConfig<T> setEncoderConfig(Consumer<EncoderConfig> func){
        func.accept(encoder);
        return this;
    }

    public MechanismConfig<T> setUseEncoderAbsolute(boolean useAbsolute){
        this.useAbsolute = useAbsolute;
        return this;
    }

    public MechanismConfig<T> setUseVoltage(boolean useVoltage){
        this.useVoltage = useVoltage;
        return this;
    }

    public MechanismConfig<T> setNeutralMode(MotorNeutralMode mode){
        this.motorNeutralMode = mode;
        return this;
    }

    public MechanismConfig<T> addLimitSwitch(GenericLimitSwitchConfig config) {
        limitSwitches.add(config);
        return this;
    }

    public MechanismConfig<T> setUseSetpointAsOutput(boolean useSetpointAsOutput) {
        this.useSetpointAsOutput = useSetpointAsOutput;
        return this;
    }

    public MechanismConfig<T> addLowerLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    public MechanismConfig<T> addLowerLimitSwitch(int id, double position, boolean stopMotors){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, -1));
    }

    public MechanismConfig<T> addUpperLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    public MechanismConfig<T> addUpperLimitSwitch(int id, double position, boolean stopMotors){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, 1));
    }

    public MechanismConfig<T> addLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    public MechanismConfig<T> addLimitSwitch(int id, double position, boolean stopMotors, int blockDirection){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, blockDirection));
    }

    public T build(){

        motors.forEach(
            (motor) -> motor.setNeutralMode(motorNeutralMode)
                            .setCurrentLimit(motorCurrentLimit)
                            .setCanbus(canbus)
                            );

        if (encoder != null) {
             encoder.setCanbus(canbus)
                    .setConversion(encoderConversion)
                    .setConversionOffset(encoderConversionOffset)
                    .setGearRatio(encoderGearRatio)
                    .setOffset(encoderOffset);
        }

        if(pidController != null){
            pidController.setTolerance(tolerance);
        }

        if(profiledPIDController != null){
            profiledPIDController.setTolerance(tolerance);
        }

        return factory.apply(this);
    }
}
