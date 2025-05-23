package ca.frc6390.athena.devices;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorControllerType;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class MotorController implements RobotSendableDevice {

    private final DoubleConsumer setSpeed, setVoltage, setCurrentLimit, setPosition;
    private final BooleanSupplier getIsConnected;
    private final DoubleSupplier getTemperature;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Consumer<PIDController> setPID;
    private final Encoder encoder;
    private boolean inverted = false;
    private double currentLimit = 0;
    private MotorNeutralMode motorNeutralMode;
    private String canbus;
    private int id;
    private MotorControllerType type;
    private double temperature;
    private PIDController pid;

    public MotorController(TalonFX controller){
        this(
            (mode) -> {
                controller.setNeutralMode(mode.asCTRE());
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
                currentConfig.SupplyCurrentLimitEnable = true;
                currentConfig.SupplyCurrentLimit = limit;
                config.withCurrentLimits(currentConfig);
                controller.getConfigurator().apply(config);
                // config.Feedback.
            },
            controller::isConnected,
            () -> controller.getDeviceTemp(true).getValueAsDouble(),
            pid -> {
                var slot0Configs = new Slot0Configs();
                slot0Configs.kP = pid.getP();
                slot0Configs.kI = pid.getI();
                slot0Configs.kD = pid.getD();
                controller.getConfigurator().apply(slot0Configs);
            },
            val -> {
                controller.setControl(new PositionDutyCycle(val).withSlot(0));
            },
            new Encoder(controller)
        );

    }

    public MotorController(SparkMax controller){
        this(
            (mode) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            },
            () -> controller.getFaults().can,
            controller::getMotorTemperature,
            pid -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(controller.configAccessor.getIdleMode());
                config.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                config.closedLoop.iZone(pid.getIZone());
            },
            val -> {
                controller.getClosedLoopController().setReference(val, ControlType.kPosition);
            },
            Encoder.newREVSparkMax(controller)
        );
    }

    public MotorController(SparkFlex controller){
        this(
            (mode) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                config.closedLoop.pid(controller.configAccessor.closedLoop.getP(), controller.configAccessor.closedLoop.getI(), controller.configAccessor.closedLoop.getD());
                config.closedLoop.iZone(controller.configAccessor.closedLoop.getIZone());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                config.closedLoop.pid(controller.configAccessor.closedLoop.getP(), controller.configAccessor.closedLoop.getI(), controller.configAccessor.closedLoop.getD());
                config.closedLoop.iZone(controller.configAccessor.closedLoop.getIZone());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            },
            () -> controller.getFaults().can,
            controller::getMotorTemperature,
            pid -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(controller.configAccessor.getIdleMode());
                config.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                config.closedLoop.iZone(pid.getIZone());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            },
            val -> {
                controller.getClosedLoopController().setReference(val, ControlType.kPosition);
            },
            Encoder.newREVSparkFlex(controller)
        );
    }

    public MotorController(Consumer<MotorNeutralMode> setNeutralMode, DoubleConsumer setSpeed, DoubleConsumer setVoltage, DoubleConsumer setCurrentLimit, BooleanSupplier getIsConnected, DoubleSupplier getTemperature, Consumer<PIDController> setPID, DoubleConsumer setPosition, Encoder encoder) {
        this.setNeutralMode = setNeutralMode;
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.getIsConnected = getIsConnected;
        this.encoder = encoder;
        this.getTemperature = getTemperature;
        this.setPID = setPID;
        this.setPosition = setPosition;
    }

    public static MotorController newREVSparkFlexBrushed(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkFlexBrushed, id));
    }

    public static MotorController newREVSparkFlexBrushless(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkFlexBrushless, id));
    }

    public static MotorController newREVSparkMaxBrushed(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkMaxBrushed, id));
    }

    public static MotorController newREVSparkMaxBrushless(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkMaxBrushless, id));
    }

    public static MotorController newCTRETalonFx(int id, String canbus){
        return fromConfig(new MotorControllerConfig(MotorControllerType.CTRETalonFX, id).setCanbus(canbus));
    }

    public static MotorController newCTRETalonFx(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.CTRETalonFX, id));
    }

    public static MotorController fromConfig(MotorControllerConfig config) {
        switch (config.type) {
            case CTRETalonFX:
            return new MotorController(new TalonFX(config.id, config.canbus)).applyConfig(config); 
            case REVSparkMaxBrushed:
            return new MotorController(new SparkMax(config.id, MotorType.kBrushed)).applyConfig(config); 
            case REVSparkMaxBrushless:
            return new MotorController(new SparkMax(config.id, MotorType.kBrushless)).applyConfig(config); 
            case REVSparkFlexBrushed:
            return new MotorController(new SparkFlex(config.id, MotorType.kBrushed)).applyConfig(config); 
            case REVSparkFlexBrushless:
            return new MotorController(new SparkFlex(config.id, MotorType.kBrushless)).applyConfig(config); 
            default:
            return null;
        }
    }

    public MotorController applyConfig(MotorControllerConfig config) {
        setCurrentLimit(config.currentLimit);
        setInverted(config.inverted);
        setNeutralMode(config.neutralMode);
        getEncoder().applyConfig(config.encoderConfig);
        canbus = config.canbus;
        id = config.id;
        type = config.type;
        return this;
    }

    public MotorController setCurrentLimit(double limit){
        setCurrentLimit.accept(limit);
        currentLimit = limit;
        return this;
    }

    public MotorController setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    public boolean isInverted(){
        return inverted;
    }

    public MotorController setPID(PIDController pid){
        setPID.accept(pid);
        return this;
    }

    public MotorController setPID(double p, double i, double d){
        return setPID(new PIDController(p, i, d));
    }

    public MotorController setNeutralMode(MotorNeutralMode mode){
        setNeutralMode.accept(mode);
        motorNeutralMode = mode;
        return this;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void setPosition(double position) {
        setPosition.accept(inverted ? -position : position);
    }

    public void setSpeed(double speed) {
        setSpeed.accept(inverted ? -speed : speed);
    }

    public void setVoltage(double voltage) {
        setVoltage.accept(inverted ? -voltage : voltage);
    }

    public void stopMotor(){
        setSpeed.accept(0);
    }

    public boolean isConnected(){
        return getIsConnected.getAsBoolean();
    }

    public double getCurrentLimit() {
        return currentLimit;
    }

    public MotorNeutralMode getNeutralMode() {
        return motorNeutralMode;
    }

    public String getCanbus() {
        return canbus;
    }

    public int getId() {
        return id;
    }

    public MotorControllerType getMotorControllerType() {
        return type;
    }

    public String getName(){
        return getCanbus()+"\\"+getId()+"\\"+getMotorControllerType().name();
    }

    public void update(){
        encoder.update();
        this.temperature = getTemperature.getAsDouble();
    }


    public double getTemperature() {
        return temperature;
    }

    public enum Motor {
        KRAKEN_X60_FOC(MotorControllerType.CTRETalonFX, 5800),
        KRAKEN_X60(MotorControllerType.CTRETalonFX,6000),
        FALCON_500_FOC(MotorControllerType.CTRETalonFX,6080),
        FALCON_500(MotorControllerType.CTRETalonFX,6380),
        NEO_V1(MotorControllerType.REVSparkMaxBrushless,5820),
        NEO_VORTEX(MotorControllerType.REVSparkFlexBrushless,6784);

        private final int freeSpeedRPM;
        private final MotorControllerType controllerType;

        Motor(MotorControllerType controllerType, int freeSpeedRPM) {
            this.controllerType = controllerType;
            this.freeSpeedRPM = freeSpeedRPM;
        }

        public int getFreeSpeedRPM() {
            return freeSpeedRPM;
        }

        public MotorControllerType getMotorControllerType(){
            return controllerType;
        }

        public MotorControllerConfig config(int id){
            return getMotorControllerType().config(id);
        }
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout parentLayout, SendableLevel level) {
        encoder.shuffleboard(parentLayout);
        ShuffleboardLayout layout = parentLayout.getLayout(getName(), BuiltInLayouts.kList);
        if(level.equals(SendableLevel.DEBUG)){
            layout.addDouble("Current Limit", this::getCurrentLimit);
            layout.addString("Neutral Mode", () -> getNeutralMode().name());
            layout.addDouble("Tempature", this::getTemperature);
            layout.addBoolean("Inverted", this::isInverted);
        }
        layout.addBoolean("Is Connected", this::isConnected);
        if(pid != null){
            var pidLayout = layout.getLayout("PID", BuiltInLayouts.kList);
            pidLayout.addDouble("P", () -> pid.getP());
            pidLayout.addDouble("I", () -> pid.getI());
            pidLayout.addDouble("D", () -> pid.getD());
        }
        return layout;
    }
}
