package ca.frc6390.athena.devices;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorControllerType;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class MotorController implements RobotSendableDevice {

    private final DoubleConsumer setSpeed, setVoltage, setCurrentLimit;
    private final BooleanSupplier getIsConnected;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Encoder encoder;
    private boolean inverted = false;
    private double currentLimit = 0;
    private MotorNeutralMode motorNeutralMode;
    private String canbus;
    private int id;
    private MotorControllerType type;

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
            },
            controller::isConnected,
            new Encoder(controller)
        );
    }

    public MotorController(SparkMax controller){
        this(
            (mode) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            () -> controller.getFaults().can,
            Encoder.newREVSparkMax(controller)
        );
    }

    public MotorController(SparkFlex controller){
        this(
            (mode) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            () -> controller.getFaults().can,
            Encoder.newREVSparkFlex(controller)
        );
    }

    public MotorController(Consumer<MotorNeutralMode> setNeutralMode, DoubleConsumer setSpeed, DoubleConsumer setVoltage, DoubleConsumer setCurrentLimit, BooleanSupplier getIsConnected, Encoder encoder) {
        this.setNeutralMode = setNeutralMode;
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.getIsConnected = getIsConnected;
        this.encoder = encoder;
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

    public MotorController setNeutralMode(MotorNeutralMode mode){
        setNeutralMode.accept(mode);
        motorNeutralMode = mode;
        return this;
    }

    public Encoder getEncoder() {
        return encoder;
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
    }

    public enum Motor {
        KRAKEN_X60_FOC(MotorControllerType.CTRETalonFX, 5800, true),
        KRAKEN_X60(MotorControllerType.CTRETalonFX,6000, false),
        FALCON_500_FOC(MotorControllerType.CTRETalonFX,6080, true),
        FALCON_500(MotorControllerType.CTRETalonFX,6380, false),
        NEO_V1(MotorControllerType.REVSparkFlexBrushless,5820, false),
        NEO_VORTEX(MotorControllerType.REVSparkFlexBrushless,6784, false);

        private final int freeSpeedRPM;
        private final boolean foc;
        private final MotorControllerType controllerType;

        Motor(MotorControllerType controllerType, int freeSpeedRPM, boolean foc) {
            this.controllerType = controllerType;
            this.freeSpeedRPM = freeSpeedRPM;
            this.foc = foc;
        }

        public int getFreeSpeedRPM() {
            return freeSpeedRPM;
        }

        public boolean isFOC() {
            return foc;
        }

        public MotorControllerType getMotorControllerType(){
            return controllerType;
        }
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        encoder.shuffleboard(layout.getLayout(encoder.getName(), BuiltInLayouts.kList));
        layout.addBoolean("Inverted", this::isInverted);
        layout.addBoolean("Is Connected", this::isConnected);
        layout.addDouble("Current Limit", this::getCurrentLimit);
        layout.addString("Neutral Mode", () -> getNeutralMode().name());
        layout.addString("Canbus", this::getCanbus);
        layout.addInteger("Id", this::getId);
        layout.addString("Motor Controller", () -> this.getMotorControllerType().name());
    
        return layout;
    }
}
