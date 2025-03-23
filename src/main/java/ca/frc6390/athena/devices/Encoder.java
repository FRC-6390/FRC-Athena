package ca.frc6390.athena.devices;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class Encoder implements RobotSendableDevice{

    private final DoubleConsumer setPosition;
    private final BooleanSupplier getIsConnected;
    private final DoubleSupplier getPosition, getAbsolutePosition, getVelocity;
    private double absolutePosition = 0, position = 0, velocity = 0, gearRatio = 1, offset = 0, conversion = 1, conversionOffset = 1;
    private boolean inverted = false;
    private String canbus;
    private int id;
    private EncoderType type;

    public Encoder(TalonFX motor){
        getAbsolutePosition = () -> motor.getRotorPosition(true).getValueAsDouble();
        getPosition = () -> motor.getPosition(true).getValueAsDouble();
        setPosition = (val) -> motor.setPosition(val);
        getVelocity = () -> motor.getRotorVelocity(true).getValueAsDouble();
        getIsConnected = () -> motor.isConnected();
    }
    

    public Encoder(CANcoder encoder){
        getAbsolutePosition = () -> encoder.getAbsolutePosition(true).getValueAsDouble();
        getPosition = () -> encoder.getPosition(true).getValueAsDouble();
        setPosition = (val) -> encoder.setPosition(val);
        getVelocity = () -> encoder.getVelocity(true).getValueAsDouble();
        getIsConnected = () -> encoder.isConnected();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoder.getConfigurator().apply(encoderConfig);
    }

    public Encoder(RelativeEncoder relativeEncoder, AbsoluteEncoder absoluteEncoder){
        this(relativeEncoder::setPosition,absoluteEncoder::getPosition, relativeEncoder::getPosition, relativeEncoder::getVelocity, () -> relativeEncoder.setPosition(relativeEncoder.getPosition()).equals(REVLibError.kCANDisconnected));
        
    }

    public Encoder(edu.wpi.first.wpilibj.Encoder encoder, double dpp){
        this( (val) -> encoder.reset(), encoder::getRaw, encoder::getDistance, encoder::getRate, () -> true);
        encoder.setDistancePerPulse(dpp);
    }

    public Encoder(DoubleConsumer setPosition, DoubleSupplier getAbsolutePosition, DoubleSupplier getPosition, DoubleSupplier getVelocity, BooleanSupplier getIsConnected){
        this.getAbsolutePosition = getAbsolutePosition;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        this.setPosition = setPosition;
        this.getIsConnected = getIsConnected;
    }

    public static Encoder newCTRECANCoder(int id){
        return new Encoder(new CANcoder(id));
    }

    public static Encoder newCTRECANCoder(int id, String canbus){
        return new Encoder(new CANcoder(id, canbus));
    }

    public static Encoder newREVSparkMax(SparkMax motor){
        return new Encoder(motor.getEncoder(), motor.getAbsoluteEncoder());
    }

    public static Encoder newREVSparkFlex(SparkFlex motor){
        return new Encoder(motor.getEncoder(), motor.getAbsoluteEncoder());
    }

    public static Encoder newWPILib(int channelA, int channelB, double ddp){
        edu.wpi.first.wpilibj.Encoder encoder = new edu.wpi.first.wpilibj.Encoder(channelA, channelB);
        return new Encoder(encoder,ddp);
    }

    public static Encoder fromConfig(EncoderConfig config) {
        switch (config.type) {
            case CTRETalonFXEncoder:
            return new Encoder(new TalonFX(config.id, config.canbus)).applyConfig(config);
            case CTRECANcoder:
            return new Encoder(new CANcoder(config.id, config.canbus)).applyConfig(config);
            case REVSparkFlexEncoder:
                DriverStation.reportError("Cannot create REVRelativeEncoder without motor, please make encoder using the motor!", null); 
            return null;
            case REVSparkMaxEncoder:
                DriverStation.reportError("Cannot create REVAbsoluteEncoder without motor, please make encoder using the motor!", null); 
            return null;
            case WPILibEncoder:
                DriverStation.reportError("Cannot create WPILibEncoder without channels, please make encoder using the newWPILib!", null); 
            return null;
            default:
            return null;
        }
    }

    public Encoder applyConfig(EncoderConfig config) {
        setGearRatio(config.gearRatio);
        setConversion(config.conversion);
        setOffset(config.offset);
        setInverted(config.inverted);
        setConversionOffset(config.conversionOffset);
        canbus = config.canbus;
        id = config.id;
        type = config.type;
        return this;
    }

    public Encoder setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public Encoder setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    public Encoder setConversion(double conversion) {
        this.conversion = conversion;
        return this;
    }
    public Encoder setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
        return this;
    }

    public Encoder setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public void setPosition(double pos) {
        setRaw(((pos + conversionOffset) / conversion + offset) / gearRatio);
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getConversion() {
        return conversion;
    }

    public double getConversionOffset() {
        return conversionOffset;
    }

    public double getOffset() {
        return offset;
    }

    public double getRawValue() {
        return position;
    }

    public double getRawAbsoluteValue() {
        return absolutePosition;
    }

    public double getRawVelocity() {
        return velocity;
    }

    public boolean isInverted() {
        return inverted;
    }

    /**
     * Get the rotations of the encoder with gearRatio and offset applied
     */
    public double getRotations() {
        return position * gearRatio - offset;
    }

     /**
     * Get the absolute rotations of the encoder with gearRatio and offset applied
     */
    public double getAbsoluteRotations() {
        return absolutePosition * gearRatio - offset;
    }

    /**
     * Get the rotations of the encoder with gearRatio, offsets and conversion applied
     */
    public double getPosition() {
        return (getRotations() * conversion) - conversionOffset;
    }

      /**
     * Get the absolute rotations of the encoder with gearRatio, offsets and conversion applied
     */
    public double getAbsolutePosition() {
        return (getAbsoluteRotations() * conversion) - conversionOffset;
    }

     /**
     * Get the rotations of the encoder with gearRatio and offset applied
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getRotations());
    }
    
      /**
     * Get the absolute rotations of the encoder with gearRatio and offset applied
     */
    public Rotation2d getAbsoluteRotation2d() {
        return Rotation2d.fromRotations(getAbsoluteRotations());
    }


     /**
     * Get the velocity in rotations of the encoder with gearRatio and offset applied
     */
    public double getRate() {
        return velocity * gearRatio + offset;
    }

     /**
     * Get the velocity of the encoder with gearRatio, offset and conversion applied
     */
    public double getVelocity() {
        return getRate() * conversion;
    }

    public boolean isConnected(){
        return getIsConnected.getAsBoolean();
    }

    public Encoder update() {
        position = inverted ? -getPosition.getAsDouble() : getPosition.getAsDouble();
        velocity = inverted ? -getVelocity.getAsDouble() : getVelocity.getAsDouble();
        absolutePosition = inverted ? -getAbsolutePosition.getAsDouble() : getAbsolutePosition.getAsDouble();
        return this;
    }

    public void setRotations(double rotations){
        setRaw((rotations + offset) / gearRatio);
    }

    public void setRaw(double raw){
        setPosition.accept(raw);
    }

    public String getCanbus() {
        return canbus;
    }

    public int getId() {
        return id;
    }

    public EncoderType getEncoderType() {
        return type;
    }

    public String getName(){
        return getCanbus()+"\\"+getId()+"\\"+getEncoderType().name();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.addDouble("Gear Ratio", this::getGearRatio);
        layout.addBoolean("Inverted", this::isInverted);
        layout.addDouble("Conversion", this::getConversion);
        layout.addDouble("Conversion Offset", this::getConversionOffset);
        layout.addDouble("Position", this::getPosition);
        layout.addDouble("Rotations", this::getRawValue);
        layout.addDouble("Absolute Position", this::getAbsolutePosition);
        layout.addDouble("Absolute Roation", this::getAbsoluteRotations);
        layout.addDouble("Raw Velocity", this::getRawVelocity);
        layout.addDouble("Velocity", this::getVelocity);
        layout.addBoolean("Is Connected", this::isConnected);
        layout.addString("Canbus", this::getCanbus);
        layout.addInteger("Id", this::getId);
        layout.addString("Encoder Type", () -> this.getEncoderType().name());
        return layout;
    }
}
