package ca.frc6390.athena.devices;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import ca.frc6390.athena.core.RobotSendableSystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Encoder implements RobotSendableSystem{

    private final DoubleConsumer setPosition;
    private final DoubleSupplier getPosition, getAbsolutePosition, getVelocity;
    private double absolutePosition = 0, position = 0, velocity = 0, gearRatio = 1, offset = 0, conversion = 1, conversionOffset = 1;
    private boolean inverted = false;

    public enum EncoderType {
        None,
        CTRETalonFX,
        CTRECANcoder,
        REVSparkFlex,
        REVSparkMax,
        WPILibEncoder,
    }

    public record EncoderConfig(EncoderType type, int id, String canbus, double gearRatio, double offset, double conversion, boolean inverted, double conversionOffset) {

        public EncoderConfig(){
            this(EncoderType.None, -1, "rio", 1, 0, 1, false,0);
        }
        
        public EncoderConfig(EncoderType type){
            this(type, -1, "rio", 1, 0, 1, false,0);
        }

        public EncoderConfig(EncoderType type, int id){
            this(type, Math.abs(id), "rio", 1, 0, 1, id < 0,0);
        }

        public EncoderConfig setCanbus(String canbus){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion, inverted, conversionOffset);
        }

        public EncoderConfig setOffset(double offset){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setGearRatio(double gearRatio){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setConversion(double conversion){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setInverted(boolean inverted){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setID(int id){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setConversionOffset(double conversionOffset){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }

        public EncoderConfig setEncoderType(EncoderType type){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted, conversionOffset);
        }
    }

    public Encoder(TalonFX motor){
        getAbsolutePosition = () -> motor.getRotorPosition(true).getValueAsDouble();
        getPosition = () -> motor.getPosition(true).getValueAsDouble();
        setPosition = (val) -> motor.setPosition(val);
        getVelocity = () -> motor.getRotorVelocity(true).getValueAsDouble();
    }
    

    public Encoder(CANcoder encoder){
        getAbsolutePosition = () -> encoder.getAbsolutePosition(true).getValueAsDouble();
        getPosition = () -> encoder.getPosition(true).getValueAsDouble();
        setPosition = (val) -> encoder.setPosition(val);
        getVelocity = () -> encoder.getVelocity(true).getValueAsDouble();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoder.getConfigurator().apply(encoderConfig);
    }

    public Encoder(RelativeEncoder relativeEncoder, AbsoluteEncoder absoluteEncoder){
        this(relativeEncoder::setPosition,absoluteEncoder::getPosition, relativeEncoder::getPosition, relativeEncoder::getVelocity);
    }

    public Encoder(edu.wpi.first.wpilibj.Encoder encoder, double dpp){
        this( (val) -> encoder.reset(), encoder::getRaw, encoder::getDistance, encoder::getRate);
        encoder.setDistancePerPulse(dpp);
    }

    public Encoder(DoubleConsumer setPosition, DoubleSupplier getAbsolutePosition, DoubleSupplier getPosition, DoubleSupplier getVelocity){
        this.getAbsolutePosition = getAbsolutePosition;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        this.setPosition = setPosition;
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
            case CTRETalonFX:
            return new Encoder(new TalonFX(config.id, config.canbus)).applyConfig(config);
            case CTRECANcoder:
            return new Encoder(new CANcoder(config.id, config.canbus)).applyConfig(config);
            case REVSparkFlex:
                DriverStation.reportError("Cannot create REVRelativeEncoder without motor, please make encoder using the motor!", null); 
            return null;
            case REVSparkMax:
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
        setPosition.accept(pos);
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
        return (getRotations() * conversion)+ conversionOffset;
    }

      /**
     * Get the absolute rotations of the encoder with gearRatio, offsets and conversion applied
     */
    public double getAbsolutePosition() {
        return (getAbsoluteRotations() * conversion) + conversionOffset;
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

    public Encoder update() {
        position = inverted ? -getPosition.getAsDouble() : getPosition.getAsDouble();
        velocity = inverted ? -getVelocity.getAsDouble() : getVelocity.getAsDouble();
        absolutePosition = inverted ? -getAbsolutePosition.getAsDouble() : getAbsolutePosition.getAsDouble();
        return this;
    }


    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        tab.addDouble("Gear Ratio", this::getGearRatio);
        tab.addBoolean("Intervetd", this::isInverted);
        tab.addDouble("Conversion", this::getConversion);
        tab.addDouble("Conversion Offset", this::getConversionOffset);
        tab.addDouble("Position", this::getPosition);
        tab.addDouble("Rotations", this::getRawValue);
        tab.addDouble("Absolute Positon", this::getAbsolutePosition);
        tab.addDouble("Absolute Roation", this::getAbsoluteRotations);
        tab.addDouble("Raw Velocity", this::getRawVelocity);
        tab.addDouble("Velocity", this::getVelocity);
        throw new UnsupportedOperationException("Unimplemented method 'shuffleboard'");
    }
}
