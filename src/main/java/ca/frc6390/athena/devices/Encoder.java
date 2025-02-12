package ca.frc6390.athena.devices;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Encoder {

    private final DoubleConsumer setPosition;
    private final DoubleSupplier getPosition, getVelocity;
    private double position, velocity, gearRatio = 1, offset = 0, conversion = 1;
    private boolean inverted = false;

    public enum EncoderType {
        None,
        CTRETalonFX,
        CTRECANcoder,
        REVRelativeEncoder,
        REVAbsoluteEncoder,
        WPILibEncoder,
    }

    public record EncoderConfig(EncoderType type, int id, String canbus, double gearRatio, double offset, double conversion, boolean inverted) {

        public EncoderConfig(){
            this(EncoderType.None, -1, "rio", 1, 0, 1, false);
        }
        
        public EncoderConfig(EncoderType type, int id){
            this(type, id, "rio", 1, 0, 1, false);
        }

        public EncoderConfig withCanbus(String canbus){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion, inverted);
        }

        public EncoderConfig withOffset(double offset){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted);
        }

        public EncoderConfig withGearRatio(double gearRatio){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted);
        }

        public EncoderConfig withConversion(double conversion){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted);
        }

        public EncoderConfig setInverted(boolean inverted){
            return new EncoderConfig(type, id, canbus, gearRatio, offset, conversion,inverted);
        }
    }

    public Encoder(TalonFX motor){
        this(motor, false);
    }

    public Encoder(TalonFX motor, boolean absolute){
        getPosition = absolute ?  () -> motor.getRotorPosition(true).getValueAsDouble() : () -> motor.getPosition(true).getValueAsDouble();
        setPosition = absolute ? (val) -> DriverStation.reportError("Cannot set position of absolute encoder!", null) : (val) -> motor.setPosition(val);
        getVelocity = absolute ? ()-> motor.getRotorVelocity(true).getValueAsDouble() : () -> motor.getVelocity(true).getValueAsDouble();
    }
    
    public Encoder(CANcoder encoder){
       this(encoder, false);
    }

    public Encoder(CANcoder encoder, boolean absolute){
        getPosition = absolute ?  () -> encoder.getAbsolutePosition(true).getValueAsDouble() : () -> encoder.getPosition(true).getValueAsDouble();
        setPosition = absolute ? (val) -> DriverStation.reportError("Cannot set position of absolute encoder!", null) : (val) -> encoder.setPosition(val);
        getVelocity = () -> encoder.getVelocity(true).getValueAsDouble();
    }

    public Encoder(RelativeEncoder encoder){
        this(encoder::setPosition, encoder::getPosition, encoder::getVelocity);
    }

    public Encoder(AbsoluteEncoder encoder){
        this(encoder::getPosition, encoder::getVelocity);
    }

    public Encoder(DoubleSupplier getPosition, DoubleSupplier getVelocity){
        this((val) -> DriverStation.reportError("Cannot set position of absolute encoder!", null), getPosition, getVelocity);
    }

    public Encoder(edu.wpi.first.wpilibj.Encoder encoder, double dpp){
        this((val) -> encoder.reset(), encoder::getDistance, encoder::getRate);
        encoder.setDistancePerPulse(dpp);
    }

    public Encoder(DoubleConsumer setPosition, DoubleSupplier getPosition, DoubleSupplier getVelocity){
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        this.setPosition = setPosition;
    }

    public static Encoder newCTRECANCoder(int id){
        return new Encoder(new CANcoder(id));
    }

    public static Encoder newCTRECANCoderAbsolute(int id){
        return new Encoder(new CANcoder(id), true);
    }

    public static Encoder newCTRECANCoder(int id, String canbus){
        return new Encoder(new CANcoder(id, canbus));
    }

    public static Encoder newCTRECANCoderAbsolute(int id, String canbus){
        return new Encoder(new CANcoder(id, canbus), true);
    }

    public static Encoder newREVRelative(SparkMax motor){
        return new Encoder(motor.getEncoder());
    }

    public static Encoder newREVAbsolute(SparkMax motor){
        return new Encoder(motor.getAbsoluteEncoder());
    }

    public static Encoder newWPILib(SparkMax motor){
        return new Encoder(motor.getAbsoluteEncoder());
    }

    public static Encoder fromConfig(EncoderConfig config) {
        switch (config.type) {
            case CTRETalonFX:
            return new Encoder(new TalonFX(config.id, config.canbus)).applyConfig(config);
            case CTRECANcoder:
            return new Encoder(new CANcoder(config.id, config.canbus)).applyConfig(config);
            case REVRelativeEncoder:
                DriverStation.reportError("Cannot create REVRelativeEncoder without motor, please make encoder using the motor!", null); 
            return null;
            case REVAbsoluteEncoder:
                DriverStation.reportError("Cannot create REVAbsoluteEncoder without motor, please make encoder using the motor!", null); 
            return null;
            case WPILibEncoder:
                DriverStation.reportError("Cannot create WPILibEncoder without motor, please make encoder using the motor!", null); 
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

    public Encoder setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public void setPosition(double pos) {
        setPosition.accept(pos);
    }

    public double getRawPosition() {
        return position;
    }

    public double getRawVelocity() {
        return velocity;
    }

    /**
     * Get the rotations of the encoder with gearRatio and offset applied
     */
    public double getRotations() {
        return position * gearRatio - offset;
    }

    /**
     * Get the rotations of the encoder with gearRatio, offset and conversion applied
     */
    public double getPosition() {
        return getRotations() * conversion;
    }

     /**
     * Get the rotations of the encoder with gearRatio and offset applied
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getRotations());
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

    public void update() {
        position = inverted ? -getPosition.getAsDouble() : getPosition.getAsDouble();
        velocity = inverted ? -getVelocity.getAsDouble() : getVelocity.getAsDouble();
    }
}
