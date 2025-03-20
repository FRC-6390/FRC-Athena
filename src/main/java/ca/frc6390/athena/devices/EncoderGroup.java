package ca.frc6390.athena.devices;

import java.util.Arrays;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class EncoderGroup implements RobotSendableDevice{
    
    private Encoder[] encoders;

    public EncoderGroup(Encoder... encoders){
        this.encoders = encoders;
    }

    public static EncoderGroup fromConfigs(EncoderConfig... configs){
        return new EncoderGroup(Arrays.stream(configs).map(Encoder::fromConfig).toArray(Encoder[]::new));
    }

    public static EncoderGroup fromMotorGroup(MotorControllerGroup motors){
        return new EncoderGroup(Arrays.stream(motors.getControllers()).map((m) -> m.getEncoder().setInverted(m.isInverted())).toArray(Encoder[]::new));
    }

    public Encoder[] getEncoders() {
        return encoders;
    }

    public double getVelocity(){
        double value = 0;
        for (Encoder encoder : encoders) {
            value += encoder.getVelocity();
        }

        return value / encoders.length;
    }

    public double getPosition(){
        double value = 0;
        for (Encoder encoder : encoders) {
            value += encoder.getPosition();
        }

        return value / encoders.length;
    }

    public double getRotations(){
        double value = 0;
        for (Encoder encoder : encoders) {
            value += encoder.getRotations();
        }

        return value / encoders.length;
    }

    public double getRate(){
        double value = 0;
        for (Encoder encoder : encoders) {
            value += encoder.getRate();
        }

        return value / encoders.length;
    }

    public void setPosition(double position){
        for (Encoder encoder : encoders) {
            encoder.setPosition(position);
        }
    }

    public Rotation2d getRotation2d(){
       return Rotation2d.fromRotations(getRotations());
    }

    public boolean allEncodersConnected(){
        for (Encoder encoder : encoders) {
            if(!encoder.isConnected()){
                return false;
            }
        }
        return true;
    }

    public void update(){
        for (Encoder encoder : encoders) {
            encoder.update();
        }
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        
        for (Encoder encoder : encoders) {
            encoder.shuffleboard(layout.getLayout(encoder.getName(), BuiltInLayouts.kList));
        }

        return layout;
    }


}
