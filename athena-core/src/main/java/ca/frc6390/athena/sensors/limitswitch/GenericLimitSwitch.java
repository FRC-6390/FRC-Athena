package ca.frc6390.athena.sensors.limitswitch;

import ca.frc6390.athena.sensors.EnhancedDigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
public class GenericLimitSwitch extends EnhancedDigitalInput {

    public record GenericLimitSwitchConfig(int id, boolean inverted, double position, boolean isHardstop, int blockDirection) {

        public static GenericLimitSwitchConfig create(int id){
            return new GenericLimitSwitchConfig(Math.abs(id) ,id<0,  Double.NaN, false, 0);
        }

        public GenericLimitSwitchConfig setPosition(double position){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection);
        }

        public GenericLimitSwitchConfig setHardstop(boolean isHardstop, int blockDirection){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection);
        }

        public GenericLimitSwitch create(){
            return new GenericLimitSwitch(id, inverted).applyConfig(this);
        }

    }

    private double position = Double.NaN;
    private boolean isHardstop;
    private int blockDirection;
    /**
     * Constructs an GenericLimitSwitch sensor.
     *
     * @param port The digital input port number where the switch sensor is connected.
     */
    public GenericLimitSwitch(int port) {
        this(port, port < 0);
    }
 
    public GenericLimitSwitch(int port, boolean inverted) {
        super(port, inverted);
    }

    public static GenericLimitSwitch fromConfig(GenericLimitSwitchConfig config){
        return new GenericLimitSwitch(config.id, config.inverted).applyConfig(config);
    }

    public GenericLimitSwitch applyConfig(GenericLimitSwitchConfig config){
        setPosition(config.position);
        setHardstop(config.isHardstop);
        setBlockDirection(config.blockDirection);
        return this;
    }

    public GenericLimitSwitch setPosition(double position) {
        this.position = position;
        return this;
    }

    public double getPosition() {
        return position;
    }

    public boolean isHardstop() {
        return isHardstop;
    }

    public void setHardstop(boolean isHardstop) {
        this.isHardstop = isHardstop;
    }

    public void setBlockDirection(int blockDirection) {
        this.blockDirection = blockDirection;
    }

    public int getBlockDirection() {
        return blockDirection;
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        super.shuffleboard(layout);
        layout.addDouble("Block Direction", this::getBlockDirection);
        layout.addBoolean("Is Hardstop", this::isHardstop);
        layout.addDouble("Position", this::getPosition);
        return layout;
    }
    
}
