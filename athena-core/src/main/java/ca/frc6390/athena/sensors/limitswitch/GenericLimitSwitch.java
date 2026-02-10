package ca.frc6390.athena.sensors.limitswitch;

import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.sensors.EnhancedDigitalInput;
import java.util.function.BooleanSupplier;
public class GenericLimitSwitch extends EnhancedDigitalInput {

    public enum BlockDirection {
        None(0),
        PositiveInput(1),
        NegativeInput(-1);

        private final int multiplier;

        BlockDirection(int multiplier) {
            this.multiplier = multiplier;
        }

        public int multiplier() {
            return multiplier;
        }

        public static BlockDirection fromMultiplier(int multiplier) {
            if (multiplier > 0) {
                return PositiveInput;
            }
            if (multiplier < 0) {
                return NegativeInput;
            }
            return None;
        }
    }

    public record GenericLimitSwitchConfig(int id, boolean inverted, double position, boolean isHardstop, BlockDirection blockDirection, String name, double delaySeconds) {

        public static GenericLimitSwitchConfig create(int id){
            return new GenericLimitSwitchConfig(Math.abs(id) ,id<0,  Double.NaN, false, BlockDirection.None, null, 0);
        }

        public GenericLimitSwitchConfig setPosition(double position){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection, name, delaySeconds);
        }

        public GenericLimitSwitchConfig setHardstop(boolean isHardstop, BlockDirection blockDirection){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection, name, delaySeconds);
        }

        /** @deprecated Use {@link #setHardstop(boolean, BlockDirection)} instead. */
        @Deprecated(forRemoval = false)
        public GenericLimitSwitchConfig setHardstop(boolean isHardstop, int blockDirection){
            return setHardstop(isHardstop, BlockDirection.fromMultiplier(blockDirection));
        }

        public GenericLimitSwitchConfig setName(String name){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection, name, delaySeconds);
        }

        public GenericLimitSwitchConfig setDelay(double delaySeconds){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop, blockDirection, name, delaySeconds);
        }

        public GenericLimitSwitch create(){
            return new GenericLimitSwitch(id, inverted).applyConfig(this);
        }

        public BooleanSupplier toSupplier(){
            GenericLimitSwitch sensor = create();
            sensor.setDelay(delaySeconds);
            return sensor::getAsBoolean;
        }

    }

    private double position = Double.NaN;
    private boolean isHardstop;
    private BlockDirection blockDirection = BlockDirection.None;
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
        setDelay(config.delaySeconds);
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

    public void setBlockDirection(BlockDirection blockDirection) {
        this.blockDirection = blockDirection != null ? blockDirection : BlockDirection.None;
    }

    /** @deprecated Use {@link #setBlockDirection(BlockDirection)} instead. */
    @Deprecated(forRemoval = false)
    public void setBlockDirection(int blockDirection) {
        setBlockDirection(BlockDirection.fromMultiplier(blockDirection));
    }

    public BlockDirection getBlockDirection() {
        return blockDirection;
    }

    public int getBlockDirectionMultiplier() {
        return blockDirection.multiplier();
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        node = super.networkTables(node);
        if (node == null || !node.robot().isPublishingEnabled()) {
            return node;
        }
        node.putDouble("blockDirection", getBlockDirectionMultiplier());
        node.putBoolean("hardstop", isHardstop());
        node.putDouble("position", getPosition());
        return node;
    }
    
}
