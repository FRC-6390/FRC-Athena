package ca.frc6390.athena.mechanisms;

import java.util.Objects;

import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;

/**
 * Builder-style config used by {@link MechanismConfig.SensorsSection} so teams don't need to inline
 * {@code new GenericLimitSwitchConfig(...)} in Constants.
 *
 * <p>This produces a {@link GenericLimitSwitchConfig} which is the runtime/data representation
 * consumed by the mechanism system.</p>
 */
public final class MechanismLimitSwitchConfig {

    private int id;
    private boolean inverted;
    private double position = Double.NaN;
    private boolean hardstop;
    private BlockDirection blockDirection = BlockDirection.None;
    private String name;
    private double delaySeconds;

    /**
     * DIO port for the switch. If negative, the switch is considered inverted.
     */
    public MechanismLimitSwitchConfig dio(int dioPort) {
        this.id = Math.abs(dioPort);
        this.inverted = dioPort < 0;
        return this;
    }

    /**
     * Explicitly sets inversion. This overrides the sign-based behavior of {@link #dio(int)}.
     */
    public MechanismLimitSwitchConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Optional position of the switch in mechanism units.
     */
    public MechanismLimitSwitchConfig position(double position) {
        this.position = position;
        return this;
    }

    /**
     * Marks this switch as a hardstop. When enabled, a block direction should be provided so the
     * mechanism can prevent motion into the stop.
     */
    public MechanismLimitSwitchConfig hardstop(boolean hardstop, BlockDirection blockDirection) {
        this.hardstop = hardstop;
        this.blockDirection = blockDirection != null ? blockDirection : BlockDirection.None;
        return this;
    }

    public MechanismLimitSwitchConfig blockDirection(BlockDirection blockDirection) {
        this.blockDirection = blockDirection != null ? blockDirection : BlockDirection.None;
        return this;
    }

    public MechanismLimitSwitchConfig name(String name) {
        this.name = Objects.requireNonNull(name, "name");
        return this;
    }

    public MechanismLimitSwitchConfig delaySeconds(double delaySeconds) {
        this.delaySeconds = delaySeconds;
        return this;
    }

    GenericLimitSwitchConfig build() {
        if (id <= 0) {
            throw new IllegalStateException("Limit switch DIO port must be > 0");
        }
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("Limit switch name must be set and non-blank");
        }
        return new GenericLimitSwitchConfig(id, inverted, position, hardstop, blockDirection, name, delaySeconds);
    }
}

