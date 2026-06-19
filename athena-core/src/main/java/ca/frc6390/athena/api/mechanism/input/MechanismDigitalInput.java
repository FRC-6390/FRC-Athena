package ca.frc6390.athena.api.mechanism.input;

import java.util.OptionalDouble;

import ca.frc6390.athena.api.mechanism.definition.MechanismDigitalInputDefinition;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;

public final class MechanismDigitalInput {
    private String name;
    private Integer port;
    private boolean inverted;
    private Double position;
    private boolean hardstop;
    private BlockDirection blockDirection = BlockDirection.None;
    private double delaySeconds;

    private MechanismDigitalInput() {
    }

    public static MechanismDigitalInput create() {
        return new MechanismDigitalInput();
    }

    public static MechanismDigitalInput create(String name) {
        return create().named(name);
    }

    public static MechanismDigitalInput from(MechanismDigitalInputDefinition definition) {
        MechanismDigitalInput input = create().named(definition.name()).port(definition.port());
        if (definition.inverted()) {
            input.inverted(true);
        }
        if (definition.position().isPresent()) {
            input.position(definition.position().getAsDouble());
        }
        if (definition.hardstop()) {
            input.hardstop(true, BlockDirection.fromMultiplier(definition.blockDirection()));
        }
        if (Double.isFinite(definition.delaySeconds()) && definition.delaySeconds() > 0.0) {
            input.delaySeconds(definition.delaySeconds());
        }
        return input;
    }

    public MechanismDigitalInput named(String name) {
        this.name = name;
        return this;
    }

    public MechanismDigitalInput port(int port) {
        this.port = Math.abs(port);
        if (port < 0) {
            this.inverted = true;
        }
        return this;
    }

    public MechanismDigitalInput inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MechanismDigitalInput position(double position) {
        this.position = position;
        return this;
    }

    public MechanismDigitalInput hardstop(boolean hardstop, BlockDirection blockDirection) {
        this.hardstop = hardstop;
        this.blockDirection = blockDirection != null ? blockDirection : BlockDirection.None;
        return this;
    }

    public MechanismDigitalInput delaySeconds(double delaySeconds) {
        this.delaySeconds = delaySeconds;
        return this;
    }

    public MechanismDigitalInputDefinition definition() {
        int resolvedPort = requiredPort();
        String resolvedName = name != null && !name.isBlank()
            ? name
            : Integer.toString(resolvedPort);
        return new MechanismDigitalInputDefinition(
            resolvedName,
            resolvedPort,
            inverted,
            position != null && Double.isFinite(position) ? OptionalDouble.of(position) : OptionalDouble.empty(),
            hardstop,
            blockDirection.multiplier(),
            delaySeconds,
            getClass());
    }

    private int requiredPort() {
        if (port == null) {
            throw new IllegalStateException("digital input port is required");
        }
        return port;
    }
}
