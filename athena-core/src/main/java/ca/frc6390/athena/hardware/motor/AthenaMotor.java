package ca.frc6390.athena.hardware.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import ca.frc6390.athena.sim.MotorSimType;

/**
 * Team-facing motor catalog that stays vendor-agnostic at the call site while mapping to concrete
 * vendor motor controller types through {@link MotorRegistry}. Each entry knows its free speed and
 * can generate a {@link DCMotor} model for simulation.
 */
public enum AthenaMotor implements MotorSimType {
    KRAKEN_X44("ctre:talonfx", 7532),
    KRAKEN_X60_FOC("ctre:talonfx", 5800),
    KRAKEN_X60("ctre:talonfx", 6000),
    FALCON_500_FOC("ctre:talonfx", 6080),
    FALCON_500("ctre:talonfx", 6380),
    NEO_V1("rev:sparkmax", 5820),
    NEO_VORTEX("rev:sparkflex", 6784);

    private final String motorKey;
    private final int freeSpeedRPM;

    AthenaMotor(String motorKey, int freeSpeedRPM) {
        this.motorKey = motorKey;
        this.freeSpeedRPM = freeSpeedRPM;
    }

    public MotorControllerType resolveController() {
        return MotorRegistry.get().motor(motorKey);
    }

    public int getFreeSpeedRPM() {
        return freeSpeedRPM;
    }

    @Override
    public DCMotor createSimMotor(int count) {
        int resolvedCount = Math.max(1, count);
        return switch (this) {
            case KRAKEN_X44 -> new DCMotor(12, 4.05, 275, 1.4,
                    Units.rotationsPerMinuteToRadiansPerSecond(7532), resolvedCount);
            case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(resolvedCount);
            case KRAKEN_X60 -> DCMotor.getKrakenX60(resolvedCount);
            case FALCON_500_FOC -> DCMotor.getFalcon500Foc(resolvedCount);
            case FALCON_500 -> DCMotor.getFalcon500(resolvedCount);
            case NEO_V1 -> DCMotor.getNEO(resolvedCount);
            case NEO_VORTEX -> DCMotor.getNeoVortex(resolvedCount);
        };
    }
}
