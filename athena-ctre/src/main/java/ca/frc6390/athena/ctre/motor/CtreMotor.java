package ca.frc6390.athena.ctre.motor;

import ca.frc6390.athena.sim.MotorSimType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * CTRE-specific motor models for simulation.
 */
public enum CtreMotor implements MotorSimType {
    KRAKEN_X44(count -> new DCMotor(12, 4.05, 275, 1.4,
            Units.rotationsPerMinuteToRadiansPerSecond(7532), count)),
    KRAKEN_X60(count -> DCMotor.getKrakenX60(count)),
    KRAKEN_X60_FOC(count -> DCMotor.getKrakenX60Foc(count)),
    FALCON_500(count -> DCMotor.getFalcon500(count)),
    FALCON_500_FOC(count -> DCMotor.getFalcon500Foc(count));

    private final MotorSimType factory;

    CtreMotor(MotorSimType factory) {
        this.factory = factory;
    }

    @Override
    public DCMotor createSimMotor(int count) {
        return factory.createSimMotor(count);
    }
}
