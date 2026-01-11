package ca.frc6390.athena.sim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Core motor enum kept vendor-free for simulation defaults.
 */
public enum Motor implements MotorSimType {
    CIM(count -> DCMotor.getCIM(count)),
    MINI_CIM(count -> DCMotor.getMiniCIM(count)),
    BAG(count -> DCMotor.getBAG(count)),
    RS_775(count -> DCMotor.getRS775(count)),
    PRO_775(count -> DCMotor.get775Pro(count));

    private final MotorSimType factory;

    Motor(MotorSimType factory) {
        this.factory = factory;
    }

    @Override
    public DCMotor createSimMotor(int count) {
        return factory.createSimMotor(count);
    }
}
