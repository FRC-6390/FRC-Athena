package ca.frc6390.athena.controllers;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ElevatorFeedForwardsSendable extends ElevatorFeedforward implements Sendable {

    public ElevatorFeedForwardsSendable(double ks, double kg, double kv, double ka) {
        super(ks, kg, kv, ka);
    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType(Builtin);
        builder.addDoubleProperty("kA", this::getKa, this::setKa);
        builder.addDoubleProperty("kG", this::getKg, this::setKg);
        builder.addDoubleProperty("kS", this::getKs, this::setKs);
        builder.addDoubleProperty("kv", this::getKv, this::setKv);
    }    
}
