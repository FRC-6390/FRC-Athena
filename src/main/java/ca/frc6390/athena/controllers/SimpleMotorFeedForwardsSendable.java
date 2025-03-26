package ca.frc6390.athena.controllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SimpleMotorFeedForwardsSendable extends SimpleMotorFeedforward implements Sendable {

    public SimpleMotorFeedForwardsSendable(double ks, double kv, double ka) {
        super(ks, kv, ka);
    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType(Builtin);
        builder.addDoubleProperty("kA", this::getKa, this::setKa);
        builder.addDoubleProperty("kS", this::getKs, this::setKs);
        builder.addDoubleProperty("kv", this::getKv, this::setKv);
    }    
}
