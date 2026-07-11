package frc.robot.mechanisms;

import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.MechanismTemplate;
import ca.frc6390.athena.mechanism.core.MotorSlot;
import ca.frc6390.athena.mechanism.core.Slots;

public final class TemplateRoller implements MechanismTemplate {
    public final MotorSlot<TemplateRoller> motor = Slots.motor(this, "motor", this::configureIfReady)
            .coast()
            .currentLimit(20);

    private SimModel simulation;

    public Action run;
    public Action stop;

    private void configureIfReady() {
        if (motor.filled()) {
            simulation = SimModel.motor(motor.get()).encoder(motor.get().encoder());
            run = motor.get().percent(0.65);
            stop = motor.get().neutral();
        }
    }
}
