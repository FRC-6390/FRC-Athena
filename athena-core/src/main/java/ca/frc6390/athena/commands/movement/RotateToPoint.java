package ca.frc6390.athena.commands.movement;

import ca.frc6390.athena.core.RobotCore;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class RotateToPoint extends RotateToAngle {

    public RotateToPoint(RobotCore<?> base, double x, double y){
        super(base);
        this.setSupplierRadians((pose) -> Math.atan2(y - pose.getY(), x - pose.getX()));
    }

    public RotateToPoint setPID(ProfiledPIDController rotationPID){
        return (RotateToPoint) super.setPID(rotationPID);
    }
}
