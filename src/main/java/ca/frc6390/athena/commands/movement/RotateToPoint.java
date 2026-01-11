package ca.frc6390.athena.commands.movement;

import ca.frc6390.athena.core.RobotCore;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;

public class RotateToPoint extends RotateToAngle {

    public RotateToPoint(RobotCore<?> base, double x, double y){
        super(base);
        this.setSupplier((pose) -> Units.radiansToDegrees(Math.atan2(y - pose.getY(), x - pose.getX())));
    }

    public RotateToPoint setPID(ProfiledPIDController rotationPID){
        return (RotateToPoint) super.setPID(rotationPID);
    }
}
