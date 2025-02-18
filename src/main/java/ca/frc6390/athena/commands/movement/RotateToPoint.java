package ca.frc6390.athena.commands.movement;

import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;

public class RotateToPoint extends RotateToAngle {

    public RotateToPoint(RobotBase<?> base, double x, double y, boolean relative){
        super(base, relative);
        this.setSupplier((pose) -> Units.radiansToDegrees(Math.atan2(y - pose.getY(), x - pose.getX())));
    }

    public RotateToPoint setPID(ProfiledPIDController rotationPID){
        return (RotateToPoint) super.setPID(rotationPID);
    }
}
