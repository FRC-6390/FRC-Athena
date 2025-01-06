package ca.frc6390.athena.drivetrains;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface DriveTrainBase {
    
    public void drive(ChassisSpeeds speeds);
}
