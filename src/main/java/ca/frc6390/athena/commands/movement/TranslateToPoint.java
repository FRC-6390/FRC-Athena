package ca.frc6390.athena.commands.movement;

import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.IMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TranslateToPoint extends Command{

    private final RobotSpeeds speeds;
    private final IMU imu;
    private PIDController translationPID;

    public TranslateToPoint(RobotDrivetrain<?> drivetrain, double x, double y){
        this.speeds = drivetrain.getRobotSpeeds();
        this.imu = drivetrain.getIMU();
        this.translationPID = new PIDController(0, 0, 0);
        this.translationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.translationPID.setTolerance(0.05);
    }

    public TranslateToPoint setPID(PIDController rotationPID){
        this.translationPID = rotationPID;
        this.translationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.translationPID.setTolerance(0.05);
        return this;
    }

    @Override
    public void initialize() {
        translationPID.reset();
    }

    @Override
    public void execute() {
       
    }

    @Override
    public boolean isFinished() {
        return translationPID.atSetpoint();
    }
}
