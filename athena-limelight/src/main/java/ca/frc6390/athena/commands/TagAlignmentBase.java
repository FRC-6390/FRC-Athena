package ca.frc6390.athena.commands;

import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.camera.LocalizationCameraCapability;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TagAlignmentBase extends Command {

   public RobotSpeeds robotSpeeds;
   public LimeLight camera;
   public long runTag;
   public Pose2d curPose;
   public MedianFilter filter;
   public ProfiledPIDController xController,yController, thetaController;
   public DelayedOutput endCommand;
   public DelayedOutput noTag;
   public final Runnable alignmentComplete;
   public final Function<Long, Double> offsetProvider;

   public TagAlignmentBase(RobotCore<SwerveDrivetrain> base, String table, Function<Long, Double> offsetProvider, Runnable alignmentComplete)
   {
    this.alignmentComplete = alignmentComplete;
    this.offsetProvider = offsetProvider;

    this.xController = base.getAutos().getXController();
    this.yController = base.getAutos().getYController();
    this.thetaController = base.getAutos().getThetaController();

    this.robotSpeeds = base.getDrivetrain().getRobotSpeeds();
    java.util.Optional<LimeLight> limelightCapability = base.getVision().getCameraCapability(table, LocalizationCameraCapability.LIMELIGHT);
    this.camera = limelightCapability.orElseThrow(() -> new IllegalArgumentException("No Limelight camera registered for table: " + table));
   }


   @Override
   public void initialize() 
   {
     runTag = -1;
     endCommand = new DelayedOutput(() -> closeEnough(), Units.millisecondsToSeconds(40));
     curPose = new Pose2d();
     filter = new MedianFilter(50);
     noTag = new DelayedOutput(() -> hasNoTag(), 0.6);

   }
 
   public boolean hasNoTag()
   {
     return !camera.hasValidTarget();
   }

   public Pose2d getBotPoseTagSpace(LimeLight ll)
   {
     double dist = camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle = getOffsetToTarget();
     double x = (Math.cos(Math.toRadians(angle)) * dist);
     double y = (Math.sin(Math.toRadians(angle)) * dist); 
 
     return new Pose2d(x,y, Rotation2d.fromDegrees(-filter.calculate(camera.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])));
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {
        
     if(!camera.hasValidTarget())
     { 
      robotSpeeds.stopSpeeds("feedback");
      return;
     }

     if(runTag == -1)
     {
       runTag = camera.getAprilTagID();
     }
     curPose = getBotPoseTagSpace(camera);

    double Xspeed = -xController.calculate(curPose.getX(), Units.inchesToMeters(12.5));
    double YSpeed = yController.calculate(curPose.getY(),0);
    double rSpeed =  thetaController.calculate(curPose.getRotation().getDegrees(), 0);

    if(camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 100000000)
    {
      robotSpeeds.setSpeeds("feedback",Xspeed, YSpeed, -rSpeed);
    }
    else if(noTag.getAsBoolean() && DriverStation.isTeleop()){
    robotSpeeds.setSpeeds("feedback",0, -0.1, 0);
    }
    else
    {
      robotSpeeds.stopSpeeds("feedback");
    }
    
    if(DriverStation.isTeleop())
    {
     if(endCommand.getAsBoolean())
     {
        alignmentComplete.run();
     }
    }
    
   }
 
   public boolean closeEnough()
   {
     return camera.hasValidTarget() && camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(getOffsetToTarget()) < 5;
   }

   public double getOffsetToTarget(){
      return camera.getTargetHorizontalOffset() +  offsetProvider.apply(runTag);
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    robotSpeeds.stopSpeeds("feedback");
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
        return endCommand.getAsBoolean() || noTag.getAsBoolean();
   }
}
