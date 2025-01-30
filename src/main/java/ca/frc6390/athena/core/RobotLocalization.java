package ca.frc6390.athena.core;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatency;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class RobotLocalization extends SubsystemBase{
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd, PoseEstimateWithLatencyType pose) {

        public RobotLocalizationConfig(double xStd, double yStd, double thetaSt, double vXStd, double vYStda, double vThetaStd) {
            this(xStd, yStd, thetaSt, vXStd, vYStda, vThetaStd, PoseEstimateWithLatencyType.BOT_POSE_BLUE);
        }

        public RobotLocalizationConfig(double xStd, double yStd, double thetaSt) {
            this(xStd, yStd, thetaSt, 0.9, 0.9, 0.9);
        }

        public Matrix<N3, N1> getStd(){
            return VecBuilder.fill(xStd,yStd,Units.degreesToRadians(thetaStd));
        }

        public Matrix<N3, N1> getVisionStd(){
            return VecBuilder.fill(vXStd,vYStda,Units.degreesToRadians(vThetaStd));
        }
    }

    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDrivetrain drivetrain;
    private final RobotVision vision;
    private final PoseEstimateWithLatencyType estimateWithLatencyType;
    private Pose2d estimatorPose;
    private Field2d field;

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config, Pose2d pose) {
        this.estimatorPose = pose;
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.field = new Field2d();
        estimateWithLatencyType = config.pose;

        estimator = config != null ?
         new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose, config.getStd(), config.getVisionStd()) : 
         new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose);
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config) {
        this(drivetrain, vision, config, new Pose2d());
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotLocalizationConfig config) {
        this(drivetrain, null, config);
    }

    public RobotLocalization(SwerveDrivetrain drivetrain) {
        this(drivetrain, null, null);
    }

    public void reset(Pose2d pose, Rotation2d heading) {
        drivetrain.getIMU().setFieldYaw(heading);
        estimator.resetPose(pose);
        estimator.resetRotation(heading);
        this.estimatorPose = pose;
    }

    public void reset(Pose2d pose) {
        reset(pose, pose.getRotation());
    }

    public void reset() {
        reset(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void resetXY() {
        reset(new Pose2d(0,0,drivetrain.getIMU().getFieldYaw()));
    }

    public void resetXY(double x, double y) {
        reset(new Pose2d(x, y, drivetrain.getIMU().getFieldYaw()));
    }

    public void setVisionStd(Matrix<N3, N1> matrix){
        estimator.setVisionMeasurementStdDevs(matrix);
    }

    public void setVisionStd(double x, double y, double theta){
        setVisionStd(VecBuilder.fill(x,y,Units.degreesToRadians(theta)));
    }

    public void update() {

        if(vision != null) {
            for (String table : vision.getCameraTables()) {
                LimeLight camera = vision.getCamera(table);
                if(camera.hasValidTarget()){
                    PoseEstimateWithLatency data = camera.getPoseEstimate(estimateWithLatencyType);
                    double timestamp = Timer.getFPGATimestamp() - (data.getLatency() / 1000.0);
                    estimator.addVisionMeasurement(data.getPose(), timestamp);
                    // drivetrain.getIMU().setFieldYawOffset(data.getPose().getRotation());
                }
            }
        }

        estimatorPose = estimator.update(drivetrain.getIMU().getFieldYaw(), drivetrain.getSwerveModulePositions());
        field.setRobotPose(estimatorPose);
    }

    public Pose2d getPose(){
        return estimatorPose;
    }

    public Field2d getField2d() {
        return field;
    }

    public ShuffleboardTab shuffleboard(String tab) {
        return shuffleboard(Shuffleboard.getTab(tab));
    }

    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        tab.add("Estimator", field);
        tab.addDouble("X", () -> getPose().getX());
        tab.addDouble("Y", () -> getPose().getY());
        tab.addDouble("Theta", () -> getPose().getRotation().getDegrees()).withWidget(BuiltInWidgets.kGyro);
        return tab;
    }

    @Override
    public void periodic() {
       update();
    }
}
