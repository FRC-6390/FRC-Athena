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
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {

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
    private Pose2d estimatorPose;
    private Field2d field;

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config, Pose2d pose) {
        this.estimatorPose = pose;
        this.drivetrain = drivetrain;
        this.vision = vision;

        estimator = config != null ?
         new SwerveDrivePoseEstimator(drivetrain.getKinematics(),drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose, config.getStd(), config.getVisionStd()) : 
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
        estimator.resetPosition(heading, drivetrain.getSwerveModulePositions(), pose);
        this.estimatorPose = pose;
    }

    public void reset(Pose2d pose) {
        reset(new Pose2d(), drivetrain.getIMU().getYaw());
    }

    public void reset() {
        reset(new Pose2d());
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
                    PoseEstimateWithLatency data = camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE);
                    double timestamp = Timer.getFPGATimestamp() - (data.getLatency() / 1000.0);
                    estimator.addVisionMeasurement(data.getPose(), timestamp);
                }
            }
        }

        estimatorPose = estimator.update(drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions());
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
