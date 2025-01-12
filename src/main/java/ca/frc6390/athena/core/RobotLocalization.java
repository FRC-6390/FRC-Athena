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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class RobotLocalization {
    
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

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDrivetrain drivetrain;
    private final RobotVision vision;
    private Pose2d odometryPose, estimatorPose;

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config, Pose2d pose) {
        this.odometryPose = pose;
        this.estimatorPose = pose;
        this.drivetrain = drivetrain;
        this.vision = vision;

        estimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), Rotation2d.fromDegrees(drivetrain.getHeading()), drivetrain.getSwerveModulePositions(), pose, config.getStd(), config.getVisionStd());
        odometry = new SwerveDriveOdometry(drivetrain.getKinematics(), Rotation2d.fromDegrees(drivetrain.getHeading()), drivetrain.getSwerveModulePositions());
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config) {
        this(drivetrain, vision, config, new Pose2d());
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotLocalizationConfig config) {
        this(drivetrain, null, config);
    }

    public void reset(Pose2d pose, Rotation2d heading) {
        resetOdometry(pose, heading);
        resetEstimator(pose, heading);
    }

    public void reset(Pose2d pose) {
        reset(new Pose2d(), drivetrain.getRotation2d());
    }

    public void reset() {
        reset(new Pose2d());
    }

    public void resetEstimator(Pose2d pose, Rotation2d heading) {
        estimator.resetPosition(heading, drivetrain.getSwerveModulePositions(), pose);
        this.estimatorPose = pose;
    }

    public void resetEstimator(Pose2d pose) {
        resetEstimator(new Pose2d(), drivetrain.getRotation2d());
    }

    public void resetEstimator() {
        resetEstimator(new Pose2d());
    }

    public void resetOdometry(Pose2d pose, Rotation2d heading) {
        odometry.resetPosition(heading, drivetrain.getSwerveModulePositions(), pose);
        this.odometryPose = pose;
    }

    public void resetOdometry(Pose2d pose) {
        resetOdometry(new Pose2d(), drivetrain.getRotation2d());
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
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

        odometryPose = odometry.update(drivetrain.getRotation2d(), drivetrain.getSwerveModulePositions());
        estimatorPose = estimator.update(drivetrain.getRotation2d(), drivetrain.getSwerveModulePositions());
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public Pose2d getEstimatorPose() {
        return estimatorPose;
    }

    public Pose2d getPose(){
        return vision != null ? getEstimatorPose() : getOdometryPose();
    }
}
