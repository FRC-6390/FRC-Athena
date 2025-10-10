package ca.frc6390.athena.core;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import ca.frc6390.athena.devices.IMU;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.LocalizationData;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.TargetObservation;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.CoordinateSpace;
public class RobotLocalization<T> extends SubsystemBase implements RobotSendableSystem{
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd, double v2XStd, double v2YStda, double v2ThetaStd, PIDConstants translation, PIDConstants rotation, boolean useVision) {

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
            this(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, vXStd, vYStda, vThetaStd, new PIDConstants(0), new PIDConstants(0), true);
        }

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
            this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
        }

        public RobotLocalizationConfig() {
            this(0.1,0.1,0.001);
        }

        public static RobotLocalizationConfig vision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig().setVision(vXStd, vYStda, vThetaStd).setVisionEnabled(true);
        }

        public static RobotLocalizationConfig defualt(){
            return new RobotLocalizationConfig();
        }

        public RobotLocalizationConfig setAutoPlannerPID(double tP, double tI, double tD, double rP, double rI, double rD){
            return setAutoPlannerPID(new PIDConstants(tP, tI, tD), new PIDConstants(rP, rI, rD));
        }

        public RobotLocalizationConfig setAutoPlannerPID(PIDConstants translation, PIDConstants rotation){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVisionEnabled(boolean useVision){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public Matrix<N3, N1> getStd(){
            return VecBuilder.fill(xStd,yStd,Units.degreesToRadians(thetaStd));
        }

        public Matrix<N3, N1> getVisionStd(){
            return VecBuilder.fill(vXStd,vYStda,Units.degreesToRadians(vThetaStd));
        }
        public Matrix<N3, N1> getVisionMultitagStd(){
            return VecBuilder.fill(v2XStd,v2YStda,Units.degreesToRadians(v2ThetaStd));
        }
    }

    private final PoseEstimator<T> fieldEstimator, relativeEstimator;
    private RobotVision vision;
    private final IMU imu;
    private final RobotSpeeds robotSpeeds;
    private final Supplier<T> wheelPositions;

    private Pose2d fieldPose, relativePose;
    private Field2d field;
    private RobotConfig robotConfig;

    private static final Pose2d ZERO_POSE = new Pose2d();
    private static final Pose2d[] EMPTY_POSES = new Pose2d[0];
    private static final double STD_EPSILON = 1e-5;

    private final Map<String, CameraDisplayState> cameraDisplayStates = new HashMap<>();
    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private PIDController rotationController, translationController;
    private AutoFactory factory;

    private BiConsumer<ChassisSpeeds, DriveFeedforwards> autoDrive;

    private boolean suppressUpdates = false;
    private StructPublisher<Pose2d> fieldPosePublisher;
    private StructPublisher<Pose2d> relativePosePublisher;
    private ShuffleboardTab activeShuffleboardTab;

    private double visionMaxLatencySeconds = 0.9;
    private double visionOutlierTranslationMeters = 2.5;
    private double visionOutlierRotationRadians = Units.degreesToRadians(35.0);
    private double visionStdDevOutlierMultiplier = 4.0;
    private double visionStdDevTranslationLimit = 5.0;
    private double visionStdDevRotationLimit = Units.degreesToRadians(120.0);
    private boolean hasAcceptedVisionMeasurement = false;

    private static class CameraDisplayState {
        final String key;
        final GenericEntry showCameraPoseEntry;
        final GenericEntry showTagLinesEntry;
        final GenericEntry showEstimatedPoseEntry;
        final GenericEntry useForLocalizationEntry;
        final GenericEntry trustDistanceEntry;
        final GenericEntry singleStdXEntry;
        final GenericEntry singleStdYEntry;
        final GenericEntry singleStdThetaDegEntry;
        final GenericEntry multiStdXEntry;
        final GenericEntry multiStdYEntry;
        final GenericEntry multiStdThetaDegEntry;
        final FieldObject2d cameraPoseObject;
        final FieldObject2d estimatedPoseObject;
        final FieldObject2d tagLineObject;
        final StructPublisher<Pose2d> cameraPosePublisher;
        final StructPublisher<Pose2d> estimatedPosePublisher;

        CameraDisplayState(
                String key,
                GenericEntry showCameraPoseEntry,
                GenericEntry showTagLinesEntry,
                GenericEntry showEstimatedPoseEntry,
                GenericEntry useForLocalizationEntry,
                GenericEntry trustDistanceEntry,
                GenericEntry singleStdXEntry,
                GenericEntry singleStdYEntry,
                GenericEntry singleStdThetaDegEntry,
                GenericEntry multiStdXEntry,
                GenericEntry multiStdYEntry,
                GenericEntry multiStdThetaDegEntry,
                FieldObject2d cameraPoseObject,
                FieldObject2d estimatedPoseObject,
                FieldObject2d tagLineObject,
                StructPublisher<Pose2d> cameraPosePublisher,
                StructPublisher<Pose2d> estimatedPosePublisher) {
            this.key = key;
            this.showCameraPoseEntry = showCameraPoseEntry;
            this.showTagLinesEntry = showTagLinesEntry;
            this.showEstimatedPoseEntry = showEstimatedPoseEntry;
            this.useForLocalizationEntry = useForLocalizationEntry;
            this.trustDistanceEntry = trustDistanceEntry;
            this.singleStdXEntry = singleStdXEntry;
            this.singleStdYEntry = singleStdYEntry;
            this.singleStdThetaDegEntry = singleStdThetaDegEntry;
            this.multiStdXEntry = multiStdXEntry;
            this.multiStdYEntry = multiStdYEntry;
            this.multiStdThetaDegEntry = multiStdThetaDegEntry;
            this.cameraPoseObject = cameraPoseObject;
            this.estimatedPoseObject = estimatedPoseObject;
            this.tagLineObject = tagLineObject;
            this.cameraPosePublisher = cameraPosePublisher;
            this.estimatedPosePublisher = estimatedPosePublisher;
        }
    }

    public RobotLocalization(PoseEstimator<T> fieldEstimator, PoseEstimator<T> relativeEstimator, RobotLocalizationConfig config, RobotSpeeds robotSpeeds, IMU imu, Supplier<T> wheelPositions) {
        this.localizationConfig = config;
        this.robotSpeeds = robotSpeeds;
        this.imu = imu;
        this.wheelPositions = wheelPositions;
        this.fieldPose = new Pose2d();
        this.relativePose = new Pose2d();
        this.autoDrive = (speeds, feed) ->  robotSpeeds.setSpeeds("auto",speeds);
      
        this.field = new Field2d();
        this.fieldPosePublisher = null;
        this.relativePosePublisher = null;

        this.visionEnabled = config.useVision;
        config = config == null ? new RobotLocalizationConfig() : config;

        
        this.fieldEstimator = fieldEstimator;
        this.relativeEstimator = relativeEstimator;
 
        imu.addVirtualAxis("relative", imu::getYaw);
        imu.addVirtualAxis("field", imu::getYaw);
        imu.setVirtualAxis("relative", new Rotation2d());
        imu.setVirtualAxis("field", new Rotation2d());

        if(config.rotation != null && config.translation != null){
            configurePathPlanner(config.translation, config.rotation);
        }  
    }

    private CameraDisplayState createCameraDisplayState(String key, LocalizationCamera camera) {
        if (activeShuffleboardTab == null) {
            return null;
        }
        ShuffleboardLayout cameraLayout = activeShuffleboardTab.getLayout("Camera/" + key, BuiltInLayouts.kGrid);

        ShuffleboardLayout toggleLayout =
                cameraLayout.getLayout("Display Toggles", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0);

        SimpleWidget showPoseWidget = toggleLayout.add("Show Camera Pose", true).withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showCameraPoseEntry = showPoseWidget.getEntry();

        SimpleWidget showLinesWidget = toggleLayout.add("Show Tag Lines", true).withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showTagLinesEntry = showLinesWidget.getEntry();

        SimpleWidget showEstimateWidget = toggleLayout.add("Show Estimated Pose", false).withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showEstimatedPoseEntry = showEstimateWidget.getEntry();

        boolean defaultUseForLocalization = camera.isUseForLocalization();
        SimpleWidget useForLocalizationWidget =
                toggleLayout.add("Use For Localization", defaultUseForLocalization)
                        .withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry useForLocalizationEntry = useForLocalizationWidget.getEntry();

        ShuffleboardLayout tuningLayout =
                cameraLayout.getLayout("Tuning", BuiltInLayouts.kGrid)
                        .withSize(4, 4)
                        .withPosition(2, 0);

        double trustDistance = camera.getConfig().getTrustDistance();
        GenericEntry trustDistanceEntry =
                tuningLayout.add("Trust Distance (m)", trustDistance)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", 0.0, "max", 10.0, "block increment", 0.05))
                        .getEntry();

        double singleStdX = camera.getSingleStdDev().get(0, 0);
        double singleStdY = camera.getSingleStdDev().get(1, 0);
        double singleStdThetaDeg = Units.radiansToDegrees(camera.getSingleStdDev().get(2, 0));
        ShuffleboardLayout singleStdLayout =
                tuningLayout.getLayout("Single Tag Std", BuiltInLayouts.kList);
        GenericEntry singleStdXEntry =
                singleStdLayout.add("X (m)", singleStdX)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry singleStdYEntry =
                singleStdLayout.add("Y (m)", singleStdY)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry singleStdThetaDegEntry =
                singleStdLayout.add("Theta (deg)", singleStdThetaDeg)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();

        double multiStdX = camera.getMultiStdDev().get(0, 0);
        double multiStdY = camera.getMultiStdDev().get(1, 0);
        double multiStdThetaDeg = Units.radiansToDegrees(camera.getMultiStdDev().get(2, 0));
        ShuffleboardLayout multiStdLayout =
                tuningLayout.getLayout("Multi Tag Std", BuiltInLayouts.kList);
        GenericEntry multiStdXEntry =
                multiStdLayout.add("X (m)", multiStdX)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry multiStdYEntry =
                multiStdLayout.add("Y (m)", multiStdY)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry multiStdThetaDegEntry =
                multiStdLayout.add("Theta (deg)", multiStdThetaDeg)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();

        FieldObject2d cameraPoseObject = field.getObject("CameraPose/" + key);
        FieldObject2d estimatedPoseObject = field.getObject("CameraEstimate/" + key);
        FieldObject2d tagLineObject = field.getObject("CameraTagLine/" + key);
        cameraPoseObject.setPoses(EMPTY_POSES);
        estimatedPoseObject.setPoses(EMPTY_POSES);
        tagLineObject.setPoses(EMPTY_POSES);

        String baseTopic = "/Shuffleboard/" + activeShuffleboardTab.getTitle() + "/Camera/" + key;
        StructPublisher<Pose2d> cameraPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(baseTopic + "/CameraPose", Pose2d.struct)
                .publish();
        cameraPosePublisher.set(ZERO_POSE);
        StructPublisher<Pose2d> estimatedPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(baseTopic + "/EstimatedPose", Pose2d.struct)
                .publish();
        estimatedPosePublisher.set(ZERO_POSE);

        return new CameraDisplayState(
                key,
                showCameraPoseEntry,
                showTagLinesEntry,
                showEstimatedPoseEntry,
                useForLocalizationEntry,
                trustDistanceEntry,
                singleStdXEntry,
                singleStdYEntry,
                singleStdThetaDegEntry,
                multiStdXEntry,
                multiStdYEntry,
                multiStdThetaDegEntry,
                cameraPoseObject,
                estimatedPoseObject,
                tagLineObject,
                cameraPosePublisher,
                estimatedPosePublisher);
    }

    public RobotLocalization<T> enableVisionForLocalization(boolean visionEnabled){
        this.visionEnabled = visionEnabled;
        return this;
    }

    public RobotLocalization<T> setRobotVision(RobotVision vision){
        this.vision = vision;
        vision.setLocalizationStdDevs(localizationConfig.getVisionStd(), localizationConfig.getVisionMultitagStd());
        ensureCameraShuffleboardEntries();
        return this;
    }

    public RobotLocalization<T> setVisionOutlierThresholds(double translationMeters, double rotationDegrees) {
        this.visionOutlierTranslationMeters = Math.max(0.0, translationMeters);
        this.visionOutlierRotationRadians = Units.degreesToRadians(Math.max(0.0, rotationDegrees));
        return this;
    }

    public RobotLocalization<T> setVisionStdDevOutlierMultiplier(double multiplier) {
        if (Double.isFinite(multiplier) && multiplier >= 1.0) {
            this.visionStdDevOutlierMultiplier = multiplier;
        }
        return this;
    }

    public RobotLocalization<T> setVisionStdDevLimits(double translationMeters, double rotationDegrees) {
        this.visionStdDevTranslationLimit = Math.max(0.0, translationMeters);
        this.visionStdDevRotationLimit = Units.degreesToRadians(Math.max(0.0, rotationDegrees));
        return this;
    }

    public RobotLocalization<T> setVisionLatencyLimit(double maxLatencySeconds) {
        if (Double.isFinite(maxLatencySeconds) && maxLatencySeconds >= 0.0) {
            this.visionMaxLatencySeconds = maxLatencySeconds;
        }
        return this;
    }

    public void setAutoDrive(BiConsumer<RobotSpeeds, ChassisSpeeds> autoDrive) {
        this.autoDrive = (speeds, feeds) -> autoDrive.accept(robotSpeeds, speeds);
    }

    private Matrix<N3, N1> sanitizeVisionStdDevs(Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) {
            return null;
        }
        double sanitizedX = sanitizeStdDevEntry(stdDevs.get(0, 0), visionStdDevTranslationLimit);
        double sanitizedY = sanitizeStdDevEntry(stdDevs.get(1, 0), visionStdDevTranslationLimit);
        double sanitizedTheta = sanitizeStdDevEntry(stdDevs.get(2, 0), visionStdDevRotationLimit);
        if (Double.isNaN(sanitizedX) || Double.isNaN(sanitizedY) || Double.isNaN(sanitizedTheta)) {
            return null;
        }
        return VecBuilder.fill(sanitizedX, sanitizedY, sanitizedTheta);
    }

    private double sanitizeStdDevEntry(double value, double maxValue) {
        double sanitized = Math.abs(value);
        if (!Double.isFinite(sanitized) || sanitized < STD_EPSILON) {
            return Double.NaN;
        }
        if (maxValue > 0.0 && sanitized >= maxValue) {
            return Double.NaN;
        }
        return sanitized;
    }

    private boolean shouldApplyVisionMeasurement(LocalizationData data, Matrix<N3, N1> stdDevs) {
        if (data == null) {
            return false;
        }
        Pose2d visionPose = data.pose();
        if (!isFinitePose(visionPose)) {
            return false;
        }
        double timestampSeconds = data.latency();
        if (!Double.isFinite(timestampSeconds)) {
            return false;
        }
        double ageSeconds = Timer.getFPGATimestamp() - timestampSeconds;
        if (visionMaxLatencySeconds > 0.0) {
            // Reject frames that are too old or unexpectedly in the future (>100 ms lead).
            if (ageSeconds > visionMaxLatencySeconds || ageSeconds < -0.1) {
                return false;
            }
        }
        if (!hasAcceptedVisionMeasurement) {
            return true;
        }
        Pose2d referencePose = fieldPose != null ? fieldPose : new Pose2d();
        Pose2d delta = visionPose.relativeTo(referencePose);
        double translationError = delta.getTranslation().getNorm();
        double rotationError = Math.abs(delta.getRotation().getRadians());

        double translationThreshold = visionOutlierTranslationMeters;
        double rotationThreshold = visionOutlierRotationRadians;
        if (stdDevs != null) {
            double translationStd = Math.max(stdDevs.get(0, 0), stdDevs.get(1, 0));
            double rotationStd = stdDevs.get(2, 0);
            if (Double.isFinite(translationStd) && translationStd > STD_EPSILON) {
                translationThreshold = Math.max(translationThreshold, translationStd * visionStdDevOutlierMultiplier);
            }
            if (Double.isFinite(rotationStd) && rotationStd > STD_EPSILON) {
                rotationThreshold = Math.max(rotationThreshold, rotationStd * visionStdDevOutlierMultiplier);
            }
        }

        return translationError <= translationThreshold && rotationError <= rotationThreshold;
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private void ensureCameraShuffleboardEntries() {
        if (vision == null || activeShuffleboardTab == null) {
            return;
        }
        Map<String, LocalizationCamera> cameras = vision.getCameras();
        for (Map.Entry<String, LocalizationCamera> entry : cameras.entrySet()) {
            cameraDisplayStates.computeIfAbsent(entry.getKey(), key -> createCameraDisplayState(key, entry.getValue()));
        }
        cameraDisplayStates.entrySet().removeIf(stateEntry -> {
            if (!cameras.containsKey(stateEntry.getKey())) {
                stateEntry.getValue().cameraPosePublisher.close();
                stateEntry.getValue().estimatedPosePublisher.close();
                return true;
            }
            return false;
        });
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants){
      return configurePathPlanner(translationConstants, rotationConstants, autoDrive);
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants, BiConsumer<ChassisSpeeds, DriveFeedforwards> output){
        try{
            robotConfig = RobotConfig.fromGUISettings();  
        }catch(Exception e){
            DriverStation.reportWarning("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
            return this;
        }
        AutoBuilder.configure(
            this::getFieldPose, 
            this::resetFieldPose, 
            () -> robotSpeeds.getSpeeds("drive"), 
            output, 
            new PPHolonomicDriveController(
            translationConstants,
            rotationConstants
            ),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }            
      );
      FollowPathCommand.warmupCommand().schedule();
      return this;
    }

    public RobotLocalization<T> configureChoreo(Subsystem drivetrain){
        rotationController = new PIDController(localizationConfig.rotation.kP, localizationConfig.rotation.kI, localizationConfig.rotation.kD);
        rotationController.setIZone(localizationConfig.rotation.iZone);

        translationController = new PIDController(localizationConfig.translation.kP, localizationConfig.translation.kI, localizationConfig.translation.kD);
        translationController.setIZone(localizationConfig.translation.iZone);

        factory = new AutoFactory(this::getFieldPose, this::resetFieldPose, 
        (sample) -> {
            Pose2d botpose = getFieldPose();
            Pose2d trajpose = sample.getPose();
            
            ChassisSpeeds speeds = sample.getChassisSpeeds();

            speeds.vxMetersPerSecond = translationController.calculate(botpose.getX(), trajpose.getX());
            speeds.vyMetersPerSecond = translationController.calculate(botpose.getY(), trajpose.getY());
            speeds.omegaRadiansPerSecond = rotationController.calculate(botpose.getRotation().getRadians(), trajpose.getRotation().getRadians());

            robotSpeeds.setSpeeds("auto", speeds);
        }, 
        true,
        drivetrain);
      return this;
    }

    public AutoFactory getChoreoAutoFactory(){
        return factory;
    }

    public void resetFieldPose(Pose2d pose) {
        fieldEstimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("field", pose.getRotation());
        this.fieldPose = pose;
        this.hasAcceptedVisionMeasurement = false;
    }

    public void resetFieldPose(double x, double y) {
        resetFieldPose(new Pose2d(x, y, imu.getVirtualAxis("field")));
    }

    public void resetFieldPose(double x, double y, double thetaDegree) {
        resetFieldPose(new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegree)));
    }

    public void resetFieldPose() {
        resetFieldPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void resetRelativePose(Pose2d pose) {
        relativeEstimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("relative", pose.getRotation());
        this.relativePose = pose;
    }

    public void resetRelativePose(double x, double y) {
        resetRelativePose(new Pose2d(x, y, imu.getVirtualAxis("relative")));
    }

    public void resetRelativePose(double x, double y, double r) {
        resetRelativePose(new Pose2d(x, y, new Rotation2d(r)));
    }

    public void resetRelativePose() {
        resetRelativePose(0,0, 0);
    }

    public void setVisionStd(Matrix<N3, N1> matrix){
        fieldEstimator.setVisionMeasurementStdDevs(matrix);
    }

    public void setVisionStd(double x, double y, double theta){
        setVisionStd(VecBuilder.fill(x,y,Units.degreesToRadians(theta)));
    }

    private static double sanitizeStdDev(double value) {
        if (Double.isNaN(value) || value < STD_EPSILON) {
            return STD_EPSILON;
        }
        return value;
    }

    private void applyCameraConfigUpdates(CameraDisplayState state, LocalizationCamera camera) {
        boolean desiredUse = state.useForLocalizationEntry.getBoolean(camera.isUseForLocalization());
        if (desiredUse != camera.isUseForLocalization()) {
            if (vision != null) {
                vision.setUseForLocalization(state.key, desiredUse);
            } else {
                camera.setUseForLocalization(desiredUse);
            }
        }

        double currentTrust = camera.getConfig().getTrustDistance();
        double desiredTrust = state.trustDistanceEntry.getDouble(currentTrust);
        if (Math.abs(desiredTrust - currentTrust) > STD_EPSILON) {
            camera.getConfig().setTrustDistance(Math.max(0.0, desiredTrust));
        }

        Matrix<N3, N1> singleStd = camera.getSingleStdDev();
        double desiredSingleX = sanitizeStdDev(state.singleStdXEntry.getDouble(singleStd.get(0, 0)));
        double desiredSingleY = sanitizeStdDev(state.singleStdYEntry.getDouble(singleStd.get(1, 0)));
        double desiredSingleThetaDeg =
                state.singleStdThetaDegEntry.getDouble(Units.radiansToDegrees(singleStd.get(2, 0)));
        double desiredSingleThetaRad = Units.degreesToRadians(desiredSingleThetaDeg);
        if (Math.abs(desiredSingleX - singleStd.get(0, 0)) > STD_EPSILON
                || Math.abs(desiredSingleY - singleStd.get(1, 0)) > STD_EPSILON
                || Math.abs(desiredSingleThetaRad - singleStd.get(2, 0)) > Units.degreesToRadians(0.01)) {
            Matrix<N3, N1> updatedSingle = VecBuilder.fill(desiredSingleX, desiredSingleY, desiredSingleThetaRad);
            camera.getConfig().setSingleStdDevs(updatedSingle);
            camera.setStdDevs(updatedSingle, null);
        }

        Matrix<N3, N1> multiStd = camera.getMultiStdDev();
        double desiredMultiX = sanitizeStdDev(state.multiStdXEntry.getDouble(multiStd.get(0, 0)));
        double desiredMultiY = sanitizeStdDev(state.multiStdYEntry.getDouble(multiStd.get(1, 0)));
        double desiredMultiThetaDeg =
                state.multiStdThetaDegEntry.getDouble(Units.radiansToDegrees(multiStd.get(2, 0)));
        double desiredMultiThetaRad = Units.degreesToRadians(desiredMultiThetaDeg);
        if (Math.abs(desiredMultiX - multiStd.get(0, 0)) > STD_EPSILON
                || Math.abs(desiredMultiY - multiStd.get(1, 0)) > STD_EPSILON
                || Math.abs(desiredMultiThetaRad - multiStd.get(2, 0)) > Units.degreesToRadians(0.01)) {
            Matrix<N3, N1> updatedMulti = VecBuilder.fill(desiredMultiX, desiredMultiY, desiredMultiThetaRad);
            camera.getConfig().setMultiStdDevs(updatedMulti);
            camera.setStdDevs(null, updatedMulti);
        }
    }

    private Pose2d computeCameraFieldPose(LocalizationCamera camera) {
        Transform3d cameraToRobot = camera.getConfig().getCameraToRobotTransform();
        Transform3d robotToCamera = cameraToRobot.inverse();
        Translation2d offset = new Translation2d(robotToCamera.getX(), robotToCamera.getY())
                .rotateBy(fieldPose.getRotation());
        Rotation2d cameraRotation = fieldPose
                .getRotation()
                .plus(Rotation2d.fromRadians(cameraToRobot.getRotation().getZ()));
        return new Pose2d(fieldPose.getTranslation().plus(offset), cameraRotation);
    }

    private void updateCameraPose(CameraDisplayState state, Pose2d cameraPose) {
        boolean show = state.showCameraPoseEntry.getBoolean(true);
        if (show && cameraPose != null) {
            state.cameraPoseObject.setPose(cameraPose);
        } else {
            state.cameraPoseObject.setPoses(EMPTY_POSES);
        }
        state.cameraPosePublisher.set(cameraPose != null ? cameraPose : ZERO_POSE);
    }

    private void updateEstimatedPose(CameraDisplayState state, LocalizationCamera camera) {
        boolean show = state.showEstimatedPoseEntry.getBoolean(false);
        Pose2d estimatePose =
                camera.hasValidTarget() ? camera.getLocalizationPose() : null;
        if (show && estimatePose != null) {
            state.estimatedPoseObject.setPose(estimatePose);
        } else {
            state.estimatedPoseObject.setPoses(EMPTY_POSES);
        }
        state.estimatedPosePublisher.set(estimatePose != null ? estimatePose : ZERO_POSE);
    }

    private void updateTagLines(CameraDisplayState state, LocalizationCamera camera, Pose2d cameraPose) {
        boolean show = state.showTagLinesEntry.getBoolean(true);
        if (!show || cameraPose == null) {
            state.tagLineObject.setPoses(EMPTY_POSES);
            return;
        }
        TargetObservation observation =
                camera.getLatestObservation(CoordinateSpace.FIELD, fieldPose, null).orElse(null);
        if (observation == null || !observation.hasTranslation()) {
            state.tagLineObject.setPoses(EMPTY_POSES);
            return;
        }
        Translation2d targetFieldTranslation = observation.translation();
        Pose2d tagPose = new Pose2d(targetFieldTranslation, new Rotation2d());
        state.tagLineObject.setPoses(cameraPose, tagPose);
    }

    private void updateCameraVisualizations() {
        if (vision == null) {
            return;
        }
        ensureCameraShuffleboardEntries();
        for (CameraDisplayState state : cameraDisplayStates.values()) {
            LocalizationCamera camera = vision.getCamera(state.key);
            if (camera == null) {
                state.cameraPoseObject.setPoses(EMPTY_POSES);
                state.estimatedPoseObject.setPoses(EMPTY_POSES);
                state.tagLineObject.setPoses(EMPTY_POSES);
                state.cameraPosePublisher.set(ZERO_POSE);
                state.estimatedPosePublisher.set(ZERO_POSE);
                continue;
            }
            applyCameraConfigUpdates(state, camera);
            Pose2d cameraPose = computeCameraFieldPose(camera);
            updateCameraPose(state, cameraPose);
            updateEstimatedPose(state, camera);
            updateTagLines(state, camera, cameraPose);
        }
    }

    public void update() {
        
        if(suppressUpdates) return;

        if(vision != null && visionEnabled) {
            vision.setRobotOrientation(fieldPose);
            // vision.getLimelights().forEach((table, ll) -> ll.setFiducialIdFilters(Arrays.stream(ll.config.filteredTags()).mapToDouble(i -> i).toArray()));
            // List<Pose2d> poses = vision.getLocalizationPoses();
            // SmartDashboard.putNumber("Localization Poses", poses.size());
            vision.getBestLocalizationData().ifPresent(data -> {
                Matrix<N3, N1> rawStdDevs = data.stdDevs();
                Matrix<N3, N1> sanitizedStdDevs = sanitizeVisionStdDevs(rawStdDevs);
                if (rawStdDevs != null && sanitizedStdDevs == null) {
                    return;
                }
                if (!shouldApplyVisionMeasurement(data, sanitizedStdDevs)) {
                    return;
                }
                if (sanitizedStdDevs != null) {
                    fieldEstimator.addVisionMeasurement(data.pose(), data.latency(), sanitizedStdDevs);
                } else {
                    fieldEstimator.addVisionMeasurement(data.pose(), data.latency());
                }
                hasAcceptedVisionMeasurement = true;
            });
        }

        fieldPose = fieldEstimator.update(imu.getVirtualAxis("field"), wheelPositions.get());
        relativePose = relativeEstimator.update(imu.getVirtualAxis("relative"), wheelPositions.get());
        field.setRobotPose(fieldPose);
        if (fieldPosePublisher != null) {
            fieldPosePublisher.set(fieldPose);
        }
        if (relativePosePublisher != null) {
            relativePosePublisher.set(relativePose);
        }
        updateCameraVisualizations();
    }

    public FieldObject2d getField2dObject(String id){
        return field.getObject(id);
    }

    public Pose2d getFieldPose(){
        return fieldPose;
    }

    public boolean isSuppressUpdates() {
        return suppressUpdates;
    }

    public void setSuppressUpdates(boolean suppressUpdates) {
        this.suppressUpdates = suppressUpdates;
    }

    public Pose2d getRelativePose(){
        return relativePose;
    }

    public Field2d getField2d() {
        return field;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        this.activeShuffleboardTab = tab;
        tab.add("Estimator", field).withPosition( 0,0).withSize(4, 3);

        String baseTopic = "/Shuffleboard/" + tab.getTitle();

        if (fieldPosePublisher == null) {
            String fieldTopic = baseTopic + "/EstimatorPose";
            fieldPosePublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(fieldTopic, Pose2d.struct)
                    .publish();
            fieldPosePublisher.set(fieldPose);
        }

        if (relativePosePublisher == null) {
            String relativeTopic = baseTopic + "/RelativePose";
            relativePosePublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(relativeTopic, Pose2d.struct)
                    .publish();
            relativePosePublisher.set(relativePose);
        }

        if(level.equals(SendableLevel.DEBUG)) {
            tab.addDouble("Field X", () -> getFieldPose().getX()).withPosition( 4,2);
            tab.addDouble("Field Y", () -> getFieldPose().getY()).withPosition( 5,2);
            tab.addDouble("Field Theta", () -> getFieldPose().getRotation().getDegrees()).withPosition( 4,0).withSize(2, 2);
        }
        tab.addDouble("Relative X", () -> getRelativePose().getX()).withPosition( 6,2);
        tab.addDouble("Relative Y", () -> getRelativePose().getY()).withPosition( 7,2);
        tab.addDouble("Relative Theta", () -> getRelativePose().getRotation().getDegrees()).withPosition( 6,0).withSize(2, 2);
        ensureCameraShuffleboardEntries();
        return tab;
    }

    @Override
    public void periodic() {
       update();
    }

} 
