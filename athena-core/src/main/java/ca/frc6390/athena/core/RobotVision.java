package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.ServiceLoader;

import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.sensors.camera.CameraProvider;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera.LocalizationData;
import ca.frc6390.athena.sensors.camera.VisionCamera.CoordinateSpace;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.core.sim.RobotVisionSimProvider;
import ca.frc6390.athena.dashboard.ShuffleboardControls;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotVision implements RobotSendableSystem {
   private static final List<CameraProvider> CAMERA_PROVIDERS = ServiceLoader
           .load(CameraProvider.class)
           .stream()
           .map(ServiceLoader.Provider::get)
           .toList();
   private static final List<RobotVisionSimProvider> VISION_SIM_PROVIDERS = ServiceLoader
           .load(RobotVisionSimProvider.class)
           .stream()
           .map(ServiceLoader.Provider::get)
           .toList();
   
   public record RobotVisionConfig(ArrayList<ConfigurableCamera> cameras) {

      public static RobotVisionConfig defualt(){
         return new RobotVisionConfig(new ArrayList<>());
      }

      @SafeVarargs
      public final <T extends ConfigurableCamera> RobotVisionConfig addCameras(T... configs){
         cameras.addAll(Arrays.asList(configs));
         return this;
      }

      public <T extends ConfigurableCamera> RobotVisionConfig addCamera(T config){
         cameras.add(config);
         return this;
      }

      public RobotVision create(){
         return new RobotVision(this);
      }
   }

   private final RobotVisionConfig config;
   private final HashMap<String, VisionCamera> cameras;
   private final EnumMap<CameraRole, List<VisionCamera>> camerasByRole;
   private final HashMap<String, Object> vendorCameras;
   private RobotLocalization<?> localization;
   
   public RobotVision(RobotVisionConfig config) {
      this.config = config;
      this.cameras = new HashMap<>();
      this.camerasByRole = new EnumMap<>(CameraRole.class);
      this.vendorCameras = new HashMap<>();

      boolean simulation = edu.wpi.first.wpilibj.RobotBase.isSimulation();

      for (ConfigurableCamera cameraConfig : config.cameras) {
         VisionCamera camera = createCamera(cameraConfig, simulation);
         if (camera != null) {
            registerCamera(cameraConfig.getTable(), camera, null);
         }
      }
   }

   public static RobotVision fromConfig(RobotVisionConfig config){
      return new RobotVision(config);
   }

   public VisionCamera getCamera(String key) {
      return cameras.get(key);
   }

   public RobotVisionSim createSimulation(AprilTagFieldLayout layout) {
      for (RobotVisionSimProvider provider : VISION_SIM_PROVIDERS) {
         if (provider.supports(this)) {
            return provider.create(this, layout);
         }
      }
      return new RobotVisionSim(this, layout);
   }

   public AprilTagFieldLayout deriveSimulationLayout() {
      try {
         return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      } catch (Exception e) {
         throw new RuntimeException("Failed to load default AprilTag layout", e);
      }
   }

   public Map<String, VisionCamera> getCameras() {
      return Collections.unmodifiableMap(cameras);
   }

   public void setUseForLocalization(String key, boolean enabled) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return;
      }
      camera.setUseForLocalization(enabled);
      refreshCameraRoles(camera);
   }

   public void attachLocalization(RobotLocalization<?> localization) {
      this.localization = localization;
   }

   public Map<CameraRole, List<VisionCamera>> getCamerasByRole() {
      EnumMap<CameraRole, List<VisionCamera>> copy = new EnumMap<>(CameraRole.class);
      camerasByRole.forEach((role, list) -> copy.put(role, Collections.unmodifiableList(list)));
      return Collections.unmodifiableMap(copy);
   }

   public List<VisionCamera> getCamerasForRole(CameraRole role) {
      return camerasByRole.containsKey(role)
              ? Collections.unmodifiableList(camerasByRole.get(role))
              : Collections.emptyList();
   }

   public Optional<VisionCamera> getFirstCameraWithRole(CameraRole role) {
      return camerasByRole.containsKey(role) && !camerasByRole.get(role).isEmpty()
              ? Optional.of(camerasByRole.get(role).get(0))
              : Optional.empty();
   }

   public <T> Optional<T> getCameraCapability(String key, VisionCameraCapability capability) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.capability(capability);
   }

   public Optional<VisionCamera.TargetObservation> getCameraObservation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getLatestObservation(space, robotPose, tagPose);
   }

   public Optional<Translation2d> getCameraTranslation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getTargetTranslation(space, robotPose, tagPose);
   }

   public Optional<VisionCamera.TargetObservation> getBestObservation(CoordinateSpace space, Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      return selectBestObservation(cameras.values(), space, robotPose, tagPoseLookup);
   }

    public Optional<VisionCamera.TargetObservation> getBestObservationForRole(CameraRole role, CoordinateSpace space,
          Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      List<VisionCamera> roleCameras = camerasByRole.get(role);
      if (roleCameras == null || roleCameras.isEmpty()) {
         return Optional.empty();
      }
      return selectBestObservation(roleCameras, space, robotPose, tagPoseLookup);
   }

   public void setRobotOrientation(Pose2d pose){
      cameras.values().forEach(camera -> camera.setRobotOrientation(pose));
   }

   public ArrayList<Pose2d> getCameraLocalizationPoses(){
      ArrayList<Pose2d> poses = new ArrayList<>();
      cameras.values().stream()
          .filter(VisionCamera::isUseForLocalization)
          .filter(VisionCamera::hasValidTarget)
          .map(VisionCamera::getLocalizationPose)
          .forEach(poses::add);
      return poses;
   }

   public void addLocalizationPoses(Consumer<LocalizationData> estimator){
      cameras.values().stream()
          .filter(VisionCamera::isUseForLocalization)
          .filter(VisionCamera::hasValidTarget)
          .map(VisionCamera::getLocalizationData)
          .forEach(estimator);
   }

   public Optional<LocalizationData> getBestLocalizationData() {
      return cameras.values().stream()
              .filter(VisionCamera::isUseForLocalization)
          .filter(VisionCamera::hasValidTarget)
          .map(VisionCamera::getLocalizationData)
          .filter(data -> data != null && data.pose2d() != null)
              .min(Comparator.comparingDouble(RobotVision::scoreLocalizationData));
   }

   public Optional<VisionCamera.VisionMeasurement> getBestVisionMeasurement() {
      return cameras.values().stream()
              .filter(VisionCamera::isUseForLocalization)
              .map(VisionCamera::getLatestVisionMeasurement)
              .flatMap(Optional::stream)
              .filter(measurement -> measurement.pose2d() != null)
              .min(Comparator.comparingDouble(RobotVision::scoreVisionMeasurement));
   }

   public List<VisionCamera.VisionMeasurement> getVisionMeasurements() {
      return cameras.values().stream()
              .filter(VisionCamera::isUseForLocalization)
              .map(VisionCamera::getLatestVisionMeasurement)
              .flatMap(Optional::stream)
              .filter(measurement -> measurement.pose2d() != null)
              .toList();
   }

   private static double scoreLocalizationData(LocalizationData data) {
      double score = 0.0;
      Matrix<N3, N1> std = data.stdDevs();
      if (std == null) {
         return Double.POSITIVE_INFINITY;
      }
      double stdX = safeStdDev(std.get(0, 0));
      double stdY = safeStdDev(std.get(1, 0));
      double stdTheta = safeStdDev(std.get(2, 0));
      score += stdX + stdY + stdTheta;
      return score;
   }

   private static double safeStdDev(double value) {
      double sanitized = Math.abs(value);
      if (Double.isNaN(sanitized) || Double.isInfinite(sanitized)) {
         return Double.POSITIVE_INFINITY;
      }
      return sanitized;
   }

   private static double scoreVisionMeasurement(VisionCamera.VisionMeasurement measurement) {
      Matrix<N3, N1> std = measurement.stdDevs();
      if (std == null) {
         return Double.POSITIVE_INFINITY;
      }
      double stdX = safeStdDev(std.get(0, 0));
      double stdY = safeStdDev(std.get(1, 0));
      double stdTheta = safeStdDev(std.get(2, 0));
      double weight = measurement.weightMultiplier();
      if (!Double.isFinite(weight) || weight <= 0.0) {
         weight = 1.0;
      }
      return (stdX + stdY + stdTheta) / weight;
   }

   public Set<String> getCameraTables() {
      return Set.copyOf(cameras.keySet());
   }

   public void setLocalizationStdDevs(Matrix<N3, N1> singleStdDevs, Matrix<N3, N1> multiStdDevs){
      cameras.values().forEach(camera -> camera.setStdDevs(singleStdDevs, multiStdDevs));
   }

   private Optional<VisionCamera.TargetObservation> selectBestObservation(Iterable<VisionCamera> cameraIterable,
         CoordinateSpace space, Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      VisionCamera.TargetObservation best = null;
      for (VisionCamera camera : cameraIterable) {
         Optional<VisionCamera.TargetObservation> baseObservation = camera.getLatestObservation(CoordinateSpace.CAMERA, robotPose, null);
         if (baseObservation.isEmpty()) continue;

         VisionCamera.TargetObservation base = baseObservation.get();
         Pose2d tagPose = null;
         if (space == CoordinateSpace.TAG && tagPoseLookup != null) {
            tagPose = tagPoseLookup.apply(base.tagId());
         } else if (space != CoordinateSpace.CAMERA && space != CoordinateSpace.ROBOT && space != CoordinateSpace.FIELD && space != CoordinateSpace.TAG) {
            continue;
         }

         Translation2d translation;
         if (space == CoordinateSpace.CAMERA) {
            translation = base.translation();
            if (translation == null) {
               continue;
            }
         } else {
            translation = camera.getTargetTranslation(space, robotPose, tagPose).orElse(null);
            if (translation == null) {
               continue;
            }
         }

         VisionCamera.TargetObservation transformed = new VisionCamera.TargetObservation(
                 base.tagId(),
                 base.yawDegrees(),
                 base.distanceMeters(),
                 translation,
                 space,
                 base.localizationData(),
                 base.confidence());

         best = pickBetterObservation(best, transformed);
      }
      return Optional.ofNullable(best);
   }

   private VisionCamera.TargetObservation pickBetterObservation(VisionCamera.TargetObservation current,
         VisionCamera.TargetObservation candidate) {
      if (candidate == null) {
         return current;
      }
      if (current == null) {
         return candidate;
      }

      double currentConfidence = current.hasConfidence() ? current.confidence() : 0.0;
      double candidateConfidence = candidate.hasConfidence() ? candidate.confidence() : 0.0;
      if (candidateConfidence > currentConfidence + 1e-9) {
         return candidate;
      }
      if (currentConfidence > candidateConfidence + 1e-9) {
         return current;
      }

      double currentDistance = current.hasDistance() ? Math.abs(current.distanceMeters()) : Double.POSITIVE_INFINITY;
      double candidateDistance = candidate.hasDistance() ? Math.abs(candidate.distanceMeters()) : Double.POSITIVE_INFINITY;
      if (candidateDistance < currentDistance) {
         return candidate;
      }

      double currentMagnitude = current.hasTranslation() ? current.translation().getNorm() : Double.POSITIVE_INFINITY;
      double candidateMagnitude = candidate.hasTranslation() ? candidate.translation().getNorm() : Double.POSITIVE_INFINITY;
      if (candidateMagnitude < currentMagnitude) {
         return candidate;
      }
      return current;
   }

   private void registerCamera(String key, VisionCamera camera, Object vendorInstance) {
      cameras.put(key, camera);
      if (vendorInstance != null) {
         vendorCameras.put(key, vendorInstance);
      }
      refreshCameraRoles(camera);
   }

   private VisionCamera createCamera(ConfigurableCamera config, boolean simulation) {
      for (CameraProvider provider : CAMERA_PROVIDERS) {
         if (provider.supports(config)) {
            return provider.create(config, simulation);
         }
      }
      return null;
   }

   private void refreshCameraRoles(VisionCamera camera) {
      for (List<VisionCamera> list : camerasByRole.values()) {
         list.remove(camera);
      }
      for (CameraRole role : camera.getRoles()) {
         camerasByRole.computeIfAbsent(role, __ -> new ArrayList<>()).add(camera);
      }
   }

   @Override
   public ShuffleboardTab shuffleboard(ShuffleboardTab tab, RobotSendableSystem.SendableLevel level) {
      if (localization != null) {
         localization.registerVisionShuffleboardTab(tab);
         if (ShuffleboardControls.enabled(ShuffleboardControls.Flag.VISION_FIELD_WIDGET)) {
            tab.add("Vision Field", localization.getVisionField2d())
                    .withPosition(0, 0)
                    .withSize(4, 3);
         }
      }
      return tab;
   }
}
