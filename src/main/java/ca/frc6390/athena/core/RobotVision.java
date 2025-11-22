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

import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.LocalizationData;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.CoordinateSpace;
import ca.frc6390.athena.sensors.camera.LocalizationCameraCapability;
import ca.frc6390.athena.sensors.camera.LocalizationCameraConfig;
import ca.frc6390.athena.sensors.camera.LocalizationCameraConfig.CameraRole;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotVision implements RobotSendableSystem {
   
   public record RobotVisionConfig(ArrayList<LimeLightConfig> limelight, ArrayList<PhotonVisionConfig> photon) {

      public static RobotVisionConfig defualt(){
         return new RobotVisionConfig(new ArrayList<>(), new ArrayList<>());
      }

      public RobotVisionConfig addLimeLight(LimeLightConfig config){
         limelight.add(config);
         return this;
      }

      public RobotVisionConfig addLimeLights(LimeLightConfig... config){
         limelight.addAll(Arrays.asList(config));
         return this;
      }

      public RobotVisionConfig addPhotonVisions(PhotonVisionConfig... config){
         photon.addAll(Arrays.asList(config));
         return this;
      }

      public RobotVisionConfig addPhotonVision(PhotonVisionConfig config){
         photon.add(config);
         return this;
      }

      @SafeVarargs
      public final <T extends ConfigurableCamera> RobotVisionConfig addCameras(T... configs){
         
         for (ConfigurableCamera c : configs) {
            addCamera(c);
         }

         return this;
      }

      public <T extends ConfigurableCamera> RobotVisionConfig addCamera(T config){
       
         if (config instanceof LimeLightConfig){
            limelight.add((LimeLightConfig)config);
         }

         if (config instanceof PhotonVisionConfig){
            photon.add((PhotonVisionConfig)config);
         }

         return this;
      }

      public RobotVision create(){
         return new RobotVision(this);
      }
   }

   private final RobotVisionConfig config;
   private final HashMap<String, LocalizationCamera> cameras;
   private final EnumMap<CameraRole, List<LocalizationCamera>> camerasByRole;
   private final HashMap<String, PhotonVision> photonVisionCameras;
   private final HashMap<String, LimeLight> limeLightCameras;
   private RobotLocalization<?> localization;
   
   public RobotVision(RobotVisionConfig config) {
      this.config = config;
      this.cameras = new HashMap<>();
      this.camerasByRole = new EnumMap<>(CameraRole.class);
      this.photonVisionCameras = new HashMap<>();
      this.limeLightCameras = new HashMap<>();

      boolean simulation = edu.wpi.first.wpilibj.RobotBase.isSimulation();

      for (LimeLightConfig c : config.limelight) {
         if (simulation) {
            PhotonVision simPhoton = new PhotonVision(c.toSimulationPhotonConfig());
            registerCamera(c.getTable(), simPhoton.getLocalizationCamera(), simPhoton, null);
         } else {
            LimeLight limeLight = new LimeLight(c);
            registerCamera(c.getTable(), limeLight.getLocalizationCamera(), null, limeLight);
         }
      }

      for (PhotonVisionConfig c : config.photon) {
         PhotonVision photonVision = new PhotonVision(c);
         registerCamera(c.getTable(), photonVision.getLocalizationCamera(), photonVision, null);
      }
   }

   public static RobotVision fromConfig(RobotVisionConfig config){
      return new RobotVision(config);
   }

   public LocalizationCamera getCamera(String key) {
      return cameras.get(key);
   }

   public RobotVisionSim createSimulation(AprilTagFieldLayout layout) {
      return new RobotVisionSim(this, layout);
   }

   public AprilTagFieldLayout deriveSimulationLayout() {
      for (PhotonVisionConfig pv : config.photon()) {
         if (pv != null && pv.fieldLayout() != null) {
            try {
               return AprilTagFieldLayout.loadField(pv.fieldLayout());
            } catch (Exception ignored) {
            }
         }
      }
      try {
         return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
      } catch (Exception e) {
         throw new RuntimeException("Failed to load default AprilTag layout", e);
      }
   }

   public Map<String, LocalizationCamera> getCameras() {
      return Collections.unmodifiableMap(cameras);
   }

   public void setUseForLocalization(String key, boolean enabled) {
      LocalizationCamera camera = cameras.get(key);
      if (camera == null) {
         return;
      }
      camera.setUseForLocalization(enabled);
      getLimeLightCamera(key).ifPresent(lime -> lime.setUseForLocalization(enabled));
      getPhotonVisionCamera(key).ifPresent(pv -> pv.setUseForLocalization(enabled));
      refreshCameraRoles(camera);
   }

   public void attachLocalization(RobotLocalization<?> localization) {
      this.localization = localization;
   }

   public Optional<PhotonVision> getPhotonVisionCamera(String key) {
      return Optional.ofNullable(photonVisionCameras.get(key));
   }

   public Optional<LimeLight> getLimeLightCamera(String key) {
      return Optional.ofNullable(limeLightCameras.get(key));
   }

   public Map<CameraRole, List<LocalizationCamera>> getCamerasByRole() {
      EnumMap<CameraRole, List<LocalizationCamera>> copy = new EnumMap<>(CameraRole.class);
      camerasByRole.forEach((role, list) -> copy.put(role, Collections.unmodifiableList(list)));
      return Collections.unmodifiableMap(copy);
   }

   public List<LocalizationCamera> getCamerasForRole(CameraRole role) {
      return camerasByRole.containsKey(role)
              ? Collections.unmodifiableList(camerasByRole.get(role))
              : Collections.emptyList();
   }

   public Optional<LocalizationCamera> getFirstCameraWithRole(CameraRole role) {
      return camerasByRole.containsKey(role) && !camerasByRole.get(role).isEmpty()
              ? Optional.of(camerasByRole.get(role).get(0))
              : Optional.empty();
   }

   public <T> Optional<T> getCameraCapability(String key, LocalizationCameraCapability capability) {
      LocalizationCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.capability(capability);
   }

   public Optional<LocalizationCamera.TargetObservation> getCameraObservation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      LocalizationCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getLatestObservation(space, robotPose, tagPose);
   }

   public Optional<Translation2d> getCameraTranslation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      LocalizationCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getTargetTranslation(space, robotPose, tagPose);
   }

   public Optional<LocalizationCamera.TargetObservation> getBestObservation(CoordinateSpace space, Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      return selectBestObservation(cameras.values(), space, robotPose, tagPoseLookup);
   }

    public Optional<LocalizationCamera.TargetObservation> getBestObservationForRole(CameraRole role, CoordinateSpace space,
          Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      List<LocalizationCamera> roleCameras = camerasByRole.get(role);
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
          .filter(LocalizationCamera::isUseForLocalization)
          .filter(LocalizationCamera::hasValidTarget)
          .map(LocalizationCamera::getLocalizationPose)
          .forEach(poses::add);
      return poses;
   }

   public void addLocalizationPoses(Consumer<LocalizationData> estimator){
      cameras.values().stream()
          .filter(LocalizationCamera::isUseForLocalization)
          .filter(LocalizationCamera::hasValidTarget)
          .map(LocalizationCamera::getLocalizationData)
          .forEach(estimator);
   }

   public Optional<LocalizationData> getBestLocalizationData() {
      return cameras.values().stream()
              .filter(LocalizationCamera::isUseForLocalization)
          .filter(LocalizationCamera::hasValidTarget)
          .map(LocalizationCamera::getLocalizationData)
          .filter(data -> data != null && data.pose2d() != null)
              .min(Comparator.comparingDouble(RobotVision::scoreLocalizationData));
   }

   public Optional<LocalizationCamera.VisionMeasurement> getBestVisionMeasurement() {
      return cameras.values().stream()
              .filter(LocalizationCamera::isUseForLocalization)
              .map(LocalizationCamera::getLatestVisionMeasurement)
              .flatMap(Optional::stream)
              .filter(measurement -> measurement.pose2d() != null)
              .min(Comparator.comparingDouble(RobotVision::scoreVisionMeasurement));
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

   private static double scoreVisionMeasurement(LocalizationCamera.VisionMeasurement measurement) {
      Matrix<N3, N1> std = measurement.stdDevs();
      if (std == null) {
         return Double.POSITIVE_INFINITY;
      }
      double stdX = safeStdDev(std.get(0, 0));
      double stdY = safeStdDev(std.get(1, 0));
      double stdTheta = safeStdDev(std.get(2, 0));
      return stdX + stdY + stdTheta;
   }

   public Set<String> getCameraTables() {
      return Set.copyOf(cameras.keySet());
   }

   public void setLocalizationStdDevs(Matrix<N3, N1> singleStdDevs, Matrix<N3, N1> multiStdDevs){
      cameras.values().forEach(camera -> camera.setStdDevs(singleStdDevs, multiStdDevs));
   }

   private Optional<LocalizationCamera.TargetObservation> selectBestObservation(Iterable<LocalizationCamera> cameraIterable,
         CoordinateSpace space, Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      LocalizationCamera.TargetObservation best = null;
      for (LocalizationCamera camera : cameraIterable) {
         Optional<LocalizationCamera.TargetObservation> baseObservation = camera.getLatestObservation(CoordinateSpace.CAMERA, robotPose, null);
         if (baseObservation.isEmpty()) continue;

         LocalizationCamera.TargetObservation base = baseObservation.get();
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

         LocalizationCamera.TargetObservation transformed = new LocalizationCamera.TargetObservation(
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

   private LocalizationCamera.TargetObservation pickBetterObservation(LocalizationCamera.TargetObservation current,
         LocalizationCamera.TargetObservation candidate) {
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

   private void registerCamera(String key, LocalizationCamera camera, PhotonVision photonVision, LimeLight limeLight) {
      cameras.put(key, camera);
      if (photonVision != null) {
         photonVisionCameras.put(key, photonVision);
      }
      if (limeLight != null) {
         limeLightCameras.put(key, limeLight);
      }
      refreshCameraRoles(camera);
   }

   private void refreshCameraRoles(LocalizationCamera camera) {
      for (List<LocalizationCamera> list : camerasByRole.values()) {
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
         tab.add("Vision Field", localization.getVisionField2d())
                 .withPosition(0, 0)
                 .withSize(4, 3);
      }
      return tab;
   }
}
