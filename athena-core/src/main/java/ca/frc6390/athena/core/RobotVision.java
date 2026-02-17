package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.ServiceLoader;

import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
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
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

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

      public static RobotVisionConfig defaults(){
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
   private final Map<String, VisionCamera> camerasView;
   private final EnumMap<CameraRole, List<VisionCamera>> camerasByRole;
   private final HashMap<String, Object> vendorCameras;
   private RobotLocalization<?> localization;
   private DiagnosticsChannel diagnosticsChannel;
   private final DiagnosticsView diagnosticsView = new DiagnosticsView();
   
   public RobotVision(RobotVisionConfig config) {
      this.config = config;
      this.cameras = new HashMap<>();
      this.camerasView = Collections.unmodifiableMap(cameras);
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

   private VisionCamera camera(String key) {
      return cameras.get(key);
   }

   public CamerasSection cameras() {
      return new CamerasSection(this);
   }

   public MeasurementsSection measurements() {
      return new MeasurementsSection(this);
   }

   public ObservationsSection observations() {
      return new ObservationsSection(this);
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

   private Map<String, VisionCamera> camerasMap() {
      return camerasView;
   }

   public RobotVision camera(String key, Consumer<VisionCamera.RuntimeSection> section) {
      VisionCamera camera = cameras.get(key);
      if (camera == null || section == null) {
         return this;
      }
      camera.config(section);
      refreshCameraRoles(camera);
      appendDiagnosticLog("INFO", "config", "updated runtime config for camera '" + key + "'");
      return this;
   }

   public void attachLocalization(RobotLocalization<?> localization) {
      this.localization = localization;
      appendDiagnosticLog(
            "INFO",
            "lifecycle",
            localization != null ? "attached localization" : "detached localization");
   }

   public RobotVision diagnostics(DiagnosticsChannel channel) {
      this.diagnosticsChannel = channel;
      if (channel != null) {
         appendDiagnosticLog("INFO", "lifecycle", "diagnostics channel attached");
         appendDiagnosticLog("INFO", "lifecycle", "camera count: " + cameras.size());
      }
      return this;
   }

   public DiagnosticsView diagnostics() {
      return diagnosticsView;
   }

   private Map<CameraRole, List<VisionCamera>> camerasByRole() {
      EnumMap<CameraRole, List<VisionCamera>> copy = new EnumMap<>(CameraRole.class);
      camerasByRole.forEach((role, list) -> copy.put(role, Collections.unmodifiableList(list)));
      return Collections.unmodifiableMap(copy);
   }

   private List<VisionCamera> camerasForRole(CameraRole role) {
      return camerasByRole.containsKey(role)
              ? Collections.unmodifiableList(camerasByRole.get(role))
              : Collections.emptyList();
   }

   private Optional<VisionCamera> firstCameraForRole(CameraRole role) {
      return camerasByRole.containsKey(role) && !camerasByRole.get(role).isEmpty()
              ? Optional.of(camerasByRole.get(role).get(0))
              : Optional.empty();
   }

   private <T> Optional<T> cameraCapability(String key, VisionCameraCapability capability) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.capability(capability);
   }

   private Optional<VisionCamera.TargetObservation> cameraObservation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getLatestObservation(space, robotPose, tagPose);
   }

   private Optional<Translation2d> cameraTranslation(String key, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
      VisionCamera camera = cameras.get(key);
      if (camera == null) {
         return Optional.empty();
      }
      return camera.getTargetTranslation(space, robotPose, tagPose);
   }

   private Optional<VisionCamera.TargetObservation> bestObservation(CoordinateSpace space, Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      return selectBestObservation(cameras.values(), space, robotPose, tagPoseLookup);
   }

    private Optional<VisionCamera.TargetObservation> bestObservationForRole(CameraRole role, CoordinateSpace space,
          Pose2d robotPose, Function<Integer, Pose2d> tagPoseLookup) {
      List<VisionCamera> roleCameras = camerasByRole.get(role);
      if (roleCameras == null || roleCameras.isEmpty()) {
         return Optional.empty();
      }
      return selectBestObservation(roleCameras, space, robotPose, tagPoseLookup);
   }

   private void robotOrientation(Pose2d pose){
      cameras.values().forEach(camera -> camera.setRobotOrientation(pose));
   }

   private ArrayList<Pose2d> localizationPoses(){
      ArrayList<Pose2d> poses = new ArrayList<>(cameras.size());
      for (VisionCamera camera : cameras.values()) {
         if (camera == null || !camera.isUseForLocalization() || !camera.hasValidTarget()) {
            continue;
         }
         poses.add(camera.getLocalizationPose());
      }
      return poses;
   }

   private void forEachLocalizationPose(Consumer<LocalizationData> estimator){
      for (VisionCamera camera : cameras.values()) {
         if (camera == null || !camera.isUseForLocalization() || !camera.hasValidTarget()) {
            continue;
         }
         estimator.accept(camera.getLocalizationData());
      }
   }

   private Optional<LocalizationData> bestLocalizationData() {
      LocalizationData best = null;
      double bestScore = Double.POSITIVE_INFINITY;
      for (VisionCamera camera : cameras.values()) {
         if (camera == null || !camera.isUseForLocalization() || !camera.hasValidTarget()) {
            continue;
         }
         LocalizationData data = camera.getLocalizationData();
         if (data == null || data.pose2d() == null) {
            continue;
         }
         double score = scoreLocalizationData(data);
         if (score < bestScore) {
            best = data;
            bestScore = score;
         }
      }
      return Optional.ofNullable(best);
   }

   private Optional<VisionCamera.VisionMeasurement> bestVisionMeasurement() {
      VisionCamera.VisionMeasurement best = null;
      double bestScore = Double.POSITIVE_INFINITY;
      for (VisionCamera camera : cameras.values()) {
         if (camera == null || !camera.isUseForLocalization()) {
            continue;
         }
         Optional<VisionCamera.VisionMeasurement> measurementOpt = camera.getLatestVisionMeasurement();
         if (measurementOpt.isEmpty()) {
            continue;
         }
         VisionCamera.VisionMeasurement measurement = measurementOpt.get();
         if (measurement.pose2d() == null) {
            continue;
         }
         double score = scoreVisionMeasurement(measurement);
         if (score < bestScore) {
            best = measurement;
            bestScore = score;
         }
      }
      return Optional.ofNullable(best);
   }

   private List<VisionCamera.VisionMeasurement> visionMeasurements() {
      ArrayList<VisionCamera.VisionMeasurement> measurements = new ArrayList<>(cameras.size());
      for (VisionCamera camera : cameras.values()) {
         if (camera == null || !camera.isUseForLocalization()) {
            continue;
         }
         Optional<VisionCamera.VisionMeasurement> measurementOpt = camera.getLatestVisionMeasurement();
         if (measurementOpt.isEmpty()) {
            continue;
         }
         VisionCamera.VisionMeasurement measurement = measurementOpt.get();
         if (measurement.pose2d() != null) {
            measurements.add(measurement);
         }
      }
      return measurements;
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

   private Set<String> cameraTables() {
      return Set.copyOf(cameras.keySet());
   }

   private void localizationStdDevs(Matrix<N3, N1> singleStdDevs, Matrix<N3, N1> multiStdDevs){
      cameras.values().forEach(camera -> camera.config().stdDevs(singleStdDevs, multiStdDevs));
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

         if (isObservationBetter(best, base.confidence(), base.distanceMeters(), translation)) {
            best = new VisionCamera.TargetObservation(
                    base.tagId(),
                    base.yawDegrees(),
                    base.distanceMeters(),
                    translation,
                    space,
                    base.localizationData(),
                    base.confidence());
         }
      }
      return Optional.ofNullable(best);
   }

   private static boolean isObservationBetter(
         VisionCamera.TargetObservation current,
         double candidateConfidence,
         double candidateDistance,
         Translation2d candidateTranslation) {
      if (current == null) {
         return true;
      }

      double currentConfidence = current.hasConfidence() ? current.confidence() : 0.0;
      double resolvedCandidateConfidence = Double.isFinite(candidateConfidence) ? candidateConfidence : 0.0;
      if (resolvedCandidateConfidence > currentConfidence + 1e-9) {
         return true;
      }
      if (currentConfidence > resolvedCandidateConfidence + 1e-9) {
         return false;
      }

      double currentDistance = current.hasDistance() ? Math.abs(current.distanceMeters()) : Double.POSITIVE_INFINITY;
      double resolvedCandidateDistance =
            Double.isFinite(candidateDistance) ? Math.abs(candidateDistance) : Double.POSITIVE_INFINITY;
      if (resolvedCandidateDistance < currentDistance) {
         return true;
      }

      double currentMagnitude = current.hasTranslation() ? current.translation().getNorm() : Double.POSITIVE_INFINITY;
      double candidateMagnitude = candidateTranslation != null ? candidateTranslation.getNorm() : Double.POSITIVE_INFINITY;
      if (candidateMagnitude < currentMagnitude) {
         return true;
      }
      return false;
   }

   private void registerCamera(String key, VisionCamera camera, Object vendorInstance) {
      cameras.put(key, camera);
      if (vendorInstance != null) {
         vendorCameras.put(key, vendorInstance);
      }
      refreshCameraRoles(camera);
      appendDiagnosticLog("INFO", "camera", "registered camera '" + key + "'");
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

   public static final class CamerasSection {
      private final RobotVision owner;

      private CamerasSection(RobotVision owner) {
         this.owner = owner;
      }

      public VisionCamera camera(String key) {
         return owner.camera(key);
      }

      public Map<String, VisionCamera> all() {
         return owner.camerasMap();
      }

      public Map<CameraRole, List<VisionCamera>> byRole() {
         return owner.camerasByRole();
      }

      public List<VisionCamera> forRole(CameraRole role) {
         return owner.camerasForRole(role);
      }

      public Optional<VisionCamera> firstForRole(CameraRole role) {
         return owner.firstCameraForRole(role);
      }

      public <T> Optional<T> capability(String key, VisionCameraCapability capability) {
         return owner.cameraCapability(key, capability);
      }

      public Set<String> tables() {
         return owner.cameraTables();
      }
   }

   public static final class MeasurementsSection {
      private final RobotVision owner;

      private MeasurementsSection(RobotVision owner) {
         this.owner = owner;
      }

      public void robotOrientation(Pose2d pose) {
         owner.robotOrientation(pose);
      }

      public void localizationStdDevs(Matrix<N3, N1> singleStdDevs, Matrix<N3, N1> multiStdDevs) {
         owner.localizationStdDevs(singleStdDevs, multiStdDevs);
      }

      public ArrayList<Pose2d> poses() {
         return owner.localizationPoses();
      }

      public Optional<LocalizationData> bestData() {
         return owner.bestLocalizationData();
      }

      public Optional<VisionCamera.VisionMeasurement> bestMeasurement() {
         return owner.bestVisionMeasurement();
      }

      public List<VisionCamera.VisionMeasurement> measurements() {
         return owner.visionMeasurements();
      }

      public void forEach(Consumer<LocalizationData> estimator) {
         owner.forEachLocalizationPose(estimator);
      }
   }

   public static final class ObservationsSection {
      private final RobotVision owner;

      private ObservationsSection(RobotVision owner) {
         this.owner = owner;
      }

      public Optional<VisionCamera.TargetObservation> camera(
            String key,
            CoordinateSpace space,
            Pose2d robotPose,
            Pose2d tagPose) {
         return owner.cameraObservation(key, space, robotPose, tagPose);
      }

      public Optional<Translation2d> translation(
            String key,
            CoordinateSpace space,
            Pose2d robotPose,
            Pose2d tagPose) {
         return owner.cameraTranslation(key, space, robotPose, tagPose);
      }

      public Optional<VisionCamera.TargetObservation> best(
            CoordinateSpace space,
            Pose2d robotPose,
            Function<Integer, Pose2d> tagPoseLookup) {
         return owner.bestObservation(space, robotPose, tagPoseLookup);
      }

      public Optional<VisionCamera.TargetObservation> bestForRole(
            CameraRole role,
            CoordinateSpace space,
            Pose2d robotPose,
            Function<Integer, Pose2d> tagPoseLookup) {
         return owner.bestObservationForRole(role, space, robotPose, tagPoseLookup);
      }
   }

   public final class DiagnosticsView {
      public DiagnosticsView log(String level, String category, String message) {
         appendDiagnosticLog(level, category, message);
         return this;
      }

      public DiagnosticsView info(String category, String message) {
         appendDiagnosticLog("INFO", category, message);
         return this;
      }

      public DiagnosticsView warn(String category, String message) {
         appendDiagnosticLog("WARN", category, message);
         return this;
      }

      public DiagnosticsView error(String category, String message) {
         appendDiagnosticLog("ERROR", category, message);
         return this;
      }

      public List<DiagnosticsChannel.Event> events(int limit) {
         DiagnosticsChannel channel = diagnosticsChannel;
         return channel != null ? channel.events(limit) : List.of();
      }

      public int count() {
         DiagnosticsChannel channel = diagnosticsChannel;
         return channel != null ? channel.eventCount() : 0;
      }

      public DiagnosticsView clear() {
         clearDiagnostics();
         return this;
      }

      public Map<String, Object> summary() {
         return diagnosticsSummary();
      }

      public Map<String, Object> snapshot(int limit) {
         return diagnosticsSnapshot(limit);
      }
   }

   private void appendDiagnosticLog(String level, String category, String message) {
      DiagnosticsChannel channel = diagnosticsChannel;
      if (channel == null || message == null || message.isBlank()) {
         return;
      }
      try {
         channel.log(level, category, message);
      } catch (RuntimeException ignored) {
         // Diagnostics should never break camera updates.
      }
   }

   private Map<String, Object> diagnosticsSummary() {
      Map<String, Object> summary = new LinkedHashMap<>();
      summary.put("cameraCount", cameras.size());
      summary.put("hasLocalization", localization != null);
      Map<String, Integer> roleCounts = new LinkedHashMap<>();
      for (Map.Entry<CameraRole, List<VisionCamera>> entry : camerasByRole.entrySet()) {
         if (entry == null || entry.getKey() == null || entry.getValue() == null) {
            continue;
         }
         roleCounts.put(entry.getKey().name(), entry.getValue().size());
      }
      summary.put("roles", roleCounts);
      DiagnosticsChannel channel = diagnosticsChannel;
      if (channel != null) {
         summary.put("channel", channel.summary());
      }
      return summary;
   }

   private Map<String, Object> diagnosticsSnapshot(int limit) {
      int resolvedLimit = limit > 0 ? Math.min(limit, 2048) : 120;
      Map<String, Object> snapshot = new LinkedHashMap<>(diagnosticsSummary());
      List<Map<String, Object>> cameraSnapshots = new ArrayList<>();
      for (Map.Entry<String, VisionCamera> entry : cameras.entrySet()) {
         if (entry == null || entry.getKey() == null || entry.getValue() == null) {
            continue;
         }
         VisionCamera camera = entry.getValue();
         Map<String, Object> cameraNode = new LinkedHashMap<>();
         cameraNode.put("key", entry.getKey());
         cameraNode.put("table", entry.getKey());
         cameraNode.put("connected", camera.isConnected());
         cameraNode.put("hasValidTarget", camera.hasValidTarget());
         cameraNode.put("useForLocalization", camera.isUseForLocalization());
         cameraNode.put("targetCount", camera.getTargetMeasurements().size());
         cameraNode.put("roles", camera.getRoles().stream().map(CameraRole::name).toList());
         cameraSnapshots.add(cameraNode);
      }
      cameraSnapshots.sort(Comparator.comparing(m -> String.valueOf(m.get("key")), String.CASE_INSENSITIVE_ORDER));
      snapshot.put("cameras", cameraSnapshots);
      DiagnosticsChannel channel = diagnosticsChannel;
      if (channel != null) {
         snapshot.put("events", channel.events(resolvedLimit));
      }
      return snapshot;
   }

   private void clearDiagnostics() {
      DiagnosticsChannel channel = diagnosticsChannel;
      if (channel != null) {
         channel.clear();
      }
   }

   @Override
   public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
      if (node == null) {
         return node;
      }
      RobotNetworkTables nt = node.robot();
      if (!nt.isPublishingEnabled()) {
         return node;
      }

      node.putDouble("cameraCount", cameras.size());
      node.putBoolean("hasLocalization", localization != null);

      return node;
   }
}
