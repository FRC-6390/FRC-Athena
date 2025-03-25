package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.LocalizationData;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RobotVision {
   
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

   private final HashMap<String, Object> cameras;

   public RobotVision(RobotVisionConfig config) {
      this.cameras = new HashMap<>();

      for (LimeLightConfig c : config.limelight) {
         cameras.put(c.getTable(), new LimeLight(c));
      }

      for (PhotonVisionConfig c : config.photon) {
         cameras.put(c.getTable(), new PhotonVision(c));
      }
   }

   public static RobotVision fromConfig(RobotVisionConfig config){
      return new RobotVision(config);
   }

   public LimeLight getLimelight(String key) {
      Object c = cameras.get(key);

      if (c instanceof LimeLight){
         return (LimeLight) c;
      }
      return null;
   }

   public PhotonVision getPhotonVision(String key) {
      Object c = cameras.get(key);

      if (c instanceof PhotonVision){
         return (PhotonVision) c;
      }
      return null;
   }

   public Map<String, LimeLight> getLimelights() {
      return cameras.entrySet().stream()
          .filter(e -> e.getValue() instanceof LimeLight)
          .collect(Collectors.toMap(
              Map.Entry::getKey,
              e -> (LimeLight) e.getValue()
          ));
   }

   public Map<String, PhotonVision> getPhotonVisions() {
      return cameras.entrySet().stream()
          .filter(e -> e.getValue() instanceof PhotonVision)
          .collect(Collectors.toMap(
              Map.Entry::getKey,
              e -> (PhotonVision) e.getValue()
          ));
   }

   public void setRobotOrientation(Pose2d pose){
      getLimelights().values().forEach(ll -> ll.setRobotOrientation(pose));
      getPhotonVisions().values().forEach(pv -> pv.setRobotOrientation(pose));

   }

   public ArrayList<Pose2d> getLimelightRobotPoses(){
      return getLimelights().values().stream().filter(ll -> ll.hasValidTarget() && ll.config.useForLocalization()).map(ll -> ll.getPoseEstimate(ll.config.localizationEstimator()).getLocalizationPose()).collect(Collectors.toCollection(ArrayList::new));
   }

   public void addLimelightRobotPoses(Consumer<LocalizationData> estimator){
      getLimelights().values().stream().filter(ll -> ll.hasValidTarget() && ll.config.useForLocalization()).forEach(ll -> estimator.accept(ll.getLocalizationData()));
   }

   public ArrayList<Pose2d> getPhotonVisionPoses(){
      return getPhotonVisions().values().stream().filter(pv -> pv.hasValidTarget() && pv.getConfig().useForLocalization()).map(pv -> pv.getLocalizationPose()).collect(Collectors.toCollection(ArrayList::new));
   }

   public void addPhotonVisionPoses(Consumer<LocalizationData> estimator){
      getPhotonVisions().values().stream().filter(pv -> pv.getConfig().useForLocalization()).forEach((pv) ->  {
            estimator.accept(pv.getLocalizationData());
         }
      );
   }

   // public ArrayList<Pose2d> getLocalizationPoses(){
   //    ArrayList<Pose2d> poses = getLimelightRobotPoses();
   //    poses.addAll(getPhotonVisionPoses());
   //    return poses;
   // }

   public void addLocalizationPoses(Consumer<LocalizationData> estimator){
     addLimelightRobotPoses(estimator);
     addPhotonVisionPoses(estimator);
   }

   public Set<String> getCameraTables() {
      return cameras.keySet();
   }

   public void setLocalizationStdDevs(Matrix<N3, N1> singleStdDevs, Matrix<N3, N1> multiStdDevs){
      getPhotonVisions().values().stream().filter(pv -> pv.getConfig().useForLocalization()).forEach(pv -> pv.setStdDevs(singleStdDevs, multiStdDevs));
      getLimelights().values().stream().filter(ll -> ll.config.useForLocalization()).forEach(ll -> ll.setStdDevs(singleStdDevs, multiStdDevs));
   }
}
