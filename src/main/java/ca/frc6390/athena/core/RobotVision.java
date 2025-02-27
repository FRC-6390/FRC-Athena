package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.stream.Collectors;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotVision {
   
   public record RobotVisionConfig(ArrayList<LimeLightConfig> limelight, ArrayList<PhotonVisionConfig> photon) {

      public static RobotVisionConfig blank(){
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

   public void setRobotOrientation(Rotation2d yaw){
      Double[] orientation = {yaw.getDegrees(),0d,0d,0d,0d,0d};
      getLimelights().values().forEach(ll -> ll.setRobotOrientation(orientation));
   }

   public ArrayList<Pose2d> getLimelightRobotPoses(){
      return getLimelights().values().stream().filter(ll -> ll.hasValidTarget()).map(ll -> ll.getPoseEstimate(ll.config.localizationEstimator()).getLocalizationPose()).collect(Collectors.toCollection(ArrayList::new));
   }

   public void setLimelightRobotPoses(BiConsumer<Pose2d, Double> estimator){
      getLimelights().values().stream().filter(ll -> ll.hasValidTarget()).forEach(ll -> estimator.accept(ll.getLocalizationPose(), ll.getLocalizationLatency()));
   }

   public ArrayList<Pose2d> getPhotonVisionPoses(){
      return getPhotonVisions().values().stream().filter(pv -> pv.hasValidTarget()).map(pv -> pv.getLocalizationPose()).collect(Collectors.toCollection(ArrayList::new));
   }

   public void setPhotonVisionPoses(BiConsumer<Pose2d, Double> estimator){
      getPhotonVisions().values().stream().filter(pv -> pv.hasValidTarget()).forEach(pv -> estimator.accept(pv.getLocalizationPose(), pv.getLocalizationLatency()));
   }

   public ArrayList<Pose2d> getLocalizationPoses(){
      ArrayList<Pose2d> poses = getLimelightRobotPoses();
      poses.addAll(getPhotonVisionPoses());
      return poses;
   }

   public void setLocalizationPoses(BiConsumer<Pose2d, Double> estimator){
     setLimelightRobotPoses(estimator);
     setPhotonVisionPoses(estimator);
   }

   public Set<String> getCameraTables() {
      return cameras.keySet();
   }
}
