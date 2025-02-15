package ca.frc6390.athena.core;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Set;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;

public class RobotVision {
   
   public record RobotVisionConfig(LimelightConfig... configs) {
      
      public static RobotVisionConfig limelight(String... limelights){
         return new RobotVisionConfig(Arrays.stream(limelights).map(LimelightConfig::new).toArray(LimelightConfig[]::new));
      }

      public static RobotVisionConfig limelight(LimelightConfig... limelights){
         return new RobotVisionConfig(limelights);
      }
   }

   private final HashMap<String, LimeLight> cameras;

   public RobotVision(String... tables) {
      this(Arrays.stream(tables)
                  .map(LimelightConfig::new)
                  .toArray(LimelightConfig[]::new));
   }

   public RobotVision(LimelightConfig... configs) {
      this(Arrays.stream(configs)
      .map(LimeLight::new)
      .toArray(LimeLight[]::new));
   }

   public RobotVision(LimeLight... camerasPopulate) {
      cameras = new HashMap<>();
      for (LimeLight limeLight : camerasPopulate) {
         cameras.put(limeLight.config.table(), limeLight);
      }
   }

   public static RobotVision fromConfig(RobotVisionConfig config){
      return new RobotVision(config.configs);
   }

   public LimeLight getCamera(String key) {
      return cameras.get(key);
   }

   public Set<String> getCameraTables() {
      return cameras.keySet();
   }
}
