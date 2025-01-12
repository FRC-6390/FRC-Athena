package ca.frc6390.athena.core;

import java.util.HashMap;
import java.util.Set;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;

public class RobotVision {

   private final HashMap<String, LimeLight> cameras;

   private final String defaultCameraKey;

   public RobotVision(String... tables) {
      cameras = new HashMap<>();
      for (int i = 0; i < tables.length; i++) {
         LimeLight limeLight = new LimeLight(new LimelightConfig(tables[i]));
         cameras.put(tables[i], limeLight);
      }
      defaultCameraKey = tables[0];
   }

   public RobotVision(LimelightConfig... configs) {
      cameras = new HashMap<>();
      for (int i = 0; i < configs.length; i++) {
         LimeLight limeLight = new LimeLight(configs[i]);
         cameras.put(configs[i].table(), limeLight);
      }
      defaultCameraKey = configs[0].table();
   }

   public RobotVision(LimeLight... camerasPopulate) {
      cameras = new HashMap<>();
      defaultCameraKey = camerasPopulate[0].config.table();
      for (LimeLight limeLight : camerasPopulate) {
         cameras.put(limeLight.config.table(), limeLight);
      }
   }

   public LimeLight getCamera(String key) {
      return cameras.get(key);
   }

   public LimeLight getDefaultCamera() {
      return cameras.get(defaultCameraKey);
   }

   public Set<String> getCameraTables() {
      return cameras.keySet();
   }
}
