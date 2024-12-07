package ca.frc6390.athena.core;

import java.util.HashMap;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;

public class RobotVision {

   private final HashMap<String, LimeLight> cameras;

   private final String defaultCameraKey;

   public RobotVision(String... tableNames) {
      cameras = new HashMap<>();
      for (int i = 0; i < tableNames.length; i++) {
         LimeLight limeLight = new LimeLight(tableNames[i]);
         cameras.put(tableNames[i], limeLight);
      }
      defaultCameraKey = tableNames[0];
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
}
