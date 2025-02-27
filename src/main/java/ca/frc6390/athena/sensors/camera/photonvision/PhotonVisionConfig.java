package ca.frc6390.athena.sensors.camera.photonvision;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public record PhotonVisionConfig(String table,Transform3d cameraRobotSpace) implements ConfigurableCamera {

    @Override
    public Rotation2d getYawRelativeToForwards() {
        return Rotation2d.fromRadians(cameraRobotSpace.getRotation().getZ());
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.PhotonVision;
    }

    public static PhotonVisionConfig transform(Transform3d cameraRobotSpace){
        return new PhotonVisionConfig("photonvision", cameraRobotSpace);
    }

    public PhotonVisionConfig setTable(String table){
        return new PhotonVisionConfig(table, cameraRobotSpace);
    }

    @Override
    public String getTable() {
       return table;
    }
    
    public PhotonVision create(){
        return new PhotonVision(table, cameraRobotSpace);
    }
}
