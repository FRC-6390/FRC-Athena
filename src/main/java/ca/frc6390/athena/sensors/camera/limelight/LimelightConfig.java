package ca.frc6390.athena.sensors.camera.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public record LimelightConfig(String table, double mountingAngle, double mountingHeightMeters, double angleRelativeToForwards) {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimelightConfig(String table){
        this(table, 0, 0, 0);
    }

    public LimelightConfig(){
        this(DEFUALT_TABLE);
    }

    public static LimelightConfig table(String table){
        return new LimelightConfig(table, 0,0,0);
    }

    public LimelightConfig setMountingAngle(double mountingAngle){
        return new LimelightConfig(table, mountingAngle, mountingHeightMeters, angleRelativeToForwards);
    }

    public LimelightConfig setMountingHeightAngle(double mountingHeightMeters){
        return new LimelightConfig(table, mountingAngle, mountingHeightMeters, angleRelativeToForwards);
    }

    public LimelightConfig setAngleRelativeToForwards(double angleRelativeToForwards){
        return new LimelightConfig(table, mountingAngle, mountingHeightMeters, angleRelativeToForwards);
    }

    public Rotation2d getRotationRelativeToForwards(){
        return Rotation2d.fromDegrees(angleRelativeToForwards);
    }
}