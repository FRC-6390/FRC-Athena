package ca.frc6390.athena.sensors.camera.limelight;

public class LimelightConfig {
    private static String DEFUALT_TABLE = "limelight"; 
    private String table;  
    public double mountingAngle;
    public double mountingHeightMeters;

    public LimelightConfig(String table, double mountingAngle, double mountingHeightMeters)
    {
        this.table = table;
        this.mountingAngle = mountingAngle;
        this.mountingHeightMeters = mountingHeightMeters;
    }

    public static LimelightConfig defualt(String table){
        return new LimelightConfig(table, 0, 0);
    }

    public static LimelightConfig defualt(){
        return new LimelightConfig(DEFUALT_TABLE, 0, 0);
    }

    public double mountingAngle()
    {
        return mountingAngle;
    }

    public double mountingHeightMeters()
    {
        return mountingHeightMeters;
    }

    public String table()
    {
        return table;
    }
}