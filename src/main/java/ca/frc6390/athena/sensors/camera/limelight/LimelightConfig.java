package ca.frc6390.athena.sensors.camera.limelight;

public record LimelightConfig(String table, double mountingAngle, double mountingHeightMeters) {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimelightConfig(String table){
        this(table, 0, 0);
    }

    public LimelightConfig(){
        this(DEFUALT_TABLE);
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