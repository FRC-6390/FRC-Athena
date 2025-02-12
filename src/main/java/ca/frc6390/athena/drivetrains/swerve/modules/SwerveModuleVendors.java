package ca.frc6390.athena.drivetrains.swerve.modules;

public class SwerveModuleVendors {
    public static SDSModules SDS;

    public enum SwerveMotor {
        KRAKEN_X60_FOC(5800, true),
        KRAKEN_X60(6000, false),
        FALCON_500_FOC(6080, true),
        FALCON_500(6380, false),
        NEO_V1(5820, false),
        NEO_VORTEX(6784, false);

        private final int freeSpeedRPM;
        private final boolean foc;

        SwerveMotor(int freeSpeedRPM, boolean foc) {
            this.freeSpeedRPM = freeSpeedRPM;
            this.foc = foc;
        }

        public int getFreeSpeedRPM() {
            return freeSpeedRPM;
        }

        public boolean isFOC() {
            return foc;
        }
    }

    public interface SwerveModule {
    
    }

}
