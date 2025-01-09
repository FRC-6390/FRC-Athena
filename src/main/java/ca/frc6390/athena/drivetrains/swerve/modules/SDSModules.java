package ca.frc6390.athena.drivetrains.swerve.modules;

import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveEncoder;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SDSModules {
    
    private final SDSMotor motor;
    private final SDSMK module;
    private final String canbus;
    public SDSModules(SDSMotor motor, SDSMK module){
        this(motor, module, "rio");
    }

    public SDSModules(SDSMotor motor, SDSMK module, String canbus){
        this.motor = motor;
        this.module = module;
        this.canbus = canbus;
    }

    public SDSMK getModule() {
        return module;
    }
    
    public SDSMotor getMotor() {
        return motor;
    }
    
    public SwerveMotor[] generateDriveMotors(int[] ids) {
        return generateDriveMotors(ids, canbus);
    }

    public SwerveMotor[] generateDriveMotors(int[] ids, String canbus) {
        return generateMotors(module.getDriveGearRatio(), ids, canbus);
    }

    public SwerveMotor[] generateRotationMotors(int[] ids){
        return generateRotationMotors(ids, canbus);
    }

    public SwerveMotor[] generateRotationMotors(int[] ids, String canbus){
        return generateMotors(module.getRotationGearRatio(), ids, canbus);
    }

    public SwerveMotor[] generateMotors(double gearRatio, int[] ids, String canbus){
        return SwerveHelpers.generateMotors(gearRatio, calculateFreeSpeedMeter(motor, module), canbus, ids);
    }

    public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets){
        return generateEncoders(ids, offsets, canbus);
    }

    public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets, String canbus){
        return SwerveHelpers.generateEncoders(module.getEncoderGearRatio(), offsets, canbus, ids);
    }

    public SwerveModuleConfig[] generateConfigs(Translation2d[] locations, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID, SwerveEncoder[] encoders) {
        return SwerveHelpers.generateConfigs(locations, module.getWheelDiameterMeters(), drive, rotation, rotationPID, encoders);
    }

    private interface SDSMK {
        public double getDriveGearRatio();
        public double getRotationGearRatio();
        public double getEncoderGearRatio();
        public double getWheelDiameterInches();
        public double getWheelDiameterMeters();
    }

    public enum SDSMotor {
        KRAKEN_X60_FOC(5800, true),
        KRAKEN_X60(6000, false),
        FALCON_500_FOC(6080, true),
        FALCON_500(6380, false),
        NEO_V1(5820, false),
        NEO_VORTEX(6784, false);

        private final int freeSpeedRPM;
        private final boolean foc;

        SDSMotor(int freeSpeedRPM, boolean foc) {
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

    public enum MK4 implements SDSMK {
        L1(8.14d,12.8, 1),
        L2(6.75d,12.8,1),
        L3(6.12d,12.8,1),
        L4(5.14,12.8,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameterInches;

        MK4(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameterInches =  4;
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
        }
        
        @Override
        public double getWheelDiameterInches() {
            return wheelDiameterInches;
        }

        @Override
        public double getRotationGearRatio() {
            return rotationGearRatio;
        }

        @Override
        public double getEncoderGearRatio() {
            return encoderGearRatio;
        }

        @Override
        public double getWheelDiameterMeters() {
            return Units.inchesToMeters(getWheelDiameterInches());
        }
    }

    public enum MK4i implements SDSMK {
        L1(8.14d,150d/7d, 1),
        L2(6.75d,150d/7d,1),
        L3(6.12d,150d/7d,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameterInches;

        MK4i(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameterInches =  4;
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        @Override
        public double getWheelDiameterInches() {
            return wheelDiameterInches;
        }

        @Override
        public double getRotationGearRatio() {
            return rotationGearRatio;
        }

        @Override
        public double getEncoderGearRatio() {
            return encoderGearRatio;
        }

        @Override
        public double getWheelDiameterMeters() {
            return Units.inchesToMeters(getWheelDiameterInches());
        }
    }

    public static double calculateFreeSpeedFeet(SDSMotor motor, SDSMK module) {
        double wheelCircumferenceFeet = Math.PI * (module.getWheelDiameterInches() / 12.0);
        double motorRPM = motor.getFreeSpeedRPM();
        double wheelRPM = motorRPM / module.getDriveGearRatio();
        return wheelCircumferenceFeet * (wheelRPM / 60.0);
    }

    public static double calculateFreeSpeedMeter(SDSMotor motor, SDSMK module) {
        return Units.feetToMeters(calculateFreeSpeedFeet(motor, module));
    }
}
