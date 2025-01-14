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
        return SwerveHelpers.generateMotors(gearRatio, getFreeSpeed(), canbus, ids);
    }

    public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets){
        return generateEncoders(ids, offsets, canbus);
    }

    public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets, String canbus){
        return SwerveHelpers.generateEncoders(module.getEncoderGearRatio(), offsets, canbus, ids);
    }

    public SwerveModuleConfig[] generateConfigs(Translation2d[] locations, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID, SwerveEncoder[] encoders) {
        return SwerveHelpers.generateConfigs(locations, module.getWheelDiameter(), drive, rotation, rotationPID, encoders);
    }

    public SwerveModuleConfig generateConfig(Translation2d location, SwerveMotor drive, SwerveMotor rotation, PIDController rotationPID, SwerveEncoder encoder) {
        return new SwerveModuleConfig(location, module.getWheelDiameter(), drive, rotation, rotationPID, encoder);
    }

    public static SwerveModuleConfig[] generateConfigs(Translation2d[] locations, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID[], SwerveEncoder[] encoders, SDSModules ...modules) {

        SwerveModuleConfig[] configs = new SwerveModuleConfig[modules.length];

        for (int i = 0; i < modules.length; i++) {
            configs[i] = modules[i].generateConfig(locations[i], drive[i], rotation[i], rotationPID[i], encoders[i]);
        }

        return configs;
    }

    public SwerveModuleConfig[] generateConfigs(Translation2d[] locations, int[] drive, int[] rotation, PIDController rotationPID, int[] encoders, double[] offsets) {
        SwerveMotor[] DRIVE_MOTORS = generateDriveMotors(drive);
        SwerveMotor[] ROTATION_MOTORS = generateRotationMotors(rotation);
        SwerveEncoder[] MODULE_ENCODERS = generateEncoders(encoders, offsets);
        return SwerveHelpers.generateConfigs(locations, module.getWheelDiameter(), DRIVE_MOTORS, ROTATION_MOTORS, rotationPID, MODULE_ENCODERS);
    }

    public double getFreeSpeed() {
        double wheelCircumferenceFeet = Math.PI * module.getWheelDiameter();
        double motorRPM = motor.getFreeSpeedRPM();
        double wheelRPM = motorRPM / module.getDriveGearRatio();

        return wheelCircumferenceFeet * (wheelRPM / 60.0);
    }

    private interface SDSMK {
        public double getDriveGearRatio();
        public double getRotationGearRatio();
        public double getEncoderGearRatio();
        public double getWheelDiameter();
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

    public enum SDSMK4 implements SDSMK {
        L1(8.14d,12.8, 1),
        L2(6.75d,12.8,1),
        L3(6.12d,12.8,1),
        L4(5.14,12.8,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        SDSMK4(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameter =  Units.inchesToMeters(4);
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
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
        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }

    public enum SDSMK4i implements SDSMK {
        L1(8.14d,150d/7d, 1),
        L2(6.75d,150d/7d,1),
        L3(6.12d,150d/7d,1),
        L1_PLUS(7.13d,150d/7d,1),
        L2_PLUS(5.9d,150d/7d,1),
        L3_PLUS(5.36d,150d/7d,1);
        
        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        SDSMK4i(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameter =  Units.inchesToMeters(4);
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
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
        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }


    public enum SDSMK4n implements SDSMK {
        L1_PLUS(7.13d,18.75d,1),
        L2_PLUS(5.9d,18.75d,1),
        L3_PLUS(5.36d,18.75d,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        SDSMK4n(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameter =  Units.inchesToMeters(4);
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
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
        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }
}
