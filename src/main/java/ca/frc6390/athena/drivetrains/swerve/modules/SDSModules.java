package ca.frc6390.athena.drivetrains.swerve.modules;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.DrivetrainIDs;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveModuleVendors.VendorSwerveModule;
import edu.wpi.first.math.util.Units;

public class SDSModules {
    
    // private final SDSMotor motor;
    // private final SDSMK module;
    // private final String canbus;
    // public SDSModules(SDSMotor motor, SDSMK module){
    //     this(motor, module, "rio");
    // }

    // public SDSModules(SDSMotor motor, SDSMK module, String canbus){
    //     this.motor = motor;
    //     this.module = module;
    //     this.canbus = canbus;
    // }

    // public SDSMK getModule() {
    //     return module;
    // }
    
    // public SDSMotor getMotor() {
    //     return motor;
    // }
    
    // public SwerveMotor[] generateDriveMotors(int[] ids) {
    //     return generateDriveMotors(ids, canbus);
    // }

    // public SwerveMotor[] generateDriveMotors(int[] ids, String canbus) {
    //     return generateMotors(module.getDriveGearRatio(), ids, canbus);
    // }

    // public SwerveMotor[] generateRotationMotors(int[] ids){
    //     return generateRotationMotors(ids, canbus);
    // }

    // public SwerveMotor[] generateRotationMotors(int[] ids, String canbus){
    //     return generateMotors(module.getSteerGearRatio(), ids, canbus);
    // }

    // public SwerveMotor[] generateMotors(double gearRatio, int[] ids, String canbus){
    //     return SwerveHelpers.generateMotors(gearRatio, getFreeSpeed(), canbus, ids);
    // }

    // public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets){
    //     return generateEncoders(ids, offsets, canbus);
    // }

    // public SwerveEncoder[] generateEncoders(int[] ids, double[] offsets, String canbus){
    //     return SwerveHelpers.generateEncoders(module.getEncoderGearRatio(), offsets, canbus, ids);
    // }

    // public SwerveModuleConfig[] generateConfigs(Translation2d[] locations, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID, SwerveEncoder[] encoders) {
    //     return SwerveHelpers.generateConfigs(locations, module.getWheelDiameter(), drive, rotation, rotationPID, encoders);
    // }

    // public SwerveModuleConfig generateConfig(Translation2d location, SwerveMotor drive, SwerveMotor rotation, PIDController rotationPID, SwerveEncoder encoder) {
    //     return new SwerveModuleConfig(location, module.getWheelDiameter(), drive, rotation, rotationPID, encoder);
    // }

    // public static SwerveModuleConfig[] generateConfigs(Translation2d[] locations, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID[], SwerveEncoder[] encoders, SDSModules ...modules) {

    //     SwerveModuleConfig[] configs = new SwerveModuleConfig[modules.length];

    //     for (int i = 0; i < modules.length; i++) {
    //         configs[i] = modules[i].generateConfig(locations[i], drive[i], rotation[i], rotationPID[i], encoders[i]);
    //     }

    //     return configs;
    // }

    // public SwerveModuleConfig[] generateConfigs(Translation2d[] locations, int[] drive, int[] rotation, PIDController rotationPID, int[] encoders, double[] offsets) {
    //     SwerveMotor[] DRIVE_MOTORS = generateDriveMotors(drive);
    //     SwerveMotor[] ROTATION_MOTORS = generateRotationMotors(rotation);
    //     SwerveEncoder[] MODULE_ENCODERS = generateEncoders(encoders, offsets);
    //     return SwerveHelpers.generateConfigs(locations, module.getWheelDiameter(), DRIVE_MOTORS, ROTATION_MOTORS, rotationPID, MODULE_ENCODERS);
    // }

    // SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.custom(
    //     SDSModules.MK4i.L1_PLUS.config(Motor.KRAKEN_X60).setOffset(0).setPID(null),
    //     SDSModules.MK4i.L1_PLUS.config(Motor.KRAKEN_X60).setOffset(0).setPID(null),
    //     SDSModules.MK4n.L1_PLUS.config(Motor.KRAKEN_X60).setOffset(0).setPID(null),
    //     SDSModules.MK4n.L1_PLUS.config(Motor.KRAKEN_X60).setOffset(0).setPID(null)
    // ).setModulueLocations(10,10).setIDs(DrivetrainIDs.SWERVE_CHASSIS_STANDARD);

    // SwerveDrivetrainConfig SAM_CONFIG = SwerveDrivetrainConfig.standard(
    //     SDSModules.MK4i.L1_PLUS.config(Motor.FALCON_500).setPID(null)
    // ).setModulueLocations(10,10).setIDs(DrivetrainIDs.SWERVE_CHASSIS_STANDARD).setOffsets(1,1,1,1);


    public static enum MK4 implements VendorSwerveModule {
        L1(1d/8.14d,12.8, 1),
        L2(1d/6.75d,12.8,1),
        L3(1d/6.12d,12.8,1),
        L4(1d/5.14,12.8,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        MK4(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
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
        public double getSteerGearRatio() {
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

    public static enum MK4i implements VendorSwerveModule {
        L1(1d/8.14d,150d/7d, 1),
        L2(1d/6.75d,150d/7d,1),
        L3(1d/6.12d,150d/7d,1),
        L1_PLUS(1d/7.13d,150d/7d,1),
        L2_PLUS(1d/5.9d,150d/7d,1),
        L3_PLUS(1d/5.36d,150d/7d,1);
        
        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        MK4i(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
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
        public double getSteerGearRatio() {
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


    public static enum MK4n implements VendorSwerveModule {
        L1_PLUS(1d/7.13d,18.75d,1),
        L2_PLUS(1d/5.9d,18.75d,1),
        L3_PLUS(1d/5.36d,18.75d,1);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        MK4n(double driveGearRatio, double rotationGearRatio, double encoderGearRatio) {
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
        public double getSteerGearRatio() {
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
