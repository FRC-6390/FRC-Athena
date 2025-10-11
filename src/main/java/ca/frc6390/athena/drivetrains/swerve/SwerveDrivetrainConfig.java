package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.EncoderIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.SteerIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.IMU.IMUConfig;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Builder-style configuration object for constructing a {@link SwerveDrivetrain}. It captures module
 * hardware layouts, controller gains, CAN IDs, and optional simulation parameters so the drivetrain
 * can be created consistently across both robot code and tests.
 */
public class SwerveDrivetrainConfig implements RobotDrivetrainConfig<SwerveDrivetrain> {
    
    /** Optional IMU configuration that provides field-centric heading. */
    private IMUConfig imu = null;
    /** Per-module hardware configuration (drive/steer motors, encoders). */
    private SwerveModuleConfig[] modules = null;
    /** PID controller used to counter uncommanded drift while driving straight. */
    private PIDController driftPID = null;
    /** Speed threshold (m/s) where drift compensation engages. */
    private double driftActivationSpeed = 0.05;
    /** Whether open-loop driving defaults to field-relative chassis speeds. */
    private boolean fieldRelative = true;
    /** CAN IDs for drive motors ordered FL, FR, BL, BR (negatives invert motors). */
    private int[] driveIds = DriveIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** CAN IDs for steer motors ordered FL, FR, BL, BR (negatives invert motors). */
    private int[] steerIDs = SteerIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** CAN IDs for steering encoders ordered FL, FR, BL, BR (negatives invert encoders). */
    private int[] encoderIds = EncoderIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** Device ID of the inertial gyro used for heading. */
    private int gryoId = DrivetrainIDs.SWERVE_CHASSIS_STANDARD.getGyro();
    /** Whether each steer motor should be inverted during configuration. */
    private boolean steerInverted = true;
    /** Whether each steering encoder should be inverted. */
    private boolean encoderInverted = false;
    /** Whether each drive motor should be inverted. */
    private boolean driveInverted = false;
    /** PID controller that maintains heading when commanded setpoints demand rotation. */
    private PIDController rotationPID = null;
    /** Drive-side current limit applied per motor (amps). */
    private double driveCurrentLimit = 80;
    /** Steer-side current limit applied per motor (amps). */
    private double steerCurrentLimit = 40;
    /** Absolute encoder offsets (radians) for each module. */
    private double[] encoderOffsets = {0,0,0,0};
    /** CAN bus identifier shared by all configured devices. */
    private String canbus = "rio";
    /** Module corner locations relative to the robot center. */
    private Translation2d[] locations = null;
    /** Optional physics configuration used when running drivetrain simulation. */
    private SwerveSimulationConfig simulationConfig = SwerveSimulationConfig.defaults();

    /**
     * Creates a configuration with default hardware IDs and explicit module locations.
     *
     * @param locations module corner offsets relative to robot center (FL, FR, BL, BR)
     */
    public static SwerveDrivetrainConfig defualt(Translation2d[] locations){
        return new SwerveDrivetrainConfig().setModuleLocations(locations);
    }

    /**
     * Creates a configuration with default hardware IDs and square module footprint.
     *
     * @param trackWidth distance (meters) between left and right modules
     */
    public static SwerveDrivetrainConfig defualt(double trackWidth){
        return new SwerveDrivetrainConfig().setModuleLocations(trackWidth);
    }

    /**
     * Creates a configuration with default hardware IDs and rectangular footprint.
     *
     * @param trackWidth distance (meters) between left and right modules
     * @param wheelbase distance (meters) between front and back modules
     */
    public static SwerveDrivetrainConfig defualt(double trackWidth, double wheelbase){
        return new SwerveDrivetrainConfig().setModuleLocations(trackWidth, wheelbase);
    }

    /**
     * Configures the standard team drivetrain IDs while using the provided module locations.
     */
    public static SwerveDrivetrainConfig standard(Translation2d[] locations){
        return SwerveDrivetrainConfig.defualt(locations).setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD);
    }

    /**
     * Configures the standard team drivetrain IDs while assuming a square footprint.
     */
    public static SwerveDrivetrainConfig standard(double trackWidth){
        return SwerveDrivetrainConfig.defualt(trackWidth).setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD);
    }

    /**
     * Configures the standard team drivetrain IDs while assuming a rectangular footprint.
     */
    public static SwerveDrivetrainConfig standard(double trackWidth, double wheelbase){
        return SwerveDrivetrainConfig.defualt(trackWidth, wheelbase).setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD);
    }

    /**
     * Declares the gyro hardware to use along with its inversion.
     *
     * @param imu IMU type installed on the robot
     * @param inverted true if yaw should be inverted
     */
    public SwerveDrivetrainConfig setIMU(IMUType imu, boolean inverted){
        this.imu = new IMUConfig(imu).setInverted(inverted);
        return this;
    }

    /**
     * Registers the swerve module configurations. Passing a single module copies it to all four
     * module positions in order FL, FR, BL, BR.
     *
     * @param modules one module definition per corner, or a single definition to clone
     */
    public SwerveDrivetrainConfig modules(SwerveModuleConfig... modules){
        if (modules.length == 1) {
            modules = new SwerveModuleConfig[]{
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0])
            };
        }
        this.modules = modules;
        return this;
    }

    /**
     * Creates a drift-correction PID controller using the provided gains.
     */
    public SwerveDrivetrainConfig setDriftPID(double kP, double kI, double kd){
        return setDriftPID(new PIDController(kP, kI, kd));
    }
    
    /**
     * Supplies a fully constructed drift-correction PID controller.
     */
    public SwerveDrivetrainConfig setDriftPID(PIDController driftPID){
        this.driftPID = driftPID;
        return this;
    }

    /**
     * Sets the minimum drive speed (m/s) required before drift correction engages.
     */
    public SwerveDrivetrainConfig setDriftActivationSpeed(double driftActivationSpeed){
        this.driftActivationSpeed = driftActivationSpeed;
        return this;
    }

    /**
     * Controls whether the drivetrain interprets inputs as field-relative by default.
     */
    public SwerveDrivetrainConfig setFieldRelative(boolean fieldRelative){
        this.fieldRelative = fieldRelative;
        return this;
    }

    /**
     * Sets the CAN IDs used for drive motors (FL, FR, BL, BR). Negative IDs invert that motor.
     */
    public SwerveDrivetrainConfig setDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    /**
     * Sets the CAN IDs used for steering motors (FL, FR, BL, BR). Negative IDs invert that motor.
     */
    public SwerveDrivetrainConfig setSteerIds(int[] steerIDs){
        this.steerIDs = steerIDs;
        return this;
    } 

    /**
     * Sets the CAN IDs used for steering encoders (FL, FR, BL, BR). Negative IDs invert that encoder.
     */
    public SwerveDrivetrainConfig setEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    /**
     * Populates drive motor IDs from a {@link DriveIDs} preset.
     */
    public SwerveDrivetrainConfig setDriveIds(DriveIDs driveIds){
       return setDriveIds(driveIds.getIDs());
    } 

    /**
     * Populates steer motor IDs from a {@link SteerIDs} preset.
     */
    public SwerveDrivetrainConfig setSteerIds(SteerIDs steerIDs){
        return setSteerIds(steerIDs.getIDs());
    } 

    /**
     * Populates encoder IDs from an {@link EncoderIDs} preset.
     */
    public SwerveDrivetrainConfig setEncoderIds(EncoderIDs encoderIds){
        return setEncoderIds(encoderIds.getIDs());
    } 

    /**
     * Declares the CAN device ID used by the IMU.
     */
    public SwerveDrivetrainConfig setIMUId(int id){
        gryoId = id;
        return this;
    }

    /**
     * Applies a common {@link DrivetrainIDs} preset covering drive, steer, encoder, and gyro IDs.
     */
    public SwerveDrivetrainConfig setIds(DrivetrainIDs ids){
        setDriveIds(ids.getDrive());
        setSteerIds(ids.getSteer());
        setEncoderIds(ids.getEncoders());
        setIMUId(ids.getGyro());
        return this;
    } 

    /**
     * Sets whether drive motors should be inverted.
     */
    public SwerveDrivetrainConfig setDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        return this;
    } 

    /**
     * Sets whether steer motors should be inverted.
     */
    public SwerveDrivetrainConfig setSteerInverted(boolean steerInverted){
        this.steerInverted = steerInverted;
        return this;
    } 

    /**
     * Sets whether steering encoders should be inverted.
     */
    public SwerveDrivetrainConfig setEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        return this;
    } 

    /**
     * Creates a rotation PID controller used for heading control with the provided gains.
     */
    public SwerveDrivetrainConfig setRotationPID(double kP, double kI, double kd){
        return setRotationPID(new PIDController(kP, kI, kd));
    }
    
    /**
     * Supplies a fully constructed rotation PID controller.
     */
    public SwerveDrivetrainConfig setRotationPID(PIDController rotationPID){
        this.rotationPID = rotationPID;
        return this;
    }
    
    /**
     * Applies the same current limit to both drive and steer motors.
     */
    public SwerveDrivetrainConfig setCurrentLimit(double currentLimit){
        return setCurrentLimit(currentLimit, currentLimit);
    } 

    /**
     * Applies the same current limit to both drive and steer motors.
     */
    public SwerveDrivetrainConfig setCurrentLimit(double driveCurrentLimit, double steerCurrentLimit){
        setDriveCurrentLimit(driveCurrentLimit);
        setSteerCurrentLimit(steerCurrentLimit);
        return this;
    } 

    /**
     * Sets the drive motor current limit (amps).
     */
    public SwerveDrivetrainConfig setDriveCurrentLimit(double currentLimit){
        this.driveCurrentLimit = currentLimit;
        return this;
    } 

    /**
     * Sets the steer motor current limit (amps).
     */
    public SwerveDrivetrainConfig setSteerCurrentLimit(double currentLimit){
        this.steerCurrentLimit = currentLimit;
        return this;
    } 


    /**
     * Provides absolute encoder offsets in radians (FL, FR, BL, BR).
     */
    public SwerveDrivetrainConfig setEncoderOffset(double... encoderOffsets){
        this.encoderOffsets = encoderOffsets;
        return this;
    } 

    /**
     * Sets the common CAN bus used by all motors, encoders, and sensors.
     */
    public SwerveDrivetrainConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    /**
     * Declares the physical module locations relative to robot center (FL, FR, BL, BR).
     */
    public SwerveDrivetrainConfig setModuleLocations(Translation2d[] locations){
        this.locations = locations;
        return this;
    }

    /**
     * Declares module locations assuming a rectangular footprint (track width x wheelbase).
     */
    public SwerveDrivetrainConfig setModuleLocations(double trackWidth, double wheelbase){
        return setModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase));
    }

    /**
     * Declares module locations assuming a square footprint (track width on both axes).
     */
    public SwerveDrivetrainConfig setModuleLocations(double trackWidth){
        return setModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, trackWidth));
    }

    /**
     * Overrides the drivetrain simulation configuration used when running in WPILib simulation.
     */
    public SwerveDrivetrainConfig setSimulationConfig(SwerveSimulationConfig simulationConfig) {
        this.simulationConfig = simulationConfig;
        return this;
    }


    /**
     * Applies all registered configuration options to the module definitions and constructs the
     * {@link SwerveDrivetrain}. Also wires up optional drift/rotation controllers and simulation
     * configuration.
     *
     * @throws Error if any module configuration arrays (IDs, offsets, locations) are mismatched in size
     */
    @Override
    public SwerveDrivetrain build() {
        if (encoderIds.length != modules.length) {
        throw new Error("ENCODER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderIds.length+")");
        }

        if (driveIds.length != modules.length) {
        throw new Error("DRIVE ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+driveIds.length+")");
        }

        if (steerIDs.length != modules.length) {
        throw new Error("STEER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+steerIDs.length+")");
        }

        if (encoderOffsets.length != modules.length) {
        throw new Error("ENCODER OFFSETS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderOffsets.length+")");
        }

        if (locations.length != modules.length) {
        throw new Error("MODULE LOCATIONS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+locations.length+")");
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i] = modules[i].setEncoderID(encoderIds[i])
                                    .setSteerID(steerIDs[i])
                                    .setDriveID(driveIds[i])
                                    .setEncoderInverted(encoderInverted)
                                    .setSteerInverted(steerInverted)
                                    .setDriveInverted(driveInverted)
                                    .setSteerCurrentLimit(steerCurrentLimit)
                                    .setDriveCurrentLimit(driveCurrentLimit)
                                    .setOffset(encoderOffsets[i])
                                    .setCanbus(canbus)
                                    .setLocation(locations[i]);
            
            if(rotationPID != null){
                modules[i] = modules[i].setPID(rotationPID);
            }
        } 
        imu = imu.setId(gryoId).setCanbus(canbus);

        SwerveDrivetrain dt = new SwerveDrivetrain(IMU.fromConfig(imu), modules);

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            dt.configureSimulation(simulationConfig != null ? simulationConfig : SwerveSimulationConfig.defaults());
        }

        if (driftPID != null){
            dt.setDriftCorrectionPID(driftPID);
            dt.setDriftCorrectionMode(true);
        }

        dt.setFieldRelative(fieldRelative);
        dt.driftActivationSpeed = driftActivationSpeed;
        return dt;
    }
}
