package ca.frc6390.athena.core;

import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface RobotDrivetrain<T extends RobotDrivetrain<T>> extends RobotSendableSystem, Subsystem {

    public interface RobotDrivetrainConfig<T extends RobotDrivetrain<T>> {
        T build();
    }

    interface ControlSection {
        Command command(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);

        void defaultCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);

        default void defaultCommand(EnhancedXboxController driverController) {
            defaultCommand(driverController.leftY, driverController.leftX, driverController.rightX);
        }

        void reset();

        void stop();
    }

    interface SpeedsSection {
        MotionLimits limits();

        ChassisSpeeds get(String source);

        ChassisSpeeds getInput(String source);

        SpeedsSection set(String source, ChassisSpeeds speeds);

        SpeedsSection set(String source, double x, double y, double theta);

        SpeedsSection stop(String source);

        SpeedsSection stop();

        SpeedsSection enabled(String source, boolean enabled);

        boolean enabled(String source);

        double maxVelocity();

        double maxAngularVelocity();

        Set<String> sources();

        default ChassisSpeeds input(String source) {
            return getInput(source);
        }

        default ChassisSpeeds output(String source) {
            return get(source);
        }

        default SpeedsSection output(String source, ChassisSpeeds speeds) {
            return set(source, speeds);
        }

        default SpeedsSection sourceEnabled(String source, boolean enabled) {
            return enabled(source, enabled);
        }

        default boolean sourceEnabled(String source) {
            return enabled(source);
        }
    }

    interface ModulesSection {
    }

    interface HardwareSection {
        void neutralMode(MotorNeutralMode mode);
    }

    interface SysIdSection {
        double rampRateVoltsPerSecond();

        void rampRateVoltsPerSecond(double voltsPerSecond);

        double stepVoltage();

        void stepVoltage(double volts);

        double timeoutSeconds();

        void timeoutSeconds(double seconds);

        boolean active();

        Command quasistatic(SysIdRoutine.Direction direction);

        Command dynamic(SysIdRoutine.Direction direction);
    }

    interface ImuSection {
        Imu device();
    }

    interface SimulationSection {
        boolean enabled();

        Pose2d pose();

        void pose(Pose2d pose);
    }

    T control(Consumer<ControlSection> section);

    ControlSection control();

    T speeds(Consumer<SpeedsSection> section);

    SpeedsSection speeds();

    T modules(Consumer<ModulesSection> section);

    ModulesSection modules();

    T hardware(Consumer<HardwareSection> section);

    HardwareSection hardware();

    T sysId(Consumer<SysIdSection> section);

    SysIdSection sysId();

    T imu(Consumer<ImuSection> section);

    ImuSection imu();

    T simulation(Consumer<SimulationSection> section);

    SimulationSection simulation();

    RobotSpeeds robotSpeeds();

    void update();

    @Override
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        Imu imu = imu().device();
        if (imu != null) {
            imu.networkTables(node.child("IMU"));
        }

        if (nt.enabled(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS)) {
            RobotNetworkTables.Node speeds = node.child("RobotSpeeds");
            for (String source : speeds().sources()) {
                RobotNetworkTables.Node sourceNode = speeds.child(source);
                ChassisSpeeds input = speeds().getInput(source);
                ChassisSpeeds output = speeds().get(source);
                RobotSpeeds robotSpeeds = robotSpeeds();
                sourceNode.putBoolean("enabled", speeds().enabled(source));
                sourceNode.putBoolean("axisXEnabled", robotSpeeds.isAxisActive(source, RobotSpeeds.SpeedAxis.X));
                sourceNode.putBoolean("axisYEnabled", robotSpeeds.isAxisActive(source, RobotSpeeds.SpeedAxis.Y));
                sourceNode.putBoolean("axisThetaEnabled", robotSpeeds.isAxisActive(source, RobotSpeeds.SpeedAxis.Theta));
                sourceNode.child("input").putDouble("vxMps", input.vxMetersPerSecond);
                sourceNode.child("input").putDouble("vyMps", input.vyMetersPerSecond);
                sourceNode.child("input").putDouble("omegaRadPerSec", input.omegaRadiansPerSecond);
                sourceNode.putDouble("vxMps", output.vxMetersPerSecond);
                sourceNode.putDouble("vyMps", output.vyMetersPerSecond);
                sourceNode.putDouble("omegaRadPerSec", output.omegaRadiansPerSecond);
            }
        }

        // Commands are intentionally not published here; dashboards can invoke actions via your own bindings.
        return node;
    }

    public class RobotDrivetrainIDs {

        public enum DrivetrainIDs {
            SWERVE_CHASSIS_STANDARD(DriveIDs.SWERVE_CHASSIS_STANDARD, SteerIDs.SWERVE_CHASSIS_STANDARD, EncoderIDs.SWERVE_CHASSIS_STANDARD, 0),
            DUAL_MOTOR_DIFFERENTIAL(DriveIDs.DUAL_MOTOR_DIFFERENTIAL, SteerIDs.DUAL_MOTOR_DIFFERENTIAL, EncoderIDs.DUAL_MOTOR_DIFFERENTIAL, 0);

            private final DriveIDs drive;
            private final SteerIDs steer;
            private final EncoderIDs encoders;
            private final int gyro;
    
            DrivetrainIDs(DriveIDs drive, SteerIDs steer, EncoderIDs encoders, int gyro) {
                this.drive = drive;
                this.steer = steer;
                this.encoders = encoders;
                this.gyro = gyro;
            }
    
            public DriveIDs getDrive() {
                return drive;
            }

            public EncoderIDs getEncoders() {
                return encoders;
            }

            public SteerIDs getSteer() {
                return steer;
            }

            public int getGyro(){
                return gyro;
            }
        }
    
        public enum DriveIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {1,2,3,4}),
            DUAL_MOTOR_DIFFERENTIAL(new int[] {1,2,3,4});

            private final int[] ids;
    
            DriveIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    
        public enum SteerIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {5,6,7,8}),
            DUAL_MOTOR_DIFFERENTIAL(new int[] {});

            private final int[] ids;
    
            SteerIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    
        public enum EncoderIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {9,10,11,12}),
            DUAL_MOTOR_DIFFERENTIAL(new int[] {5,6});
    
            private final int[] ids;
    
            EncoderIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    }
}
