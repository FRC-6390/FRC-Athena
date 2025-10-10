package ca.frc6390.athena.drivetrains.swerve.sim;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrivetrainSimulation {

    private final SwerveDrivetrain drivetrain;
    private final SwerveSimulationConfig config;
    private final SwerveModuleSimulation[] moduleSimulations;
    private Pose2d pose = new Pose2d();

    public SwerveDrivetrainSimulation(SwerveDrivetrain drivetrain, SwerveSimulationConfig config) {
        this.drivetrain = drivetrain;
        this.config = config;

        SwerveModule[] modules = drivetrain.swerveModules;
        moduleSimulations = new SwerveModuleSimulation[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleSimulations[i] = new SwerveModuleSimulation(modules[i], config, modules.length);
            modules[i].attachSimulation(moduleSimulations[i]);
            moduleSimulations[i].applyToSensors();
        }
    }

    public ChassisSpeeds update(double dtSeconds, ChassisSpeeds referenceSpeeds) {
        SwerveModuleState[] states = new SwerveModuleState[moduleSimulations.length];

        for (int i = 0; i < moduleSimulations.length; i++) {
            moduleSimulations[i].update(dtSeconds);
            states[i] = moduleSimulations[i].getModuleState();
        }

        ChassisSpeeds measured = drivetrain.getKinematics().toChassisSpeeds(states);

        double vx = measured.vxMetersPerSecond;
        double vy = measured.vyMetersPerSecond;
        double omega = measured.omegaRadiansPerSecond;

        if (referenceSpeeds != null) {
            omega = referenceSpeeds.omegaRadiansPerSecond;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, omega);

        Twist2d twist = new Twist2d(
                chassisSpeeds.vxMetersPerSecond * dtSeconds,
                chassisSpeeds.vyMetersPerSecond * dtSeconds,
                chassisSpeeds.omegaRadiansPerSecond * dtSeconds);

        pose = pose.exp(twist);

        drivetrain.getIMU().setSimulatedHeading(pose.getRotation(), Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond));
        return chassisSpeeds;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        for (SwerveModuleSimulation moduleSimulation : moduleSimulations) {
            moduleSimulation.reset(0, 0);
        }
        drivetrain.getIMU().setSimulatedHeading(pose.getRotation(), Rotation2d.fromDegrees(0));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[moduleSimulations.length];
        for (int i = 0; i < moduleSimulations.length; i++) {
            positions[i] = moduleSimulations[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[moduleSimulations.length];
        for (int i = 0; i < moduleSimulations.length; i++) {
            states[i] = moduleSimulations[i].getModuleState();
        }
        return states;
    }

    public SwerveModuleSimulation[] getModuleSimulations() {
        return moduleSimulations;
    }

    public SwerveSimulationConfig getConfig() {
        return config;
    }
}
