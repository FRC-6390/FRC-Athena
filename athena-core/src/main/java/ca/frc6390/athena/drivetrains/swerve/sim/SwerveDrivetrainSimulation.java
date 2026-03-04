package ca.frc6390.athena.drivetrains.swerve.sim;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrivetrainSimulation {

    private final SwerveDrivetrain drivetrain;
    private final SwerveSimulationConfig config;
    private final SwerveModuleSimulation[] moduleSimulations;
    private Pose2d pose = new Pose2d();
    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
    private Rotation2d lastChassisHeading = Rotation2d.kZero;
    private boolean hasLastChassisHeading = false;

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

        ChassisSpeeds measured = drivetrain.kinematicsModel().toChassisSpeeds(states);

        double vx = measured.vxMetersPerSecond;
        double vy = measured.vyMetersPerSecond;
        double omega = measured.omegaRadiansPerSecond;

        // Apply traction-limited acceleration to avoid unrealistic jumps in sim.
        double tractionScaleAccel = 0.8; // keep sim responsive while still traction-limited
        double tractionScaleDecel = 1.1; // allow slightly stronger braking than acceleration
        double maxAccel = Math.max(0.0, tractionScaleAccel * config.getWheelCoefficientOfFriction() * 9.81); // m/s^2
        double maxDecel = Math.max(0.0, tractionScaleDecel * config.getWheelCoefficientOfFriction() * 9.81); // m/s^2

        // Apply linear acceleration limiting in field frame to avoid artificial axis coupling
        // when robot-relative velocity components rotate under nonzero omega.
        Rotation2d heading = pose.getRotation();
        Rotation2d lastHeading = hasLastChassisHeading ? lastChassisHeading : heading;
        ChassisSpeeds fieldMeasured = ChassisSpeeds.fromRobotRelativeSpeeds(
                vx,
                vy,
                omega,
                heading);
        ChassisSpeeds fieldLast = ChassisSpeeds.fromRobotRelativeSpeeds(
                lastChassisSpeeds.vxMetersPerSecond,
                lastChassisSpeeds.vyMetersPerSecond,
                lastChassisSpeeds.omegaRadiansPerSecond,
                lastHeading);

        double dvxField = fieldMeasured.vxMetersPerSecond - fieldLast.vxMetersPerSecond;
        double dvyField = fieldMeasured.vyMetersPerSecond - fieldLast.vyMetersPerSecond;
        double lastSpeedNorm = Math.hypot(fieldLast.vxMetersPerSecond, fieldLast.vyMetersPerSecond);
        double decelDot = dvxField * fieldLast.vxMetersPerSecond + dvyField * fieldLast.vyMetersPerSecond;
        double maxDelta = (lastSpeedNorm > 1e-6 && decelDot < 0.0) ? maxDecel * dtSeconds : maxAccel * dtSeconds;
        double deltaNorm = Math.hypot(dvxField, dvyField);
        if (deltaNorm > maxDelta && deltaNorm > 1e-6) {
            double scale = maxDelta / deltaNorm;
            fieldMeasured = new ChassisSpeeds(
                    fieldLast.vxMetersPerSecond + dvxField * scale,
                    fieldLast.vyMetersPerSecond + dvyField * scale,
                    omega);
            ChassisSpeeds limitedRobot = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldMeasured.vxMetersPerSecond,
                    fieldMeasured.vyMetersPerSecond,
                    omega,
                    heading);
            vx = limitedRobot.vxMetersPerSecond;
            vy = limitedRobot.vyMetersPerSecond;
        }

        double maxVelCapability = drivetrain.speeds().maxVelocity();
        double maxOmegaCapability = drivetrain.speeds().maxAngularVelocity();
        double speedScale = config.getMaxSpeedScale();
        if (!Double.isFinite(speedScale) || speedScale <= 0.0) {
            speedScale = 1.0;
        }
        maxVelCapability *= speedScale;
        maxOmegaCapability *= speedScale;
        double effectiveRadius = (maxOmegaCapability > 1e-6) ? (maxVelCapability / maxOmegaCapability) : 1.0;
        double maxOmegaAccel = effectiveRadius > 1e-6 ? maxAccel / effectiveRadius : maxAccel;
        double maxOmegaDecel = effectiveRadius > 1e-6 ? maxDecel / effectiveRadius : maxDecel;
        double domega = omega - lastChassisSpeeds.omegaRadiansPerSecond;
        double maxDeltaOmega = Math.abs(lastChassisSpeeds.omegaRadiansPerSecond) > 1e-6 && domega * lastChassisSpeeds.omegaRadiansPerSecond < 0.0
                ? maxOmegaDecel * dtSeconds
                : maxOmegaAccel * dtSeconds;
        omega = lastChassisSpeeds.omegaRadiansPerSecond + MathUtil.clamp(domega, -maxDeltaOmega, maxDeltaOmega);

        // Clamp simulated chassis speeds to the drivetrain capability to prevent runaway motion in sim.
        if (Double.isFinite(maxVelCapability) && maxVelCapability > 1e-6) {
            double norm = Math.hypot(vx, vy);
            if (norm > maxVelCapability) {
                double scale = maxVelCapability / norm;
                vx *= scale;
                vy *= scale;
            }
        }
        if (Double.isFinite(maxOmegaCapability) && maxOmegaCapability > 1e-6) {
            omega = Math.copySign(Math.min(Math.abs(omega), maxOmegaCapability), omega);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, omega);

        Twist2d twist = new Twist2d(
                chassisSpeeds.vxMetersPerSecond * dtSeconds,
                chassisSpeeds.vyMetersPerSecond * dtSeconds,
                chassisSpeeds.omegaRadiansPerSecond * dtSeconds);

        pose = pose.exp(twist);
        lastChassisSpeeds = chassisSpeeds;
        lastChassisHeading = heading;
        hasLastChassisHeading = true;

        drivetrain.imu().device().setSimulatedHeading(pose.getRotation(), Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond));
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
        lastChassisSpeeds = new ChassisSpeeds();
        lastChassisHeading = pose.getRotation();
        hasLastChassisHeading = false;
        drivetrain.imu().device().setSimulatedHeading(pose.getRotation(), Rotation2d.fromDegrees(0));
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
