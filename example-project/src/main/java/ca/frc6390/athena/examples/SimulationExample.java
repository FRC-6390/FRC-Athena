package ca.frc6390.athena.examples;

import ca.frc6390.athena.sim.drivetrain.SimDifferentialDrive;
import ca.frc6390.athena.sim.drivetrain.SimSwerveDrive;
import ca.frc6390.athena.sim.mechanism.SimMechanism;
import ca.frc6390.athena.sim.world.SimWorld;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;

/**
 * Simulation example using the in-memory simulation world.
 */
public final class SimulationExample {
    private SimulationExample() {
    }

    /**
     * Creates a sample simulation world with motor, IMU, and camera state.
     *
     * @return sample simulation world
     */
    public static SimWorld createWorld() {
        SimWorld world = new SimWorld();
        world.motor("drive.left").percentOutput(0.5).velocityPerSecond(2.0);
        world.imu("robot").yawRateDegreesPerSecond(45.0);
        world.camera(VisionExample.FRONT_CAMERA)
                .frame(VisionFrame.of(VisionObservation.tag(7, -3.2, -1.5, 2.4, 0.92)));
        return world;
    }

    /**
     * Creates a world with a mechanism state applied through the simulation
     * binding.
     *
     * @return world containing simulated shooter motor state
     */
    public static SimWorld createMechanismWorld() {
        SimWorld world = new SimWorld();
        new SimMechanism(world, ShooterExample.CONFIG.toSpec()).applyState("speaker");
        return world;
    }

    /**
     * Creates a world with differential drivetrain velocity targets applied.
     *
     * @return world containing simulated differential drivetrain state
     */
    public static SimWorld createDifferentialDriveWorld() {
        SimWorld world = new SimWorld();
        new SimDifferentialDrive(world, DriveExample.CONFIG.toSpec()).tankVelocity(2.0, 1.5);
        return world;
    }

    /**
     * Creates a world with swerve module targets applied.
     *
     * @return world containing simulated swerve drivetrain state
     */
    public static SimWorld createSwerveDriveWorld() {
        SimWorld world = new SimWorld();
        new SimSwerveDrive(world, SwerveExample.CONFIG.toSpec()).driveAll(3.0, 45.0);
        return world;
    }
}
