package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.hardware.SimDigitalInputHandle;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import java.util.List;

/**
 * Coordinates simulation services for the normal Athena runtime path.
 *
 * <p>It provides simulation-backed hardware, shared world state, physics
 * stepping, and test inspection APIs used by the same {@code RobotRuntime}
 * that real robots use.</p>
 */
public final class SimulationSession {
    private final SimulationCoordinator coordinator;

    private SimulationSession(SimulationCoordinator coordinator) {
        this.coordinator = coordinator == null ? new SimulationCoordinator() : coordinator;
    }

    /**
     * Creates a simulation session.
     *
     * @return session
     */
    public static SimulationSession create() {
        return new SimulationSession(new SimulationCoordinator());
    }

    /**
     * Returns the simulated hardware graph.
     *
     * @return hardware graph
     */
    public HardwareGraph hardwareGraph() {
        return coordinator.hardwareGraph();
    }

    /**
     * Registers a simulation model.
     *
     * @param name model name
     * @param model model declaration
     * @return this session
     */
    public SimulationSession model(String name, SimModel model) {
        coordinator.model(name, model);
        return this;
    }

    /**
     * Uses a specific physics engine for registered simulation models.
     *
     * @param physicsEngine physics engine
     * @return this session
     */
    public SimulationSession physicsEngine(SimPhysicsEngine physicsEngine) {
        coordinator.physicsEngine(physicsEngine);
        return this;
    }

    /**
     * Returns the simulated field pose.
     *
     * @return pose
     */
    public PoseSnapshot pose() {
        return coordinator.pose();
    }

    /**
     * Resets the simulated field pose.
     *
     * @param pose pose
     * @return this session
     */
    public SimulationSession resetPose(PoseSnapshot pose) {
        coordinator.resetPose(pose);
        return this;
    }

    /**
     * Advances the simulated field pose by a chassis velocity.
     *
     * @param xMetersPerSecond forward field velocity
     * @param yMetersPerSecond left field velocity
     * @param headingRadiansPerSecond angular velocity
     * @param seconds timestep in seconds
     * @return this session
     */
    public SimulationSession drivePose(
            double xMetersPerSecond,
            double yMetersPerSecond,
            double headingRadiansPerSecond,
            double seconds) {
        coordinator.drivePose(xMetersPerSecond, yMetersPerSecond, headingRadiansPerSecond, seconds);
        return this;
    }

    /**
     * Advances simulated hardware and physics.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        coordinator.step(this, seconds);
    }

    /**
     * Returns registered simulation models.
     *
     * @return models
     */
    public List<SimModel> registeredModels() {
        return coordinator.registeredModels();
    }

    /**
     * Registers a vision simulation updated from the simulated field pose.
     *
     * @param simulation vision simulation
     * @return this session
     */
    public SimulationSession vision(VisionSimulation simulation) {
        coordinator.vision(simulation);
        return this;
    }

    /**
     * Sets the field target layout used by discovered vision simulation providers.
     *
     * @param field vision simulation field
     * @return this session
     */
    public SimulationSession visionField(VisionSimulationField field) {
        coordinator.visionField(field);
        return this;
    }

    /**
     * Runs work inside this session's digital input runtime scope.
     *
     * @param work work
     */
    public void withDigitalInputs(Runnable work) {
        DigitalInputDevice.withRuntime(coordinator.digitalInputScope(), work);
    }

    /**
     * Runs work inside this session's digital input runtime scope.
     *
     * @param work work
     * @param <T> result type
     * @return result
     */
    public <T> T withDigitalInputs(java.util.function.Supplier<T> work) {
        return DigitalInputDevice.withRuntime(coordinator.digitalInputScope(), work);
    }

    /**
     * Returns the field target layout used by discovered vision simulation providers.
     *
     * @return vision simulation field
     */
    public VisionSimulationField visionField() {
        return coordinator.visionField();
    }

    /**
     * Returns registered vision simulations.
     *
     * @return vision simulations
     */
    public List<VisionSimulation> visionSimulations() {
        return coordinator.visionSimulations();
    }

    /**
     * Returns the simulated motor handle for a real motor declaration.
     *
     * @param device motor declaration
     * @return simulated motor handle
     */
    public SimMotorHandle motor(MotorDevice device) {
        return coordinator.motor(device);
    }

    /**
     * Returns the simulated encoder handle for a real encoder declaration.
     *
     * @param device encoder declaration
     * @return simulated encoder handle
     */
    public SimEncoderHandle encoder(EncoderDevice device) {
        return coordinator.encoder(device);
    }

    /**
     * Returns the simulated digital input handle for a real input declaration.
     *
     * @param device digital input declaration
     * @return simulated digital input handle
     */
    public SimDigitalInputHandle digitalInput(DigitalInputDevice device) {
        return coordinator.digitalInput(device);
    }

    /**
     * Returns the simulated IMU handle for a real IMU declaration.
     *
     * @param device IMU declaration
     * @return simulated IMU handle
     */
    public SimImuHandle imu(ImuDevice device) {
        return coordinator.imu(device);
    }
}
