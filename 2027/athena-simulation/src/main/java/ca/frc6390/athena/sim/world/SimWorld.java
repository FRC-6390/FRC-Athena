package ca.frc6390.athena.sim.world;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

import ca.frc6390.athena.vision.spec.CameraSpec;

/**
 * Small in-memory simulation world for examples and tests.
 */
public final class SimWorld {
    private final Map<String, SimMotorState> motors = new LinkedHashMap<>();
    private final Map<String, SimImuState> imus = new LinkedHashMap<>();
    private final Map<String, SimVisionCamera> cameras = new LinkedHashMap<>();

    /**
     * Returns an existing motor state or creates it.
     *
     * @param name motor name
     * @return motor state
     */
    public SimMotorState motor(String name) {
        return motors.computeIfAbsent(normalize(name, "motor"), SimMotorState::new);
    }

    /**
     * Returns an existing IMU state or creates it.
     *
     * @param name IMU name
     * @return IMU state
     */
    public SimImuState imu(String name) {
        return imus.computeIfAbsent(normalize(name, "imu"), SimImuState::new);
    }

    /**
     * Adds or replaces a simulated vision camera.
     *
     * @param spec camera spec
     * @return simulated camera
     */
    public SimVisionCamera camera(CameraSpec spec) {
        SimVisionCamera camera = new SimVisionCamera(spec);
        cameras.put(spec.path(), camera);
        return camera;
    }

    /**
     * Finds a simulated camera by spec path.
     *
     * @param path camera spec path
     * @return simulated camera if present
     */
    public Optional<SimVisionCamera> findCamera(String path) {
        return Optional.ofNullable(cameras.get(path));
    }

    /**
     * Advances all time-based simulation states.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        motors.values().forEach(motor -> motor.step(seconds));
        imus.values().forEach(imu -> imu.step(seconds));
    }

    private static String normalize(String name, String fallback) {
        return name == null || name.isBlank() ? fallback : name;
    }
}
