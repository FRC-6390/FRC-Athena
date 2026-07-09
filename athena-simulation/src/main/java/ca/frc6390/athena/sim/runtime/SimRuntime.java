package ca.frc6390.athena.sim.runtime;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;

/**
 * Simulation runtime root for in-memory hardware handles.
 */
public final class SimRuntime {
    private final Map<HardwareIdentity, SimMotorHandle> motors = new LinkedHashMap<>();
    private final Map<HardwareIdentity, SimImuHandle> imus = new LinkedHashMap<>();
    private final Map<String, SimModel> models = new LinkedHashMap<>();

    /**
     * Registers a simulation model and materializes its hardware handles.
     *
     * @param name model name
     * @param model simulation model declaration
     * @return this runtime
     */
    public SimRuntime model(String name, SimModel model) {
        Objects.requireNonNull(model, "model");
        String key = normalizeName(name, "model-" + models.size());
        models.put(key, model);
        model.motors().forEach(this::motor);
        return this;
    }

    /**
     * Registers simulation models using generated names.
     *
     * @param models simulation model declarations
     * @return this runtime
     */
    public SimRuntime models(SimModel... models) {
        if (models != null) {
            for (SimModel model : models) {
                if (model != null) {
                    model("model-" + this.models.size(), model);
                }
            }
        }
        return this;
    }

    /**
     * Returns registered simulation models.
     *
     * @return models
     */
    public List<SimModel> registeredModels() {
        return List.copyOf(models.values());
    }

    /**
     * Creates a hardware graph backed by this simulation runtime.
     *
     * @return hardware graph
     */
    public HardwareGraph hardwareGraph() {
        return HardwareGraph.using(BackendRegistry.of(
                List.of(new RuntimeMotorBackend()),
                List.of(),
                List.of(new RuntimeImuBackend())));
    }

    /**
     * Returns an existing motor handle or creates it.
     *
     * @param device motor declaration
     * @return motor handle
     */
    public SimMotorHandle motor(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity key = HardwareIdentity.motor(device);
        return motors.computeIfAbsent(key, ignored -> new SimMotorHandle(device));
    }

    /**
     * Returns an existing IMU handle or creates it.
     *
     * @param device IMU declaration
     * @return IMU handle
     */
    public SimImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity key = HardwareIdentity.imu(device);
        return imus.computeIfAbsent(key, ignored -> new SimImuHandle(device));
    }

    /**
     * Advances all time-based simulation states.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        motors.values().forEach(handle -> handle.step(seconds));
        imus.values().forEach(handle -> handle.step(seconds));
    }

    private static String normalizeName(String requested, String fallback) {
        return requested == null || requested.isBlank() ? fallback : requested.trim();
    }

    private final class RuntimeMotorBackend implements MotorBackend {
        @Override
        public boolean supports(MotorKind kind) {
            return kind.key().startsWith("sim:");
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            return SimRuntime.this.motor(device);
        }
    }

    private final class RuntimeImuBackend implements ImuBackend {
        @Override
        public boolean supports(ImuKind kind) {
            return kind.key().startsWith("sim:");
        }

        @Override
        public ImuHandle create(ImuDevice device) {
            return SimRuntime.this.imu(device);
        }
    }
}
