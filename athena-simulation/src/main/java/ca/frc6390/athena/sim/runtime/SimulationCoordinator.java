package ca.frc6390.athena.sim.runtime;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.hardware.SimDigitalInputHandle;
import ca.frc6390.athena.sim.hardware.SimEncoderBackend;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorBackend;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;

final class SimulationCoordinator {
    private final Map<HardwareIdentity, SimMotorHandle> motors = new LinkedHashMap<>();
    private final Map<HardwareIdentity, SimEncoderHandle> encoders = new LinkedHashMap<>();
    private final Map<HardwareIdentity, SimImuHandle> imus = new LinkedHashMap<>();
    private final Map<DigitalInputDevice, SimDigitalInputHandle> digitalInputs = new LinkedHashMap<>();
    private final DigitalInputDevice.RuntimeScope digitalInputScope = DigitalInputDevice.runtimeScope();
    private final Map<String, SimModel> models = new LinkedHashMap<>();
    private final List<VisionSimulation> visionSimulations = new java.util.ArrayList<>();
    private final Set<HardwareIdentity> modeledMotors = new LinkedHashSet<>();
    private final RuntimeMotorBackend motorBackend = new RuntimeMotorBackend();
    private final RuntimeEncoderBackend encoderBackend = new RuntimeEncoderBackend();
    private final RuntimeImuBackend imuBackend = new RuntimeImuBackend();
    private SimPhysicsEngine physicsEngine = new SimModelRunner();
    private PoseSnapshot pose = new PoseSnapshot(0.0, 0.0, 0.0);
    private VisionSimulationField visionField = VisionSimulationField.EMPTY;

    /**
     * Registers a simulation model and materializes its hardware handles.
     *
     * @param name model name
     * @param model simulation model declaration
     * @return this runtime
     */
    SimulationCoordinator model(String name, SimModel model) {
        Objects.requireNonNull(model, "model");
        String key = normalizeName(name, "model-" + models.size());
        models.put(key, model);
        model.motors().forEach(motor -> {
            this.motor(motor);
            modeledMotors.add(HardwareIdentity.motor(motor));
        });
        model.encoders().forEach(this::encoder);
        model.dependencies().forEach(this::materializeDependency);
        return this;
    }

    /**
     * Registers simulation models using generated names.
     *
     * @param models simulation model declarations
     * @return this runtime
     */
    SimulationCoordinator models(SimModel... models) {
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
    List<SimModel> registeredModels() {
        return List.copyOf(models.values());
    }

    /**
     * Registers a vision simulation that will be updated from the simulated field pose.
     *
     * @param simulation vision simulation
     * @return this runtime
     */
    SimulationCoordinator vision(VisionSimulation simulation) {
        visionSimulations.add(Objects.requireNonNull(simulation, "simulation"));
        return this;
    }

    /**
     * Returns registered vision simulations.
     *
     * @return vision simulations
     */
    List<VisionSimulation> visionSimulations() {
        return List.copyOf(visionSimulations);
    }

    SimulationCoordinator visionField(VisionSimulationField field) {
        visionField = field == null ? VisionSimulationField.EMPTY : field;
        return this;
    }

    VisionSimulationField visionField() {
        return visionField;
    }

    /**
     * Uses a specific physics engine for registered simulation models.
     *
     * @param physicsEngine physics engine
     * @return this runtime
     */
    SimulationCoordinator physicsEngine(SimPhysicsEngine physicsEngine) {
        this.physicsEngine = Objects.requireNonNull(physicsEngine, "physicsEngine");
        return this;
    }

    /**
     * Returns the simulated field pose.
     *
     * @return pose
     */
    PoseSnapshot pose() {
        return pose;
    }

    /**
     * Resets the simulated field pose.
     *
     * @param pose pose
     * @return this runtime
     */
    SimulationCoordinator resetPose(PoseSnapshot pose) {
        this.pose = Objects.requireNonNull(pose, "pose");
        syncImuYaw();
        return this;
    }

    /**
     * Advances the simulated field pose by a chassis velocity.
     *
     * @param xMetersPerSecond forward field velocity
     * @param yMetersPerSecond left field velocity
     * @param headingRadiansPerSecond angular velocity
     * @param seconds timestep in seconds
     * @return this runtime
     */
    SimulationCoordinator drivePose(
            double xMetersPerSecond,
            double yMetersPerSecond,
            double headingRadiansPerSecond,
            double seconds) {
        if (seconds <= 0.0
                || !Double.isFinite(xMetersPerSecond)
                || !Double.isFinite(yMetersPerSecond)
                || !Double.isFinite(headingRadiansPerSecond)) {
            return this;
        }
        pose = new PoseSnapshot(
                pose.xMeters() + xMetersPerSecond * seconds,
                pose.yMeters() + yMetersPerSecond * seconds,
                pose.headingRadians() + headingRadiansPerSecond * seconds);
        syncImuYaw();
        return this;
    }

    /**
     * Creates a hardware graph backed by this simulation runtime.
     *
     * @return hardware graph
     */
    HardwareGraph hardwareGraph() {
        return HardwareGraph.using(BackendRegistry.of(
                List.of(motorBackend),
                List.of(encoderBackend),
                List.of(imuBackend)));
    }

    /**
     * Returns an existing motor handle or creates it.
     *
     * @param device motor declaration
     * @return motor handle
     */
    SimMotorHandle motor(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity key = HardwareIdentity.motor(device);
        return motors.computeIfAbsent(key, ignored -> new SimMotorHandle(device));
    }

    /**
     * Returns an existing encoder handle or creates it.
     *
     * @param device encoder declaration
     * @return encoder handle
     */
    SimEncoderHandle encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity key = HardwareIdentity.encoder(device);
        return encoders.computeIfAbsent(key, ignored -> new SimEncoderHandle(device));
    }

    /**
     * Returns an existing digital input handle or creates it.
     *
     * @param device digital input declaration
     * @return digital input handle
     */
    SimDigitalInputHandle digitalInput(DigitalInputDevice device) {
        Objects.requireNonNull(device, "device");
        SimDigitalInputHandle handle = digitalInputs.computeIfAbsent(device, SimDigitalInputHandle::new);
        digitalInputScope.bind(device, handle::raw);
        return handle;
    }

    DigitalInputDevice.RuntimeScope digitalInputScope() {
        return digitalInputScope;
    }

    /**
     * Returns an existing IMU handle or creates it.
     *
     * @param device IMU declaration
     * @return IMU handle
     */
    SimImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity key = HardwareIdentity.imu(device);
        return imus.computeIfAbsent(key, ignored -> new SimImuHandle(device));
    }

    /**
     * Advances all time-based simulation Actions.
     *
     * @param seconds timestep in seconds
     */
    void step(SimulationSession session, double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0.0) {
            return;
        }
        physicsEngine.step(models.values(), session, seconds);
        motors.forEach((identity, handle) -> {
            if (!modeledMotors.contains(identity)) {
                handle.step(seconds);
            }
        });
        imus.values().forEach(handle -> handle.step(seconds));
        visionSimulations.forEach(simulation -> simulation.update(pose));
    }

    private void syncImuYaw() {
        double yawDegrees = Math.toDegrees(pose.headingRadians());
        imus.values().forEach(handle -> handle.yawDegrees(yawDegrees));
    }

    private static String normalizeName(String requested, String fallback) {
        return requested == null || requested.isBlank() ? fallback : requested.trim();
    }

    private void materializeDependency(Object dependency) {
        if (dependency instanceof EncoderDevice encoder) {
            encoder(encoder);
        } else if (dependency instanceof DigitalInputDevice digitalInput) {
            digitalInput(digitalInput);
        } else if (dependency instanceof SimLimit limit) {
            digitalInput(limit.sensor());
        }
    }

    Map<HardwareIdentity, SimMotorHandle> motorHandles() {
        return motors;
    }

    Map<HardwareIdentity, SimEncoderHandle> encoderHandles() {
        return encoders;
    }

    Map<DigitalInputDevice, SimDigitalInputHandle> digitalInputHandles() {
        return digitalInputs;
    }

    private final class RuntimeMotorBackend implements MotorBackend {
        private final SimMotorBackend delegate = new SimMotorBackend();

        @Override
        public boolean supports(MotorKind kind) {
            return delegate.supports(kind);
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            return SimulationCoordinator.this.motor(device);
        }
    }

    private final class RuntimeEncoderBackend implements EncoderBackend {
        private final SimEncoderBackend delegate = new SimEncoderBackend();

        @Override
        public boolean supports(EncoderKind kind) {
            return delegate.supports(kind);
        }

        @Override
        public EncoderHandle create(EncoderDevice device) {
            return SimulationCoordinator.this.encoder(device);
        }
    }

    private final class RuntimeImuBackend implements ImuBackend {
        private final ca.frc6390.athena.sim.hardware.SimImuBackend delegate = new ca.frc6390.athena.sim.hardware.SimImuBackend();

        @Override
        public boolean supports(ImuKind kind) {
            return delegate.supports(kind);
        }

        @Override
        public ImuHandle create(ImuDevice device) {
            return SimulationCoordinator.this.imu(device);
        }
    }
}
