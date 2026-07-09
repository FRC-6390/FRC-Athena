package ca.frc6390.athena.vision.runtime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.ref.CameraDevice;
import ca.frc6390.athena.vision.ref.PoseSignal;
import ca.frc6390.athena.vision.ref.TargetSignal;

/**
 * Runtime graph for camera declarations and cached vision measurements.
 */
public final class VisionGraph {
    private final Map<String, CameraRuntime> cameras;

    private VisionGraph(Map<String, CameraRuntime> cameras) {
        this.cameras = Map.copyOf(cameras);
    }

    /**
     * Builds a graph from camera declarations.
     *
     * @param cameras camera declarations
     * @return vision graph
     */
    public static VisionGraph of(CameraDevice... cameras) {
        Map<String, CameraRuntime> runtimes = new LinkedHashMap<>();
        if (cameras != null) {
            for (CameraDevice camera : cameras) {
                if (camera != null) {
                    CameraRuntime runtime = new CameraRuntime(camera);
                    runtimes.put(runtime.key(), runtime);
                }
            }
        }
        return new VisionGraph(runtimes);
    }

    /**
     * Refreshes cached measurements for every camera.
     */
    public void refresh() {
        cameras.values().forEach(CameraRuntime::refresh);
    }

    /**
     * Returns runtimes for every camera in declaration order.
     *
     * @return camera runtimes
     */
    public List<CameraRuntime> cameras() {
        return List.copyOf(cameras.values());
    }

    /**
     * Returns the runtime for a camera declaration.
     *
     * @param camera camera
     * @return camera runtime
     */
    public CameraRuntime camera(CameraDevice camera) {
        Objects.requireNonNull(camera, "camera");
        CameraRuntime runtime = cameras.get(key(camera));
        if (runtime == null) {
            throw new IllegalArgumentException("Camera is not part of this vision graph: " + camera.name());
        }
        return runtime;
    }

    /**
     * Returns the latest cached pose measurements from every camera.
     *
     * @return pose measurements
     */
    public List<Measurement> poseMeasurements() {
        List<Measurement> values = new ArrayList<>();
        cameras.values().forEach(camera -> values.addAll(camera.poseMeasurements()));
        return List.copyOf(values);
    }

    /**
     * Returns the latest cached target measurements from every camera.
     *
     * @return target measurements
     */
    public List<Measurement> targetMeasurements() {
        List<Measurement> values = new ArrayList<>();
        cameras.values().forEach(camera -> values.addAll(camera.targetMeasurements()));
        return List.copyOf(values);
    }

    private static String key(CameraDevice camera) {
        return camera.kind().key() + ":" + camera.name();
    }

    /**
     * Runtime cache for one camera declaration.
     */
    public static final class CameraRuntime {
        private final CameraDevice camera;
        private final PoseSignal poseSignal;
        private final TargetSignal targetSignal;
        private List<Measurement> poseMeasurements = List.of();
        private List<Measurement> targetMeasurements = List.of();

        private CameraRuntime(CameraDevice camera) {
            this.camera = Objects.requireNonNull(camera, "camera");
            poseSignal = camera.pose();
            targetSignal = camera.targets();
        }

        /**
         * Refreshes this camera's cached signal values.
         */
        public void refresh() {
            poseMeasurements = snapshot(poseSignal.measurements());
            targetMeasurements = snapshot(targetSignal.measurements());
        }

        /**
         * Returns the camera declaration.
         *
         * @return camera declaration
         */
        public CameraDevice camera() {
            return camera;
        }

        /**
         * Returns a pose signal backed by the graph cache.
         *
         * @return cached pose signal
         */
        public PoseSignal poseSignal() {
            return new CachedPoseSignal(this);
        }

        /**
         * Returns a target signal backed by the graph cache.
         *
         * @return cached target signal
         */
        public TargetSignal targetSignal() {
            return new CachedTargetSignal(this);
        }

        /**
         * Returns cached pose measurements.
         *
         * @return pose measurements
         */
        public List<Measurement> poseMeasurements() {
            return poseMeasurements;
        }

        /**
         * Returns cached target measurements.
         *
         * @return target measurements
         */
        public List<Measurement> targetMeasurements() {
            return targetMeasurements;
        }

        private Map<String, Object> poseMetadata() {
            return poseSignal.metadata();
        }

        private Map<String, Object> targetMetadata() {
            return targetSignal.metadata();
        }

        private String key() {
            return VisionGraph.key(camera);
        }
    }

    private record CachedPoseSignal(CameraRuntime runtime) implements PoseSignal {
        private CachedPoseSignal {
            Objects.requireNonNull(runtime, "runtime");
        }

        @Override
        public CameraDevice camera() {
            return runtime.camera();
        }

        @Override
        public List<Measurement> measurements() {
            return runtime.poseMeasurements();
        }

        @Override
        public Map<String, Object> metadata() {
            return runtime.poseMetadata();
        }
    }

    private record CachedTargetSignal(CameraRuntime runtime) implements TargetSignal {
        private CachedTargetSignal {
            Objects.requireNonNull(runtime, "runtime");
        }

        @Override
        public CameraDevice camera() {
            return runtime.camera();
        }

        @Override
        public List<Measurement> measurements() {
            return runtime.targetMeasurements();
        }

        @Override
        public Map<String, Object> metadata() {
            return runtime.targetMetadata();
        }
    }

    private static List<Measurement> snapshot(List<Measurement> measurements) {
        return measurements == null || measurements.isEmpty() ? List.of() : List.copyOf(measurements);
    }
}
