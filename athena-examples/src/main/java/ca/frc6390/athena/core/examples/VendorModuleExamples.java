package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.AutoRegistry;
import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.sensors.camera.CameraRegistry;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import java.util.Map;
import java.util.Objects;

/**
 * Example lookups for vendor module keys and ServiceLoader descriptors.
 */
public final class VendorModuleExamples {
    private VendorModuleExamples() {}

    private static final Map<String, String> AUTO_ENGINE_KEYS = Map.of(
            "athena-choreo", "auto:choreo",
            "athena-pathplanner", "auto:pathplanner");

    private static final Map<String, String> CAMERA_SOFTWARE_KEYS = Map.of(
            "athena-limelight", "camera:limelight",
            "athena-photonvision", "camera:photonvision",
            "athena-helios", "camera:helios");

    public static Map<String, String> expectedAutoEngineKeys() {
        return AUTO_ENGINE_KEYS;
    }

    public static Map<String, String> expectedCameraSoftwareKeys() {
        return CAMERA_SOFTWARE_KEYS;
    }

    public static RobotAuto.AutoSource requireAutoEngine(String key) {
        return AutoRegistry.get().engine(key);
    }

    public static ConfigurableCamera.CameraSoftware requireCameraSoftware(String key) {
        return CameraRegistry.get().camera(key);
    }

    public static String serviceDescriptorPath(String serviceInterface) {
        String value = Objects.requireNonNull(serviceInterface, "serviceInterface").trim();
        if (value.isEmpty()) {
            throw new IllegalArgumentException("serviceInterface must not be blank");
        }
        return "META-INF/services/" + value;
    }
}
