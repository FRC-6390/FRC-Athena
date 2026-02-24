package ca.frc6390.athena.core.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import ca.frc6390.athena.core.examples.VendorModuleExamples;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

final class VendorModuleServiceContractsTest {

    private record ServiceContract(String module, String serviceInterface, List<String> implementations) {}

    @Test
    void serviceDescriptorsMatchExpectedImplementationsAndSourceFiles() throws IOException {
        Path repoRoot = repoRoot();

        for (ServiceContract contract : serviceContracts()) {
            Path moduleRoot = repoRoot.resolve(contract.module());
            assertTrue(Files.isDirectory(moduleRoot), "missing module folder: " + moduleRoot);

            Path descriptor = moduleRoot.resolve("src/main/resources")
                    .resolve(VendorModuleExamples.serviceDescriptorPath(contract.serviceInterface()));
            assertTrue(Files.exists(descriptor), "missing service descriptor: " + descriptor);

            List<String> lines = Files.readAllLines(descriptor).stream()
                    .map(String::trim)
                    .filter(line -> !line.isEmpty())
                    .filter(line -> !line.startsWith("#"))
                    .collect(Collectors.toList());
            assertEquals(contract.implementations(), lines, "descriptor mismatch for " + descriptor);

            for (String implementation : lines) {
                Path sourceFile = moduleRoot.resolve("src/main/java")
                        .resolve(implementation.replace('.', '/') + ".java");
                assertTrue(Files.exists(sourceFile), "missing provider source: " + sourceFile);
            }
        }
    }

    @Test
    void publishedVendorKeysUseStablePrefixes() {
        Map<String, String> autoKeys = VendorModuleExamples.expectedAutoEngineKeys();
        Map<String, String> cameraKeys = VendorModuleExamples.expectedCameraSoftwareKeys();

        assertFalse(autoKeys.isEmpty());
        assertFalse(cameraKeys.isEmpty());
        assertTrue(autoKeys.values().stream().allMatch(key -> key.startsWith("auto:")));
        assertTrue(cameraKeys.values().stream().allMatch(key -> key.startsWith("camera:")));

        assertEquals("auto:pathplanner", autoKeys.get("athena-pathplanner"));
        assertEquals("auto:choreo", autoKeys.get("athena-choreo"));
        assertEquals("camera:photonvision", cameraKeys.get("athena-photonvision"));
        assertEquals("camera:limelight", cameraKeys.get("athena-limelight"));
    }

    @Test
    void providerSourcesPublishDocumentedStringBackedKeys() throws IOException {
        Path repoRoot = repoRoot();
        Map<String, List<String>> sourceKeyExpectations = Map.of(
                "athena-pathplanner/src/main/java/ca/frc6390/athena/pathplanner/PathPlannerAutoProvider.java",
                List.of("\"auto:pathplanner\""),
                "athena-choreo/src/main/java/ca/frc6390/athena/choreo/ChoreoAutoProvider.java",
                List.of("\"auto:choreo\""),
                "athena-photonvision/src/main/java/ca/frc6390/athena/sensors/camera/photonvision/PhotonVisionProvider.java",
                List.of("\"camera:photonvision\""),
                "athena-limelight/src/main/java/ca/frc6390/athena/sensors/camera/limelight/LimelightProvider.java",
                List.of("\"camera:limelight\""));

        for (Map.Entry<String, List<String>> entry : sourceKeyExpectations.entrySet()) {
            Path sourceFile = repoRoot.resolve(entry.getKey());
            assertTrue(Files.exists(sourceFile), "missing source file: " + sourceFile);
            String source = Files.readString(sourceFile);
            for (String expectedToken : entry.getValue()) {
                assertTrue(source.contains(expectedToken),
                        "expected token " + expectedToken + " in " + sourceFile);
            }
        }
    }

    private static List<ServiceContract> serviceContracts() {
        return List.of(
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.motor.MotorRegistry$Provider",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareProvider")),
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.encoder.EncoderRegistry$Provider",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareProvider")),
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.imu.ImuRegistry$Provider",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareProvider")),
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.factory.MotorControllerFactory",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareFactory")),
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.factory.EncoderFactory",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareFactory")),
                new ServiceContract(
                        "athena-ctre",
                        "ca.frc6390.athena.hardware.factory.ImuFactory",
                        List.of("ca.frc6390.athena.ctre.CtreHardwareFactory")),
                new ServiceContract(
                        "athena-rev",
                        "ca.frc6390.athena.hardware.motor.MotorRegistry$Provider",
                        List.of("ca.frc6390.athena.rev.RevHardwareProvider")),
                new ServiceContract(
                        "athena-rev",
                        "ca.frc6390.athena.hardware.encoder.EncoderRegistry$Provider",
                        List.of("ca.frc6390.athena.rev.RevHardwareProvider")),
                new ServiceContract(
                        "athena-rev",
                        "ca.frc6390.athena.hardware.factory.MotorControllerFactory",
                        List.of("ca.frc6390.athena.rev.RevHardwareFactory")),
                new ServiceContract(
                        "athena-rev",
                        "ca.frc6390.athena.hardware.factory.EncoderFactory",
                        List.of("ca.frc6390.athena.rev.RevHardwareFactory")),
                new ServiceContract(
                        "athena-studica",
                        "ca.frc6390.athena.hardware.imu.ImuRegistry$Provider",
                        List.of("ca.frc6390.athena.studica.imu.StudicaImuProvider")),
                new ServiceContract(
                        "athena-studica",
                        "ca.frc6390.athena.hardware.factory.ImuFactory",
                        List.of("ca.frc6390.athena.studica.imu.StudicaImuFactory")),
                new ServiceContract(
                        "athena-photonvision",
                        "ca.frc6390.athena.sensors.camera.CameraRegistry$Provider",
                        List.of("ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionProvider")),
                new ServiceContract(
                        "athena-photonvision",
                        "ca.frc6390.athena.sensors.camera.CameraProvider",
                        List.of("ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionCameraProvider")),
                new ServiceContract(
                        "athena-photonvision",
                        "ca.frc6390.athena.core.sim.RobotVisionSimProvider",
                        List.of("ca.frc6390.athena.sensors.camera.photonvision.sim.PhotonVisionSimProvider")),
                new ServiceContract(
                        "athena-limelight",
                        "ca.frc6390.athena.sensors.camera.CameraRegistry$Provider",
                        List.of("ca.frc6390.athena.sensors.camera.limelight.LimelightProvider")),
                new ServiceContract(
                        "athena-limelight",
                        "ca.frc6390.athena.sensors.camera.CameraProvider",
                        List.of("ca.frc6390.athena.sensors.camera.limelight.LimelightCameraProvider")),
                new ServiceContract(
                        "athena-pathplanner",
                        "ca.frc6390.athena.core.AutoRegistry$Provider",
                        List.of("ca.frc6390.athena.pathplanner.PathPlannerAutoProvider")),
                new ServiceContract(
                        "athena-pathplanner",
                        "ca.frc6390.athena.core.auto.AutoBackend",
                        List.of("ca.frc6390.athena.pathplanner.PathPlannerAutoBackend")),
                new ServiceContract(
                        "athena-choreo",
                        "ca.frc6390.athena.core.AutoRegistry$Provider",
                        List.of("ca.frc6390.athena.choreo.ChoreoAutoProvider")),
                new ServiceContract(
                        "athena-choreo",
                        "ca.frc6390.athena.core.auto.AutoBackend",
                        List.of("ca.frc6390.athena.choreo.ChoreoAutoBackend")));
    }

    private static Path repoRoot() {
        Path start = Paths.get("").toAbsolutePath();
        for (Path cursor = start; cursor != null; cursor = cursor.getParent()) {
            if (Files.exists(cursor.resolve("settings.gradle"))) {
                return cursor;
            }
        }
        fail("Unable to locate repo root from " + start);
        return start;
    }
}
