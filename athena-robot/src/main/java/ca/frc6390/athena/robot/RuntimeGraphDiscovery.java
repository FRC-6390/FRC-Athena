package ca.frc6390.athena.robot;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.vision.device.CameraDevice;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Discovers runtime-owned service graphs declared inside a mechanism tree. */
final class RuntimeGraphDiscovery {
    private RuntimeGraphDiscovery() {
    }

    static Result inspect(Mechanism root) {
        List<Localization> declared = new ArrayList<>();
        List<AutoChooser> autoChoosers = new ArrayList<>();
        List<PathProvider> pathProviders = new ArrayList<>();
        Map<Localization, Mechanism> localizationOwners = new IdentityHashMap<>();
        collectMechanism(
                root,
                identitySet(),
                identitySet(),
                declared,
                identitySet(),
                autoChoosers,
                identitySet(),
                pathProviders,
                localizationOwners);

        Set<Localization> nested = identitySet();
        for (Localization localization : declared) {
            collectNestedLocalizations(localization, nested, identitySet());
        }

        List<Localization> roots = declared.stream()
                .filter(localization -> !nested.contains(localization))
                .toList();
        List<CameraDevice> cameras = new ArrayList<>();
        Set<CameraDevice> seenCameras = identitySet();
        Set<MeasurementSignal> seenSignals = identitySet();
        Map<CameraDevice, Mechanism> cameraOwners = new IdentityHashMap<>();
        for (Localization localization : roots) {
            int firstCamera = cameras.size();
            collectCameras(localization, seenSignals, seenCameras, cameras);
            Mechanism owner = localizationOwners.get(localization);
            if (owner != null) {
                for (int i = firstCamera; i < cameras.size(); i++) cameraOwners.put(cameras.get(i), owner);
            }
        }
        return new Result(roots, cameras, cameraOwners, autoChoosers, pathProviders);
    }

    private static void collectMechanism(
            Mechanism mechanism,
            Set<Mechanism> seenMechanisms,
            Set<Localization> seenLocalizations,
            List<Localization> localizations,
            Set<AutoChooser> seenAutoChoosers,
            List<AutoChooser> autoChoosers,
            Set<PathProvider> seenPathProviders,
            List<PathProvider> pathProviders,
            Map<Localization, Mechanism> localizationOwners) {
        if (!seenMechanisms.add(mechanism)) {
            return;
        }
        for (Field field : fields(mechanism.getClass())) {
            if (field.isSynthetic() || Modifier.isStatic(field.getModifiers())) {
                continue;
            }
            Object value = read(field, mechanism);
            if (value instanceof Mechanism child) {
                collectMechanism(
                        child,
                        seenMechanisms,
                        seenLocalizations,
                        localizations,
                        seenAutoChoosers,
                        autoChoosers,
                        seenPathProviders,
                        pathProviders,
                        localizationOwners);
            }
            if (value instanceof Localization localization && seenLocalizations.add(localization)) {
                localizations.add(localization);
                localizationOwners.put(localization, mechanism);
            }
            if (value instanceof AutoChooser chooser && seenAutoChoosers.add(chooser)) {
                autoChoosers.add(chooser);
            }
            if (value instanceof PathProvider provider && seenPathProviders.add(provider)) {
                pathProviders.add(provider);
            }
        }
    }

    private static void collectNestedLocalizations(
            Localization localization,
            Set<Localization> nested,
            Set<Localization> visited) {
        if (!visited.add(localization)) {
            return;
        }
        for (PoseSignal input : localization.inputs()) {
            if (input instanceof Localization child) {
                nested.add(child);
                collectNestedLocalizations(child, nested, visited);
            }
        }
    }

    private static void collectCameras(
            MeasurementSignal signal,
            Set<MeasurementSignal> seenSignals,
            Set<CameraDevice> seenCameras,
            List<CameraDevice> cameras) {
        if (!seenSignals.add(signal)) {
            return;
        }
        if (signal instanceof ca.frc6390.athena.vision.signal.PoseSignal cameraSignal
                && seenCameras.add(cameraSignal.camera())) {
            cameras.add(cameraSignal.camera());
        }
        if (signal instanceof Localization localization) {
            localization.inputs().forEach(input -> collectCameras(input, seenSignals, seenCameras, cameras));
        }
        signal.sources().forEach(source -> collectCameras(source, seenSignals, seenCameras, cameras));
    }

    private static List<Field> fields(Class<?> type) {
        Map<String, Field> fields = new LinkedHashMap<>();
        for (Class<?> current = type; current != null && current != Object.class; current = current.getSuperclass()) {
            for (Field field : current.getDeclaredFields()) {
                fields.putIfAbsent(field.getName(), field);
            }
        }
        return List.copyOf(fields.values());
    }

    private static Object read(Field field, Object instance) {
        try {
            if (!field.canAccess(instance)) {
                field.setAccessible(true);
            }
            return field.get(instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to inspect field " + field.getName(), exception);
        }
    }

    private static <T> Set<T> identitySet() {
        return Collections.newSetFromMap(new IdentityHashMap<>());
    }

    record Result(
            List<Localization> localizations,
            List<CameraDevice> cameras,
            Map<CameraDevice, Mechanism> cameraOwners,
            List<AutoChooser> autoChoosers,
            List<PathProvider> pathProviders) {
        Result {
            localizations = List.copyOf(localizations);
            cameras = List.copyOf(cameras);
            cameraOwners = Map.copyOf(cameraOwners);
            autoChoosers = List.copyOf(autoChoosers);
            pathProviders = List.copyOf(pathProviders);
        }
    }
}
