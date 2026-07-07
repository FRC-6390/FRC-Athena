package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;
import ca.frc6390.athena.hardware.ref.ImuRef;
import ca.frc6390.athena.mechanism.core.ControlRef;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Discovers swerve drivebase modules and geometry from refs and nested classes.
 */
public final class SwerveDrivebaseIntrospector {
    private SwerveDrivebaseIntrospector() {
    }

    public static SwerveDrivebaseDefinition inspect(SwerveDrivebase drivebase) {
        Objects.requireNonNull(drivebase, "drivebase");
        TrackWidth trackWidth = null;
        WheelBase wheelBase = null;
        ImuRef imu = null;
        List<DiscoveredModule> discovered = new ArrayList<>();

        for (Field field : fields(drivebase.getClass())) {
            if (field.isSynthetic()) {
                continue;
            }
            Object value = read(field, drivebase);
            if (value instanceof TrackWidth width) {
                trackWidth = width;
            } else if (value instanceof WheelBase base) {
                wheelBase = base;
            } else if (value instanceof ImuRef ref) {
                imu = ref;
            } else if (value instanceof SwerveModule module) {
                discovered.add(new DiscoveredModule(
                        field.getName(),
                        module,
                        order(field, module.getClass()),
                        discovered.size()));
            }
        }

        for (Class<?> nested : drivebase.getClass().getDeclaredClasses()) {
            if (SwerveModule.class.isAssignableFrom(nested) && !alreadyDiscovered(nested, discovered)) {
                SwerveModule module = constructModule(nested);
                discovered.add(new DiscoveredModule(defaultName(nested), module, order(nested), discovered.size()));
            }
        }

        validate(discovered);
        discovered.sort(Comparator
                .comparingInt(DiscoveredModule::sortOrder)
                .thenComparingInt(DiscoveredModule::declarationIndex));

        List<SwerveModuleDefinition> modules = new ArrayList<>();
        boolean canDeriveRectangularLocations = discovered.size() == 4 && trackWidth != null && wheelBase != null;
        for (int index = 0; index < discovered.size(); index++) {
            DiscoveredModule discoveredModule = discovered.get(index);
            ModuleLocationRef explicitLocation = moduleLocation(discoveredModule.module());
            if (explicitLocation != null) {
                modules.add(new SwerveModuleDefinition(
                        discoveredModule.name(),
                        discoveredModule.module(),
                        index,
                        explicitLocation,
                        true,
                        controls(discoveredModule.module())));
            } else if (canDeriveRectangularLocations) {
                modules.add(new SwerveModuleDefinition(
                        discoveredModule.name(),
                        discoveredModule.module(),
                        index,
                        derivedLocation(index, trackWidth, wheelBase),
                        false,
                        controls(discoveredModule.module())));
            } else {
                throw new IllegalStateException(
                        "Swerve module " + discoveredModule.name()
                                + " needs a ModuleLocationRef or a four-module drivebase with TrackWidth and WheelBase.");
            }
        }

        return new SwerveDrivebaseDefinition(drivebase, trackWidth, wheelBase, imu, modules);
    }

    private static Field[] fields(Class<?> type) {
        Map<String, Field> fields = new LinkedHashMap<>();
        Class<?> current = type;
        while (current != null && current != Object.class) {
            for (Field field : current.getDeclaredFields()) {
                fields.putIfAbsent(field.getName(), field);
            }
            current = current.getSuperclass();
        }
        return fields.values().toArray(Field[]::new);
    }

    private static Object read(Field field, Object instance) {
        try {
            if (!field.canAccess(Modifier.isStatic(field.getModifiers()) ? null : instance)) {
                field.setAccessible(true);
            }
            return field.get(Modifier.isStatic(field.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to inspect field " + field.getName(), exception);
        }
    }

    private static Integer order(Field field, Class<?> moduleType) {
        SwerveModuleOrder order = field.getAnnotation(SwerveModuleOrder.class);
        if (order != null) {
            return order.value();
        }
        return order(moduleType);
    }

    private static Integer order(Class<?> type) {
        SwerveModuleOrder order = type.getAnnotation(SwerveModuleOrder.class);
        return order == null ? null : order.value();
    }

    private static void validate(List<DiscoveredModule> discovered) {
        if (discovered.size() < 3) {
            throw new IllegalStateException("Swerve drivebase needs at least three modules.");
        }

        Map<Integer, String> orders = new HashMap<>();
        for (DiscoveredModule module : discovered) {
            if (module.order() == null) {
                continue;
            }
            if (module.order() < 0) {
                throw new IllegalStateException("Swerve module order must be zero or greater: " + module.name());
            }
            String previous = orders.putIfAbsent(module.order(), module.name());
            if (previous != null) {
                throw new IllegalStateException(
                        "Swerve module order " + module.order() + " is used by both " + previous + " and "
                                + module.name() + ".");
            }
        }
    }

    private static boolean alreadyDiscovered(Class<?> type, List<DiscoveredModule> discovered) {
        for (DiscoveredModule module : discovered) {
            if (module.module().getClass() == type) {
                return true;
            }
        }
        return false;
    }

    private static SwerveModule constructModule(Class<?> type) {
        try {
            Constructor<?> constructor = type.getDeclaredConstructor();
            if (!constructor.canAccess(null)) {
                constructor.setAccessible(true);
            }
            return (SwerveModule) constructor.newInstance();
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException(
                    "Unable to construct nested swerve module " + type.getName()
                            + ". Nested module classes need a no-argument constructor.",
                    exception);
        }
    }

    private static String defaultName(Class<?> type) {
        return type.getSimpleName();
    }

    private static ModuleLocationRef moduleLocation(SwerveModule module) {
        for (Field field : fields(module.getClass())) {
            if (field.isSynthetic()) {
                continue;
            }
            Object value = read(field, module);
            if (value instanceof ModuleLocationRef location) {
                return location;
            }
        }
        return null;
    }

    private static Map<String, ControlRef> controls(SwerveModule module) {
        Map<String, ControlRef> controls = new LinkedHashMap<>();
        for (Field field : fields(module.getClass())) {
            if (field.isSynthetic()) {
                continue;
            }
            Object value = read(field, module);
            if (value instanceof ControlRef control) {
                controls.put(field.getName(), control);
            }
        }
        return controls;
    }

    private static ModuleLocationRef derivedLocation(int index, TrackWidth trackWidth, WheelBase wheelBase) {
        double halfWheelBase = wheelBase.meters() / 2.0;
        double halfTrackWidth = trackWidth.meters() / 2.0;
        return switch (index) {
            case 0 -> ModuleLocationRef.meters(halfWheelBase, halfTrackWidth);
            case 1 -> ModuleLocationRef.meters(halfWheelBase, -halfTrackWidth);
            case 2 -> ModuleLocationRef.meters(-halfWheelBase, halfTrackWidth);
            case 3 -> ModuleLocationRef.meters(-halfWheelBase, -halfTrackWidth);
            default -> throw new IllegalArgumentException("Rectangular swerve location derivation requires four modules.");
        };
    }

    private record DiscoveredModule(String name, SwerveModule module, Integer order, int declarationIndex) {
        private int sortOrder() {
            return order == null ? Integer.MAX_VALUE : order;
        }
    }
}
