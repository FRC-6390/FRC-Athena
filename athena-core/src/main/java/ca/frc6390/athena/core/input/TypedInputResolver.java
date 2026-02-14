package ca.frc6390.athena.core.input;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Shared typed input lookup helper used by context implementations.
 */
public final class TypedInputResolver {

    public enum ValueMode {
        STRICT,
        LENIENT
    }

    public interface MutableInputs {
        default boolean hasBool(String key) {
            return false;
        }

        default boolean bool(String key) {
            return false;
        }

        default boolean hasDouble(String key) {
            return false;
        }

        default double dbl(String key) {
            return Double.NaN;
        }

        default boolean hasInt(String key) {
            return false;
        }

        default int intVal(String key) {
            return 0;
        }

        default boolean hasString(String key) {
            return false;
        }

        default String str(String key) {
            return "";
        }

        default boolean hasPose2d(String key) {
            return false;
        }

        default Pose2d pose2d(String key) {
            return null;
        }

        default boolean hasPose3d(String key) {
            return false;
        }

        default Pose3d pose3d(String key) {
            return null;
        }
    }

    public static final MutableInputs NO_MUTABLES = new MutableInputs() {};

    private final String ownerLabel;
    private final ValueMode valueMode;
    private final MutableInputs mutableInputs;
    private final Map<String, BooleanSupplier> boolInputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;

    public TypedInputResolver(
            String ownerLabel,
            ValueMode valueMode,
            MutableInputs mutableInputs,
            Map<String, BooleanSupplier> boolInputs,
            Map<String, DoubleSupplier> doubleInputs,
            Map<String, IntSupplier> intInputs,
            Map<String, Supplier<String>> stringInputs,
            Map<String, Supplier<Pose2d>> pose2dInputs,
            Map<String, Supplier<Pose3d>> pose3dInputs,
            Map<String, Supplier<?>> objectInputs) {
        this.ownerLabel = ownerLabel != null ? ownerLabel : "context";
        this.valueMode = valueMode != null ? valueMode : ValueMode.STRICT;
        this.mutableInputs = mutableInputs != null ? mutableInputs : NO_MUTABLES;
        this.boolInputs = boolInputs != null ? boolInputs : Map.of();
        this.doubleInputs = doubleInputs != null ? doubleInputs : Map.of();
        this.intInputs = intInputs != null ? intInputs : Map.of();
        this.stringInputs = stringInputs != null ? stringInputs : Map.of();
        this.pose2dInputs = pose2dInputs != null ? pose2dInputs : Map.of();
        this.pose3dInputs = pose3dInputs != null ? pose3dInputs : Map.of();
        this.objectInputs = objectInputs != null ? objectInputs : Map.of();
    }

    public boolean boolVal(String key) {
        if (mutableInputs.hasBool(key)) {
            return mutableInputs.bool(key);
        }
        BooleanSupplier supplier = boolInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return false;
            }
            throw missing("bool", key);
        }
        return supplier.getAsBoolean();
    }

    public BooleanSupplier boolSupplier(String key) {
        if (mutableInputs.hasBool(key)) {
            return () -> mutableInputs.bool(key);
        }
        BooleanSupplier supplier = boolInputs.get(key);
        if (supplier == null) {
            throw missing("bool", key);
        }
        return supplier;
    }

    public double doubleVal(String key) {
        if (mutableInputs.hasDouble(key)) {
            return mutableInputs.dbl(key);
        }
        DoubleSupplier supplier = doubleInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return Double.NaN;
            }
            throw missing("double", key);
        }
        return supplier.getAsDouble();
    }

    public DoubleSupplier doubleSupplier(String key) {
        if (mutableInputs.hasDouble(key)) {
            return () -> mutableInputs.dbl(key);
        }
        DoubleSupplier supplier = doubleInputs.get(key);
        if (supplier == null) {
            throw missing("double", key);
        }
        return supplier;
    }

    public int intVal(String key) {
        if (mutableInputs.hasInt(key)) {
            return mutableInputs.intVal(key);
        }
        IntSupplier supplier = intInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return 0;
            }
            throw missing("int", key);
        }
        return supplier.getAsInt();
    }

    public IntSupplier intSupplier(String key) {
        if (mutableInputs.hasInt(key)) {
            return () -> mutableInputs.intVal(key);
        }
        IntSupplier supplier = intInputs.get(key);
        if (supplier == null) {
            throw missing("int", key);
        }
        return supplier;
    }

    public String stringVal(String key) {
        if (mutableInputs.hasString(key)) {
            return mutableInputs.str(key);
        }
        Supplier<String> supplier = stringInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return "";
            }
            throw missing("string", key);
        }
        return supplier.get();
    }

    public Supplier<String> stringSupplier(String key) {
        if (mutableInputs.hasString(key)) {
            return () -> mutableInputs.str(key);
        }
        Supplier<String> supplier = stringInputs.get(key);
        if (supplier == null) {
            throw missing("string", key);
        }
        return supplier;
    }

    public Pose2d pose2dVal(String key) {
        if (mutableInputs.hasPose2d(key)) {
            return mutableInputs.pose2d(key);
        }
        Supplier<Pose2d> supplier = pose2dInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return null;
            }
            throw missing("Pose2d", key);
        }
        return supplier.get();
    }

    public Supplier<Pose2d> pose2dSupplier(String key) {
        if (mutableInputs.hasPose2d(key)) {
            return () -> mutableInputs.pose2d(key);
        }
        Supplier<Pose2d> supplier = pose2dInputs.get(key);
        if (supplier == null) {
            throw missing("Pose2d", key);
        }
        return supplier;
    }

    public Pose3d pose3dVal(String key) {
        if (mutableInputs.hasPose3d(key)) {
            return mutableInputs.pose3d(key);
        }
        Supplier<Pose3d> supplier = pose3dInputs.get(key);
        if (supplier == null) {
            if (valueMode == ValueMode.LENIENT) {
                return null;
            }
            throw missing("Pose3d", key);
        }
        return supplier.get();
    }

    public Supplier<Pose3d> pose3dSupplier(String key) {
        if (mutableInputs.hasPose3d(key)) {
            return () -> mutableInputs.pose3d(key);
        }
        Supplier<Pose3d> supplier = pose3dInputs.get(key);
        if (supplier == null) {
            throw missing("Pose3d", key);
        }
        return supplier;
    }

    public <V> V objectVal(String key, Class<V> type) {
        Supplier<?> supplier = objectInputs.get(key);
        if (supplier == null) {
            return null;
        }
        Object value = supplier.get();
        if (value == null) {
            return null;
        }
        if (!type.isInstance(value)) {
            throw new IllegalArgumentException(
                    "Input '" + key + "' is not of type " + type.getSimpleName());
        }
        return type.cast(value);
    }

    public <V> Supplier<V> objectSupplier(String key, Class<V> type) {
        Supplier<?> supplier = objectInputs.get(key);
        if (supplier == null) {
            throw missing("object", key);
        }
        return () -> objectVal(key, type);
    }

    private IllegalArgumentException missing(String type, String key) {
        return new IllegalArgumentException("No " + type + " input found for key " + key + " in " + ownerLabel);
    }
}
