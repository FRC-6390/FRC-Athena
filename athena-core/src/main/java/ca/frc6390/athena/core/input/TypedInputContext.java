package ca.frc6390.athena.core.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Shared typed input access contract for runtime contexts.
 */
public interface TypedInputContext {
    boolean input(String key);

    default BooleanSupplier inputSupplier(String key) {
        return () -> input(key);
    }

    default boolean boolVal(String key) {
        return input(key);
    }

    default BooleanSupplier boolValSupplier(String key) {
        return inputSupplier(key);
    }

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    default double doubleVal(String key) {
        return doubleInput(key);
    }

    default DoubleSupplier doubleValSupplier(String key) {
        return doubleInputSupplier(key);
    }

    int intVal(String key);

    IntSupplier intValSupplier(String key);

    String stringVal(String key);

    Supplier<String> stringValSupplier(String key);

    Pose2d pose2dVal(String key);

    Supplier<Pose2d> pose2dValSupplier(String key);

    Pose3d pose3dVal(String key);

    Supplier<Pose3d> pose3dValSupplier(String key);

    <V> V objectInput(String key, Class<V> type);

    <V> Supplier<V> objectInputSupplier(String key, Class<V> type);
}
