package ca.frc6390.athena.util;

import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import java.util.function.IntPredicate;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * General-purpose helpers for working with suppliers and their optional values across Athena.
 */
public final class SupplierUtil {

    private SupplierUtil() {}

    public static BooleanSupplier wrapBoolean(BooleanSupplier supplier, boolean fallback) {
        return supplier != null ? supplier : () -> fallback;
    }

    public static DoubleSupplier wrapDouble(DoubleSupplier supplier, double fallback) {
        return supplier != null ? supplier : () -> fallback;
    }

    public static IntSupplier wrapInt(IntSupplier supplier, int fallback) {
        return supplier != null ? supplier : () -> fallback;
    }

    public static <T> Supplier<T> wrapSupplier(Supplier<T> supplier, Supplier<T> fallback) {
        return supplier != null ? supplier : fallback;
    }

    public static <T> Consumer<T> wrapConsumer(Consumer<T> consumer, Consumer<T> fallback) {
        return consumer != null ? consumer : fallback;
    }

    public static OptionalDouble optionalDouble(DoubleSupplier supplier) {
        return optionalDouble(supplier, Double::isNaN);
    }

    public static OptionalDouble optionalDouble(DoubleSupplier supplier, DoublePredicate invalidPredicate) {
        double value = supplier.getAsDouble();
        return invalidPredicate != null && invalidPredicate.test(value)
                ? OptionalDouble.empty()
                : OptionalDouble.of(value);
    }

    public static OptionalInt optionalInt(IntSupplier supplier) {
        return optionalInt(supplier, val -> val < 0);
    }

    public static OptionalInt optionalInt(IntSupplier supplier, IntPredicate invalidPredicate) {
        int value = supplier.getAsInt();
        return invalidPredicate != null && invalidPredicate.test(value)
                ? OptionalInt.empty()
                : OptionalInt.of(value);
    }
}
