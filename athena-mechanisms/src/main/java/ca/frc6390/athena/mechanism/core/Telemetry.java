package ca.frc6390.athena.mechanism.core;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import ca.frc6390.athena.mechanism.persist.Locks;
import java.lang.annotation.Target;

/**
 * Publishes a mechanism field or zero-argument method as telemetry.
 *
 * <p>Methods and fields are read-only unless a mutable field explicitly sets
 * {@link #writable()} to {@code true}. Existing {@link TelemetryValue} declarations remain the
 * escape hatch for custom readers and writers.</p>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface Telemetry {
    /** Relative topic path. The field or method name is used when empty. */
    String value() default "";

    /** Allows NetworkTables to update the annotated field. Methods cannot be writable. */
    boolean writable() default false;

    /** Minimum accepted value for writable numeric fields. */
    double min() default -Double.MAX_VALUE;

    /** Maximum accepted value for writable numeric fields. */
    double max() default Double.MAX_VALUE;

    /**
     * Stores the value in the persistent store under its telemetry path. A stored value overrides the
     * code default when the mechanism is registered, and later writes (dashboard or code) are saved.
     * Only writable numeric, boolean, and string fields can persist.
     */
    boolean persist() default false;

    /**
     * Whether the value is sent to NetworkTables. {@code false} keeps the value local (and persisted when
     * {@link #persist()} is set).
     */
    boolean publish() default true;

    /**
     * Lock group that gates dashboard writes to a writable field. Defaults to {@link Locks.InMatch}.
     * Ignored for read-only fields.
     */
    Class<?> lock() default Locks.InMatch.class;
}
