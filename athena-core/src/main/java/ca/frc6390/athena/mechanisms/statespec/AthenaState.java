package ca.frc6390.athena.mechanisms.statespec;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a state enum for Athena plugin-backed enum DSL transformation.
 *
 * <p>Optional value mode:
 * {@code @AthenaState(SomeType.class)} enables generated
 * {@code SetpointProvider<SomeType>} plumbing from enum constant args.</p>
 */
@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.TYPE)
public @interface AthenaState {
    Class<?> value() default Void.class;
}
