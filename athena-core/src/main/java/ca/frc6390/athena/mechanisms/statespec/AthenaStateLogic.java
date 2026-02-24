package ca.frc6390.athena.mechanisms.statespec;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Attaches logic to a state constant.
 *
 * <p>When the Athena state DSL compiler plugin is enabled, enum identifiers are accepted
 * directly (for example: {@code @AthenaStateLogic(UNJAM)}).</p>
 */
@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.METHOD)
public @interface AthenaStateLogic {
    String value();
}
