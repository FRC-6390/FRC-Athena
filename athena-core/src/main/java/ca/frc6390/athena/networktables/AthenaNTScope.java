package ca.frc6390.athena.networktables;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Optional class-level scope prefix applied by {@link AthenaNT#bind(String, Object)} and
 * {@link NtScope#bind(Object)}.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface AthenaNTScope {
    String value();
}
