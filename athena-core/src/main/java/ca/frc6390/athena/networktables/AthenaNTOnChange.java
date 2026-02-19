package ca.frc6390.athena.networktables;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Called when a matching {@link AthenaNTTunable} value changes.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AthenaNTOnChange {
    String key();
    String scope() default "Tuning";
}
