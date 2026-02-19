package ca.frc6390.athena.networktables;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Declares a tunable field that can be adjusted from NetworkTables.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface AthenaNTTunable {
    String key();
    String scope() default "Tuning";
    double min() default -Double.MAX_VALUE;
    double max() default Double.MAX_VALUE;
    double step() default 0.0;
}
