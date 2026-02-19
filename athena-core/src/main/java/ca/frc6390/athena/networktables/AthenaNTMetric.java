package ca.frc6390.athena.networktables;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Publishes a field/method value to NetworkTables.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface AthenaNTMetric {
    String key();
    String scope() default "";
    int periodMs() default 100;
    double epsilon() default 0.0;
}
