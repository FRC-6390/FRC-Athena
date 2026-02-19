package ca.frc6390.athena.networktables;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Invokes a zero-arg method when a boolean topic is triggered.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AthenaNTAction {
    String key();
    String scope() default "Actions";
    boolean risingEdgeOnly() default true;
}
