package ca.frc6390.athena.api.mechanism.annotation.behavior.control;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import ca.frc6390.athena.mechanisms.OutputType;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface ControlLoop {
    String value() default "";

    OutputType output() default OutputType.PERCENT;
}
