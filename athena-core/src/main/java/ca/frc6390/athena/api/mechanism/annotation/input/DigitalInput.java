package ca.frc6390.athena.api.mechanism.annotation.input;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface DigitalInput {
    String value() default "";

    int port();

    double position() default Double.NaN;

    boolean hardstop() default false;

    int blockDirection() default 0;

    double delaySeconds() default 0.0;
}
