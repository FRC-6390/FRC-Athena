package ca.frc6390.athena.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface Telemetry {
    String key() default "";
    TelemetryDestination destination() default TelemetryDestination.BOTH;
    int periodMs() default 100;
    double epsilon() default 0.0;
}
