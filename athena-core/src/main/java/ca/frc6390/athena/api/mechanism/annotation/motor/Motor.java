package ca.frc6390.athena.api.mechanism.annotation.motor;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface Motor {
    String value() default "";

    AthenaMotor type() default AthenaMotor.KRAKEN_X60;

    int id() default Integer.MIN_VALUE;

    String bus() default "";

    double currentLimit() default Double.NaN;

    MotorNeutralMode neutralMode() default MotorNeutralMode.Brake;
}
