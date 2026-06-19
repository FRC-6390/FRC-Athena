package ca.frc6390.athena.api.mechanism.annotation.encoder;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface Encoder {
    String value() default "";

    AthenaEncoder type() default AthenaEncoder.INTERNAL;

    int id() default Integer.MIN_VALUE;

    String bus() default "";

    double gearRatio() default Double.NaN;

    double conversion() default Double.NaN;

    double offset() default Double.NaN;

    double conversionOffset() default Double.NaN;

    MechanismEncoderUnit unit() default MechanismEncoderUnit.ROTATIONS;

    double wrapsEvery() default Double.NaN;
}
