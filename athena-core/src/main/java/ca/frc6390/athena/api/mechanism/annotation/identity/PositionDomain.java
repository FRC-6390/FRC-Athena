package ca.frc6390.athena.api.mechanism.annotation.identity;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface PositionDomain {
    PositionDomainKind value();

    PositionUnit units() default PositionUnit.NONE;
}
