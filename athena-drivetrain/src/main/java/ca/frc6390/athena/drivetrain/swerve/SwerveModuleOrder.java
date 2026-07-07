package ca.frc6390.athena.drivetrain.swerve;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Optional explicit ordering for an introspected swerve module.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.TYPE, ElementType.FIELD})
public @interface SwerveModuleOrder {
    /**
     * Zero-based module order used by drivetrain kinematics.
     *
     * @return module order
     */
    int value();
}
