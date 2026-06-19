package ca.frc6390.athena.api.mechanism;

import ca.frc6390.athena.api.mechanism.behavior.MechanismBehavior;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.identity.IdentityConfig;
import ca.frc6390.athena.api.mechanism.input.MechanismInputs;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotors;

public interface MechanismConfig {
    default String name() {
        return "";
    }

    default boolean disabled() {
        return false;
    }

    default void identity(IdentityConfig identity) {
    }

    default void motors(MechanismMotors motors) {
    }

    default void encoders(MechanismEncoders encoders) {
    }

    default void inputs(MechanismInputs inputs) {
    }

    default void behavior(MechanismBehavior behavior) {
    }

    default MechanismDefinition definition() {
        return MechanismDefinitions.structured(this);
    }
}
