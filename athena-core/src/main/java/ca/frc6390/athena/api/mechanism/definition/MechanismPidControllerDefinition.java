package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;

public record MechanismPidControllerDefinition(
    double kP,
    double kI,
    double kD,
    OptionalDouble tolerance,
    OptionalDouble maxVelocity,
    OptionalDouble maxAcceleration,
    MechanismInputSource inputSource,
    MechanismSetpointSource setpointSource
) implements MechanismLoopControllerDefinition {
    public MechanismPidControllerDefinition {
        tolerance = Objects.requireNonNull(tolerance, "tolerance");
        maxVelocity = Objects.requireNonNull(maxVelocity, "maxVelocity");
        maxAcceleration = Objects.requireNonNull(maxAcceleration, "maxAcceleration");
        inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
        setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
    }
}
