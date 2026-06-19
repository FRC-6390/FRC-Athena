package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;

public record MechanismBangBangControllerDefinition(
    double highOutput,
    double lowOutput,
    OptionalDouble tolerance,
    MechanismInputSource inputSource,
    MechanismSetpointSource setpointSource
) implements MechanismLoopControllerDefinition {
    public MechanismBangBangControllerDefinition {
        tolerance = Objects.requireNonNull(tolerance, "tolerance");
        inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
        setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
    }
}
