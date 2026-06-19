package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.mechanisms.MechanismSetpointSource;

public record MechanismFeedforwardControllerDefinition(
    MechanismFeedforwardModel model,
    double kS,
    double kG,
    double kV,
    double kA,
    OptionalDouble tolerance,
    MechanismSetpointSource setpointSource
) implements MechanismLoopControllerDefinition {
    public MechanismFeedforwardControllerDefinition {
        model = Objects.requireNonNull(model, "model");
        tolerance = Objects.requireNonNull(tolerance, "tolerance");
        setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
    }
}
