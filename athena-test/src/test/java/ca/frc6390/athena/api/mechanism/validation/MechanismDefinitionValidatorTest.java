package ca.frc6390.athena.api.mechanism.validation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardModel;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;
import ca.frc6390.athena.api.mechanism.identity.MechanismIdentity;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainSpec;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.identity.TravelRange;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.mechanisms.OutputType;

class MechanismDefinitionValidatorTest {
    private enum State {
        Aim
    }

    @Test
    void rejectsConflictingIdentityAndManualSchedulingRules() {
        MechanismDefinition definition = new MechanismDefinition(
            "Turret",
            false,
            Optional.of(State.class),
            Optional.of(State.Aim.name()),
            0.0,
            new MechanismIdentity(
                Optional.of(new PositionDomainSpec(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)),
                Optional.of(new TravelRange(-120.0, 120.0)),
                true,
                false),
            List.of(),
            List.of(
                new MechanismEncoderDefinition(
                    "encA",
                    AthenaEncoder.CANCODER,
                    OptionalInt.of(40),
                    Optional.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    Optional.empty(),
                    OptionalDouble.empty(),
                    true,
                    Object.class),
                new MechanismEncoderDefinition(
                    "encB",
                    AthenaEncoder.CANCODER,
                    OptionalInt.of(41),
                    Optional.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    Optional.empty(),
                    OptionalDouble.empty(),
                    true,
                    Object.class)),
            List.of(),
            List.of(
                new MechanismLoopDefinition(
                    "mainPid",
                    OutputType.VOLTAGE,
                    new LoopActivation(LoopMode.MANUAL, List.of("Aim")),
                    LoopDeclarationKind.FIELD,
                    new MechanismPidControllerDefinition(
                        0.12,
                        0.0,
                        0.0,
                        OptionalDouble.empty(),
                        OptionalDouble.empty(),
                        OptionalDouble.empty(),
                        null,
                        null),
                    Object.class),
                new MechanismLoopDefinition(
                    "mainPid",
                    OutputType.VOLTAGE,
                    LoopActivation.enabled(),
                    LoopDeclarationKind.FIELD,
                    MechanismCustomControllerDefinition.EMPTY,
                    Object.class),
                new MechanismLoopDefinition(
                    "counterRotation",
                    OutputType.VOLTAGE,
                    LoopActivation.enabled(),
                    LoopDeclarationKind.FIELD,
                    new MechanismFeedforwardControllerDefinition(
                        MechanismFeedforwardModel.SIMPLE,
                        0.1,
                        0.0,
                        0.2,
                        0.3,
                        OptionalDouble.of(-1.0),
                        null),
                    Object.class)),
            List.of());

        List<MechanismValidationIssue> issues = MechanismDefinitionValidator.validate(definition);

        assertEquals(5, issues.size());
    }
}
