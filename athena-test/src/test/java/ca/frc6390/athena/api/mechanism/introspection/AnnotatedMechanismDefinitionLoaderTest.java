package ca.frc6390.athena.api.mechanism.introspection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateEnter;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateExit;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStatePeriodic;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.annotation.input.BooleanInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DigitalInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDigitalInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;
import ca.frc6390.athena.api.mechanism.input.MechanismBooleanInput;
import ca.frc6390.athena.api.mechanism.input.MechanismDoubleInput;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;

class AnnotatedMechanismDefinitionLoaderTest {
    @Test
    void loadsAnnotatedMembersIntoOneDefinition() {
        MechanismDefinition definition = AnnotatedMechanismDefinitionLoader.load(SampleTurret.class);

        assertEquals("Turret", definition.name());
        assertEquals(1, definition.motors().size());
        assertEquals(1, definition.encoders().stream().filter(e -> e.defaultPositionSource()).count());
        assertEquals(3, definition.inputs().size());
        assertEquals(2, definition.loops().size());
        assertEquals(3, definition.automation().size());
        assertEquals("Off", definition.initialStateName().orElseThrow());
        assertEquals(State.class, definition.stateType().orElseThrow());
        assertEquals(PositionDomainKind.ANGULAR, definition.identity().positionDomain().orElseThrow().kind());
        assertEquals(PositionUnit.DEGREES, definition.identity().positionDomain().orElseThrow().units());
        assertEquals(-134.0, definition.identity().travelRange().orElseThrow().min());
        assertEquals(220.0, definition.identity().travelRange().orElseThrow().max());
    }

    @Test
    void normalizesSignedDigitalPortsIntoInversion() {
        MechanismDefinition definition = AnnotatedMechanismDefinitionLoader.load(SampleTurret.class);

        MechanismDigitalInputDefinition home = definition.inputs().stream()
            .filter(MechanismDigitalInputDefinition.class::isInstance)
            .map(MechanismDigitalInputDefinition.class::cast)
            .findFirst()
            .orElseThrow();

        assertEquals("home", home.name());
        assertEquals(6, home.port());
        assertTrue(home.inverted());
    }

    @Test
    void infersFieldAndMethodLoopNames() {
        MechanismDefinition definition = AnnotatedMechanismDefinitionLoader.load(SampleTurret.class);

        assertTrue(definition.loops().stream().anyMatch(loop ->
            "mainPid".equals(loop.name()) && loop.declarationKind() == LoopDeclarationKind.FIELD));
        assertTrue(definition.loops().stream().anyMatch(loop ->
            "hubTargetingLoop".equals(loop.name()) && loop.declarationKind() == LoopDeclarationKind.METHOD));
        assertTrue(definition.loops().stream().anyMatch(loop ->
            "hubTargetingLoop".equals(loop.name())
                && loop.controller() instanceof MechanismCustomControllerDefinition custom
                && custom.callback().isPresent()));
        assertFalse(definition.loops().stream().anyMatch(loop -> loop.name().isBlank()));
        assertTrue(definition.loops().stream().anyMatch(loop ->
            loop.controller() instanceof MechanismPidControllerDefinition pid && pid.kP() == 0.12));
    }

    @SuppressWarnings("unused")
    @Mechanism("Turret")
    @InitialState("Off")
    @PositionDomain(value = PositionDomainKind.ANGULAR, units = PositionUnit.DEGREES)
    @TravelRange(min = -134.0, max = 220.0)
    private static final class SampleTurret implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X60, id = 13, bus = "can")
        static final Object turretMotor = new Object();

        @Encoder(type = AthenaEncoder.CANCODER, id = 40, bus = "can")
        static final Object absoluteEncoderA = new Object();

        @Encoder(type = AthenaEncoder.CANCODER, id = 41, bus = "can")
        static final Object absoluteEncoderB = new Object();

        @Encoder
        @DefaultPositionSource
        static final Object turretCrt = new Object();

        @DigitalInput(port = -6)
        static final Object home = new Object();

        @BooleanInput
        static final MechanismBooleanInput allowCounterRotation = MechanismBooleanInput.create()
            .defaultValue(true);

        @DoubleInput
        static final MechanismDoubleInput fieldHeadingDeg = MechanismDoubleInput.create()
            .defaultValue(180.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        static final MechanismPid mainPid = MechanismPid.create()
            .kp(0.12)
            .ki(0.0)
            .kd(0.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.ENABLED, states = {"Aim"})
        double hubTargetingLoop() {
            return 0.0;
        }

        @OnStateEnter({"Aim"})
        void aimEnter() {
        }

        @OnStatePeriodic({"Aim"})
        void aimPeriodic() {
        }

        @OnStateExit({"Aim"})
        void aimExit() {
        }

        @Override
        public State initialState() {
            return State.Off;
        }
    }

    private enum State {
        Off,
        Aim
    }
}
