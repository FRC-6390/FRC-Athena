package ca.frc6390.athena.api.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateEnter;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.annotation.input.BooleanInput;
import ca.frc6390.athena.api.mechanism.behavior.MechanismBehavior;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.identity.IdentityConfig;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.input.MechanismBooleanInput;
import ca.frc6390.athena.api.mechanism.input.MechanismInputs;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotors;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.statespec.StateId;
import ca.frc6390.athena.mechanisms.statespec.StateSet;

class MechanismDefinitionsIntegrationTest {
    @Test
    void flowConfigBuildsDefinitionAndDirectRuntime() {
        var definition = Mechanisms.stateful("Turret", State.Off)
            .identity(identity -> identity
                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
                .travelRange(-134.0, 220.0))
            .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 13))
            .encoders(encoders -> encoders.add(
                ca.frc6390.athena.api.mechanism.encoder.MechanismEncoder.create("turretCrt")
                    .type(AthenaEncoder.CANCODER)
                    .id(40)
                    .defaultPositionSource()))
            .inputs(inputs -> inputs.booleanInput("allowCounterRotation", true))
            .behavior(behavior -> behavior.control(control -> control
                .pid("mainPid", pid -> pid
                    .output(OutputType.VOLTAGE)
                    .manual()
                    .kp(0.12)
                    .ki(0.0)
                    .kd(0.0))))
            .definition();

        assertEquals("Turret", definition.name());
        assertEquals("Off", definition.initialStateName().orElseThrow());
        assertEquals(1, definition.motors().size());
        assertEquals(1, definition.encoders().size());
        assertEquals(1, definition.loops().size());

        var runtimeDefinition = Mechanisms.stateful("TurretRuntime", State.Off)
            .behavior(behavior -> behavior.control(control -> control
                .pid("mainPid", pid -> pid
                    .manual()
                    .output(OutputType.VOLTAGE)
                    .kp(0.12)
                    .ki(0.0)
                    .kd(0.0))))
            .definition();

        assertNotNull(MechanismDefinitions.build(runtimeDefinition));
    }

    @Test
    void structuredConfigMergesAnnotationsAndSections() {
        var definition = new TurretStructured() {
        }.definition();

        assertEquals("StructuredTurret", definition.name());
        assertEquals("Off", definition.initialStateName().orElseThrow());
        assertEquals(State.class, definition.stateType().orElseThrow());
        assertTrue(definition.encoders().stream().anyMatch(encoder -> encoder.defaultPositionSource()));
        assertTrue(definition.inputs().stream().anyMatch(input -> "allowCounterRotation".equals(input.name())));
        assertEquals(1, definition.automation().size());
    }

    @Test
    void structuredConfigCanBeBuiltFromDeclarationClass() {
        var definition = MechanismDefinitions.structured(TurretStructured.class);

        assertEquals("StructuredTurret", definition.name());
        assertEquals("Off", definition.initialStateName().orElseThrow());
        assertEquals(State.class, definition.stateType().orElseThrow());
        assertTrue(definition.encoders().stream().anyMatch(encoder -> encoder.defaultPositionSource()));
    }

    @Test
    void staticConfigureSpecBuildsDefinitionFromPlainDeclarationClass() {
        var definition = MechanismDefinitions.structured(PlainSpecTurret.class);

        assertEquals("PlainSpecTurret", definition.name());
        assertEquals("Stow", definition.initialStateName().orElseThrow());
        assertEquals(StateId.class, definition.stateType().orElseThrow());
        assertEquals(1, definition.motors().size());
        assertEquals(1, definition.loops().size());
    }

    @Test
    void customLoopLowersWithExecutableCallbackAndManualSchedule() {
        MechanismPid mainPid = MechanismPid.create("mainPid")
            .manual()
            .output(OutputType.VOLTAGE)
            .kp(0.12)
            .ki(0.0)
            .kd(0.0);

        var definition = Mechanisms.stateful("TurretCustom", State.Off)
            .inputs(inputs -> inputs.doubleInput("fieldHeadingDeg", 180.0))
            .behavior(behavior -> behavior.control(control -> control
                .add(mainPid)
                .customLoop("hubTargeting", loop -> loop
                    .manual()
                    .output(OutputType.VOLTAGE)
                    .custom(ctx -> ctx.calculate(mainPid, ctx.doubleInput("fieldHeadingDeg"), ctx.setpoint())))))
            .definition();

        var mechanism = MechanismDefinitions.build(definition);
        mechanism.runInitHooks();
        assertFalse(mechanism.loops().enabled("hubTargeting"));
    }

    @Test
    void buildUsesDefinitionAsSourceKey() {
        var definition = Mechanisms.stateful("BuiltTurret", State.Off)
            .behavior(behavior -> behavior.control(control -> control
                .pid("mainPid", pid -> pid.manual().kp(0.1).ki(0.0).kd(0.0))))
            .definition();

        var mechanism = MechanismDefinitions.build(definition);

        assertNotNull(mechanism);
        assertTrue(mechanism.getSourceKey() == definition);
    }

    @Test
    void annotationDeclarationLoadsStateTypeAndExecutableCallbacks() {
        var definition = MechanismDefinitions.annotation(AnnotatedTurret.class);

        assertEquals(State.class, definition.stateType().orElseThrow());
        assertEquals("Off", definition.initialStateName().orElseThrow());
        assertEquals(1, definition.automation().size());
        @SuppressWarnings("unchecked")
        var mechanism = (ca.frc6390.athena.mechanisms.StatefulMechanism<State>) MechanismDefinitions.build(definition);
        mechanism.runInitHooks();
        assertTrue(mechanism.loops().names().contains("hubTargetingLoop"));
        mechanism.stateMachine().force(State.Aim);
        mechanism.periodic();
        assertFalse(mechanism.loops().enabled("hubTargetingLoop"));
    }

    @Test
    void stateAutomationExecutesInDirectRuntime() {
        var definition = Mechanisms.stateful("TurretAutomation", State.Off)
            .behavior(behavior -> behavior
                .control(control -> control
                    .pid("mainPid", pid -> pid.manual().kp(0.12).ki(0.0).kd(0.0))
                    .pid("counterRotation", pid -> pid.manual().kp(0.05).ki(0.0).kd(0.0)))
                .automation(automation -> automation
                    .onStateEnter(ctx -> ctx.enableControlLoop("mainPid"), State.Aim)
                    .onStatePeriodic(ctx -> ctx.disableControlLoop("counterRotation"), State.Aim)
                    .onStateExit(ctx -> ctx.disableControlLoop("mainPid"), State.Aim)))
            .definition();

        assertEquals(3, definition.automation().size());
        @SuppressWarnings("unchecked")
        var mechanism = (ca.frc6390.athena.mechanisms.StatefulMechanism<State>) MechanismDefinitions.build(definition);
        mechanism.runInitHooks();
        assertFalse(mechanism.loops().enabled("mainPid"));
        assertFalse(mechanism.loops().enabled("counterRotation"));
        mechanism.stateMachine().queue(State.Aim);
        for (int i = 0; i < 4; i++) {
            mechanism.periodic();
        }
        assertTrue(mechanism.loops().enabled("mainPid"));
        assertFalse(mechanism.loops().enabled("counterRotation"));
        mechanism.stateMachine().queue(State.Off);
        for (int i = 0; i < 4; i++) {
            mechanism.periodic();
        }
        assertFalse(mechanism.loops().enabled("mainPid"));
    }

    @Test
    void stateSetMechanismBuildsAndRunsWithoutEnumState() {
        var definition = Mechanisms.stateful("StateSetArm", ArmStates.Stow)
            .behavior(behavior -> behavior
                .control(control -> control
                    .pid("hold", pid -> pid.manual().kp(0.1).ki(0.0).kd(0.0)))
                .automation(automation -> automation
                    .onStateEnter(ctx -> ctx.enableControlLoop("hold"), ArmStates.Out)))
            .definition();

        assertEquals("Stow", definition.initialStateName().orElseThrow());
        assertEquals(StateId.class, definition.stateType().orElseThrow());
        assertTrue(definition.initialState().orElseThrow() == ArmStates.Stow);

        @SuppressWarnings("unchecked")
        var mechanism = (ca.frc6390.athena.mechanisms.StatefulMechanism<StateId>) MechanismDefinitions.build(definition);
        mechanism.runInitHooks();
        assertEquals(ArmStates.Stow, mechanism.stateMachine().goal());
        assertEquals(0.0, mechanism.stateMachine().setpoint(), 1e-9);
        mechanism.stateMachine().queue(ArmStates.Out);
        for (int i = 0; i < 4; i++) {
            mechanism.periodic();
        }
        assertEquals(ArmStates.Out, mechanism.stateMachine().goal());
        assertTrue(mechanism.loops().enabled("hold"));
    }

    enum State implements ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider<Double> {
        Off(0.0),
        Aim(1.0);

        private final double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    static final class ArmStates extends StateSet {
        static final StateId Stow = state("Stow", 0.0);
        static final StateId Out = state("Out", 1.0);
    }

    @Mechanism("StructuredTurret")
    @PositionDomain(value = PositionDomainKind.ANGULAR, units = PositionUnit.DEGREES)
    @TravelRange(min = -134.0, max = 220.0)
    interface TurretStructured extends TypedStatefulMechanismConfig<State> {
        @BooleanInput
        MechanismBooleanInput allowCounterRotation = MechanismBooleanInput.create()
            .defaultValue(true);

        @Encoder(type = AthenaEncoder.CANCODER, id = 40, bus = "can")
        @DefaultPositionSource
        Object turretCrt = new Object();

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        MechanismPid mainPid = MechanismPid.create()
            .kp(0.12)
            .ki(0.0)
            .kd(0.0);

        @OnStateEnter({"Aim"})
        default void aimEnter(ca.frc6390.athena.mechanisms.MechanismContext<?, ?> context) {
            context.enableControlLoop("secondaryPid");
        }

        @Override
        default State initialState() {
            return State.Off;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.travelRange(-140.0, 225.0);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X60, 13);
        }

        @Override
        default void encoders(MechanismEncoders encoders) {
            encoders.add("absoluteB", AthenaEncoder.CANCODER, 41);
        }

        @Override
        default void inputs(MechanismInputs inputs) {
            inputs.doubleInput("fieldHeadingDeg", 180.0);
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior.control(control -> control.pid("secondaryPid", pid -> pid
                .manual()
                .kp(0.2)
                .ki(0.0)
                .kd(0.0)));
        }
    }

    @Mechanism("AnnotatedTurret")
    @ca.frc6390.athena.api.mechanism.annotation.identity.InitialState("Off")
    static final class AnnotatedTurret implements TypedStatefulMechanismConfig<State> {
        @Override
        public State initialState() {
            return State.Off;
        }

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.ENABLED, states = {"Aim"})
        double hubTargetingLoop(ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext context) {
            return context.setpoint();
        }

        @OnStateEnter({"Aim"})
        void onAimEnter(ca.frc6390.athena.mechanisms.MechanismContext<?, ?> context) {
            context.disableControlLoop("hubTargetingLoop");
        }
    }

    interface PlainSpecTurret {
        static void configure(MechanismSpec mech) {
            identity(mech);
            hardware(mech);
            controls(mech);
        }

        private static void identity(MechanismSpec mech) {
            mech.name("PlainSpecTurret");
            mech.initialState(ArmStates.Stow);
            mech.identity()
                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
                .travelRange(-90.0, 90.0);
        }

        private static void hardware(MechanismSpec mech) {
            mech.motors().add(AthenaMotor.KRAKEN_X60, 13);
        }

        private static void controls(MechanismSpec mech) {
            mech.behavior().control(control -> control
                .pid("hold", pid -> pid.manual().kp(0.1).ki(0.0).kd(0.0)));
        }
    }
}
