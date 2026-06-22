package ca.frc6390.athena.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.commands.CommandRunner;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.mechanism.config.Mechanisms;
import ca.frc6390.athena.mechanism.runtime.MechanismController;
import ca.frc6390.athena.sim.hardware.SimMotorBackend;
import ca.frc6390.athena.superstructure.runtime.SuperstructureCommands;
import ca.frc6390.athena.superstructure.runtime.SuperstructureController;
import ca.frc6390.athena.superstructure.config.Superstructures;
import ca.frc6390.athena.superstructure.runtime.SuperstructurePlanner;
import ca.frc6390.athena.superstructure.spec.SuperstructurePartSpec;

class SuperstructureConfigTest {
    @Test
    void lowersPartsAndStates() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput());

        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .state("idle", state -> state.part("intake", "stopped"))
                .toSpec();

        assertEquals("robot", spec.name());
        assertEquals(1, spec.parts().size());
        assertEquals("idle", spec.states().get(0).name());
    }

    @Test
    void validatesChildMechanismsWithBackendContext() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput());
        var spec = Superstructures.create("robot").part("intake", intake).toSpec();

        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));

        assertFalse(spec.validate(context).hasErrors());
    }

    @Test
    void reportsUnknownPartTargets() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1));
        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .state("bad", state -> state.part("shooter", "ready"))
                .toSpec();

        var report = spec.validate(AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend())));

        assertTrue(report.hasErrors());
        assertEquals("superstructure.unknown-part", report.errors().get(0).code());
    }

    @Test
    void validatesNestedSuperstructureParts() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .state("stowed", state -> state.target(0.0))
                .state("feed", state -> state.target(0.6));
        var handoff = Superstructures.create("handoff")
                .part("intake", intake)
                .state("idle", state -> state.part("intake", "stowed"))
                .state("feeding", state -> state.part("intake", "feed"));
        var spec = Superstructures.create("robot")
                .part("handoff", handoff)
                .state("idle", state -> state.part("handoff", "idle"))
                .state("score", state -> state.part("handoff", "feeding"))
                .toSpec();

        var report = spec.validate(AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend())));

        assertFalse(report.hasErrors());
        assertEquals(SuperstructurePartSpec.Kind.SUPERSTRUCTURE, spec.parts().get(0).kind());
    }

    @Test
    void reportsUnknownChildStateTargets() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("stowed", state -> state.target(0.0));
        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .state("bad", state -> state.part("intake", "missing"))
                .toSpec();

        var report = spec.validate(AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend())));

        assertTrue(report.hasErrors());
        assertEquals("superstructure.unknown-target", report.errors().get(0).code());
    }

    @Test
    void reportsDuplicatePartsAndStates() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1));
        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .part("intake", intake)
                .state("idle", state -> state.part("intake", "idle"))
                .state("idle", state -> state.part("intake", "idle"))
                .toSpec();

        var report = spec.validate(AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend())));

        assertFalse(report.errorsWithCode("superstructure.duplicate-part").isEmpty());
        assertFalse(report.errorsWithCode("superstructure.duplicate-state").isEmpty());
    }

    @Test
    void plansNestedTransitionsToLeafMechanismTargets() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("stowed", state -> state.target(0.0))
                .state("feed", state -> state.target(0.6));
        var shooter = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .state("idle", state -> state.target(0.0))
                .state("speaker", state -> state.target(4600.0));
        var handoff = Superstructures.create("handoff")
                .part("intake", intake)
                .part("shooter", shooter)
                .state("idle", state -> state
                        .part("intake", "stowed")
                        .part("shooter", "idle"))
                .state("feeding", state -> state
                        .part("intake", "feed")
                        .part("shooter", "speaker"));
        var spec = Superstructures.create("robot")
                .part("handoff", handoff)
                .state("idle", state -> state.part("handoff", "idle"))
                .state("score", state -> state.part("handoff", "feeding"))
                .toSpec();

        var plan = new SuperstructurePlanner(spec).plan("idle", "score");

        assertEquals("robot", plan.superstructureName());
        assertEquals("idle", plan.currentState());
        assertEquals("score", plan.targetState());
        assertEquals("handoff.intake", plan.targets().get(0).path());
        assertEquals("feed", plan.targets().get(0).stateName());
        assertEquals(0.6, plan.targets().get(0).targetValue().orElseThrow(), 1.0e-9);
        assertEquals("handoff.shooter", plan.targets().get(1).path());
        assertEquals(4600.0, plan.targets().get(1).targetValue().orElseThrow(), 1.0e-9);
    }

    @Test
    void plansOnlyPartsNamedByTheTargetState() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("feed", state -> state.target(0.6));
        var shooter = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .state("speaker", state -> state.target(4600.0));
        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .part("shooter", shooter)
                .state("feedOnly", state -> state.part("intake", "feed"))
                .state("score", state -> state
                        .part("intake", "feed")
                        .part("shooter", "speaker"))
                .toSpec();

        var plan = new SuperstructurePlanner(spec).plan("feedOnly");

        assertEquals(1, plan.targets().size());
        assertEquals("intake", plan.targets().get(0).path());
    }

    @Test
    void rejectsUnknownAndGuardedTransitions() {
        var intake = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("stowed", state -> state.target(0.0))
                .state("feed", state -> state.target(0.6));
        var spec = Superstructures.create("robot")
                .part("intake", intake)
                .state("idle", state -> state.part("intake", "stowed"))
                .state("score", state -> state.part("intake", "feed"))
                .toSpec();
        var guarded = new SuperstructurePlanner(spec, (current, target) -> !"idle".equals(current));

        assertThrows(IllegalArgumentException.class, () -> new SuperstructurePlanner(spec).plan("missing"));
        assertThrows(IllegalStateException.class, () -> guarded.plan("idle", "score"));
    }

    @Test
    void controllerAppliesPlannedTargetsToMechanisms() {
        var intakeConfig = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .state("stowed", state -> state.target(0.0))
                .state("feed", state -> state.target(0.6));
        var shooterConfig = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .control(control -> control.velocity())
                .state("idle", state -> state.target(0.0))
                .state("speaker", state -> state.target(82.0));
        var handoff = Superstructures.create("handoff")
                .part("intake", intakeConfig)
                .part("shooter", shooterConfig)
                .state("idle", state -> state
                        .part("intake", "stowed")
                        .part("shooter", "idle"))
                .state("feeding", state -> state
                        .part("intake", "feed")
                        .part("shooter", "speaker"));
        var spec = Superstructures.create("robot")
                .part("handoff", handoff)
                .state("idle", state -> state.part("handoff", "idle"))
                .state("score", state -> state.part("handoff", "feeding"))
                .toSpec();
        var intake = intakeConfig.toSpec();
        var shooter = shooterConfig.toSpec();
        var intakeMotor = new RecordingMotor(intake.motors().get(0));
        var shooterMotor = new RecordingMotor(shooter.motors().get(0));

        var controller = SuperstructureController.builder(spec)
                .mechanism("handoff.intake", MechanismController.of(intake, java.util.List.of(intakeMotor)))
                .mechanism("handoff.shooter", MechanismController.of(shooter, java.util.List.of(shooterMotor)))
                .build();

        var plan = controller.applyState("score");

        assertEquals("score", controller.currentState());
        assertEquals(2, plan.targets().size());
        assertEquals(0.6, intakeMotor.percent, 1.0e-9);
        assertEquals(82.0, shooterMotor.velocity, 1.0e-9);
    }

    @Test
    void commandFactoryAppliesAndStopsSuperstructureControllers() {
        var intakeConfig = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .state("stowed", state -> state.target(0.0))
                .state("feed", state -> state.target(0.6));
        var spec = Superstructures.create("robot")
                .part("intake", intakeConfig)
                .state("idle", state -> state.part("intake", "stowed"))
                .state("score", state -> state.part("intake", "feed"))
                .toSpec();
        var intake = intakeConfig.toSpec();
        var intakeMotor = new RecordingMotor(intake.motors().get(0));
        var controller = SuperstructureController.builder(spec)
                .mechanism("intake", MechanismController.of(intake, java.util.List.of(intakeMotor)))
                .build();

        var apply = SuperstructureCommands.applyState(controller, "score", "superstructure");
        var stop = SuperstructureCommands.stop(controller, "superstructure");

        assertEquals("superstructure:score", apply.name());
        assertTrue(apply.requirements().contains("superstructure"));
        assertTrue(new CommandRunner(apply).step());
        assertEquals("score", controller.currentState());
        assertEquals(0.6, intakeMotor.percent, 1.0e-9);

        assertTrue(new CommandRunner(stop).step());
        assertEquals(0.0, intakeMotor.percent, 1.0e-9);
    }

    private static final class RecordingMotor implements MotorDevice {
        private final MotorSpec spec;
        private double percent;
        private double velocity;

        private RecordingMotor(MotorSpec spec) {
            this.spec = spec;
        }

        @Override
        public MotorSpec spec() {
            return spec;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }
    }
}
