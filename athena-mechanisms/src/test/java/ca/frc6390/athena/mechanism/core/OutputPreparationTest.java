package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.effort.Effort;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class OutputPreparationTest {
    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        int commands;
        double volts;
        double speed;
        double position;
        int positionReads;
        int velocityReads;
        Motor(int id) { device = MotorDevice.of(MotorKinds.KRAKEN_X60, id); }
        Motor(int id, PowerProfile profile) { device = MotorDevice.of(MotorKinds.KRAKEN_X60, id).power(profile); }
        @Override public MotorDevice device() { return device; }
        @Override public double integratedPositionRotations() { positionReads++; return position; }
        @Override public double integratedVelocityRotationsPerSecond() { velocityReads++; return speed; }
        @Override public double appliedVoltage() { return volts; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public void setVoltage(double value) { commands++; volts = value; }
        @Override public void stop() { commands++; volts = 0.0; }
    }

    private static final class Loop implements ControlLoop {
        final double volts;
        int calculations;
        int notifications;
        double appliedVolts;
        Loop(double volts) { this.volts = volts; }
        @Override public ControlLoopRuntime bind(ControlLoopBinding binding) {
            return new ControlLoopRuntime() {
                @Override public ControlOutput calculate(ControlLoopContext context) {
                    calculations++;
                    return ControlOutput.voltage(volts);
                }
                @Override public void applied(ControlLoopContext context, Output requested, Output applied) {
                    notifications++;
                    appliedVolts = applied instanceof Output.Voltage voltage ? voltage.volts() : Double.NaN;
                }
            };
        }
    }

    @Test
    void unchangedControllerRequestsStillReevaluateLiveOutputConstraintsAndNotifyOnce() {
        Motor motor = new Motor(222);
        Loop loop = new Loop(16.0);
        double[] allowed = {20.0};
        int[] evaluations = {0};
        var liveClamp = new ca.frc6390.athena.mechanism.constraint.Constraint<Double>() {
            @Override
            public ca.frc6390.athena.mechanism.constraint.ConstraintResult<Double> evaluate(
                    ca.frc6390.athena.mechanism.constraint.ConstraintContext<Double> context) {
                evaluations[0]++;
                return new ca.frc6390.athena.mechanism.constraint.ConstraintResult.Corrected<>(
                        context.requested(), Math.min(context.requested(), allowed[0]));
            }

            @Override
            public ca.frc6390.athena.mechanism.constraint.ConstraintStage stage() {
                return ca.frc6390.athena.mechanism.constraint.ConstraintStage.OUTPUT;
            }
        };
        ControlBinding<?> control = Controls.velocity(motor.device, Units.Rotations).loop(loop).constraints(liveClamp);
        OutputApplier applier = OutputApplier.using(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ignored) { return motor; }
        });
        for (int cycle = 0; cycle < 3; cycle++) {
            applier.beginPreparation();
            apply(applier, control);
            applier.endCycle();
            applier.commitPrepared();
            assertEquals(12.0, motor.volts, "nominal voltage saturation still bounds an unchanged request");
        }
        allowed[0] = 4.0;
        applier.beginPreparation();
        apply(applier, control);
        applier.endCycle();
        applier.commitPrepared();

        assertEquals(4.0, motor.volts, "a live constraint change is observed without changing the controller request");
        assertEquals(4, evaluations[0]);
        assertEquals(4, loop.calculations);
        assertEquals(4, loop.notifications);
        assertEquals(4.0, loop.appliedVolts);
    }

    @Test
    void deferredRequestsAndRetainedLoopContextsKeepTheirOwnStagingAndFeedback() {
        Motor first = new Motor(220);
        Motor second = new Motor(221);
        first.position = 2.0;
        first.speed = 3.0;
        second.position = 4.0;
        second.speed = 5.0;
        var contexts = new java.util.ArrayList<ControlLoopContext>();
        ControlLoop loop = binding -> new ControlLoopRuntime() {
            @Override public ControlOutput calculate(ControlLoopContext context) {
                contexts.add(context);
                // PID, feedforward, and custom loops may inspect these values repeatedly.
                assertEquals(context.position(), context.position());
                assertEquals(context.velocity(), context.velocity());
                return ControlOutput.voltage(1.0);
            }
        };
        ControlBinding<?> firstControl = Controls.velocity(first.device, Units.Rotations).loop(loop);
        ControlBinding<?> secondControl = Controls.velocity(second.device, Units.Rotations).loop(loop);
        OutputApplier applier = OutputApplier.using(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice motor) { return motor == first.device ? first : second; }
        });
        applier.captureTrace(true);
        applier.beginPreparation();
        Output firstTarget = Outputs.velocity(11.0);
        Output secondTarget = Outputs.velocity(22.0);
        applier.apply(new ResolvedOutput(OutputRequest.of(firstControl, firstTarget), firstTarget));
        applier.apply(new ResolvedOutput(OutputRequest.of(secondControl, secondTarget), secondTarget));
        applier.endCycle();
        first.position = 100.0;
        first.speed = 200.0;
        applier.commitPrepared();

        assertEquals(11.0, applier.controlTraces().get(0).goal());
        assertEquals(22.0, applier.controlTraces().get(1).goal());
        assertEquals(2.0, contexts.get(0).position());
        assertEquals(3.0, contexts.get(0).velocity());
        assertEquals(4.0, contexts.get(1).position());
        assertEquals(5.0, contexts.get(1).velocity());
        assertEquals(1, first.positionReads);
        assertEquals(1, first.velocityReads);

        applier.beginPreparation();
        apply(applier, firstControl);
        applier.endCycle();
        applier.commitPrepared();
        assertEquals(2.0, contexts.get(0).position(), "a retained callback context is never overwritten by the next cycle");
        assertEquals(11.0, contexts.get(0).target());
        assertEquals(100.0, contexts.get(2).position());
    }

    @Test
    void preparationEvaluatesEachControllerOnceAndCommitsStableOutputs() {
        Motor first = new Motor(201);
        Motor second = new Motor(202);
        Loop firstLoop = new Loop(6.0);
        Loop secondLoop = new Loop(3.0);
        ControlBinding<?> firstControl = Controls.velocity(first.device, Units.Rotations).loop(firstLoop);
        ControlBinding<?> secondControl = Controls.velocity(second.device, Units.Rotations).loop(secondLoop);
        OutputApplier applier = OutputApplier.using(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice motor) {
                return motor == first.device ? first : second;
            }
        });
        applier.captureTrace(true);

        applier.beginPreparation();
        apply(applier, firstControl);
        apply(applier, secondControl);
        applier.endCycle();

        assertEquals(1, firstLoop.calculations);
        assertEquals(1, secondLoop.calculations);
        assertEquals(0, first.commands);
        assertEquals(0, second.commands);
        assertEquals(0, firstLoop.notifications);
        assertTrue(applier.controlTraces().isEmpty());

        applier.commitPrepared();

        assertEquals(1, firstLoop.calculations);
        assertEquals(1, secondLoop.calculations);
        assertEquals(1, first.commands);
        assertEquals(1, second.commands);
        assertEquals(6.0, first.volts);
        assertEquals(3.0, second.volts);
        assertEquals(1, firstLoop.notifications);
        assertEquals(6.0, firstLoop.appliedVolts);
        assertEquals(3.0, secondLoop.appliedVolts);
        assertEquals(2, applier.controlTraces().size());
    }

    @Test
    void releasedHoldIsPreparedBeforeCommitWithoutDrivingEarly() {
        Motor motor = new Motor(203);
        Loop loop = new Loop(4.0);
        ControlBinding<?> control = Controls.velocity(motor.device, Units.Rotations).loop(loop).release(Release.HOLD);
        OutputApplier applier = OutputApplier.using(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ignored) { return motor; }
        });
        applier.beginPreparation();
        apply(applier, control);
        applier.endCycle();
        applier.commitPrepared();

        applier.beginPreparation();
        applier.endCycle();

        assertEquals(2, loop.calculations, "release HOLD participates in the preparation pass");
        assertEquals(1, motor.commands, "held output waits for the global allocation");
        assertTrue(applier.heldMotors().contains(motor.device));
        applier.commitPrepared();
        assertEquals(2, motor.commands);
        assertEquals(2, loop.notifications);
    }

    @Test
    void allocationSeesTheUnclippedControllerCandidateAndNotifiesItsActualCurrentClamp() {
        Motor motor = new Motor(204);
        Loop loop = new Loop(12.0);
        ControlBinding<?> control = Controls.velocity(motor.device, Units.Rotations).loop(loop)
                .effort(Effort.max(Units.Amps.of(10.0)));
        ActionContext context = new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ignored) { return motor; }
        };
        PowerRuntime power = power();
        OutputApplier applier = OutputApplier.using(context);
        applier.electrical(power);
        power.beginCycle(List.of(motor.device), context, new RuntimeOverrides(), 0.0);
        applier.beginPreparation();
        apply(applier, control);
        applier.endCycle();
        power.planCommands();
        assertTrue(power.snapshot().motors().get(0).demandAmps() > 300.0,
                "allocation receives the full 12 V candidate rather than an already-clipped 10 A request");
        assertEquals(0, motor.commands);
        applier.commitPrepared();
        assertEquals(10.0, current(motor, motor.volts), 1e-7);
        assertEquals(motor.volts, loop.appliedVolts, 1e-12);
        assertEquals(1, loop.calculations);
        assertEquals(1, loop.notifications);
    }

    @Test
    void sharedProfilePreservesRequestedCurrentRatioAtDifferentBackEmfs() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(40.0));
        Motor first = new Motor(205, profile);
        Motor second = new Motor(206, profile);
        second.speed = 40.0;
        Loop firstLoop = new Loop(12.0);
        Loop secondLoop = new Loop(8.0);
        ControlBinding<?> firstControl = Controls.velocity(first.device, Units.Rotations).loop(firstLoop);
        ControlBinding<?> secondControl = Controls.velocity(second.device, Units.Rotations).loop(secondLoop);
        ActionContext context = new ActionContext() {
            @Override public MotorHandle motor(MotorDevice motor) { return motor == first.device ? first : second; }
        };
        PowerRuntime power = power();
        OutputApplier applier = OutputApplier.using(context);
        applier.electrical(power);
        power.beginCycle(List.of(first.device, second.device), context, new RuntimeOverrides(), 0.0);
        applier.beginPreparation();
        apply(applier, firstControl);
        apply(applier, secondControl);
        applier.endCycle();
        power.planCommands();
        applier.commitPrepared();

        double firstScale = current(first, first.volts) / current(first, firstLoop.volts);
        double secondScale = current(second, second.volts) / current(second, secondLoop.volts);
        assertEquals(firstScale, secondScale, 1e-8,
                "coupling scales winding current around each motor's own back-EMF");
        assertTrue(firstScale > 0.0 && firstScale < 1.0);
        assertEquals(40.0, current(first, first.volts), 1e-7);
        assertTrue(Math.abs(current(second, second.volts)) <= 40.0 + 1e-7);
        assertEquals(1, first.commands);
        assertEquals(1, second.commands);
    }

    @Test
    void deviceTargetWithoutHostVoltageLoopCannotBypassPowerLimits() {
        Motor motor = new Motor(207, PowerProfile.of().sustained(Units.Amps.of(20.0)));
        ActionContext context = new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ignored) { return motor; }
        };
        PowerRuntime power = power();
        OutputApplier applier = OutputApplier.using(context);
        applier.electrical(power);
        power.beginCycle(List.of(motor.device), context, new RuntimeOverrides(), 0.0);
        applier.beginPreparation();
        assertThrows(IllegalStateException.class, () -> apply(applier,
                Controls.velocity(motor.device, Units.Rotations)));
        applier.endCycle();
        applier.commitPrepared();
        assertEquals(0, motor.commands, "a rejected device target is never queued for commit");
    }

    private static PowerRuntime power() {
        PowerRuntime power = new PowerRuntime();
        power.busVoltage(() -> 12.0);
        power.policy(PowerPolicy.defaults().estimateBatteryResistance(false)
                .staggerStarts(Units.Seconds.of(0.0)));
        return power;
    }

    private static double current(Motor motor, double volts) {
        MotorConstants constants = MotorConstants.forKind(motor.device.kind()).orElseThrow();
        double emf = motor.speed * 2.0 * Math.PI / constants.raw().speedConstantRadPerSecPerVolt();
        return (volts - emf) / constants.resistance().in(Units.Ohms);
    }

    private static void apply(OutputApplier applier, ControlBinding<?> control) {
        Output output = Outputs.velocity(10.0);
        applier.apply(new ResolvedOutput(OutputRequest.of(control, output), output));
    }
}
