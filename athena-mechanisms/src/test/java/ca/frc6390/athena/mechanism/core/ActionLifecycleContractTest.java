package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.IdentityHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Executable contract for request, arbitration, completion, and mode transitions. */
class ActionLifecycleContractTest {
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 71);
    private static final MotorDevice ROLLERS = MotorDevice.of(MotorKinds.KRAKEN_X44, 72);

    @Test
    void requestRunsUntilCancellationThenNeutralizesExactlyOnce() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.holdArm);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertTrue(scheduler.isRunning(mechanism.holdArm));
        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(2, hardware.motor(ARM).percentCalls);

        scheduler.cancel(mechanism.holdArm);
        scheduler.teleopPeriodic(0.04, 0.02);
        scheduler.teleopPeriodic(0.06, 0.02);

        assertFalse(scheduler.isRunning(mechanism.holdArm));
        assertFalse(scheduler.isComplete(mechanism.holdArm));
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(1, hardware.motor(ARM).stopCalls);
    }

    @Test
    void finiteCompletionReleasesNeutralizesAndCanBeRequestedAgain() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.timedArm);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.60, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.teleopPeriodic(0.02, 0.02);
        assertFalse(scheduler.isRunning(mechanism.timedArm));
        assertTrue(scheduler.isComplete(mechanism.timedArm));
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(1, hardware.motor(ARM).stopCalls);

        scheduler.request(mechanism.timedArm);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertTrue(scheduler.isRunning(mechanism.timedArm));
        assertFalse(scheduler.isComplete(mechanism.timedArm));
        assertEquals(0.60, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void newestConflictWinsAndOlderLeaseResumesAfterRelease() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.holdArm);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(mechanism.overrideArm);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertTrue(scheduler.isRunning(mechanism.holdArm));
        assertTrue(scheduler.isRunning(mechanism.overrideArm));
        assertEquals(-0.80, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.cancel(mechanism.overrideArm);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertTrue(scheduler.isRunning(mechanism.holdArm));
        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0, hardware.motor(ARM).stopCalls);
    }

    @Test
    void dynamicBranchSwitchesChildrenWithoutASecondRequest() {
        boolean[] armSelected = {true};
        RecordingContext hardware = new RecordingContext();
        BranchMechanism mechanism = new BranchMechanism(armSelected);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.selectedOutput);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.40, hardware.motor(ARM).percent, 1.0e-9);

        armSelected[0] = false;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(1, hardware.motor(ARM).stopCalls);
        assertEquals(0.70, hardware.motor(ROLLERS).percent, 1.0e-9);

        armSelected[0] = true;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.40, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.0, hardware.motor(ROLLERS).percent, 1.0e-9);
        assertEquals(1, hardware.motor(ROLLERS).stopCalls);
    }

    @Test
    void disableSuspendsOutputWithoutCompletingAndEnableResumes() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.holdArm);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.disabledPeriodic(0.02, 0.02);

        assertTrue(scheduler.isRunning(mechanism.holdArm));
        assertFalse(scheduler.isComplete(mechanism.holdArm));
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void teleopHookReleasesOutsideTeleopAndReacquiresOnReturn() {
        RecordingContext hardware = new RecordingContext();
        TeleopMechanism mechanism = new TeleopMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(0.55, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.autoPeriodic(0.02, 0.02);
        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);

        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.55, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void directRequestsAreNotTeleopGated() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.holdArm);
        scheduler.autoPeriodic(0.0, 0.02);

        assertEquals(0.25, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void actionCreatedAfterRegistrationInfersOwnerFromItsTarget() {
        RecordingContext hardware = new RecordingContext();
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);
        Action methodCreatedAction = mechanism.arm.percent(0.90);

        scheduler.request(methodCreatedAction);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.90, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void actionWithoutAnOwnedDeclarationFailsAtRequest() {
        MotorDevice unknown = MotorDevice.of(MotorKinds.KRAKEN_X60, 99);
        ContractMechanism mechanism = new ContractMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        IllegalArgumentException error = assertThrows(
                IllegalArgumentException.class,
                () -> scheduler.request(unknown.percent(0.5)));

        assertTrue(error.getMessage().contains("not owned by a registered mechanism"));
    }

    private static final class ContractMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final Action holdArm = arm.percent(0.25);
        private final Action overrideArm = arm.percent(-0.80);
        private final Action timedArm = arm.percent(0.60).timeout(0.02);
    }

    private static final class BranchMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final MotorDevice rollers = ROLLERS;
        private final Action selectedOutput;

        private BranchMechanism(boolean[] armSelected) {
            selectedOutput = Actions.when(() -> armSelected[0])
                    .run(arm.percent(0.40))
                    .otherwise(rollers.percent(0.70));
        }
    }

    private static final class TeleopMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final Action drive = arm.percent(0.55);
        private final HookBinding driverControl = Events.teleopPeriodic().whileActive(drive);
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new IdentityHashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double percent = Double.NaN;
        private int percentCalls;
        private int stopCalls;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
            percentCalls++;
        }

        @Override
        public void stop() {
            percent = 0.0;
            stopCalls++;
        }
    }
}
