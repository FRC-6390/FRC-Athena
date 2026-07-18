package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class CompiledActionGraphTest {
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 91);
    private static final MotorDevice ROLLERS = MotorDevice.of(MotorKinds.KRAKEN_X44, 92);

    @Test
    void computedSupplierRunsOnlyDuringPeriodicEvaluation() {
        AtomicInteger evaluations = new AtomicInteger();
        ExplicitShapeMechanism mechanism = new ExplicitShapeMechanism(evaluations);
        RecordingContext hardware = new RecordingContext();

        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);
        assertEquals(0, evaluations.get());

        scheduler.request(mechanism.dynamic);
        assertEquals(0, evaluations.get());

        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(2, evaluations.get());
        assertEquals(0.65, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void explicitlyOwnedComputedActionDoesNotRunDuringCompilation() {
        AtomicInteger evaluations = new AtomicInteger();
        ShapeLessMechanism mechanism = new ShapeLessMechanism(evaluations);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        scheduler.request(mechanism.dynamic);
        assertEquals(0, evaluations.get());

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(1, evaluations.get());
    }

    @Test
    void compiledShapeReservesFutureDynamicResources() {
        boolean[] armSelected = {true};
        ReservedShapeMechanism mechanism = new ReservedShapeMechanism(armSelected);
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(mechanism.olderRollers);
        scheduler.request(mechanism.dynamic);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.80, hardware.motor(ARM).percent, 1.0e-9);
        assertTrue(Double.isNaN(hardware.motor(ROLLERS).percent));

        armSelected[0] = false;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(0.0, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.90, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void dynamicChildOutsideCompiledShapeFailsImmediately() {
        InvalidShapeMechanism mechanism = new InvalidShapeMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);
        scheduler.request(mechanism.dynamic);

        IllegalStateException error = assertThrows(
                IllegalStateException.class,
                () -> scheduler.teleopPeriodic(0.0, 0.02));

        assertTrue(error.getMessage().contains("outside its compiled resource graph"));
        assertTrue(error.getMessage().contains("computed action ownership"));
    }

    @Test
    void methodCreatedComputedActionInfersExplicitOwnership() {
        ShapeOwnerMechanism mechanism = new ShapeOwnerMechanism();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);
        Action dynamic = Actions.compute(() -> mechanism.arm.percent(0.75), mechanism.arm);

        scheduler.request(dynamic);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.75, hardware.motor(ARM).percent, 1.0e-9);
    }

    @Test
    void newerPathDoesNotReserveUnrelatedAimResource() {
        PathAndAimMechanism mechanism = new PathAndAimMechanism();
        RecordingContext hardware = new RecordingContext();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism)
                .path(mechanism.path, new PathRuntime() {
                    @Override public Action output(PathAction path, MechanismContext context) {
                        return mechanism.drive.percent(0.60);
                    }

                    @Override public List<?> ownership(PathAction path) {
                        return List.of(mechanism.drive);
                    }

                    @Override public boolean isFinished(PathAction path, MechanismContext context) {
                        return false;
                    }
                });

        scheduler.request(mechanism.aim);
        scheduler.request(mechanism.path);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(0.60, hardware.motor(ARM).percent, 1.0e-9);
        assertEquals(0.35, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    private static final class ExplicitShapeMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final Action dynamic;

        private ExplicitShapeMechanism(AtomicInteger evaluations) {
            dynamic = Actions.compute(() -> {
                evaluations.incrementAndGet();
                return arm.percent(0.65);
            }, arm);
        }
    }

    private static final class ShapeLessMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final Action dynamic;

        private ShapeLessMechanism(AtomicInteger evaluations) {
            dynamic = Actions.compute(() -> {
                evaluations.incrementAndGet();
                return arm.percent(0.50);
            }, arm);
        }
    }

    private static final class ReservedShapeMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final MotorDevice rollers = ROLLERS;
        private final Action olderRollers = rollers.percent(0.30);
        private final Action dynamic;

        private ReservedShapeMechanism(boolean[] armSelected) {
            dynamic = Actions.compute(
                    () -> armSelected[0] ? arm.percent(0.80) : rollers.percent(0.90),
                    arm,
                    rollers);
        }
    }

    private static final class InvalidShapeMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
        private final MotorDevice rollers = ROLLERS;
        private final Action dynamic = Actions.compute(() -> rollers.percent(0.90), arm);
    }

    private static final class ShapeOwnerMechanism implements Mechanism {
        private final MotorDevice arm = ARM;
    }

    private static final class PathAndAimMechanism implements Mechanism {
        private final MotorDevice drive = ARM;
        private final MotorDevice aimMotor = ROLLERS;
        private final Action aim = aimMotor.percent(0.35);
        private final PathAction path = Paths.of("test", "drive");
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
        }

        @Override
        public void stop() {
            percent = 0.0;
        }
    }
}
