package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.staging.Staging;
import edu.wpi.first.units.Units;
import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

/** One test per {@code State/Reason} cause that needs the scheduler (constraints, staging, power, faults). */
class MechanismReasonCausesTest {
    private static final MotorDevice BLOCKED_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 931);
    private static final MotorDevice GUARD_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 932);
    private static final EncoderDevice GUARD_ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 932);
    private static final MotorDevice STAGE_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 933);
    private static final MotorDevice POWER_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 934)
            .currentLimit(400)
            .power(PowerProfile.of().sustained(Units.Amps.of(60)));
    private static final MotorDevice THERMAL_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 935)
            .currentLimit(100)
            .thermal(Thermal.derate(Units.Celsius.of(40), Units.Celsius.of(60), 0.5));
    private static final MotorDevice FAULT_MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 936);

    enum Health { NORMAL, HOT }

    @Test
    void blockedNamesTheConstraintThatRejectedTheTarget() {
        Hardware hardware = new Hardware();
        BlockedRobot robot = new BlockedRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);

        scheduler.request(robot.push);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals("Blocked: require constraint rejected target", scheduler.reason(robot));
    }

    @Test
    void guardReportsTheRangeEdgeThatClippedTheOutput() {
        Hardware hardware = new Hardware();
        hardware.encoder(GUARD_ENCODER).position = 1.0;
        GuardRobot robot = new GuardRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);

        scheduler.request(robot.push);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals("Guard: at max edge", scheduler.reason(robot));
    }

    @Test
    void stageNamesTheActiveStageThatCapsTheOutput() {
        Hardware hardware = new Hardware();
        StageRobot robot = new StageRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);

        scheduler.request(robot.drive);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals("Running: drive", scheduler.reason(robot));

        robot.hot = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals("Stage HOT", scheduler.reason(robot));
    }

    @Test
    void powerCapReportsTheProfileCapInAmps() {
        Hardware hardware = new Hardware();
        Motor motor = hardware.motor(POWER_MOTOR);
        motor.appliedVolts = 12.0;
        motor.supplyAmps = 364.0;
        PowerRobot robot = new PowerRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);

        scheduler.request(robot.drive);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals("Power cap 60 A", scheduler.reason(robot));
    }

    @Test
    void thermalDerateReportsThePercentOfTheLimitThatRemains() throws Exception {
        Hardware hardware = new Hardware();
        Motor motor = hardware.motor(THERMAL_MOTOR);
        motor.temperature = OptionalDouble.of(100.0);
        // The lumped thermal model only exists while the motor is bound to a runtime.
        AutoCloseable binding = THERMAL_MOTOR.bindRuntime(new RuntimeScope("reason-thermal-test"), motor);
        try {
            ThermalRobot robot = new ThermalRobot();
            MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);

            scheduler.request(robot.drive);
            double now = 0.0;
            // The model moves toward the vendor reading with a 2 s time constant; 30 s is well past the knee.
            for (int cycle = 0; cycle < 1500; cycle++) {
                scheduler.teleopPeriodic(now, 0.02);
                now += 0.02;
            }

            assertEquals("Thermal derate 50%", scheduler.reason(robot));
        } finally {
            binding.close();
        }
    }

    @Test
    void faultReasonIsReportedWhileTheFaultIsContained() {
        Hardware hardware = new Hardware();
        FaultRobot robot = new FaultRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).isolateFaults(true).register(robot);

        scheduler.request(robot.drive);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertTrue(scheduler.reason(robot).startsWith("Fault: "), scheduler.reason(robot));
        assertEquals(scheduler.fault(robot), scheduler.reason(robot).substring("Fault: ".length()));
    }

    private static final class BlockedRobot implements Mechanism {
        private final ControlBinding<?> control = Controls.velocity(BLOCKED_MOTOR)
                .constraint(Constraints.require(() -> false));
        public final Action push = control.percent(0.8);
    }

    private static final class GuardRobot implements Mechanism {
        private final ControlBinding<?> control = Controls.position(GUARD_MOTOR)
                .feedback(GUARD_ENCODER)
                .constraint(Constraints.range(Range.of(0.0, 1.0)));
        public final Action push = control.percent(0.2);
    }

    private static final class StageRobot implements Mechanism {
        boolean hot;
        private final ControlBinding<?> control = Controls.velocity(STAGE_MOTOR);
        @SuppressWarnings("unused")
        private final Staging<Health> health = Staging.of(Health.NORMAL)
                .stage(Health.HOT, () -> hot, control.limitEffort(0.5))
                .exitAfter(Units.Seconds.of(0.0));
        public final Action drive = control.percent(0.6);
    }

    private static final class PowerRobot implements Mechanism {
        private final ControlBinding<?> control = Controls.velocity(POWER_MOTOR);
        public final Action drive = control.percent(0.6);
    }

    private static final class ThermalRobot implements Mechanism {
        private final ControlBinding<?> control = Controls.velocity(THERMAL_MOTOR);
        public final Action drive = control.percent(0.6);
    }

    private static final class FaultRobot implements Mechanism {
        private final ControlBinding<?> control = Controls.velocity(FAULT_MOTOR)
                .constraint(Constraints.require(context -> {
                    throw new IllegalStateException("constraint crashed");
                }));
        public final Action drive = control.percent(0.6);
    }

    private static final class Hardware implements ActionContext {
        final Map<MotorDevice, Motor> motors = new HashMap<>();
        final Map<EncoderDevice, Encoder> encoders = new HashMap<>();

        @Override
        public Motor motor(MotorDevice device) {
            return motors.computeIfAbsent(device, Motor::new);
        }

        @Override
        public Encoder encoder(EncoderDevice device) {
            return encoders.computeIfAbsent(device, Encoder::new);
        }
    }

    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        double appliedVolts;
        double supplyAmps;
        double statorAmps;
        OptionalDouble temperature = OptionalDouble.empty();

        Motor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public boolean supportsRuntimeConfiguration() {
            return true;
        }

        @Override
        public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        }

        @Override
        public double appliedVoltage() {
            return appliedVolts;
        }

        @Override
        public double supplyCurrentAmps() {
            return supplyAmps;
        }

        @Override
        public double statorCurrentAmps() {
            return statorAmps;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return 0.0;
        }

        @Override
        public OptionalDouble temperatureCelsius() {
            return temperature;
        }
    }

    private static final class Encoder implements EncoderHandle {
        final EncoderDevice device;
        double position;

        Encoder(EncoderDevice device) {
            this.device = device;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return 0.0;
        }
    }
}
