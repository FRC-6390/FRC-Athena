package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotDrivetrainSysIdExamplesTest {

    @AfterEach
    void cleanupScheduler() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void bindHelpersTriggerExpectedCommands() {
        StubSysIdSection section = new StubSysIdSection();
        AtomicBoolean qForward = new AtomicBoolean(false);
        AtomicBoolean qReverse = new AtomicBoolean(false);
        AtomicBoolean dForward = new AtomicBoolean(false);
        AtomicBoolean dReverse = new AtomicBoolean(false);

        section.bindQuasistaticForward(new Trigger(qForward::get));
        section.bindQuasistaticReverse(new Trigger(qReverse::get));
        section.bindDynamicForward(new Trigger(dForward::get));
        section.bindDynamicReverse(new Trigger(dReverse::get));

        pressOnce(qForward);
        pressOnce(qReverse);
        pressOnce(dForward);
        pressOnce(dReverse);

        assertEquals(1, section.qForwardCount);
        assertEquals(1, section.qReverseCount);
        assertEquals(1, section.dForwardCount);
        assertEquals(1, section.dReverseCount);
    }

    @Test
    void voltageLimitRoundTripMatchesExampleUsage() {
        StubSysIdSection section = new StubSysIdSection();
        section.voltageLimit(3.0);
        assertEquals(3.0, section.voltageLimit(), 1e-9);
    }

    private static void pressOnce(AtomicBoolean signal) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.run();
        signal.set(true);
        scheduler.run();
        signal.set(false);
        scheduler.run();
    }

    private static final class StubSysIdSection implements RobotDrivetrain.SysIdSection {
        private double rampRate = 1.0;
        private double stepVoltage = 1.0;
        private double timeoutSeconds = 1.0;
        private double voltageLimit = Double.NaN;
        private int qForwardCount = 0;
        private int qReverseCount = 0;
        private int dForwardCount = 0;
        private int dReverseCount = 0;

        @Override
        public double rampRateVoltsPerSecond() {
            return rampRate;
        }

        @Override
        public void rampRateVoltsPerSecond(double voltsPerSecond) {
            rampRate = voltsPerSecond;
        }

        @Override
        public double stepVoltage() {
            return stepVoltage;
        }

        @Override
        public void stepVoltage(double volts) {
            stepVoltage = volts;
        }

        @Override
        public double timeoutSeconds() {
            return timeoutSeconds;
        }

        @Override
        public void timeoutSeconds(double seconds) {
            timeoutSeconds = seconds;
        }

        @Override
        public double voltageLimit() {
            return voltageLimit;
        }

        @Override
        public void voltageLimit(double volts) {
            voltageLimit = volts;
        }

        @Override
        public boolean active() {
            return false;
        }

        @Override
        public Command quasistatic(SysIdRoutine.Direction direction) {
            return Commands.runOnce(() -> {
                if (direction == SysIdRoutine.Direction.kForward) {
                    qForwardCount++;
                } else {
                    qReverseCount++;
                }
            }).ignoringDisable(true);
        }

        @Override
        public Command dynamic(SysIdRoutine.Direction direction) {
            return Commands.runOnce(() -> {
                if (direction == SysIdRoutine.Direction.kForward) {
                    dForwardCount++;
                } else {
                    dReverseCount++;
                }
            }).ignoringDisable(true);
        }
    }
}
