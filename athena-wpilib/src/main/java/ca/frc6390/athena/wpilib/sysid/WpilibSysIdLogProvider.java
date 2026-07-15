package ca.frc6390.athena.wpilib.sysid;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdLogProvider;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

/** Writes Athena control characterization samples in WPILib SysId format. */
public final class WpilibSysIdLogProvider implements SysIdLogProvider {
    @Override
    public SysIdLog open(String routineName, String motorName) {
        DataLogManager.start();
        SysIdRoutineLog log = new SysIdRoutineLog(routineName);
        SysIdRoutineLog.MotorLog motor = log.motor(motorName);
        return new WpilibLog(log, motor);
    }

    private record WpilibLog(SysIdRoutineLog log, SysIdRoutineLog.MotorLog motor) implements SysIdLog {
        @Override
        public void record(SysIdSample sample) {
            motor.voltage(Volts.of(sample.appliedVoltage()));
            if (sample.angular()) {
                motor.angularPosition(Rotations.of(sample.position()))
                        .angularVelocity(RotationsPerSecond.of(sample.velocity()));
            } else {
                motor.linearPosition(Meters.of(sample.position()))
                        .linearVelocity(MetersPerSecond.of(sample.velocity()));
            }
            if (Double.isFinite(sample.currentAmps())) {
                motor.current(Amps.of(sample.currentAmps()));
            }
            log.recordState(state(sample.state()));
        }

        @Override
        public void end() {
            log.recordState(SysIdRoutineLog.State.kNone);
        }

        private static SysIdRoutineLog.State state(SysIdState state) {
            return switch (state) {
                case QUASISTATIC_FORWARD -> SysIdRoutineLog.State.kQuasistaticForward;
                case QUASISTATIC_REVERSE -> SysIdRoutineLog.State.kQuasistaticReverse;
                case DYNAMIC_FORWARD -> SysIdRoutineLog.State.kDynamicForward;
                case DYNAMIC_REVERSE -> SysIdRoutineLog.State.kDynamicReverse;
                case NONE -> SysIdRoutineLog.State.kNone;
            };
        }
    }
}
