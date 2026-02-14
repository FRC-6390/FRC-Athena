package ca.frc6390.athena.hardware.motor;

import edu.wpi.first.math.controller.PIDController;

final class MotorSectionSupport {
    private MotorSectionSupport() {}

    static abstract class OutputSectionBase<S> {
        protected abstract S self();

        protected abstract void applySpeed(double speed);

        protected abstract void applyVoltage(double voltage);

        protected abstract void applyPosition(double position);

        protected abstract void applyVelocity(double velocity);

        protected abstract void applyStop();

        public S speed(double speed) {
            applySpeed(speed);
            return self();
        }

        public S voltage(double voltage) {
            applyVoltage(voltage);
            return self();
        }

        public S position(double position) {
            applyPosition(position);
            return self();
        }

        public S velocity(double rotationsPerSecond) {
            applyVelocity(rotationsPerSecond);
            return self();
        }

        public S stop() {
            applyStop();
            return self();
        }
    }

    static abstract class ConfigSectionBase<S> {
        protected abstract S self();

        protected abstract void applyNeutralMode(MotorNeutralMode mode);

        protected abstract void applyCurrentLimit(double amps);

        protected abstract void applyPid(PIDController pid);

        public S neutralMode(MotorNeutralMode mode) {
            applyNeutralMode(mode);
            return self();
        }

        public S currentLimit(double amps) {
            applyCurrentLimit(amps);
            return self();
        }

        public S pid(PIDController pid) {
            applyPid(pid);
            return self();
        }

        public S pid(double p, double i, double d) {
            applyPid(new PIDController(p, i, d));
            return self();
        }
    }
}
