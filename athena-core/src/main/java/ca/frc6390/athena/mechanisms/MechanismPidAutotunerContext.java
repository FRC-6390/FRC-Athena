package ca.frc6390.athena.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;

public interface MechanismPidAutotunerContext {
    Mechanism mechanism();

    String pidName();

    MechanismConfig.PidProfile pidProfile();

    String dashboardPath();

    OutputType outputType();

    double measurement();

    double setpoint();

    void output(double output);

    void stopOutput();

    default Command relayPosition() {
        return MechanismPidAutotuners.relayPosition(this);
    }

    default Command relayPosition(MechanismPidAutotuners.RelayPositionConfig config) {
        return MechanismPidAutotuners.relayPosition(this, config);
    }
}
