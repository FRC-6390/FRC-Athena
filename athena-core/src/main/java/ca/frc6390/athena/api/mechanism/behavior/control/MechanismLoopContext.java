package ca.frc6390.athena.api.mechanism.behavior.control;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.mechanisms.Mechanism;

public interface MechanismLoopContext {
    RobotCore<?> robotCore();

    Mechanism mechanism();

    double controlLoopDtSeconds();

    double setpoint();

    Object state();

    boolean input(String key);

    double doubleInput(String key);

    int intVal(String key);

    String stringVal(String key);

    <V> V objectInput(String key, Class<V> type);

    double calculate(MechanismPid pid, double measurement, double setpoint);

    double calculate(MechanismFeedforward feedforward, double velocity);

    double calculate(MechanismFeedforward feedforward, double measurement, double setpoint, double velocity);

    double calculate(MechanismFeedforward feedforward, double currentVelocity, double nextVelocity);

    double calculate(
        MechanismFeedforward feedforward,
        double measurement,
        double setpoint,
        double currentVelocity,
        double nextVelocity);
}
