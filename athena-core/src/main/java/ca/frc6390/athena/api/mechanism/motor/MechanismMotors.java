package ca.frc6390.athena.api.mechanism.motor;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.api.mechanism.definition.MechanismMotorDefinition;
import ca.frc6390.athena.hardware.motor.AthenaMotor;

public final class MechanismMotors {
    private final List<MechanismMotor> motors = new ArrayList<>();

    private MechanismMotors() {
    }

    public static MechanismMotors create() {
        return new MechanismMotors();
    }

    public static MechanismMotors from(List<MechanismMotorDefinition> definitions) {
        MechanismMotors motors = create();
        definitions.forEach(definition -> motors.add(MechanismMotor.from(definition)));
        return motors;
    }

    public MechanismMotors add(AthenaMotor type, int id) {
        return add(MechanismMotor.create().type(type).id(id));
    }

    public MechanismMotors add(String name, AthenaMotor type, int id) {
        return add(MechanismMotor.create(name).type(type).id(id));
    }

    public MechanismMotors add(MechanismMotor motor) {
        motors.add(motor);
        return this;
    }

    public MechanismMotors merge(MechanismMotors other) {
        if (other != null) {
            other.motors.forEach(this::add);
        }
        return this;
    }

    public List<MechanismMotorDefinition> definitions() {
        return motors.stream().map(MechanismMotor::definition).toList();
    }
}
