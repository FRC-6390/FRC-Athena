// package ca.frc6390.athena.mechanisms;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertFalse;
// import static org.junit.jupiter.api.Assertions.assertTrue;

// import ca.frc6390.athena.hardware.encoder.Encoder;
// import ca.frc6390.athena.hardware.motor.MotorController;
// import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
// import ca.frc6390.athena.hardware.motor.MotorControllerType;
// import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
// import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.RobotBase;
// import org.junit.jupiter.api.Assumptions;
// import org.junit.jupiter.api.Test;

// final class MechanismSimulationOutputTest {

//     @Test
//     void manualOutputsAreVisibleToSimulation() {
//         Assumptions.assumeTrue(RobotBase.isSimulation());

//         MotorControllerGroup motors = new MotorControllerGroup(new FakeMotor());
//         Mechanism mechanism = new Mechanism(
//                 motors,
//                 null,
//                 null,
//                 null,
//                 false,
//                 false,
//                 new GenericLimitSwitch[0],
//                 false,
//                 0.02,
//                 null,
//                 null,
//                 null,
//                 false,
//                 null);

//         mechanism.setSpeed(0.4);
//         assertEquals(0.4, mechanism.getOutput(), 1e-6);
//         assertFalse(mechanism.isOutputVoltage());

//         mechanism.setVoltage(6.5);
//         assertEquals(6.5, mechanism.getOutput(), 1e-6);
//         assertTrue(mechanism.isOutputVoltage());
//     }

//     private static final class FakeMotor implements MotorController {
//         private final MotorControllerType type = () -> "test:fake";

//         @Override
//         public int getId() {
//             return 1;
//         }

//         @Override
//         public String getCanbus() {
//             return "sim";
//         }

//         @Override
//         public MotorControllerType getType() {
//             return type;
//         }

//         @Override
//         public void setSpeed(double percent) {
//         }

//         @Override
//         public void setVoltage(double volts) {
//         }

//         @Override
//         public void setCurrentLimit(double amps) {
//         }

//         @Override
//         public void setPosition(double rotations) {
//         }

//         @Override
//         public void setNeutralMode(MotorNeutralMode mode) {
//         }

//         @Override
//         public void setPid(PIDController pid) {
//         }

//         @Override
//         public boolean isConnected() {
//             return true;
//         }

//         @Override
//         public double getTemperatureCelsius() {
//             return 0.0;
//         }

//         @Override
//         public Encoder getEncoder() {
//             return null;
//         }
//     }
// }
