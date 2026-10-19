package frc.robot.examples;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;

/** Explicit simulation-session harness for tests and non-WPILib hosts. */
public final class SimulationHarness {
    private final MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 30);
    private final SimulationSession session = SimulationSession.create()
            .model("testMotor", SimModel.motor(motor))
            .visionField(VisionSimulationField.of(
                    VisionSimulationTarget.aprilTag(1, Meters.of(1.0), Meters.of(1.0), Meters.of(1.3), Radians.of(0.0)),
                    VisionSimulationTarget.aprilTag(2, Meters.of(15.0), Meters.of(7.0), Meters.of(1.3), Radians.of(Math.PI))))
            .resetPose(new PoseSnapshot(2.0, 4.0, 0.0));

    public SimulationHarness() {
        session.physicsEngine((models, activeSession, seconds) -> activeSession.motor(motor).step(seconds));
    }

    public void step(double seconds) {
        session.motor(motor).setVoltage(6.0);
        session.step(seconds);
    }

    public SimulationSession session() {
        return session;
    }
}
