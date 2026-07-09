package ca.frc6390.athena.sim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;

class SimRuntimeTest {
    @Test
    void reusesHandlesByStableHardwareIdentity() {
        SimRuntime runtime = new SimRuntime();
        MotorDevice rioMotor = MotorDevice.of(MotorKinds.SIM, 1);
        MotorDevice canivoreMotor = rioMotor.canbus("canivore");
        ImuDevice imu = ImuDevice.of(ImuKinds.SIM, 2);

        assertSame(runtime.motor(rioMotor), runtime.motor(rioMotor));
        assertSame(runtime.imu(imu), runtime.imu(imu));
        assertEquals(rioMotor, runtime.motor(rioMotor).device());
        assertEquals(canivoreMotor, runtime.motor(canivoreMotor).device());
    }

    @Test
    void motorOutputsClampAndIntegrateDuringStep() {
        SimRuntime runtime = new SimRuntime();
        SimMotorHandle motor = runtime.motor(MotorDevice.of(MotorKinds.SIM, 3));

        motor.setPercentOutput(2.0);
        runtime.step(0.5);
        assertEquals(1.0, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.5, motor.integratedPositionRotations(), 1.0e-9);

        motor.setVoltage(-6.0);
        runtime.step(2.0);
        assertEquals(-0.5, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(-0.5, motor.integratedPositionRotations(), 1.0e-9);

        motor.setPositionTargetRotations(4.0);
        assertEquals(4.0, motor.integratedPositionRotations(), 1.0e-9);
        assertEquals(0.0, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);

        motor.setVelocityTargetRotationsPerSecond(3.0);
        runtime.step(Double.NaN);
        assertEquals(4.0, motor.integratedPositionRotations(), 1.0e-9);
        runtime.step(0.25);
        assertEquals(4.75, motor.integratedPositionRotations(), 1.0e-9);
    }

    @Test
    void imuYawRateIntegratesDuringStep() {
        SimRuntime runtime = new SimRuntime();
        SimImuHandle imu = runtime.imu(ImuDevice.of(ImuKinds.SIM, 4))
                .yawDegrees(10.0)
                .yawRateDegreesPerSecond(90.0);

        runtime.step(0.5);

        assertEquals(55.0, imu.yawDegrees(), 1.0e-9);
        assertEquals(55.0, imu.angleDegrees(), 1.0e-9);

        imu.zeroYaw();
        assertEquals(0.0, imu.yawDegrees(), 1.0e-9);
        runtime.step(0.5);
        assertEquals(45.0, imu.yawDegrees(), 1.0e-9);

        imu.reset();
        runtime.step(0.5);
        assertEquals(0.0, imu.yawDegrees(), 1.0e-9);
    }

    @Test
    void registeringModelsMaterializesMotorsAndPreservesModelList() {
        SimRuntime runtime = new SimRuntime();
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 5);

        runtime.model("drive", SimModels.motor(motor));

        assertEquals(1, runtime.registeredModels().size());
        assertSame(runtime.motor(motor), runtime.motor(motor));
    }

    @Test
    void hardwareGraphUsesRuntimeBackedSimulationHandles() {
        SimRuntime runtime = new SimRuntime();
        HardwareGraph graph = runtime.hardwareGraph();
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 6);
        ImuDevice imu = ImuDevice.of(ImuKinds.SIM, 7);

        MotorHandle graphMotor = graph.motor(motor);

        assertSame(runtime.motor(motor), graphMotor);
        assertSame(runtime.imu(imu), graph.imu(imu));

        graphMotor.setVelocityTargetRotationsPerSecond(2.0);
        runtime.step(0.5);
        assertEquals(1.0, runtime.motor(motor).integratedPositionRotations(), 1.0e-9);
        assertSame(graph.encoder(motor.encoder()), graph.encoder(motor.encoder()));
        assertEquals(1.0, graph.encoder(motor.encoder()).positionRotations(), 1.0e-9);
    }

}
