package ca.frc6390.athena.mechanism.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Output;
import ca.frc6390.athena.mechanism.motion.MotionReference;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class PidHoldDeadbandTest {
    private static ControlLoopRuntime runtime(PidGains gains) {
        ControlBinding<?> binding = Controls.position(MotorDevice.of(MotorKinds.NEO, 30), Units.Degrees)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 30).measures(Units.Degrees));
        return gains.bind(new ControlLoopBinding(binding, ActionContext.empty()));
    }

    private static ControlLoopContext at(double target, double position) {
        return new ControlLoopContext() {
            @Override public Output request() { return (Output.Position) () -> target; }
            @Override public double position() { return position; }
            @Override public MotionReference reference() { return MotionReference.stationary(target); }
        };
    }

    private static double volts(ControlOutput output) {
        return ((Output.Voltage) output.output()).volts();
    }

    @Test
    void errorsInsideTheBandDrawNoCorrection() {
        ControlLoopRuntime pid = runtime(PidGains.of(0.5, 0.0, 0.0).holdDeadband(1.0));
        assertEquals(0.0, volts(pid.calculate(at(30.0, 29.4))), 1e-12, "0.6 deg inside the 1 deg band");
        assertEquals(0.0, volts(pid.calculate(at(30.0, 30.9))), 1e-12, "either side");
    }

    @Test
    void correctionGrowsSmoothlyFromTheEdgeWithoutAKick() {
        ControlLoopRuntime pid = runtime(PidGains.of(0.5, 0.0, 0.0).holdDeadband(1.0));
        assertEquals(0.5 * 2.0, volts(pid.calculate(at(30.0, 27.0))), 1e-9, "3 deg out: P acts on the 2 deg beyond");
        double justInside = volts(pid.calculate(at(30.0, 29.001)));
        double justOutside = volts(pid.calculate(at(30.0, 28.999)));
        assertTrue(Math.abs(justOutside - justInside) < 0.001, "continuous at the edge: " + justInside + " vs " + justOutside);
    }

    @Test
    void withoutABandItIsUnchangedAndOffloadable() {
        ControlLoopRuntime pid = runtime(PidGains.of(0.5, 0.0, 0.0));
        assertEquals(0.3, volts(pid.calculate(at(30.0, 29.4))), 1e-9);
        assertEquals(ControlLoopRole.DEVICE_CONFIGURABLE, PidGains.of(0.5, 0.0, 0.0).role());
        assertEquals(ControlLoopRole.SOFTWARE_OUTPUT, PidGains.of(0.5, 0.0, 0.0).holdDeadband(1.0).role(),
                "a motor controller cannot apply the band, so the loop stays in Athena");
    }
}
