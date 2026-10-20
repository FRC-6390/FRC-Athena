package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.interpolation.CurveMapping;
import ca.frc6390.athena.mechanism.interpolation.InterpolationModel;
import ca.frc6390.athena.runtime.control.ControlSink;
import edu.wpi.first.units.measure.Time;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

/**
 * Guards the typed-only control API: the unit-less overloads removed in the typed migration must not return.
 * Each removed signature is looked up reflectively and must be absent.
 */
class RemovedUntypedApiTest {

    @Test
    void controlBindingHasNoUnitlessTargetsOrTolerances() {
        assertAbsent(ControlBinding.class, "position", double.class);
        assertAbsent(ControlBinding.class, "position", DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "velocity", double.class);
        assertAbsent(ControlBinding.class, "velocity", DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "set", double.class);
        assertAbsent(ControlBinding.class, "set", DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "voltage", double.class);
        assertAbsent(ControlBinding.class, "voltage", DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "error", double.class);
        assertAbsent(ControlBinding.class, "isAt", double.class, double.class);
        assertAbsent(ControlBinding.class, "at", double.class, double.class);
        assertAbsent(ControlBinding.class, "at", DoubleSupplier.class, double.class);
        assertAbsent(ControlBinding.class, "continuous", double.class, double.class);
        assertAbsent(ControlBinding.class, "slipCheck", EncoderDevice.class, GearRatio.class, double.class);
        assertAbsent(ControlBinding.class, "interpolate", InterpolationModel.class, DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "interpolate", CurveMapping.class, DoubleSupplier.class);
        assertAbsent(ControlBinding.class, "armFf", double.class, double.class, double.class, double.class);
    }

    @Test
    void actionHasNoUnitlessTolerances() {
        assertAbsent(Action.class, "untilWithin", double.class);
        assertAbsent(Action.class, "maxVelocity", double.class);
        assertAbsent(Action.class, "untilSettled", double.class, Time.class);
        assertAbsent(Actions.class, "untilWithin", Action.class, double.class);
        assertAbsent(Actions.class, "maxVelocity", Action.class, double.class);
    }

    @Test
    void interpolatedControlActionHasNoUnitlessPoints() {
        assertAbsent(InterpolatedControlAction.class, "at", double.class, double.class);
        assertAbsent(InterpolatedControlAction.class, "at", double.class, DoubleSupplier.class);
        assertAbsent(InterpolatedControlAction.class, "at", DoubleSupplier.class, double.class);
        assertAbsent(InterpolatedControlAction.class, "at", DoubleSupplier.class, DoubleSupplier.class);
    }

    @Test
    void constraintsHaveNoUnitlessRangesOrMotionLimits() {
        assertAbsent(Constraints.class, "clamp", double.class, double.class);
        assertAbsent(Constraints.class, "clamp", double.class);
        assertAbsent(Constraints.class, "motion", double.class, double.class);
    }

    @Test
    void armGainsTakeAnAngleAndExposeOneAngleAccessor() {
        assertAbsent(FeedforwardGains.class, "arm", double.class, double.class, double.class, double.class);
        assertAbsent(FeedforwardGains.class, "gravityOffsetRadians");
    }

    @Test
    void controlFactoriesRequireAUnit() {
        assertAbsent(Controls.class, "position", MotorDevice.class);
        assertAbsent(Controls.class, "velocity", MotorDevice.class);
        assertAbsent(Controls.class, "position", ControlSink.class);
        assertAbsent(Controls.class, "velocity", ControlSink.class);
        assertAbsent(Controls.class, "position", MotorDevice.class, MotorDevice[].class);
        assertAbsent(Controls.class, "velocity", MotorDevice.class, MotorDevice[].class);
    }

    private static void assertAbsent(Class<?> owner, String name, Class<?>... parameters) {
        assertThrows(NoSuchMethodException.class, () -> owner.getMethod(name, parameters),
                owner.getSimpleName() + "." + name + " must not exist");
    }
}
