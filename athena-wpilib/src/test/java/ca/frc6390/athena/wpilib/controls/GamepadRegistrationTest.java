package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

class GamepadRegistrationTest {
    @Test
    void physicalAndDerivedBindingsAreDiscoveredExactlyOnce() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        gamepad.a().onPress(() -> {});
        gamepad.b().toggle().onTrue(() -> {});
        gamepad.x().doubleClick().onTrue(() -> {});
        gamepad.rightTrigger().above(0.6, 0.4).onTrue(() -> {});

        Map<String, ?> hooks = gamepad.hooks();

        assertEquals(4, hooks.size());
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.startsWith("a.")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("b.toggle")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("x.clicks[2]")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("rightTrigger.above")));
    }

    @Test
    void unboundIntermediateSignalsDoNotCreateEmptyRuntimeHooks() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        gamepad.a().debounce(java.time.Duration.ofMillis(20)).doubleClick();
        gamepad.leftTrigger(0.5);

        assertTrue(gamepad.hooks().isEmpty());
    }

    @Test
    void toggleResetsOnDisconnectUnlessExplicitlyConfiguredToPersist() {
        FakeInput input = new FakeInput();
        Gamepad gamepad = new Gamepad(input);
        ToggleSignal safe = gamepad.a().toggle();
        ToggleSignal persistent = gamepad.b().toggle().resetOnDisconnect(false);

        ControlSignalTest.sample(safe, 0.0);
        ControlSignalTest.sample(persistent, 0.0);
        input.buttons[1] = true;
        input.buttons[2] = true;
        assertTrue(ControlSignalTest.sample(safe, 0.02));
        assertTrue(ControlSignalTest.sample(persistent, 0.02));

        input.connected = false;
        assertFalse(ControlSignalTest.sample(safe, 0.04));
        assertTrue(ControlSignalTest.sample(persistent, 0.04));
    }

    @Test
    void supplierAxesAreExposedForLiveControllerTuning() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        gamepad.leftY().named("Forward").deadband(0.08).curve(AxisCurves.expo(0.2)).slew(4.0).toSupplier();
        gamepad.leftY().named("Forward").squared().toSupplier();

        Map<String, ?> telemetry = gamepad.telemetry();
        assertTrue(telemetry.containsKey("Connected"));
        assertTrue(telemetry.containsKey("Axes/Forward/State/Raw"));
        assertTrue(telemetry.containsKey("Axes/Forward/Curve/Output"));
        assertTrue(telemetry.containsKey("Axes/Forward/Slew/PositiveRate"));
        assertTrue(telemetry.containsKey("Axes/Forward2/State/Output"));
    }

    @Test
    void severalRegisteredAxesCanShareOneLiveCurve() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        AxisCurves.SuperRateCurve shared = AxisCurves.superRate();
        gamepad.leftY().named("Forward").curve(shared).toSupplier();
        gamepad.leftX().named("Strafe").curve(shared).toSupplier();
        gamepad.rightX().named("Rotation").curve(shared).toSupplier();

        Map<String, ca.frc6390.athena.mechanism.core.TelemetryValue> telemetry = gamepad.telemetry();
        telemetry.get("Axes/Strafe/Curve/Config/Expo").set(0.6);

        assertEquals(0.6, telemetry.get("Axes/Forward/Curve/Config/Expo").number(), 1.0e-9);
        assertEquals(0.6, telemetry.get("Axes/Rotation/Curve/Config/Expo").number(), 1.0e-9);
        assertTrue(telemetry.containsKey("Axes/Forward/Visualization/Curve"));
        assertTrue(telemetry.containsKey("Axes/Strafe/Visualization/Curve"));
        assertTrue(telemetry.containsKey("Axes/Rotation/Visualization/Curve"));
    }

    @Test
    void controllerTelemetryIsDiscoveredAlongsideControllerHooks() {
        GamepadTelemetryMechanism mechanism = new GamepadTelemetryMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        assertTrue(runtime.mechanismTelemetrySchema().values().keySet().stream()
                .anyMatch(path -> path.endsWith("/Values/driver/Axes/Forward/State/Raw")));
        assertTrue(runtime.mechanismTelemetrySchema().values().keySet().stream()
                .anyMatch(path -> path.endsWith("/Values/driver/Axes/Forward/Curve/Output")));
        assertTrue(runtime.mechanismTelemetrySchema().values().keySet().stream()
                .noneMatch(path -> path.contains("/Values/forwardCurve/")));
    }

    private static final class GamepadTelemetryMechanism implements Mechanism {
        public final Gamepad driver = new Gamepad(new FakeInput());
        public final AxisCurves.SuperRateCurve forwardCurve = AxisCurves.superRate();
        @SuppressWarnings("unused")
        private final DoubleSupplier forward = driver.leftY()
                .named("Forward")
                .curve(forwardCurve)
                .slew(3.0)
                .toSupplier();
    }

    private static final class FakeInput implements Gamepad.Input {
        private final boolean[] buttons = new boolean[16];
        private boolean connected = true;

        @Override
        public double axis(int axis) {
            return 0.0;
        }

        @Override
        public boolean button(int button) {
            return buttons[button];
        }

        @Override
        public int pov() {
            return -1;
        }

        @Override
        public boolean connected() {
            return connected;
        }
    }
}
