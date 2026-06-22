package ca.frc6390.athena.examples;

import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.runtime.control.Debouncer;
import ca.frc6390.athena.runtime.control.ManualClock;
import ca.frc6390.athena.runtime.control.ModifiedAxis;
import ca.frc6390.athena.runtime.filter.FilteredPose;
import ca.frc6390.athena.runtime.filter.FilteredValue;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.wpilib.controllers.WpilibControllerBindings;
import ca.frc6390.athena.wpilib.controllers.WpilibDriverStationProfile;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Example controller shaping and filter pipelines.
 */
public final class ControlUtilityExample {
    private ControlUtilityExample() {
    }

    /**
     * Creates a shaped driver axis.
     *
     * @return modified axis
     */
    public static ModifiedAxis driverTurnAxis() {
        return new ModifiedAxis(() -> 0.5, 0.1)
                .squared(true)
                .inverted(false);
    }

    /**
     * Creates a shaped turn axis from a WPILib Xbox controller.
     *
     * @param controller WPILib Xbox controller
     * @return modified axis
     */
    public static ModifiedAxis xboxTurnAxis(XboxController controller) {
        return WpilibControllerBindings.axis(controller, XboxController.Axis.kRightX, 0.1)
                .squared(true);
    }

    /**
     * Creates the same shaped turn axis from an indexed axis reader.
     *
     * @param reader indexed axis reader
     * @return modified axis
     */
    public static ModifiedAxis hidTurnAxis(WpilibControllerBindings.AxisReader reader) {
        return WpilibControllerBindings.axis(reader, XboxController.Axis.kRightX.value, 0.1)
                .squared(true);
    }

    /**
     * Creates a named driver-station profile.
     *
     * @return driver-station profile
     */
    public static WpilibDriverStationProfile driverStationProfile() {
        return WpilibDriverStationProfile.standard(0, 1)
                .deadzone(0.1)
                .debounceSeconds(0.25);
    }

    /**
     * Creates a shaped turn axis from the named driver profile.
     *
     * @param reader indexed axis reader
     * @return modified axis
     */
    public static ModifiedAxis profiledDriverTurnAxis(WpilibControllerBindings.AxisReader reader) {
        return driverStationProfile().driverAxis(reader, XboxController.Axis.kRightX)
                .squared(true);
    }

    /**
     * Creates a deterministic debouncer example.
     *
     * @param clock manual clock
     * @return debouncer
     */
    public static Debouncer heldButton(ManualClock clock) {
        return new Debouncer(() -> true, 0.25, clock);
    }

    /**
     * Creates a filtered scalar value.
     *
     * @return filtered value
     */
    public static FilteredValue offsetTarget() {
        return new FilteredValue(() -> 1.5).addOffset(0.25);
    }

    /**
     * Creates a filtered pose pipeline.
     *
     * @param pose mutable pose source
     * @return filtered pose
     */
    public static FilteredPose smoothedPose(AtomicReference<PoseSnapshot> pose) {
        return new FilteredPose(pose::get).addMovingAverage(2);
    }
}
