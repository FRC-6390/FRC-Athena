package ca.frc6390.athena.arcp.examples;

import ca.frc6390.athena.core.arcp.ARCP;

/**
 * Writable and command examples for ARCP control flows.
 */
public final class ArcpControlExamples {
    private ArcpControlExamples() {}

    public interface DriveController {
        void setEnabled(boolean enabled);

        void setProfileIndex(long profileIndex);

        void setKp(double kp);

        void setMode(String mode);
    }

    public interface DriveCommands {
        void zeroGyro();

        void syncOdometry();

        void clearFaults();
    }

    public static void registerWritableSignals(ARCP arcp, DriveController controller) {
        arcp.writableBoolean("Athena/Drive/Enabled")
                .onSetBoolean(controller::setEnabled)
                .widget(ARCP.Widgets.toggle())
                .meta("label", "Drive Enabled");

        arcp.writableI64("Athena/Drive/ProfileIndex")
                .onSetI64(controller::setProfileIndex)
                .widget(ARCP.Widgets.numberInput().min(0).max(8).step(1))
                .meta("label", "Profile Index");

        arcp.writableDouble("Athena/Drive/Kp")
                .onSetDouble(controller::setKp)
                .widget(ARCP.Widgets.numberInput().min(0.0).max(2.0).step(0.001))
                .meta("label", "kP");

        arcp.writable("Athena/Drive/Mode", String.class)
                .onSet(controller::setMode)
                .meta("label", "Drive Mode")
                .meta("ui.widget", "dropdown")
                .meta("ui.options", "open_loop,closed_loop,coast");
    }

    public static void registerCommandSignals(ARCP arcp, DriveCommands commands) {
        arcp.command("Athena/Drive/ZeroGyro")
                .onInvoke(commands::zeroGyro)
                .widget(ARCP.Widgets.button("Zero Gyro"));

        arcp.command("Athena/Drive/SyncOdometry")
                .onInvoke(commands::syncOdometry)
                .widget(ARCP.Widgets.button("Sync Odometry"));

        arcp.command("Athena/Drive/ClearFaults")
                .onInvoke(commands::clearFaults)
                .widget(ARCP.Widgets.button("Clear Faults"));
    }

    public static void invokeCommandLocally(ARCP arcp) {
        arcp.command("Athena/Drive/ZeroGyro").invoke();
    }

    public static void dispatchInboundEvents(ARCP arcp) {
        // Call this during your periodic loop to deliver writable/command events.
        arcp.dispatchPendingEvents();
    }
}
