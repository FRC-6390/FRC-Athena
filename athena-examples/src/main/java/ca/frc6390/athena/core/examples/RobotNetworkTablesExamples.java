package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotNetworkTables;

/**
 * Examples for configuring RobotNetworkTables runtime publishing flags.
 */
public final class RobotNetworkTablesExamples {
    private RobotNetworkTablesExamples() {}

    public static void configureFastLocalPublishing(RobotNetworkTables nt) {
        nt.setPublishingEnabled(true);
        nt.setAsyncPublishingEnabled(false);
        nt.setDefaultPeriodSeconds(0.05);
        nt.enable(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS);
        nt.enable(RobotNetworkTables.Flag.LOCALIZATION_HEALTH_WIDGETS);
    }

    public static RobotNetworkTables.MechanismToggles enableMechanismTelemetry(
            RobotNetworkTables nt,
            String mechanismPath) {
        RobotNetworkTables.Node mechanismNode = nt.root().child(mechanismPath);
        return nt.mechanismConfig(mechanismNode)
                .details(true)
                .motors(true)
                .encoder(true)
                .constraints(true)
                .sensors(true)
                .control(true)
                .inputs(true)
                .simulation(true)
                .sysId(true);
    }
}
