package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;

/** Athena-native marker metadata for a custom provider that owns marker timing. */
public final class CustomProviderMarkersAuto {
    private CustomProviderMarkersAuto() {
    }

    public static AutoRoutine create(AutoContext context, PathProvider provider) {
        return Autos.path(
                "Custom 1 - Provider markers",
                provider,
                "custom-provider-path",
                Autos.marker(
                        "custom-intake",
                        WpilibCommands.wrap(context.mechanisms.intakeUntilCaptured()
                                .withName("custom-intake-marker"))),
                Autos.marker(
                        "custom-stow",
                        WpilibCommands.wrap(context.mechanisms.stow()
                                .withName("custom-stow-marker"))));
    }
}
