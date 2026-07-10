package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathProvider;

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
                        ExampleCommands.fromWpilib(
                                "custom-intake-marker",
                                context.mechanisms.intakeUntilCaptured())),
                Autos.marker(
                        "custom-stow",
                        ExampleCommands.fromWpilib(
                                "custom-stow-marker",
                                context.mechanisms.stow())));
    }
}
