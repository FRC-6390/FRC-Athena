package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandAction;
import java.util.Objects;

/**
 * Binds a named path marker to a command Action.
 *
 * @param marker marker name
 * @param Action command Action
 */
public record PathMarkerBinding(String marker, CommandAction Action) {
    public PathMarkerBinding {
        marker = marker == null || marker.isBlank() ? "marker" : marker.trim();
        Objects.requireNonNull(Action, "Action");
    }
}
