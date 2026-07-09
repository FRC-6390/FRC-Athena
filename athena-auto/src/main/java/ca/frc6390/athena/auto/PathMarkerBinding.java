package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandState;
import java.util.Objects;

/**
 * Binds a named path marker to a command state.
 *
 * @param marker marker name
 * @param state command state
 */
public record PathMarkerBinding(String marker, CommandState state) {
    public PathMarkerBinding {
        marker = marker == null || marker.isBlank() ? "marker" : marker.trim();
        Objects.requireNonNull(state, "state");
    }
}
