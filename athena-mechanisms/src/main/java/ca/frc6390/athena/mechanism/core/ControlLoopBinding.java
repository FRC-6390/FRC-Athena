package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;
import java.util.Objects;

/**
 * Runtime binding data available when a control-loop ref is activated.
 */
public record ControlLoopBinding(ControlRef control, ActionContext refs) {
    public ControlLoopBinding {
        Objects.requireNonNull(control, "control");
        refs = refs == null ? ActionContext.empty() : refs;
    }
}
