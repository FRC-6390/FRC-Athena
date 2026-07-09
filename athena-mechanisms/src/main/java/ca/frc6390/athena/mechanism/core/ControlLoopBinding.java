package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.Objects;

/**
 * Runtime binding data available when a control-loop ref is activated.
 */
public record ControlLoopBinding(ControlBinding control, ActionContext actionContext) {
    public ControlLoopBinding {
        Objects.requireNonNull(control, "control");
        actionContext = actionContext == null ? ActionContext.empty() : actionContext;
    }
}
