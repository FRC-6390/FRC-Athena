package ca.frc6390.athena.api.mechanism.behavior;

import java.util.List;

import ca.frc6390.athena.api.mechanism.behavior.automation.AutomationConfigurer;
import ca.frc6390.athena.api.mechanism.behavior.automation.MechanismAutomation;
import ca.frc6390.athena.api.mechanism.behavior.control.ControlConfigurer;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismControl;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;

public final class MechanismBehavior {
    private final MechanismControl control;
    private final MechanismAutomation automation;

    private MechanismBehavior() {
        this.control = MechanismControl.create();
        this.automation = MechanismAutomation.create();
    }

    public static MechanismBehavior create() {
        return new MechanismBehavior();
    }

    public static MechanismBehavior from(
            List<MechanismLoopDefinition> loops,
            List<MechanismAutomationDefinition> automation) {
        MechanismBehavior behavior = create();
        behavior.control.merge(MechanismControl.from(loops));
        behavior.automation.merge(MechanismAutomation.from(automation));
        return behavior;
    }

    public MechanismBehavior control(ControlConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(control);
        }
        return this;
    }

    public MechanismBehavior automation(AutomationConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(automation);
        }
        return this;
    }

    public MechanismBehavior merge(MechanismBehavior other) {
        if (other != null) {
            control.merge(other.control);
            automation.merge(other.automation);
        }
        return this;
    }

    public MechanismControl controlSection() {
        return control;
    }

    public MechanismAutomation automationSection() {
        return automation;
    }

    public List<MechanismLoopDefinition> loopDefinitions() {
        return control.definitions();
    }

    public List<MechanismAutomationDefinition> automationDefinitions() {
        return automation.definitions();
    }
}
