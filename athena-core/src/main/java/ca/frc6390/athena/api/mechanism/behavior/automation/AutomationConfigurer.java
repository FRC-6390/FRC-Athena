package ca.frc6390.athena.api.mechanism.behavior.automation;

@FunctionalInterface
public interface AutomationConfigurer {
    MechanismAutomation apply(MechanismAutomation automation);
}
