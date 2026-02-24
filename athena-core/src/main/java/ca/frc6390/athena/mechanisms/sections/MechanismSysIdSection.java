package ca.frc6390.athena.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class MechanismSysIdSection {
    private final Mechanism owner;

    MechanismSysIdSection(Mechanism owner) {
        this.owner = owner;
    }

    public MechanismSysIdSection rampRateVoltsPerSecond(double value) {
        owner.sysIdRampRateVoltsPerSecondInternal(value);
        return this;
    }

    public MechanismSysIdSection stepVoltage(double value) {
        owner.sysIdStepVoltageInternal(value);
        return this;
    }

    public MechanismSysIdSection timeoutSeconds(double value) {
        owner.sysIdTimeoutSecondsInternal(value);
        return this;
    }

    public MechanismSysIdSection voltageLimit(double value) {
        owner.sysIdVoltageLimitInternal(value);
        return this;
    }

    public double rampRateVoltsPerSecond() {
        return owner.sysIdRampRateVoltsPerSecondInternal();
    }

    public double stepVoltage() {
        return owner.sysIdStepVoltageInternal();
    }

    public double timeoutSeconds() {
        return owner.sysIdTimeoutSecondsInternal();
    }

    public double voltageLimit() {
        return owner.sysIdVoltageLimitInternal();
    }

    public boolean active() {
        return owner.sysIdActiveInternal();
    }

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return owner.sysIdQuasistaticInternal(direction);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return owner.sysIdDynamicInternal(direction);
    }

    public MechanismSysIdSection bindQuasistatic(Trigger trigger, SysIdRoutine.Direction direction) {
        if (trigger != null) {
            trigger.onTrue(Commands.runOnce(
                    () -> owner.scheduleSysIdCommandInternal(quasistatic(direction))).ignoringDisable(true));
        }
        return this;
    }

    public MechanismSysIdSection bindDynamic(Trigger trigger, SysIdRoutine.Direction direction) {
        if (trigger != null) {
            trigger.onTrue(Commands.runOnce(
                    () -> owner.scheduleSysIdCommandInternal(dynamic(direction))).ignoringDisable(true));
        }
        return this;
    }

    public MechanismSysIdSection bindQuasistaticForward(Trigger trigger) {
        return bindQuasistatic(trigger, SysIdRoutine.Direction.kForward);
    }

    public MechanismSysIdSection bindQuasistaticReverse(Trigger trigger) {
        return bindQuasistatic(trigger, SysIdRoutine.Direction.kReverse);
    }

    public MechanismSysIdSection bindDynamicForward(Trigger trigger) {
        return bindDynamic(trigger, SysIdRoutine.Direction.kForward);
    }

    public MechanismSysIdSection bindDynamicReverse(Trigger trigger) {
        return bindDynamic(trigger, SysIdRoutine.Direction.kReverse);
    }

    public MechanismSysIdSection bindCancel(Trigger trigger) {
        if (trigger != null) {
            trigger.onTrue(Commands.runOnce(owner::cancelSysIdCommandInternal).ignoringDisable(true));
        }
        return this;
    }

    public MechanismSysIdSection cancel() {
        owner.cancelSysIdCommandInternal();
        return this;
    }
}
