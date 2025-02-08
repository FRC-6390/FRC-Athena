package ca.frc6390.athena.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RunnableTrigger extends Trigger {

    public RunnableTrigger(BooleanSupplier condition) {
        super(condition);
    }
    
    /**
   * Starts the runnable when the condition changes.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public Trigger onChange(Runnable runnable) {   
    return onChange(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable whenever the condition changes from `false` to `true`.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public Trigger onTrue(Runnable runnable) {
    return onTrue(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable whenever the condition changes from `true` to `false`.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public Trigger onFalse(Runnable runnable) {
    return onFalse(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable when the condition changes to `true` and cancels it when the condition
   * changes to `false`.
   *
   * <p>Doesn't re-start the runnable if it ends while the condition is still `true`. If the runnable
   * should restart, see {@link edu.wpi.first.wpilibj2.runnable.Repeatrunnable}.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public Trigger whileTrue(Runnable runnable) {
    return whileTrue(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable when the condition changes to `false` and cancels it when the
   * condition changes to `true`.
   *
   * <p>Doesn't re-start the runnable if it ends while the condition is still `false`. If the runnable
   * should restart, see {@link edu.wpi.first.wpilibj2.runnable.Repeatrunnable}.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public Trigger whileFalse(Runnable runnable) {
    return whileFalse(new InstantCommand(runnable));
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public Trigger toggleOnTrue(Runnable runnable) {
    return toggleOnTrue(runnable);
  }

  /**
   * Toggles a command when the condition changes from `true` to `false`.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public Trigger toggleOnFalse(Runnable runnable) {
    return toggleOnFalse(runnable);
  }

}
