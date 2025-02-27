package ca.frc6390.athena.commands;

import java.util.function.BooleanSupplier;

import ca.frc6390.athena.controllers.DelayedOutput;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
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
  public RunnableTrigger onChange(Runnable runnable) {   
    return (RunnableTrigger) super.onChange(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable whenever the condition changes from `false` to `true`.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger onTrue(Runnable runnable) {
    return (RunnableTrigger) super.onTrue(new InstantCommand(runnable));
  }

  /**
   * Starts the given runnable whenever the condition changes from `true` to `false`.
   *
   * @param runnable the runnable to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger onFalse(Runnable runnable) {
    return (RunnableTrigger) super.onFalse(new InstantCommand(runnable));
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
  public RunnableTrigger whileTrue(Runnable runnable) {
    return (RunnableTrigger) super.whileTrue(new InstantCommand(runnable));
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
  public RunnableTrigger whileFalse(Runnable runnable) {
    return (RunnableTrigger) super.whileFalse(new InstantCommand(runnable));
  }

  /**
   * Toggles a runnable when the condition changes from `false` to `true`.
   *
   * @param runnable the runnable to toggle
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger toggleOnTrue(Runnable runnable) {
    return (RunnableTrigger) super.toggleOnTrue(new InstantCommand(runnable));
  }

  /**
   * Toggles a runnable when the condition changes from `true` to `false`.
   *
   * @param runnable the runnable to toggle
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger toggleOnFalse(Runnable runnable) {
    return (RunnableTrigger) super.toggleOnFalse(new InstantCommand(runnable));
  }



    /**
   * Starts the runnable when the condition changes.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger onChange(Command command) {   
    return (RunnableTrigger) super.onChange(command);
  }

  /**
   * Starts the given runnable whenever the condition changes from `false` to `true`.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger onTrue(Command command) {
    return (RunnableTrigger) super.onTrue(command);
  }

  /**
   * Starts the given runnable whenever the condition changes from `true` to `false`.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger onFalse(Command command) {
    return (RunnableTrigger) super.onFalse(command);
  }

  /**
   * Starts the given runnable when the condition changes to `true` and cancels it when the condition
   * changes to `false`.
   *
   * <p>Doesn't re-start the runnable if it ends while the condition is still `true`. If the runnable
   * should restart, see {@link edu.wpi.first.wpilibj2.runnable.Repeatrunnable}.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger whileTrue(Command command) {
    return (RunnableTrigger) super.whileTrue(command);
  }

  /**
   * Starts the given runnable when the condition changes to `false` and cancels it when the
   * condition changes to `true`.
   *
   * <p>Doesn't re-start the runnable if it ends while the condition is still `false`. If the runnable
   * should restart, see {@link edu.wpi.first.wpilibj2.runnable.Repeatrunnable}.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger whileFalse(Command command) {
    return (RunnableTrigger) super.whileFalse(command);
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger toggleOnTrue(Command command) {
    return (RunnableTrigger) super.toggleOnTrue(command);
  }

  /**
   * Toggles a command when the condition changes from `true` to `false`.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public RunnableTrigger toggleOnFalse(Command command) {
    return (RunnableTrigger) super.toggleOnFalse(command);
  }


   /**
   * Creates a new debounced trigger from this trigger - it will become active when this trigger has
   * been active for longer than the specified period.
   *
   * @param seconds The debounce period.
   * @return The debounced trigger (rising edges debounced only)
   */
  public RunnableTrigger debounce(double seconds) {
    return (RunnableTrigger) super.debounce(seconds);
  }

  /**
   * Creates a new debounced trigger from this trigger - it will become active when this trigger has
   * been active for longer than the specified period.
   *
   * @param seconds The debounce period.
   * @param type The debounce type.
   * @return The debounced trigger.
   */
  public RunnableTrigger debounce(double seconds, DebounceType type) {
      return (RunnableTrigger) super.debounce(seconds, type);
  }


   /**
   * Composes two triggers with logical AND.
   *
   * @param trigger the condition to compose with
   * @return A trigger which is active when both component triggers are active.
   */
  public RunnableTrigger and(BooleanSupplier trigger) {
    return (RunnableTrigger) super.and(trigger);
  }

  /**
   * Composes two triggers with logical OR.
   *
   * @param trigger the condition to compose with
   * @return A trigger which is active when either component trigger is active.
   */
  public RunnableTrigger or(BooleanSupplier trigger) {
    return (RunnableTrigger) super.or(trigger);
  }

  /**
   * Creates a new trigger that is active when this trigger is inactive, i.e. that acts as the
   * negation of this trigger.
   *
   * @return the negated trigger
   */
  public RunnableTrigger negate() {
    return (RunnableTrigger) super.negate();
  }

  public RunnableTrigger after(double seconds) {
    return new RunnableTrigger(new DelayedOutput(this, seconds));
  }

}
