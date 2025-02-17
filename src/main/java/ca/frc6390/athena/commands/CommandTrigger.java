package ca.frc6390.athena.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandTrigger extends Command{

    private final BooleanSupplier trigger;
    private final Command command;

    public CommandTrigger(BooleanSupplier trigger, Runnable command){
        this(trigger, new InstantCommand(command));
    }

    public CommandTrigger(BooleanSupplier trigger, Command command){
        this.command = command;
        this.trigger = trigger;
    }

    public static CommandTrigger event(String event, Command command){
        return new CommandTrigger(new EventTrigger(event), command);
    }

    public static CommandTrigger event(String event, Runnable command){
        return event(event, new InstantCommand(command));
    }

    @Override
    public void execute() {
        if(!command.isScheduled() && trigger.getAsBoolean()){
            command.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isScheduled() ? command.isFinished() : false;
    }
}
