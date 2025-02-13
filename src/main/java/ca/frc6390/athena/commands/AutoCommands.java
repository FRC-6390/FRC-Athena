package ca.frc6390.athena.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoCommands {
    
    public static Command registerCommand(String name, Runnable command, Subsystem... requirements){
        return AutoCommands.registerCommand(name, new InstantCommand(command, requirements));
    }

    public static Command registerCommand(String name, Command command){
        NamedCommands.registerCommand(name, command);
        return command;
    }
}
