package ca.frc6390.athena.core;

import java.io.IOException;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotAuto {
    
    public static ArrayList<String> autos = new ArrayList<>();
    
    public static void registerPathplannerAuto(String auto){
        autos.add(auto);
    }

    public static SendableChooser<Command> getSendableChooser(String defualt){
        SendableChooser<Command> chooser = new SendableChooser<>();
        for (String auto : autos) {
            chooser.addOption(auto, new PathPlannerAuto(auto));
        }
        chooser.setDefaultOption(defualt, new PathPlannerAuto(defualt));
        return chooser;  
    }
}