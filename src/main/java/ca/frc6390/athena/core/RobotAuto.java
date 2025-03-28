package ca.frc6390.athena.core;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotAuto {
    
    public ArrayList<String> autos = new ArrayList<>();
    private SendableChooser<Command> chooser;
    private ProfiledPIDController xController, yController, thetaController;
    
    public RobotAuto(){

    }

    public void registerPathplannerAuto(String... auto){
        for (String name : auto) {
            autos.add(name);
        }
    }

    public SendableChooser<Command> getSendableChooser(String defualt){
        chooser = new SendableChooser<>();
        for (String auto : autos) {
            chooser.addOption(auto, new PathPlannerAuto(auto));
        }
        chooser.setDefaultOption(defualt, new PathPlannerAuto(defualt));
        return chooser;  
    }

    public SendableChooser<Command> getAutoChooser() {
        return chooser;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public ProfiledPIDController getXController() {
        return xController;
    }

    public ProfiledPIDController getYController() {
        return yController;
    }
}