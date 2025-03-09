package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface RobotSendableSystem {

    public interface RobotSendableDevice {
      ShuffleboardLayout shuffleboard(ShuffleboardLayout layout);
    }
    
    default RobotSendableSystem shuffleboard(String tab) {
      shuffleboard(Shuffleboard.getTab(tab));
      return this;
    }

    ShuffleboardTab shuffleboard(ShuffleboardTab tab);
}



