package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface RobotSendableSystem {

    public interface RobotSendableDevice {
      ShuffleboardLayout shuffleboard(ShuffleboardLayout layout);
    }
    
    default ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
    }

    ShuffleboardTab shuffleboard(ShuffleboardTab tab);
}



