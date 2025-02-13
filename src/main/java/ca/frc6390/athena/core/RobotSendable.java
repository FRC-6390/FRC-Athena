package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface RobotSendable {
    
    default ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
    }

    ShuffleboardTab shuffleboard(ShuffleboardTab tab);
}
