package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface RobotSendableSystem {

    public static enum SendableLevel {
      COMP,
      DEBUG,
    }

    public interface RobotSendableDevice {

      default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        return shuffleboard(layout, SendableLevel.COMP);
      }

      ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level);
    }
    
    default RobotSendableSystem shuffleboard(String tab) {
      shuffleboard(tab, SendableLevel.COMP);
      return this;
    }

    default RobotSendableSystem shuffleboard(String tab, SendableLevel level) {
      shuffleboard(Shuffleboard.getTab(tab), SendableLevel.COMP);
      return this;
    }

    ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level);
}



