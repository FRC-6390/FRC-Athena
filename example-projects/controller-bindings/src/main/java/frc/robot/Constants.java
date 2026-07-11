package frc.robot;

import ca.frc6390.athena.hardware.device.HardwareBus;

public interface Constants {
    HardwareBus RIO = HardwareBus.rio();

    interface Driver {
        int PORT = 0;
        double DEADBAND = 0.08;
        double SLOW_SCALE = 0.4;
    }
}
