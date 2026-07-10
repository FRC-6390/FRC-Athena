package frc.robot;

import ca.frc6390.athena.hardware.device.HardwareBus;

public interface Constants {
    HardwareBus RIO = HardwareBus.rio();

    interface Operator {
        int PORT = 0;
    }
}
