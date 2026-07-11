package frc.robot;

import ca.frc6390.athena.hardware.device.HardwareBus;

public interface Constants {
    HardwareBus RIO = HardwareBus.rio();

    interface Driver {
        int PORT = 0;
        double DEADBAND = 0.08;
    }

    interface ModuleOffsets {
        // Replace these with the measured forward-facing absolute encoder rotations.
        double FRONT_LEFT = 0.0;
        double FRONT_RIGHT = 0.0;
        double BACK_LEFT = 0.0;
        double BACK_RIGHT = 0.0;
    }
}
