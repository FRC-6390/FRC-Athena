package ca.frc6390.athena.wpilib.lifecycle;

/**
 * Robot modes exposed by the WPILib lifecycle.
 */
public enum RobotMode {
    /**
     * Disabled robot mode.
     */
    DISABLED,

    /**
     * Autonomous robot mode.
     */
    AUTONOMOUS,

    /**
     * Teleoperated robot mode.
     */
    TELEOP,

    /**
     * Test robot mode.
     */
    TEST,

    /**
     * Simulation periodic loop.
     */
    SIMULATION
}
