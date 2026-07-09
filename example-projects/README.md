# Athena Example Projects

- `blank` - blank robot project template
- `tank-drive` - tank drive robot
- `swerve-drive` - swerve drive robot

Each folder is a standalone WPILib robot project.

The base vendordep is:

- `FRC6390-Athena.json`

The drive examples declare real CTRE-style motor and encoder kinds. They still build with only the Athena vendordep because Athena's adapter jars do not transitively install third-party vendor libraries. For real hardware, install the normal vendor vendordep for the hardware on the robot.
