# Mechanism SysId Examples

## Configure safer SysId and bind directly to buttons

```java
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.mechanisms.Mechanism;

public final class RobotContainer {
    private final EnhancedXboxController operator = new EnhancedXboxController(1);
    private final Mechanism arm = buildArmMechanism();

    public RobotContainer() {
        arm.sysId(sysId -> sysId
                .rampRateVoltsPerSecond(0.25)
                .stepVoltage(1.5)
                .timeoutSeconds(4.0)
                .voltageLimit(2.0)
                .bindQuasistaticForward(operator.leftBumper)
                .bindQuasistaticReverse(operator.rightBumper)
                .bindDynamicForward(operator.x)
                .bindDynamicReverse(operator.y)
                .bindCancel(operator.start));
    }

    private static Mechanism buildArmMechanism() {
        // your normal mechanism construction
        return null;
    }
}
```

## Build commands directly when needed

```java
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// Useful if you want custom trigger composition.
operator.a.onTrue(arm.sysId().quasistatic(SysIdRoutine.Direction.kForward));
operator.b.onTrue(arm.sysId().dynamic(SysIdRoutine.Direction.kReverse));
```

