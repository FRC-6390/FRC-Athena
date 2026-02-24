# Drivetrain SysId Examples

## Swerve: safer config + direct button binding

```java
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;

public final class RobotContainer {
    private final EnhancedXboxController driver = new EnhancedXboxController(0);
    private final SwerveDrivetrain drivetrain = buildSwerve();

    public RobotContainer() {
        drivetrain.sysId(sysId -> {
            // Conservative defaults for protecting mechanisms/modules.
            sysId.rampRateVoltsPerSecond(0.4);
            sysId.stepVoltage(2.0);
            sysId.timeoutSeconds(5.0);
            sysId.voltageLimit(3.0);

            // No Shuffleboard/Elastic required: bind directly to controller buttons.
            sysId.bindQuasistaticForward(driver.a);
            sysId.bindQuasistaticReverse(driver.b);
            sysId.bindDynamicForward(driver.x);
            sysId.bindDynamicReverse(driver.y);
        });
    }

    private static SwerveDrivetrain buildSwerve() {
        // your normal drivetrain construction
        return null;
    }
}
```

## Differential: same API

```java
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;

public final class RobotContainer {
    private final EnhancedXboxController driver = new EnhancedXboxController(0);
    private final DifferentialDrivetrain drivetrain = buildDiff();

    public RobotContainer() {
        drivetrain.sysId(sysId -> {
            sysId.rampRateVoltsPerSecond(0.3);
            sysId.stepVoltage(1.8);
            sysId.timeoutSeconds(4.0);
            sysId.voltageLimit(2.5);

            sysId.bindQuasistaticForward(driver.a);
            sysId.bindQuasistaticReverse(driver.b);
            sysId.bindDynamicForward(driver.x);
            sysId.bindDynamicReverse(driver.y);
        });
    }

    private static DifferentialDrivetrain buildDiff() {
        // your normal drivetrain construction
        return null;
    }
}
```

