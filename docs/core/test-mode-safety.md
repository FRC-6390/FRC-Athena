# Test-Mode Safety Examples

## What now happens in test mode

- Core lifecycle hooks and core control loops are not executed in test.
- Mechanism periodic hooks are not executed in test.
- Mechanism PID/feedforward/control-loop outputs are forced inactive in test.
- SysId commands still run in test (and are already test-gated).

## Example pattern: keep non-SysId controls out of test

```java
import edu.wpi.first.wpilibj.DriverStation;

// Only allow this normal action outside test mode.
driver.rightBumper
        .and(() -> !DriverStation.isTest())
        .onTrue(runNormalScoringActionCommand);

// SysId bindings are safe to leave test-only:
drivetrain.sysId(sysId -> {
    sysId.voltageLimit(3.0);
    sysId.bindDynamicForward(driver.x);
});
```

## Example: explicitly disable custom mechanism hooks/loops at setup time

```java
// Optional extra hardening for mechanisms with custom hook/loop logic:
arm.control(c -> c.disableAllHooksAndControlLoops());
```

