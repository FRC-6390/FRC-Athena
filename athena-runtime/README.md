# athena-runtime

Validation reports, errors, lifecycle, registries, and runtime support shared by
Athena feature modules.

## Current Slice

`ValidationReport` is the common result type for specs. Use `hasErrors()` when
you want to inspect failures, or `assertValid()` when robot/bootstrap code should
fail fast.

```java
ValidationReport report = spec.validate(context);
report.assertValid();
```

`AthenaValidationException` keeps the original report for tooling and tests.

Diagnostics live in runtime because they are shared by robot bootstrap,
telemetry, mechanisms, and adapters:

```java
DiagnosticsChannel channel = new DiagnosticsChannel("drive", 16)
        .summary("state", "ready");
channel.info("enabled");
DiagnosticsSnapshot snapshot = channel.snapshot(10);
```

- `BoundedEventLog` retains recent events with monotonic sequence numbers.
- `DiagnosticsChannel` owns a named log and summary fields.
- `DiagnosticsSnapshot` is immutable and dashboard/test friendly.

Controller and filter helpers are dependency-free runtime utilities:

- `ModifiedAxis` applies deadzone shaping, optional squaring, and inversion.
- `Debouncer` and `DelayedOutput` support deterministic clocks for tests.
- `FilteredValue` and `FilteredPose` provide small chainable filter pipelines.
- `MotionLimits` aggregates conservative drive and axis motion limits.
- `TimedRunner` provides deterministic periodic scheduling for small tasks.
- `RobotSpeeds` blends named drive, autonomous, feedback, and custom velocity
  sources without depending on WPILib `ChassisSpeeds`.

## Dependencies

- Production: `athena-api`.
- Test-only: none.
