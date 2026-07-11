# Custom controls and feedback

This robot keeps the common feedback path simple:

```java
Controls.position(motor)
        .feedback(motor.encoder());
```

`AdvancedFeedback` demonstrates the explicit path used only when position and velocity come from
different processing chains:

```java
Controls.position(motor)
        .feedback(new FeedbackBinding(trackedPosition, filteredVelocity));
```

The example includes:

- a 17-rotation modular/Vernier absolute decoder using `1:1` and `16:17` single-turn encoders;
- circular candidate scoring and ambiguity rejection rather than directly differentiating CRT output;
- absolute-relative fusion that initializes the turn count, follows the motor encoder, and gates corrections;
- a one-pole velocity filter;
- median position fusion that continues with one invalid sensor; and
- ordinary one-encoder feedback beside the advanced cases;
- voltage-based PID and arbitrary feedforward loops that add their outputs in volts;
- controller composition through `ControlSignal`, including teleop gating and release actions; and
- automatically discovered flywheel and arm simulation models.

The modular range is half-open: `[0, 17)`. At 17 rotations the physical phase pattern repeats and is
indistinguishable from zero.

Controller bindings:

- B: modular absolute move;
- D-pad up: ordinary relative-encoder move;
- D-pad right: redundant median-feedback move; and
- D-pad down: stop both example mechanisms.

`Robot` owns every mechanism and constructs `Controls` with itself. Athena discovers the complete graph,
including controller hooks, hardware declarations, control bindings, and simulation models; no manual
`configure()` or `register()` calls are required.
