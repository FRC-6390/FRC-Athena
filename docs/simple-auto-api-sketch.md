# Sketch: a small Athena motion and routine API

Status: design exploration. This document intentionally comes before an implementation commitment.

## Problem to solve

Choreo and PathPlanner are good at authored full-field motion, but they are heavy for a two-meter
assist, a live scoring target, or a routine that shares some drivetrain authority with the driver.
Authored vendor motion still crosses Athena Actions and the vendor execution layer. Interruption,
marker ownership, and driver takeover therefore need to remain explicit even though selectable autos
are now ordinary Actions in an auto-discovered `AutoChooser`.

The small API should optimize for:

- short on-the-fly motion from the current localized pose;
- the same routine primitives in autonomous and teleop;
- explicit ownership of translation, heading, and mechanism resources;
- immediate, predictable driver override;
- simple conditions, timeouts, markers, and multiple segments;
- adapters to authored vendor paths without making vendors the core abstraction.

It should not initially promise obstacle avoidance, global pathfinding through arbitrary field
geometry, or replacement of Choreo's dynamics-quality trajectories.

## Recommended shape: intent arbitration plus a routine DSL

The most useful foundation is not another `AutoBuilder`. It is a drivetrain intent arbiter. Every
producer submits only the axes it wants to own:

```java
drive.submit(DriveIntent.driver("sticks", driverTranslation, driverOmega));
drive.submit(DriveIntent.heading("face-speaker", targetHeading).priority(20));
drive.submit(DriveIntent.translation("dock", autoTranslation).priority(30));
```

The arbiter resolves one translation owner and one rotation owner each cycle, then emits exactly one
drivetrain request. This permits driver translation plus auto heading without scheduling two commands
that conflict on the entire drivetrain. A full path follower simply claims both axes.

On top of that, use a small immutable routine description:

```java
Routine twoPiece = AutoPlan.named("dynamic-two-piece")
        .resetPose(startPose)
        .run(superstructure.score())
        .goTo(() -> detectedPiecePose, MotionLimits.assist())
            .event(0.60, superstructure.intake())
            .timeout(2.0)
        .choose(hasPiece,
                plan -> plan.goTo(scorePose).run(superstructure.score()),
                plan -> plan.goTo(safeExitPose))
        .build();
```

`Routine` is mode-agnostic. `AutoSelection` chooses one for the autonomous period; a controller
binding can run the same routine in teleop with a different cancellation policy.

## Core types

### `DriveIntent`

- Optional translation demand.
- Optional angular demand or heading target.
- Priority, source ID, timestamp, and expiry.
- Frame (`ROBOT`, `FIELD_BLUE`, or alliance-relative).
- A takeover policy: cancel immediately, blend down, or ignore small stick noise.
- Debug fields exposing which source won each axis.

An intent expires unless refreshed. This avoids a crashed or interrupted helper leaving stale drive
output behind.

### `MotionGoal`

Start with three goals:

- `PoseGoal`: reach a pose within translation/rotation tolerances.
- `TranslationGoal`: reach a point while another source owns heading.
- `HeadingGoal`: rotate toward an angle or live target while another source owns translation.

Goals should take suppliers so a target can move, but the caller must choose whether to snapshot the
supplier on start or track it continuously.

### `MotionSegment`

For the MVP, generate a smooth direct segment from the current pose using trapezoidal limits and a
holonomic pose controller. A segment owns both axes unless constructed as translation-only or
heading-only. Useful policies:

- `replanOnError(distance)` regenerates the segment when localization diverges.
- `finishWithin(positionTolerance, angleTolerance)`.
- `timeout(seconds)` and `onFailure(...)`.
- `keepVelocity()` to join multiple segments without stopping.

This is deliberately “go directly to this nearby pose,” not a global obstacle-avoiding pathfinder.
An optional field-zone guard can reject a segment that intersects known keep-out polygons.

### `RoutineStep`

Keep the first set small:

- `run(action)`
- `goTo(goal)`
- `waitSeconds` / `waitUntil`
- `sequence`, `parallel`, `race`, and `deadline`
- `choose(condition, ifTrue, ifFalse)`
- `repeatWhile`
- `event(progress, action)`
- `timeout` and explicit failure/cancel handlers

Conditions should document evaluation time. A choice normally evaluates once when entered; a guard
may evaluate continuously and interrupt its child.

### `RoutineRunner`

Use one lifecycle implementation for both auto and teleop. It must carry an interruption reason:

```java
enum EndReason { FINISHED, DRIVER_OVERRIDE, DISABLED, TIMEOUT, FAULT, REPLACED }
```

That fixes a current limitation where `CommandAction.onEnd()` cannot distinguish completion from
interruption. Vendor commands can be leaf steps adapted into this runner until their integrations are
migrated.

## Driver-assist policies

Provide policies rather than embedding joystick knowledge in path code:

- `whileHeld`: cancel with `DRIVER_OVERRIDE` on release.
- `untilStickMagnitude(0.15)`: path owns translation until intentional driver input.
- `headingAssist`: driver always owns translation; assist owns rotation.
- `nudgeablePath(0.25)`: path demand plus a bounded driver translation offset.
- `confirmBeforeMechanismAction`: motion may align automatically, but scoring waits for a button.

The resolution order should be visible in telemetry. A driver should be able to see “translation:
dynamic-score-path; rotation: face-target” rather than infer control ownership from scheduled command
names.

## Vendor integration

Treat vendor paths as leaf motion steps:

```java
plan.follow(Paths.pathPlanner("four-piece-authored"));
plan.follow(Paths.choreo("pickup-sprint").split(1));
plan.goTo(livePose); // switches back to Athena's simple local motion
```

Markers need one typed event bus. Vendor adapters translate their native event into `RoutineEvent`;
custom segments emit progress events directly. Deduplicate events by `(runId, segmentId, eventId)` so
an adapter cannot fire the same marker twice after replanning.

Avoid making `PathGraph` global across all selected routines. Each routine run should own an event
scope, allowing two files to reuse a sensible marker name such as `intake`.

## Three implementation options considered

1. Build only a nicer wrapper around WPILib commands. Lowest effort, but subsystem-level drive
   requirements still prevent mixed driver/assist ownership.
2. Build a complete custom scheduler and trajectory engine. Maximum control, but too large and risky
   for the first iteration.
3. Build axis-level intent arbitration, a small nearby-pose controller, and a routine runner that can
   host vendor commands as leaves. This is the recommended middle path.

## Proposed delivery slices

1. Add drive-intent arbitration and telemetry, then migrate the default drive command and one
   heading assist. No new path generator yet.
2. Add `PoseGoal` with direct local motion, timeouts, tolerances, and driver-release cancellation.
3. Add immutable sequence/condition/parallel routine steps and a single mode-agnostic runner.
4. Add scoped typed events and mechanism actions.
5. Adapt PathPlanner and Choreo as leaf steps; preserve their native authored-path strengths.
6. Only after field testing, consider keep-out zones or a lightweight local obstacle planner.

## Questions to settle before implementation

- Is translation/rotation axis ownership enough, or does the drivetrain need separate X and Y
  ownership for alignment assists?
- Should a live goal track continuously by default, or require an explicit `.tracking()` policy?
- Which coordinate frame is canonical across alliance flipping and localization resets?
- Does a mechanism action use Athena `Action`, `CommandAction`, WPILib `Command`, or a new common leaf
  interface during migration?
- How should a routine resume after brief vision loss: hold, dead-reckon, fall back, or fail?
- Which safety zones must the direct segment reject, and who supplies those zones each season?
- Should driver override cancel the whole routine or only the active motion step while mechanism
  steps continue?

The first prototype should answer the ownership and cancellation questions with one heading assist
and one short scoring alignment before expanding the public API.
