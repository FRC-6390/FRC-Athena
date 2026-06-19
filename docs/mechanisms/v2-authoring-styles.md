# V2 Mechanism Authoring Styles

Athena V2 currently supports three mechanism authoring styles:

- annotation
- structured
- flow

Annotation is the primary style for new examples and docs. The example files under
`athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/v2`
keep annotation as the live code and comment the structured and flow versions in the
same file so one mechanism can show all three surfaces side by side.

## Primary Example Files

- `SimpleMotorMechanismExamples.java`
- `FlywheelMechanismExamples.java`
- `ArmMechanismExamples.java`
- `ElevatorMechanismExamples.java`
- `TurretMechanismExamples.java`

## Style Guidance

- Prefer annotation for repo-facing examples and onboarding material.
- Use structured when you want source-visible Java methods without the dense fluent chain.
- Use flow when you want the compact lambda-heavy builder style.
- Keep all three styles lowering into the same `MechanismDefinition`.

## Current Boundary

- Mechanism V2 supports annotation, structured, and flow examples.
- Superstructure V2 currently has flow and structured examples; there is no matching annotation path yet.
