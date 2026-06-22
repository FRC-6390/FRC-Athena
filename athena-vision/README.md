# athena-vision

Generic camera declarations and target observations for Athena 2027.

Vendor-specific PhotonVision, Limelight, and other camera adapters do not belong
in this module. They should translate vendor data into the generic observation
types here.

## Current Slice

- Stable camera keys and reusable camera aliases from `athena-api`.
- Camera declarations with robot-relative mount pose values.
- Target observation frames with deterministic best-target selection.
- Camera target views with no-target-safe accessors.
- Validation for finite mount poses and target observations.

## Dependencies

- Production: `athena-api`, `athena-runtime`.
- Test-only: none.
