# Typed Input Resolver Behavior Contract

This document defines expected behavior for strict, lenient, and mutable typed input resolution.

## Resolution Priority

1. Mutable inputs (if key exists)
2. Registered supplier maps
3. Fallback behavior based on resolver mode

## Modes

- `STRICT`
  - Missing keys throw `IllegalArgumentException`.
- `LENIENT`
  - Missing bool: `false`
  - Missing double: `NaN`
  - Missing int: `0`
  - Missing string: `""`
  - Missing pose/object: `null`

## Object Type Safety

- `objectVal(key, type)` returns `null` if missing/null.
- If value exists and type does not match, throws `IllegalArgumentException`.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/TypedInputResolverExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/input/TypedInputResolverExamplesTest.java`
