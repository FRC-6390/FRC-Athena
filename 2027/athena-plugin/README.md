# athena-plugin

Gradle feature and vendor detection implementation for Athena 2027.

The plugin id is:

```text
ca.frc6390.athena
```

It detects installed vendor dependencies and adds only the matching
`athena-vendor-*` artifacts.

## Current Slice

- `AthenaGradlePlugin` adds selected Athena dependencies to `implementation`.
- `AthenaExtension` exposes `features(...)`, `vendors(...)`,
  `vendordepUuids(...)`, `version`, `group`, and `autoDetectVendors`.
- `AthenaFeature` defines default and optional Athena artifacts.
- `VendorFeature` defines vendor dependency, vendordep, and adapter artifact
  metadata.
- `VendorMetadataLoader` loads built-in metadata from
  `META-INF/athena/vendors/*.json`.
- `FeatureSelector` maps explicit features, explicit vendors, detected Gradle
  dependencies, and detected vendordep UUIDs into dependency coordinates.
- `META-INF/athena/vendors/*.json` is the resource shape third-party adapters
  should copy.

## Vendor Metadata

Vendor metadata resources use unversioned adapter coordinates. The plugin adds
the Athena version when it selects the adapter.

```json
{
  "feature": "acme",
  "displayName": "Acme Robotics",
  "detect": {
    "vendordepUuids": ["aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"],
    "dependencies": ["com.acme.frc:AcmeLib-java"]
  },
  "artifacts": [
    "com.acme.frc:athena-vendor-acme"
  ]
}
```

## Example

```groovy
plugins {
    id 'java'
    id 'ca.frc6390.athena'
}

athena {
    version = '2027.0.0'
    features 'drivetrain', 'vision', 'auto', 'localization', 'wpilib', 'dashboard'
    vendors 'ctre', 'photonvision', 'pathplanner', 'choreo'
}
```

Supported non-default feature names are:

```text
drivetrain
superstructure
vision
auto
localization
wpilib
dashboard
simulation
```

## Dependencies

- Production: `athena-api`.
- Test-only: none.
