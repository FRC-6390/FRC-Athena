# Athena HeliOS

Preserved legacy HeliOS/PDLib camera adapter workspace.

This code is not part of the promoted root Gradle build because the V3
replacement does not yet have a dedicated HeliOS vendor adapter. The previous
Gradle file is retained as `build.gradle.legacy` for reference only; it depends
on evicted V2 modules and should not be treated as an active build.

The intended migration target is a future optional `athena-vendor-helios`
artifact with the same vendor metadata and dependency-isolation rules used by
the other V3 vendor adapters.
