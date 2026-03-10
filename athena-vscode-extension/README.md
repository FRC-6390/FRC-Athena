# Athena Robot Tools

VS Code extension for Athena robot projects.

Current features:

- Athena DSL completion fallbacks for Java nested lambdas when JDT fails
- Athena JDT language-server bundle for `@AthenaState` false constructor diagnostics
- Athena activity-bar icon using the ARCP visual identity
- Robot Toolkit sidebar for quick file jumps and code templates
- Workspace helper command to run `./gradlew athenaDoctor`

Development:

- Source lives in `athena-vscode-extension/`
- JDT bundle source lives in `athena-vscode-extension/jdtls/`
- Build and sync the JDT bundle with `./gradlew :athena-vscode-extension-jdtls:syncBundle`
- Launch in a VS Code extension host with `--extensionDevelopmentPath=/path/to/FRC-Athena/athena-vscode-extension`
- The activity-bar icon uses the same ARCP mark bundled under `media/`
