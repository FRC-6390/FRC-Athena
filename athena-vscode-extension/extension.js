"use strict";

const fs = require("fs");
const os = require("os");
const path = require("path");

const LOG_FILE = path.join(os.tmpdir(), "athena-intellisense-local.log");
const WORKSPACE_EXCLUDE_GLOB = "**/{.git,node_modules,build,.gradle,bin,out,.settings}/**";
const COMMANDS = {
  openConstants: "athenaTools.openConstants",
  openStates: "athenaTools.openStates",
  openRobot: "athenaTools.openRobot",
  openMain: "athenaTools.openMain",
  openBuildGradle: "athenaTools.openBuildGradle",
  openVendordep: "athenaTools.openVendordep",
  insertMechanismConfig: "athenaTools.insertMechanismConfig",
  insertAthenaStateEnum: "athenaTools.insertAthenaStateEnum",
  insertAthenaStateEntry: "athenaTools.insertAthenaStateEntry",
  runAthenaDoctor: "athenaTools.runAthenaDoctor",
  refreshSidebar: "athenaTools.refreshSidebar"
};

function appendLog(message) {
  const line = `${new Date().toISOString()} ${message}\n`;
  try {
    fs.appendFileSync(LOG_FILE, line, "utf8");
  } catch (_) {}
}

function escapeRegex(text) {
  return text.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function matchTrailingIdentifier(textBeforeCursor) {
  const match = /([A-Za-z_$][\w$]*)(?:\s*\.\s*)?\s*$/.exec(textBeforeCursor);
  return match ? match[1] : null;
}

function detectAthenaContext(textBeforeCursor) {
  const windowText = textBeforeCursor.slice(-12000);
  const identifier = matchTrailingIdentifier(windowText);
  if (!identifier) {
    return null;
  }

  const escapedIdentifier = escapeRegex(identifier);
  const stateCtxPattern = new RegExp(
    "@AthenaState[\\s\\S]*?\\.until\\s*\\([\\s\\S]*?\\b" +
      escapedIdentifier +
      "\\s*->\\s*" +
      escapedIdentifier +
      "(?:\\s*\\.\\s*)?\\s*$"
  );
  if (stateCtxPattern.test(windowText)) {
    return "stateCtx";
  }

  const stateBuilderPattern = new RegExp(
    "@AthenaState[\\s\\S]*?\\(\\s*" +
      escapedIdentifier +
      "\\s*->\\s*" +
      escapedIdentifier +
      "(?:\\s*\\.\\s*)?\\s*$"
  );
  if (stateBuilderPattern.test(windowText)) {
    return "stateBuilder";
  }

  const encoderSourcePattern = new RegExp(
    "\\.encoders\\s*\\([\\s\\S]*?\\.add\\s*\\([\\s\\S]*?,\\s*" +
      escapedIdentifier +
      "\\s*->\\s*" +
      escapedIdentifier +
      "(?:\\s*\\.\\s*)?\\s*$"
  );
  if (encoderSourcePattern.test(windowText)) {
    return "encoderSource";
  }

  return null;
}

function completionSpecsForContext(context) {
  switch (context) {
    case "encoderSource":
      return [
        {
          label: "config",
          detail: "Athena encoder source: config(EncoderConfig config)",
          snippet: "config(${1:config})",
          documentation: "Apply a full encoder config object."
        },
        {
          label: "encoder",
          detail: "Athena encoder source: encoder(AthenaEncoder type, int id)",
          snippet: "encoder(${1:AthenaEncoder.INTERNAL}, ${2:id})",
          documentation: "Use an Athena encoder enum source."
        },
        {
          label: "encoder",
          detail: "Athena encoder source: encoder(EncoderType type, int id)",
          snippet: "encoder(${1:EncoderType}, ${2:id})",
          documentation: "Use a raw encoder type source."
        },
        {
          label: "virtual",
          detail: "Athena encoder source: virtual(DoubleSupplier positionSupplier)",
          snippet: "virtual(${1:() -> 0.0})",
          documentation: "Provide a virtual position supplier."
        },
        {
          label: "virtual",
          detail: "Athena encoder source: virtual(VirtualSourceSection section)",
          snippet: "virtual(${1:v} -> ${1:v}.$0)",
          documentation: "Configure a virtual encoder section."
        },
        {
          label: "crt",
          detail: "Athena encoder source: crt(CrtSourceSection section)",
          snippet: "crt(${1:crt} -> ${1:crt}.$0)",
          documentation: "Configure a Chinese remainder encoder source."
        },
        {
          label: "filter",
          detail: "Athena encoder source: filter(FilterSourceSection section)",
          snippet: "filter(${1:filter} -> ${1:filter}.$0)",
          documentation: "Configure a filtered encoder source."
        },
        {
          label: "differentiate",
          detail: "Athena encoder source: differentiate(DifferentiateSourceSection section)",
          snippet: "differentiate(${1:diff} -> ${1:diff}.$0)",
          documentation: "Configure a differentiated encoder source."
        },
        {
          label: "average",
          detail: "Athena encoder source: average(AverageSourceSection section)",
          snippet: "average(${1:avg} -> ${1:avg}.$0)",
          documentation: "Configure an averaged encoder source."
        },
        {
          label: "difference",
          detail: "Athena encoder source: difference(DifferenceSourceSection section)",
          snippet: "difference(${1:diff} -> ${1:diff}.$0)",
          documentation: "Configure a differential encoder source."
        },
        {
          label: "calibrationMap",
          detail: "Athena encoder source: calibrationMap(CalibrationMapSourceSection section)",
          snippet: "calibrationMap(${1:map} -> ${1:map}.$0)",
          documentation: "Configure a calibration-map encoder source."
        },
        {
          label: "canbus",
          detail: "Athena encoder source: canbus(String canbus)",
          snippet: "canbus(${1:\"can\"})",
          documentation: "Set the encoder CAN bus."
        },
        {
          label: "gearRatio",
          detail: "Athena encoder source: gearRatio(double gearRatio)",
          snippet: "gearRatio(${1:1.0})",
          documentation: "Set the encoder gear ratio."
        },
        {
          label: "conversion",
          detail: "Athena encoder source: conversion(double conversion)",
          snippet: "conversion(${1:1.0})",
          documentation: "Set the encoder conversion factor."
        },
        {
          label: "offset",
          detail: "Athena encoder source: offset(double offset)",
          snippet: "offset(${1:0.0})",
          documentation: "Set the encoder offset."
        },
        {
          label: "unit",
          detail: "Athena encoder source: unit(MechanismEncoderUnit unit)",
          snippet: "unit(${1:MechanismEncoderUnit.ROTATIONS})",
          documentation: "Set the encoder unit."
        },
        {
          label: "wrapsEvery",
          detail: "Athena encoder source: wrapsEvery(double wrapsEvery)",
          snippet: "wrapsEvery(${1:360.0})",
          documentation: "Set the encoder wrap span."
        }
      ];
    case "stateBuilder":
      return [
        {
          label: "setpoint",
          detail: "Athena state DSL: setpoint(double value)",
          snippet: "setpoint(${1:value})",
          documentation: "Set the state's target setpoint."
        },
        {
          label: "manualPercent",
          detail: "Athena state DSL: manualPercent(double value)",
          snippet: "manualPercent(${1:value})",
          documentation: "Set the state's manual percent output."
        },
        {
          label: "until",
          detail: "Athena state DSL: until(Predicate<StateCtx<E>> predicate)",
          snippet: "until(${1:ctx} -> ${1:ctx}.$0)",
          documentation: "Add a state exit predicate."
        },
        {
          label: "then",
          detail: "Athena state DSL: then(E state)",
          snippet: "then(${1:state})",
          documentation: "Transition to another enum state."
        },
        {
          label: "then",
          detail: "Athena state DSL: then(String stateName)",
          snippet: "then(${1:\"StateName\"})",
          documentation: "Transition by state name."
        },
        {
          label: "thenNamed",
          detail: "Athena state DSL: thenNamed(String stateName)",
          snippet: "thenNamed(${1:\"StateName\"})",
          documentation: "Transition by state name with an IDE-friendly helper."
        }
      ];
    case "stateCtx":
      return [
        {
          label: "state",
          detail: "Athena state context: state()",
          snippet: "state()",
          documentation: "Get the current enum state."
        },
        {
          label: "timeInState",
          detail: "Athena state context: timeInState()",
          snippet: "timeInState()",
          documentation: "Get time elapsed in the current state."
        },
        {
          label: "mechanism",
          detail: "Athena state context: mechanism()",
          snippet: "mechanism()",
          documentation: "Get the active mechanism."
        },
        {
          label: "limitSwitch",
          detail: "Athena state context: limitSwitch(int index)",
          snippet: "limitSwitch(${1:0})",
          documentation: "Read a mechanism limit switch by index."
        },
        {
          label: "stalled",
          detail: "Athena state context: stalled()",
          snippet: "stalled()",
          documentation: "Check whether the mechanism is stalled."
        }
      ];
    default:
      return [];
  }
}

function createCompletionItem(vscode, spec, index) {
  const separatorIndex = spec.detail.indexOf(": ");
  const context = separatorIndex >= 0 ? spec.detail.slice(0, separatorIndex) : "Athena";
  const signature = separatorIndex >= 0 ? spec.detail.slice(separatorIndex + 2) : spec.detail;
  const signatureSuffix = signature.startsWith(spec.label)
    ? signature.slice(spec.label.length)
    : "";
  const item = new vscode.CompletionItem(
    {
      label: spec.label,
      detail: signatureSuffix,
      description: context
    },
    vscode.CompletionItemKind.Method
  );
  item.detail = signature;
  item.documentation = new vscode.MarkdownString().appendCodeblock(signature, "java").appendMarkdown(`\n\n${spec.documentation}`);
  item.insertText = new vscode.SnippetString(spec.snippet);
  item.filterText = spec.label;
  item.sortText = `000${String(index).padStart(2, "0")}-${spec.detail}`;
  return item;
}

function provideAthenaCompletionItems(vscode, document, position) {
  const fullText = document.getText();
  const offset = document.offsetAt(position);
  const context = detectAthenaContext(fullText.slice(0, offset));
  if (!context) {
    return undefined;
  }
  const specs = completionSpecsForContext(context);
  return specs.map((spec, index) => createCompletionItem(vscode, spec, index));
}

function findProbePosition(document, pattern, anchor) {
  const text = document.getText();
  const startIndex = anchor ? text.indexOf(anchor) : 0;
  if (startIndex < 0) {
    return null;
  }
  const index = text.indexOf(pattern, startIndex);
  if (index < 0) {
    return null;
  }
  return document.positionAt(index + pattern.length);
}

async function runCompletionProbe(vscode) {
  if (process.env.ATHENA_INTELLISENSE_LOCAL_PROBE !== "1") {
    return;
  }

  const workspaceFolders = vscode.workspace.workspaceFolders || [];
  if (workspaceFolders.length === 0) {
    appendLog("probe skipped: no workspace folders");
    return;
  }

  const root = workspaceFolders[0].uri.fsPath;
  const probes = [
    {
      name: "constants-encoder",
      file: path.join(root, "src/main/java/frc/robot/Constants.java"),
      pattern: '.add("main", src -> src\n                ',
      anchor: "MechanismConfig<StatefulMechanism<IndexerState>> Indexer_CONFIG"
    },
    {
      name: "states-builder",
      file: path.join(root, "src/main/java/frc/robot/States.java"),
      pattern: "Stow(s -> s\n                "
    },
    {
      name: "states-ctx",
      file: path.join(root, "src/main/java/frc/robot/States.java"),
      pattern: "until(ctx -> ctx."
    }
  ];

  for (const probe of probes) {
    try {
      const uri = vscode.Uri.file(probe.file);
      const document = await vscode.workspace.openTextDocument(uri);
      const position = findProbePosition(document, probe.pattern, probe.anchor);
      if (!position) {
        appendLog(`probe ${probe.name}: pattern not found`);
        continue;
      }
      const result = await vscode.commands.executeCommand(
        "vscode.executeCompletionItemProvider",
        uri,
        position,
        "."
      );
      const items = Array.isArray(result)
        ? result
        : result && Array.isArray(result.items)
          ? result.items
          : [];
      const athenaItems = items
        .filter(item => {
          if (typeof item.label === "object" && typeof item.label.description === "string") {
            return item.label.description.startsWith("Athena");
          }
          return typeof item.detail === "string" && item.detail.startsWith("Athena ");
        })
        .map(item => typeof item.label === "string"
          ? item.label
          : `${item.label.label}${item.label.detail || ""} ${item.label.description || ""}`.trim())
        .slice(0, 10);
      appendLog(
        `probe ${probe.name}: total=${items.length} athena=${athenaItems.length} labels=${athenaItems.join(",")}`
      );
    } catch (error) {
      appendLog(`probe ${probe.name}: error=${error && error.stack ? error.stack : error}`);
    }
  }
}

async function findFirstWorkspaceFile(vscode, patterns) {
  for (const pattern of patterns) {
    const matches = await vscode.workspace.findFiles(pattern, WORKSPACE_EXCLUDE_GLOB, 1);
    if (matches.length > 0) {
      return matches[0];
    }
  }
  return null;
}

async function openWorkspaceFile(vscode, patterns, label) {
  const target = await findFirstWorkspaceFile(vscode, patterns);
  if (!target) {
    void vscode.window.showWarningMessage(`Could not find ${label} in the current workspace.`);
    return;
  }
  const document = await vscode.workspace.openTextDocument(target);
  await vscode.window.showTextDocument(document, { preview: false });
}

function requireActiveJavaEditor(vscode, actionLabel) {
  const editor = vscode.window.activeTextEditor;
  if (!editor || editor.document.languageId !== "java") {
    void vscode.window.showWarningMessage(
      `${actionLabel} needs an active Java editor. Open a Java file and place the cursor first.`
    );
    return null;
  }
  return editor;
}

async function insertJavaSnippet(vscode, snippet, actionLabel) {
  const editor = requireActiveJavaEditor(vscode, actionLabel);
  if (!editor) {
    return;
  }
  await editor.insertSnippet(new vscode.SnippetString(snippet));
}

function createMechanismConfigSnippet() {
  return [
    "MechanismConfig<StatefulMechanism<${1:StateEnum}>> ${2:Mechanism}_CONFIG = MechanismConfig.stateMachineGeneric(${1:StateEnum}.${3:Off})",
    "    .named(\"${2:Mechanism}\")",
    "    .motors(m -> m",
    "        .add(${4:AthenaMotor.KRAKEN_X60}, ${5:1})",
    "        .neutralMode(${6:MotorNeutralMode.Brake}))",
    "    .encoders(e -> e",
    "        .add(\"main\", src -> src",
    "            .encoder(${7:AthenaEncoder.INTERNAL}, ${8:1})",
    "            .gearRatio(${9:1d})",
    "            .offset(${10:0d})))",
    "    .control(c -> c",
    "        .setpointAsOutput(${11:true}))",
    "    ;"
  ].join("\n");
}

function createAthenaStateEnumSnippet() {
  return [
    "@AthenaState(${1:Double}.class)",
    "enum ${2:MechanismState} {",
    "    ${3:Off}(0d),",
    "    ${4:Active}(s -> s",
    "        .manualPercent(${5:0.2})",
    "        .then(${3:Off}));",
    "}"
  ].join("\n");
}

function createAthenaStateEntrySnippet() {
  return [
    "${1:StateName}(s -> s",
    "    .manualPercent(${2:0.0})",
    "    .until(ctx -> ctx.limitSwitch(${3:0}))",
    "    .then(${4:NextState})),"
  ].join("\n");
}

function createAthenaTooltip(vscode, title, description) {
  return new vscode.MarkdownString(`**${title}**\n\n${description}`);
}

class AthenaSectionItem {
  constructor(vscode, label, description, iconId, children) {
    this.label = label;
    this.description = description;
    this.children = children;
    this.collapsibleState = vscode.TreeItemCollapsibleState.Expanded;
    this.iconPath = new vscode.ThemeIcon(iconId);
    this.contextValue = "athenaSection";
  }
}

class AthenaActionItem {
  constructor(vscode, label, description, command, iconId, tooltip) {
    this.label = label;
    this.description = description;
    this.collapsibleState = vscode.TreeItemCollapsibleState.None;
    this.command = command;
    this.iconPath = new vscode.ThemeIcon(iconId);
    this.tooltip = createAthenaTooltip(vscode, label, tooltip || description || label);
    this.contextValue = "athenaAction";
  }
}

class AthenaSidebarProvider {
  constructor(vscode) {
    this.vscode = vscode;
    this._onDidChangeTreeData = new vscode.EventEmitter();
    this.onDidChangeTreeData = this._onDidChangeTreeData.event;
  }

  refresh() {
    this._onDidChangeTreeData.fire();
  }

  getTreeItem(element) {
    return element;
  }

  getChildren(element) {
    if (element && Array.isArray(element.children)) {
      return element.children;
    }

    return [
      new AthenaSectionItem(
        this.vscode,
        "Jump To",
        "Open key robot files",
        "file-directory",
        this.buildJumpToItems()
      ),
      new AthenaSectionItem(
        this.vscode,
        "Create",
        "Insert Athena code templates",
        "new-file",
        this.buildCreateItems()
      ),
      new AthenaSectionItem(
        this.vscode,
        "Tools",
        "Workspace helpers",
        "tools",
        this.buildToolItems()
      )
    ];
  }

  buildJumpToItems() {
    return [
      new AthenaActionItem(
        this.vscode,
        "Constants.java",
        "Mechanism configs and wiring",
        { command: COMMANDS.openConstants, title: "Open Constants.java" },
        "file-code",
        "Open frc.robot.Constants.java."
      ),
      new AthenaActionItem(
        this.vscode,
        "States.java",
        "State enums and transitions",
        { command: COMMANDS.openStates, title: "Open States.java" },
        "symbol-enum",
        "Open frc.robot.States.java."
      ),
      new AthenaActionItem(
        this.vscode,
        "Robot.java",
        "Robot lifecycle entry point",
        { command: COMMANDS.openRobot, title: "Open Robot.java" },
        "symbol-class",
        "Open frc.robot.Robot.java."
      ),
      new AthenaActionItem(
        this.vscode,
        "Main.java",
        "Robot bootstrap",
        { command: COMMANDS.openMain, title: "Open Main.java" },
        "play",
        "Open frc.robot.Main.java."
      ),
      new AthenaActionItem(
        this.vscode,
        "build.gradle",
        "Project build config",
        { command: COMMANDS.openBuildGradle, title: "Open build.gradle" },
        "gear",
        "Open the robot build.gradle file."
      ),
      new AthenaActionItem(
        this.vscode,
        "Athena Vendordep",
        "Core vendordep descriptor",
        { command: COMMANDS.openVendordep, title: "Open Athena vendordep" },
        "symbol-object",
        "Open vendordeps/FRC6390-Athena-Core.json."
      )
    ];
  }

  buildCreateItems() {
    return [
      new AthenaActionItem(
        this.vscode,
        "MechanismConfig Template",
        "Insert a fluent Athena mechanism block",
        { command: COMMANDS.insertMechanismConfig, title: "Insert MechanismConfig Template" },
        "symbol-field",
        "Insert a starter MechanismConfig block into the active Java editor."
      ),
      new AthenaActionItem(
        this.vscode,
        "State Enum Template",
        "Insert an @AthenaState enum shell",
        { command: COMMANDS.insertAthenaStateEnum, title: "Insert Athena State Enum Template" },
        "symbol-enum",
        "Insert a starter Athena state enum into the active Java editor."
      ),
      new AthenaActionItem(
        this.vscode,
        "State Entry Template",
        "Insert a single Athena state entry",
        { command: COMMANDS.insertAthenaStateEntry, title: "Insert Athena State Entry Template" },
        "add",
        "Insert a single state entry into the active Java editor."
      )
    ];
  }

  buildToolItems() {
    return [
      new AthenaActionItem(
        this.vscode,
        "Run athenaDoctor",
        "Open a terminal health check",
        { command: COMMANDS.runAthenaDoctor, title: "Run athenaDoctor" },
        "tools",
        "Run ./gradlew athenaDoctor in a VS Code terminal."
      )
    ];
  }
}

function activate(extensionContext) {
  const vscode = require("vscode");
  const sidebarProvider = new AthenaSidebarProvider(vscode);
  const provider = {
    provideCompletionItems(document, position) {
      const items = provideAthenaCompletionItems(vscode, document, position);
      if (process.env.ATHENA_INTELLISENSE_LOCAL_TRACE === "1") {
        const count = Array.isArray(items) ? items.length : 0;
        appendLog(
          `provider ${path.basename(document.fileName)}:${position.line + 1}:${position.character + 1} count=${count}`
        );
      }
      return items;
    }
  };

  extensionContext.subscriptions.push(
    vscode.languages.registerCompletionItemProvider(
      [
        { language: "java", scheme: "file" },
        { language: "java", scheme: "untitled" }
      ],
      provider,
      "."
    )
  );

  extensionContext.subscriptions.push(
    vscode.window.registerTreeDataProvider("athenaRobot", sidebarProvider)
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.refreshSidebar, () => sidebarProvider.refresh())
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openConstants, () =>
      openWorkspaceFile(
        vscode,
        ["src/main/java/frc/robot/Constants.java", "**/src/main/java/**/Constants.java"],
        "Constants.java"
      )
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openStates, () =>
      openWorkspaceFile(
        vscode,
        ["src/main/java/frc/robot/States.java", "**/src/main/java/**/States.java"],
        "States.java"
      )
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openRobot, () =>
      openWorkspaceFile(
        vscode,
        ["src/main/java/frc/robot/Robot.java", "**/src/main/java/**/Robot.java"],
        "Robot.java"
      )
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openMain, () =>
      openWorkspaceFile(
        vscode,
        ["src/main/java/frc/robot/Main.java", "**/src/main/java/**/Main.java"],
        "Main.java"
      )
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openBuildGradle, () =>
      openWorkspaceFile(vscode, ["build.gradle", "build.gradle.kts"], "build.gradle")
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.openVendordep, () =>
      openWorkspaceFile(
        vscode,
        ["vendordeps/FRC6390-Athena-Core.json", "**/vendordeps/FRC6390-Athena-Core.json"],
        "the Athena vendordep"
      )
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.insertMechanismConfig, () =>
      insertJavaSnippet(vscode, createMechanismConfigSnippet(), "MechanismConfig template")
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.insertAthenaStateEnum, () =>
      insertJavaSnippet(vscode, createAthenaStateEnumSnippet(), "Athena state enum template")
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.insertAthenaStateEntry, () =>
      insertJavaSnippet(vscode, createAthenaStateEntrySnippet(), "Athena state entry template")
    )
  );

  extensionContext.subscriptions.push(
    vscode.commands.registerCommand(COMMANDS.runAthenaDoctor, () => {
      const terminal = vscode.window.createTerminal("Athena Doctor");
      terminal.show(true);
      terminal.sendText("./gradlew athenaDoctor", true);
    })
  );

  appendLog("extension activated");
  setTimeout(() => {
    runCompletionProbe(vscode).catch(error => {
      appendLog(`probe runner error=${error && error.stack ? error.stack : error}`);
    });
  }, 5000);
}

function deactivate() {}

module.exports = {
  activate,
  deactivate,
  __test: {
    detectAthenaContext,
    completionSpecsForContext
  }
};
