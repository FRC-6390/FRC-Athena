<script lang="ts">
  import { getContext } from 'svelte';
  import type { SignalRow } from '../lib/arcp';
  import type { DashboardWidget, WidgetConfigRecord } from '../lib/dashboard';
  import SignalMapField from './SignalMapField.svelte';
  import {
    addGraphSeries,
    addStatusMatrixSignal,
    formatDropdownOptions,
    parseDropdownOptionsInput,
    readBarConfig,
    readCameraOverlayConfig,
    readChoiceConfig,
    readControllerConfig,
    readDialConfig,
    readDioConfig,
    readDifferentialDriveConfig,
    readDropdownConfig,
    readFieldConfig,
    readGraphConfig,
    readImu3dConfig,
    readInputConfig,
    readLayoutGridConfig,
    readLayoutTitleConfig,
    readMotorConfig,
    readStateMachineConfig,
    readEncoderConfig,
    readImuConfig,
    readMech2dConfig,
    readSwerveDriveConfig,
    readSwerveModuleConfig,
    readStatusMatrixConfig,
    readTextAreaConfig,
    readTimerConfig,
    readToggleConfig,
    type ControllerParam,
    type ControllerWidgetConfig,
    type DioWidgetConfig,
    type EncoderWidgetConfig,
    type ChoiceWidgetCommit,
    type ChoiceWidgetDirection,
    type GraphSeriesStyle,
    type HtmlInputType,
    type InputWidgetCommit,
    type ImuWidgetConfig,
    type MotorWidgetConfig,
    type StateMachineWidgetConfig,
    type TextAreaWidgetCommit,
    type ToggleWidgetStyle
  } from '../lib/widget-config';

  type Props = {
    selectedWidget: DashboardWidget;
    selectedSignal: SignalRow | null;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    onUpdateConfig: (widgetId: string, config: WidgetConfigRecord | undefined) => void;
  };

  type SignalMapRequest = {
    title: string;
    candidates: SignalRow[];
    selectedSignalId: number | null;
    allowNone: boolean;
    noneLabel: string;
    onPick: (signalId: number | null) => void;
  };

  let { selectedWidget, selectedSignal, signals, signalById, onUpdateConfig }: Props = $props();

  let dropdownInput = $state('');
  let dropdownInputWidgetId = $state('');
  let choiceInput = $state('');
  let choiceInputWidgetId = $state('');

  const primarySignal = $derived(
    signalById.get(selectedWidget.signalId) ?? selectedSignal ?? signals[0] ?? null
  );

  const numericSignals = $derived(
    signals
      .filter((signal) => signal.signal_type === 'f64' || signal.signal_type === 'i64')
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const boolSignals = $derived(
    signals.filter((signal) => signal.signal_type === 'bool').sort((a, b) => a.path.localeCompare(b.path))
  );

  const stringSignals = $derived(
    signals
      .filter((signal) => signal.signal_type === 'string' || signal.signal_type === 'string[]')
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const writableNumericSignals = $derived(
    numericSignals.filter((signal) => signal.access === 'write')
  );

  const writableBoolSignals = $derived(
    boolSignals.filter((signal) => signal.access === 'write')
  );

  const vectorSignals = $derived(
    signals
      .filter((signal) => signal.signal_type === 'f64[]' || signal.signal_type === 'i64[]')
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const actionSignals = $derived(
    signals
      .filter((signal) => signal.access === 'invoke' || signal.kind === 'command')
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const modeSignals = $derived(
    signals
      .filter(
        (signal) =>
          signal.signal_type === 'bool' || signal.signal_type === 'string' || signal.signal_type === 'string[]'
      )
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const writableMotorCommandSignals = $derived(
    writableNumericSignals.filter((signal) => {
      const token = normalizeToken(leafPath(signal.path));
      if (!token) return false;
      if (token.includes('currentlimit') || token.includes('limitamps')) return false;
      if (token.includes('canid')) return false;
      return (
        token.includes('command') ||
        token.includes('setpoint') ||
        token.includes('target') ||
        token.includes('output') ||
        token.includes('percent') ||
        token.includes('duty') ||
        token.includes('voltage') ||
        token.includes('volt') ||
        token.includes('velocity') ||
        token.includes('speed') ||
        token.includes('rpm')
      );
    })
  );

  const preferredMotorCommandSignals = $derived(
    writableMotorCommandSignals.length > 0 ? writableMotorCommandSignals : writableNumericSignals
  );

  const requestSignalMap =
    getContext<((request: SignalMapRequest) => void) | undefined>('arcp.signal-map-request');

  function setConfig(config: WidgetConfigRecord | undefined) {
    onUpdateConfig(selectedWidget.id, config);
  }

  function isNumeric(signal: SignalRow | null): signal is SignalRow {
    if (!signal) return false;
    return signal.signal_type === 'f64' || signal.signal_type === 'i64';
  }

  function isWritableNumeric(signal: SignalRow | null): signal is SignalRow {
    return !!signal && isNumeric(signal) && signal.access === 'write';
  }

  function leafPath(path: string): string {
    const parts = path.split('/').filter(Boolean);
    return parts[parts.length - 1] ?? path;
  }

  function normalizeToken(value: string): string {
    return value.toLowerCase().replace(/[^a-z0-9]+/g, '');
  }

  function defaultSignalIdFor(candidates: SignalRow[]): number | null {
    if (selectedSignal && candidates.some((entry) => entry.signal_id === selectedSignal.signal_id)) {
      return selectedSignal.signal_id;
    }
    return candidates[0]?.signal_id ?? null;
  }

  function toggleSignalMapping(
    currentSignalId: number | null,
    enabled: boolean,
    candidates: SignalRow[]
  ): number | null {
    if (!enabled) return null;
    return currentSignalId ?? defaultSignalIdFor(candidates);
  }

  function normalizeControllerKey(raw: string): string {
    const normalized = raw
      .trim()
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, '_')
      .replace(/^_+|_+$/g, '');
    return normalized || 'value';
  }

  function nextControllerKey(existing: ControllerParam[], preferred: string): string {
    const base = normalizeControllerKey(preferred);
    const used = new Set(existing.map((entry) => normalizeControllerKey(entry.key)));
    if (!used.has(base)) return base;
    let index = 2;
    while (used.has(`${base}_${index}`)) {
      index += 1;
    }
    return `${base}_${index}`;
  }

  function updateControllerParam(index: number, patch: Partial<ControllerParam>) {
    if (!primarySignal) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    setConfig({
      ...controller,
      params: controller.params.map((entry, entryIndex) =>
        entryIndex === index ? { ...entry, ...patch } : entry
      )
    } satisfies ControllerWidgetConfig);
  }

  function removeControllerParam(index: number) {
    if (!primarySignal) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    if (controller.params.length <= 1) return;
    setConfig({
      ...controller,
      params: controller.params.filter((_, entryIndex) => entryIndex !== index)
    } satisfies ControllerWidgetConfig);
  }

  function addControllerParam(signal: SignalRow) {
    if (!primarySignal || !isWritableNumeric(signal)) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    if (controller.params.some((entry) => entry.signalId === signal.signal_id)) return;
    const nextLabel = leafPath(signal.path);
    setConfig({
      ...controller,
      params: [
        ...controller.params,
        {
          key: nextControllerKey(controller.params, nextLabel),
          label: nextLabel,
          signalId: signal.signal_id
        }
      ]
    } satisfies ControllerWidgetConfig);
  }

  function addSelectedControllerParam() {
    if (!isWritableNumeric(selectedSignal)) return;
    addControllerParam(selectedSignal);
  }

  function requestGraphSignalFromExplorer() {
    if (!primarySignal) return;
    if (!requestSignalMap) {
      addSelectedSignalToGraph();
      return;
    }
    requestSignalMap({
      title: 'Add graph series',
      candidates: numericSignals,
      selectedSignalId: isNumeric(selectedSignal) ? selectedSignal.signal_id : null,
      allowNone: false,
      noneLabel: 'Required',
      onPick: (signalId) => {
        if (signalId === null) return;
        const signal = signalById.get(signalId);
        if (!isNumeric(signal)) return;
        setConfig(addGraphSeries(selectedWidget.config, primarySignal, signal));
      }
    });
  }

  function requestControllerSignalFromExplorer() {
    if (!requestSignalMap) {
      addSelectedControllerParam();
      return;
    }
    requestSignalMap({
      title: 'Add controller parameter signal',
      candidates: writableNumericSignals,
      selectedSignalId: isWritableNumeric(selectedSignal) ? selectedSignal.signal_id : null,
      allowNone: false,
      noneLabel: 'Required',
      onPick: (signalId) => {
        if (signalId === null) return;
        const signal = signalById.get(signalId);
        if (!isWritableNumeric(signal)) return;
        addControllerParam(signal);
      }
    });
  }

  function updateGraphSeriesStyle(signalId: number, style: GraphSeriesStyle) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...graph,
      series: graph.series.map((series) =>
        series.signalId === signalId ? { ...series, style } : series
      )
    });
  }

  function updateGraphSeriesColor(signalId: number, color: string) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...graph,
      series: graph.series.map((series) =>
        series.signalId === signalId ? { ...series, color } : series
      )
    });
  }

  function removeGraphSeries(signalId: number) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    if (graph.series.length <= 1) return;
    setConfig({
      ...graph,
      series: graph.series.filter((series) => series.signalId !== signalId)
    });
  }

  function addSelectedSignalToGraph() {
    if (!primarySignal || !isNumeric(selectedSignal)) return;
    setConfig(addGraphSeries(selectedWidget.config, primarySignal, selectedSignal));
  }

  function addSelectedBoolToMatrix() {
    if (!primarySignal || !selectedSignal) return;
    if (selectedSignal.signal_type !== 'bool') return;
    setConfig(addStatusMatrixSignal(selectedWidget.config, primarySignal, signals, selectedSignal));
  }

  function requestStatusMatrixSignalFromExplorer() {
    if (!primarySignal) return;
    if (!requestSignalMap) {
      addSelectedBoolToMatrix();
      return;
    }
    requestSignalMap({
      title: 'Add status board signal',
      candidates: boolSignals,
      selectedSignalId: selectedSignal?.signal_type === 'bool' ? selectedSignal.signal_id : null,
      allowNone: false,
      noneLabel: 'Required',
      onPick: (signalId) => {
        if (signalId === null) return;
        const signal = signalById.get(signalId);
        if (!signal || signal.signal_type !== 'bool') return;
        setConfig(addStatusMatrixSignal(selectedWidget.config, primarySignal, signals, signal));
      }
    });
  }

  $effect(() => {
    if (selectedWidget.kind !== 'dropdown') return;
    if (!primarySignal) return;
    if (dropdownInputWidgetId === selectedWidget.id) return;
    const dropdown = readDropdownConfig(selectedWidget.config, primarySignal);
    dropdownInputWidgetId = selectedWidget.id;
    dropdownInput = formatDropdownOptions(dropdown.options);
  });

  function commitDropdownOptions() {
    if (!primarySignal) return;
    const dropdown = readDropdownConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...dropdown,
      options: parseDropdownOptionsInput(dropdownInput)
    });
  }

  $effect(() => {
    if ((selectedWidget.kind !== 'button_group' && selectedWidget.kind !== 'radio') || !primarySignal) return;
    if (choiceInputWidgetId === selectedWidget.id) return;
    const choice = readChoiceConfig(selectedWidget.config, primarySignal);
    choiceInputWidgetId = selectedWidget.id;
    choiceInput = formatDropdownOptions(choice.options);
  });

  function commitChoiceOptions() {
    if (!primarySignal) return;
    const choice = readChoiceConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...choice,
      options: parseDropdownOptionsInput(choiceInput)
    });
  }
</script>

{#if !primarySignal && selectedWidget.kind !== 'layout_list' && selectedWidget.kind !== 'layout_grid' && selectedWidget.kind !== 'layout_divider' && selectedWidget.kind !== 'layout_spacer' && selectedWidget.kind !== 'layout_section' && selectedWidget.kind !== 'layout_title'}
  <p class="empty">Config unavailable: no signal bound to this widget.</p>
{:else if selectedWidget.kind === 'graph' && primarySignal}
  {@const graph = readGraphConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Graph</h4>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={graph.showLegend}
        onchange={(event) => {
          setConfig({ ...graph, showLegend: (event.currentTarget as HTMLInputElement).checked });
        }}
      />
      Show legend
    </label>

    <div class="row-grid">
      <label>
        Y Min
        <input
          type="number"
          value={graph.yMin ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value;
            const value = raw.trim() === '' ? null : Number(raw);
            setConfig({ ...graph, yMin: Number.isFinite(value) ? value : null });
          }}
        />
      </label>
      <label>
        Y Max
        <input
          type="number"
          value={graph.yMax ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value;
            const value = raw.trim() === '' ? null : Number(raw);
            setConfig({ ...graph, yMax: Number.isFinite(value) ? value : null });
          }}
        />
      </label>
    </div>

    <button class="btn" disabled={numericSignals.length === 0} onclick={requestGraphSignalFromExplorer}>
      Add signal from explorer
    </button>

    <div class="series-list">
      {#each graph.series as series (series.signalId)}
        {@const bound = signalById.get(series.signalId)}
        <div class="series-row">
          <strong>{bound?.path ?? `Signal ${series.signalId}`}</strong>
          <select
            value={series.style}
            onchange={(event) =>
              updateGraphSeriesStyle(
                series.signalId,
                (event.currentTarget as HTMLSelectElement).value as GraphSeriesStyle
              )}
          >
            <option value="line">Line</option>
            <option value="step">Step</option>
            <option value="dot">Dot</option>
          </select>
          <input
            type="color"
            value={series.color}
            oninput={(event) =>
              updateGraphSeriesColor(series.signalId, (event.currentTarget as HTMLInputElement).value)}
          />
          <button class="btn btn-danger" disabled={graph.series.length <= 1} onclick={() => removeGraphSeries(series.signalId)}>
            Remove
          </button>
        </div>
      {/each}
    </div>
  </section>
{:else if selectedWidget.kind === 'controller' && primarySignal}
  {@const controller = readControllerConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>PID/FF Editor</h4>
    <p class="hint">Auto-maps writable siblings like kP/kI/kD/kS/kV/kA.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readControllerConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>

    <button
      class="btn"
      disabled={writableNumericSignals.length === 0}
      onclick={requestControllerSignalFromExplorer}
    >
      Add writable signal from explorer
    </button>

    <div class="series-list">
      {#each controller.params as param, index (`${param.key}-${param.signalId}-${index}`)}
        <div class="series-row series-row-map">
          <input
            value={param.label}
            aria-label={`Parameter ${index + 1} label`}
            oninput={(event) =>
              updateControllerParam(index, { label: (event.currentTarget as HTMLInputElement).value })}
          />
          <input
            value={param.key}
            aria-label={`Parameter ${index + 1} key`}
            oninput={(event) =>
              updateControllerParam(index, {
                key: normalizeControllerKey((event.currentTarget as HTMLInputElement).value)
              })}
          />
          <SignalMapField
            label={`Parameter ${index + 1} signal`}
            selectedSignalId={param.signalId}
            candidates={writableNumericSignals}
            {signalById}
            allowNone={false}
            noneLabel="Required"
            onChange={(signalId) => {
              if (signalId === null) return;
              updateControllerParam(index, { signalId });
            }}
          />
          <button class="btn btn-danger" disabled={controller.params.length <= 1} onclick={() => removeControllerParam(index)}>
            Remove
          </button>
        </div>
      {/each}
    </div>

    <details class="config-collapsible">
      <summary>Autotuner</summary>
      <label class="check-row">
        <input
          type="checkbox"
          checked={controller.autotune.enabled}
          onchange={(event) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                enabled: (event.currentTarget as HTMLInputElement).checked
              }
            } satisfies ControllerWidgetConfig)}
        />
        Enable autotuner controls
      </label>
      <div class="row-grid">
        <SignalMapField
          label="Mode signal"
          selectedSignalId={controller.autotune.modeSignalId}
          candidates={modeSignals}
          {signalById}
          optional={true}
          enabled={controller.autotune.modeSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                modeSignalId: toggleSignalMapping(
                  controller.autotune.modeSignalId,
                  enabled,
                  modeSignals
                )
              }
            } satisfies ControllerWidgetConfig)}
          onChange={(signalId) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                modeSignalId: signalId
              }
            } satisfies ControllerWidgetConfig)}
        />
        <SignalMapField
          label="Status signal"
          selectedSignalId={controller.autotune.statusSignalId}
          candidates={signals}
          {signalById}
          optional={true}
          enabled={controller.autotune.statusSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                statusSignalId: toggleSignalMapping(
                  controller.autotune.statusSignalId,
                  enabled,
                  signals
                )
              }
            } satisfies ControllerWidgetConfig)}
          onChange={(signalId) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                statusSignalId: signalId
              }
            } satisfies ControllerWidgetConfig)}
        />
        <SignalMapField
          label="Result signal"
          selectedSignalId={controller.autotune.resultSignalId}
          candidates={signals}
          {signalById}
          optional={true}
          enabled={controller.autotune.resultSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                resultSignalId: toggleSignalMapping(
                  controller.autotune.resultSignalId,
                  enabled,
                  signals
                )
              }
            } satisfies ControllerWidgetConfig)}
          onChange={(signalId) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                resultSignalId: signalId
              }
            } satisfies ControllerWidgetConfig)}
        />
        <SignalMapField
          label="Start action"
          selectedSignalId={controller.autotune.startActionSignalId}
          candidates={actionSignals}
          {signalById}
          optional={true}
          enabled={controller.autotune.startActionSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                startActionSignalId: toggleSignalMapping(
                  controller.autotune.startActionSignalId,
                  enabled,
                  actionSignals
                )
              }
            } satisfies ControllerWidgetConfig)}
          onChange={(signalId) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                startActionSignalId: signalId
              }
            } satisfies ControllerWidgetConfig)}
        />
        <SignalMapField
          label="Stop action"
          selectedSignalId={controller.autotune.stopActionSignalId}
          candidates={actionSignals}
          {signalById}
          optional={true}
          enabled={controller.autotune.stopActionSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                stopActionSignalId: toggleSignalMapping(
                  controller.autotune.stopActionSignalId,
                  enabled,
                  actionSignals
                )
              }
            } satisfies ControllerWidgetConfig)}
          onChange={(signalId) =>
            setConfig({
              ...controller,
              autotune: {
                ...controller.autotune,
                stopActionSignalId: signalId
              }
            } satisfies ControllerWidgetConfig)}
        />
      </div>
    </details>
  </section>
{:else if selectedWidget.kind === 'state_machine' && primarySignal}
  {@const stateMachine = readStateMachineConfig(selectedWidget.config, primarySignal, signals)}
  {@const mappedCount = [
    stateMachine.currentStateSignalId,
    stateMachine.goalStateSignalId,
    stateMachine.nextStateSignalId,
    stateMachine.queueSignalId,
    stateMachine.atGoalSignalId,
    stateMachine.transitionCountSignalId,
    stateMachine.lastTransitionSignalId,
    stateMachine.availableStatesSignalId,
    stateMachine.clearQueueCommandSignalId
  ].filter((id) => id !== null).length}
  <section class="config-block">
    <h4>State Machine</h4>
    <p class="hint">Maps state telemetry, writable toggles, and command actions into one panel.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readStateMachineConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="chips">
      <span>{mappedCount} mapped fields</span>
      {#if stateMachine.goalStateSignalId !== null}<span>goal setpoint</span>{/if}
      {#if stateMachine.clearQueueCommandSignalId !== null}<span>clear queue</span>{/if}
    </div>
    <div class="row-grid">
      <SignalMapField
        label="Current state signal"
        selectedSignalId={stateMachine.currentStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.currentStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            currentStateSignalId: toggleSignalMapping(stateMachine.currentStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, currentStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Goal state signal"
        selectedSignalId={stateMachine.goalStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.goalStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            goalStateSignalId: toggleSignalMapping(stateMachine.goalStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, goalStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Next state signal"
        selectedSignalId={stateMachine.nextStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.nextStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            nextStateSignalId: toggleSignalMapping(stateMachine.nextStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, nextStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Queue signal"
        selectedSignalId={stateMachine.queueSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.queueSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            queueSignalId: toggleSignalMapping(stateMachine.queueSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, queueSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="At-goal signal"
        selectedSignalId={stateMachine.atGoalSignalId}
        candidates={boolSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.atGoalSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            atGoalSignalId: toggleSignalMapping(stateMachine.atGoalSignalId, enabled, boolSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, atGoalSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Transition-count signal"
        selectedSignalId={stateMachine.transitionCountSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.transitionCountSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            transitionCountSignalId: toggleSignalMapping(
              stateMachine.transitionCountSignalId,
              enabled,
              numericSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, transitionCountSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Last-transition signal"
        selectedSignalId={stateMachine.lastTransitionSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.lastTransitionSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            lastTransitionSignalId: toggleSignalMapping(
              stateMachine.lastTransitionSignalId,
              enabled,
              stringSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, lastTransitionSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Available-states signal"
        selectedSignalId={stateMachine.availableStatesSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.availableStatesSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            availableStatesSignalId: toggleSignalMapping(
              stateMachine.availableStatesSignalId,
              enabled,
              stringSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, availableStatesSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Clear-queue action"
        selectedSignalId={stateMachine.clearQueueCommandSignalId}
        candidates={actionSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.clearQueueCommandSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            clearQueueCommandSignalId: toggleSignalMapping(
              stateMachine.clearQueueCommandSignalId,
              enabled,
              actionSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, clearQueueCommandSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
    </div>
  </section>
{:else if selectedWidget.kind === 'motor' && primarySignal}
  {@const motor = readMotorConfig(selectedWidget.config, primarySignal, signals)}
  {@const mappedCount = [
    motor.canIdSignalId,
    motor.canbusSignalId,
    motor.typeSignalId,
    motor.connectedSignalId,
    motor.stalledSignalId,
    motor.neutralModeSignalId,
    motor.currentLimitSignalId,
    motor.invertedSignalId,
    motor.outputSignalId,
    motor.velocitySignalId,
    motor.positionSignalId,
    motor.currentSignalId,
    motor.temperatureSignalId,
    motor.voltageSignalId,
    motor.commandSignalId
  ].filter((id) => id !== null).length}
  <section class="config-block">
    <h4>Motor</h4>
    <p class="hint">Auto-maps identity, status, telemetry, and writable control siblings.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readMotorConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="chips">
      <span>{mappedCount} mapped fields</span>
      {#if motor.commandSignalId !== null}<span>output command</span>{/if}
      {#if motor.currentLimitSignalId !== null}<span>current limit</span>{/if}
      {#if motor.invertedSignalId !== null}<span>inverted</span>{/if}
      {#if motor.neutralModeSignalId !== null}<span>neutral mode</span>{/if}
    </div>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={motor.showOtherFields}
        onchange={(event) =>
          setConfig({
            ...motor,
            showOtherFields: (event.currentTarget as HTMLInputElement).checked
          } satisfies MotorWidgetConfig)}
      />
      Show "Other Motor Fields" panel
    </label>
    <div class="row-grid">
      <SignalMapField
        label="Output signal"
        selectedSignalId={motor.outputSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.outputSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            outputSignalId: toggleSignalMapping(motor.outputSignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, outputSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Command signal"
        selectedSignalId={motor.commandSignalId}
        candidates={preferredMotorCommandSignals}
        {signalById}
        optional={true}
        enabled={motor.commandSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            commandSignalId: toggleSignalMapping(
              motor.commandSignalId,
              enabled,
              preferredMotorCommandSignals
            )
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, commandSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Velocity signal"
        selectedSignalId={motor.velocitySignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.velocitySignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            velocitySignalId: toggleSignalMapping(motor.velocitySignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, velocitySignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Position signal"
        selectedSignalId={motor.positionSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.positionSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            positionSignalId: toggleSignalMapping(motor.positionSignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, positionSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Current signal"
        selectedSignalId={motor.currentSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.currentSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            currentSignalId: toggleSignalMapping(motor.currentSignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, currentSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Temperature signal"
        selectedSignalId={motor.temperatureSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.temperatureSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            temperatureSignalId: toggleSignalMapping(motor.temperatureSignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, temperatureSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Voltage signal"
        selectedSignalId={motor.voltageSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={motor.voltageSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            voltageSignalId: toggleSignalMapping(motor.voltageSignalId, enabled, numericSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, voltageSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Connected signal"
        selectedSignalId={motor.connectedSignalId}
        candidates={boolSignals}
        {signalById}
        optional={true}
        enabled={motor.connectedSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            connectedSignalId: toggleSignalMapping(motor.connectedSignalId, enabled, boolSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, connectedSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Neutral mode signal"
        selectedSignalId={motor.neutralModeSignalId}
        candidates={modeSignals}
        {signalById}
        optional={true}
        enabled={motor.neutralModeSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            neutralModeSignalId: toggleSignalMapping(motor.neutralModeSignalId, enabled, modeSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, neutralModeSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Current limit signal"
        selectedSignalId={motor.currentLimitSignalId}
        candidates={writableNumericSignals}
        {signalById}
        optional={true}
        enabled={motor.currentLimitSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            currentLimitSignalId: toggleSignalMapping(
              motor.currentLimitSignalId,
              enabled,
              writableNumericSignals
            )
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, currentLimitSignalId: signalId } satisfies MotorWidgetConfig)}
      />
      <SignalMapField
        label="Inverted signal"
        selectedSignalId={motor.invertedSignalId}
        candidates={boolSignals}
        {signalById}
        optional={true}
        enabled={motor.invertedSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...motor,
            invertedSignalId: toggleSignalMapping(motor.invertedSignalId, enabled, boolSignals)
          } satisfies MotorWidgetConfig)}
        onChange={(signalId) => setConfig({ ...motor, invertedSignalId: signalId } satisfies MotorWidgetConfig)}
      />
    </div>

    <details class="config-collapsible">
      <summary>Other motor fields</summary>
      <div class="row-grid">
        <SignalMapField
          label="CAN ID signal"
          selectedSignalId={motor.canIdSignalId}
          candidates={numericSignals}
          {signalById}
          optional={true}
          enabled={motor.canIdSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...motor,
              canIdSignalId: toggleSignalMapping(motor.canIdSignalId, enabled, numericSignals)
            } satisfies MotorWidgetConfig)}
          onChange={(signalId) => setConfig({ ...motor, canIdSignalId: signalId } satisfies MotorWidgetConfig)}
        />
        <SignalMapField
          label="CAN bus signal"
          selectedSignalId={motor.canbusSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={motor.canbusSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...motor,
              canbusSignalId: toggleSignalMapping(motor.canbusSignalId, enabled, stringSignals)
            } satisfies MotorWidgetConfig)}
          onChange={(signalId) => setConfig({ ...motor, canbusSignalId: signalId } satisfies MotorWidgetConfig)}
        />
        <SignalMapField
          label="Type signal"
          selectedSignalId={motor.typeSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={motor.typeSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...motor,
              typeSignalId: toggleSignalMapping(motor.typeSignalId, enabled, stringSignals)
            } satisfies MotorWidgetConfig)}
          onChange={(signalId) => setConfig({ ...motor, typeSignalId: signalId } satisfies MotorWidgetConfig)}
        />
        <SignalMapField
          label="Stalled signal"
          selectedSignalId={motor.stalledSignalId}
          candidates={boolSignals}
          {signalById}
          optional={true}
          enabled={motor.stalledSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...motor,
              stalledSignalId: toggleSignalMapping(motor.stalledSignalId, enabled, boolSignals)
            } satisfies MotorWidgetConfig)}
          onChange={(signalId) => setConfig({ ...motor, stalledSignalId: signalId } satisfies MotorWidgetConfig)}
        />
      </div>
    </details>
  </section>
{:else if selectedWidget.kind === 'dio' && primarySignal}
  {@const dio = readDioConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>DIO</h4>
    <p class="hint">Maps digital inputs/outputs (buttons, beam breaks, switches, DIO channels).</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readDioConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <label class="check-row">
      <input
        type="checkbox"
        checked={dio.showOtherFields}
        onchange={(event) =>
          setConfig({ ...dio, showOtherFields: (event.currentTarget as HTMLInputElement).checked } satisfies DioWidgetConfig)}
      />
      Show Other DIO Fields
    </label>
    <div class="row-grid">
      <SignalMapField
        label="Value signal"
        selectedSignalId={dio.valueSignalId}
        candidates={boolSignals}
        {signalById}
        optional={true}
        enabled={dio.valueSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            valueSignalId: toggleSignalMapping(dio.valueSignalId, enabled, boolSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, valueSignalId: signalId } satisfies DioWidgetConfig)}
      />
      <SignalMapField
        label="Output signal"
        selectedSignalId={dio.outputSignalId}
        candidates={writableBoolSignals}
        {signalById}
        optional={true}
        enabled={dio.outputSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            outputSignalId: toggleSignalMapping(dio.outputSignalId, enabled, writableBoolSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, outputSignalId: signalId } satisfies DioWidgetConfig)}
      />
      <SignalMapField
        label="Inverted signal"
        selectedSignalId={dio.invertedSignalId}
        candidates={writableBoolSignals}
        {signalById}
        optional={true}
        enabled={dio.invertedSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            invertedSignalId: toggleSignalMapping(dio.invertedSignalId, enabled, writableBoolSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, invertedSignalId: signalId } satisfies DioWidgetConfig)}
      />
      <SignalMapField
        label="Channel signal"
        selectedSignalId={dio.channelSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={dio.channelSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            channelSignalId: toggleSignalMapping(dio.channelSignalId, enabled, numericSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, channelSignalId: signalId } satisfies DioWidgetConfig)}
      />
      <SignalMapField
        label="Port signal"
        selectedSignalId={dio.portSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={dio.portSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            portSignalId: toggleSignalMapping(dio.portSignalId, enabled, numericSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, portSignalId: signalId } satisfies DioWidgetConfig)}
      />
      <SignalMapField
        label="Mode signal"
        selectedSignalId={dio.modeSignalId}
        candidates={modeSignals}
        {signalById}
        optional={true}
        enabled={dio.modeSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...dio,
            modeSignalId: toggleSignalMapping(dio.modeSignalId, enabled, modeSignals)
          } satisfies DioWidgetConfig)}
        onChange={(signalId) => setConfig({ ...dio, modeSignalId: signalId } satisfies DioWidgetConfig)}
      />
    </div>

    <details class="config-collapsible">
      <summary>Other DIO fields</summary>
      <div class="row-grid">
        <SignalMapField
          label="Name signal"
          selectedSignalId={dio.nameSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={dio.nameSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...dio,
              nameSignalId: toggleSignalMapping(dio.nameSignalId, enabled, stringSignals)
            } satisfies DioWidgetConfig)}
          onChange={(signalId) => setConfig({ ...dio, nameSignalId: signalId } satisfies DioWidgetConfig)}
        />
        <SignalMapField
          label="Type signal"
          selectedSignalId={dio.typeSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={dio.typeSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...dio,
              typeSignalId: toggleSignalMapping(dio.typeSignalId, enabled, stringSignals)
            } satisfies DioWidgetConfig)}
          onChange={(signalId) => setConfig({ ...dio, typeSignalId: signalId } satisfies DioWidgetConfig)}
        />
      </div>
    </details>
  </section>
{:else if selectedWidget.kind === 'encoder' && primarySignal}
  {@const encoder = readEncoderConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Encoder</h4>
    <p class="hint">Auto-maps encoder telemetry, conversion fields, CAN identity, and pose-view settings.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readEncoderConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="row-grid">
      <SignalMapField
        label="Position signal"
        selectedSignalId={encoder.positionSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, positionSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Velocity signal"
        selectedSignalId={encoder.velocitySignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, velocitySignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Absolute signal"
        selectedSignalId={encoder.absoluteSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, absoluteSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Connected signal"
        selectedSignalId={encoder.connectedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, connectedSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Ratio signal"
        selectedSignalId={encoder.ratioSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, ratioSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Offset signal"
        selectedSignalId={encoder.offsetSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, offsetSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
    </div>

    <details class="config-collapsible">
      <summary>Other encoder fields</summary>
      <div class="row-grid">
        <SignalMapField
          label="CAN ID signal"
          selectedSignalId={encoder.canIdSignalId}
          candidates={numericSignals}
          {signalById}
          optional={true}
          enabled={encoder.canIdSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              canIdSignalId: toggleSignalMapping(encoder.canIdSignalId, enabled, numericSignals)
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) => setConfig({ ...encoder, canIdSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
        <SignalMapField
          label="CAN bus signal"
          selectedSignalId={encoder.canbusSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={encoder.canbusSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              canbusSignalId: toggleSignalMapping(encoder.canbusSignalId, enabled, stringSignals)
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) => setConfig({ ...encoder, canbusSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
        <SignalMapField
          label="Type signal"
          selectedSignalId={encoder.typeSignalId}
          candidates={stringSignals}
          {signalById}
          optional={true}
          enabled={encoder.typeSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              typeSignalId: toggleSignalMapping(encoder.typeSignalId, enabled, stringSignals)
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) => setConfig({ ...encoder, typeSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
        <SignalMapField
          label="Inverted signal"
          selectedSignalId={encoder.invertedSignalId}
          candidates={boolSignals}
          {signalById}
          optional={true}
          enabled={encoder.invertedSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              invertedSignalId: toggleSignalMapping(encoder.invertedSignalId, enabled, boolSignals)
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) => setConfig({ ...encoder, invertedSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
        <SignalMapField
          label="Supports simulation signal"
          selectedSignalId={encoder.supportsSimulationSignalId}
          candidates={boolSignals}
          {signalById}
          optional={true}
          enabled={encoder.supportsSimulationSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              supportsSimulationSignalId: toggleSignalMapping(
                encoder.supportsSimulationSignalId,
                enabled,
                boolSignals
              )
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) =>
            setConfig({ ...encoder, supportsSimulationSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
        <SignalMapField
          label="Raw absolute signal"
          selectedSignalId={encoder.rawAbsoluteSignalId}
          candidates={numericSignals}
          {signalById}
          optional={true}
          enabled={encoder.rawAbsoluteSignalId !== null}
          onToggleEnabled={(enabled) =>
            setConfig({
              ...encoder,
              rawAbsoluteSignalId: toggleSignalMapping(encoder.rawAbsoluteSignalId, enabled, numericSignals)
            } satisfies EncoderWidgetConfig)}
          onChange={(signalId) => setConfig({ ...encoder, rawAbsoluteSignalId: signalId } satisfies EncoderWidgetConfig)}
        />
      </div>
    </details>

    <details class="config-collapsible">
      <summary>Pose dial views</summary>
      <div class="row-grid">
        <label>
          Position view
          <select
            value={encoder.positionViewMode}
            onchange={(event) =>
              setConfig({
                ...encoder,
                positionViewMode: (event.currentTarget as HTMLSelectElement).value as EncoderWidgetConfig['positionViewMode']
              } satisfies EncoderWidgetConfig)}
          >
            <option value="continuous">Continuous turns</option>
            <option value="zero_to_one">0..1 turns</option>
            <option value="neg180_to_180">-180..180 deg</option>
            <option value="zero_to_360">0..360 deg</option>
            <option value="distance">Distance/custom</option>
          </select>
        </label>
        <label>
          Position unit
          <input
            value={encoder.positionUnit}
            oninput={(event) =>
              setConfig({ ...encoder, positionUnit: (event.currentTarget as HTMLInputElement).value } satisfies EncoderWidgetConfig)}
          />
        </label>
        <label>
          Position min
          <input
            type="number"
            value={encoder.positionMin}
            oninput={(event) => {
              const parsed = Number((event.currentTarget as HTMLInputElement).value);
              if (!Number.isFinite(parsed)) return;
              setConfig({ ...encoder, positionMin: parsed } satisfies EncoderWidgetConfig);
            }}
          />
        </label>
        <label>
          Position max
          <input
            type="number"
            value={encoder.positionMax}
            oninput={(event) => {
              const parsed = Number((event.currentTarget as HTMLInputElement).value);
              if (!Number.isFinite(parsed)) return;
              setConfig({ ...encoder, positionMax: parsed } satisfies EncoderWidgetConfig);
            }}
          />
        </label>

        <label>
          Absolute view
          <select
            value={encoder.absoluteViewMode}
            onchange={(event) =>
              setConfig({
                ...encoder,
                absoluteViewMode: (event.currentTarget as HTMLSelectElement).value as EncoderWidgetConfig['absoluteViewMode']
              } satisfies EncoderWidgetConfig)}
          >
            <option value="zero_to_one">0..1 turns</option>
            <option value="continuous">Continuous turns</option>
            <option value="neg180_to_180">-180..180 deg</option>
            <option value="zero_to_360">0..360 deg</option>
            <option value="distance">Distance/custom</option>
          </select>
        </label>
        <label>
          Absolute unit
          <input
            value={encoder.absoluteUnit}
            oninput={(event) =>
              setConfig({ ...encoder, absoluteUnit: (event.currentTarget as HTMLInputElement).value } satisfies EncoderWidgetConfig)}
          />
        </label>
        <label>
          Absolute min
          <input
            type="number"
            value={encoder.absoluteMin}
            oninput={(event) => {
              const parsed = Number((event.currentTarget as HTMLInputElement).value);
              if (!Number.isFinite(parsed)) return;
              setConfig({ ...encoder, absoluteMin: parsed } satisfies EncoderWidgetConfig);
            }}
          />
        </label>
        <label>
          Absolute max
          <input
            type="number"
            value={encoder.absoluteMax}
            oninput={(event) => {
              const parsed = Number((event.currentTarget as HTMLInputElement).value);
              if (!Number.isFinite(parsed)) return;
              setConfig({ ...encoder, absoluteMax: parsed } satisfies EncoderWidgetConfig);
            }}
          />
        </label>
      </div>
    </details>
  </section>
{:else if selectedWidget.kind === 'imu' && primarySignal}
  {@const imu = readImuConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>IMU</h4>
    <p class="hint">Auto-maps orientation, accel/gyro/mag vectors, plus CAN metadata controls.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readImuConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="row-grid">
      <SignalMapField
        label="Roll signal"
        selectedSignalId={imu.rollSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, rollSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Pitch signal"
        selectedSignalId={imu.pitchSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, pitchSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Yaw signal"
        selectedSignalId={imu.yawSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, yawSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Heading signal"
        selectedSignalId={imu.headingSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, headingSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Connected signal"
        selectedSignalId={imu.connectedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, connectedSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Accel vector signal"
        selectedSignalId={imu.accelSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, accelSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Gyro vector signal"
        selectedSignalId={imu.gyroSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, gyroSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Mag vector signal"
        selectedSignalId={imu.magSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, magSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="CAN ID signal"
        selectedSignalId={imu.canIdSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, canIdSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="CAN bus signal"
        selectedSignalId={imu.canbusSignalId}
        candidates={stringSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, canbusSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Type signal"
        selectedSignalId={imu.typeSignalId}
        candidates={stringSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, typeSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Inverted signal"
        selectedSignalId={imu.invertedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, invertedSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <label>
        Units
        <select
          value={imu.units}
          onchange={(event) =>
            setConfig({ ...imu, units: (event.currentTarget as HTMLSelectElement).value === 'rad' ? 'rad' : 'deg' } satisfies ImuWidgetConfig)}
        >
          <option value="deg">Degrees</option>
          <option value="rad">Radians</option>
        </select>
      </label>
      <label>
        Orientation view
        <select
          value={imu.orientationViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              orientationViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['orientationViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Accel view
        <select
          value={imu.accelViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              accelViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['accelViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Gyro view
        <select
          value={imu.gyroViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              gyroViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['gyroViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Mag view
        <select
          value={imu.magViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              magViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['magViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'bar'}
  {@const bar = readBarConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Bar/Meter</h4>
    <div class="row-grid">
      <label>
        Min
        <input
          type="number"
          value={bar.min}
          oninput={(event) => setConfig({ ...bar, min: Number((event.currentTarget as HTMLInputElement).value) || 0 })}
        />
      </label>
      <label>
        Max
        <input
          type="number"
          value={bar.max}
          oninput={(event) => setConfig({ ...bar, max: Number((event.currentTarget as HTMLInputElement).value) || 1 })}
        />
      </label>
      <label>
        Unit
        <input
          value={bar.unit}
          oninput={(event) => setConfig({ ...bar, unit: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Mode
        <select
          value={bar.mode}
          onchange={(event) =>
            setConfig({ ...bar, mode: (event.currentTarget as HTMLSelectElement).value as 'progress' | 'usage' })}
        >
          <option value="progress">Progress</option>
          <option value="usage">Usage</option>
        </select>
      </label>
      <label>
        Warn (0-1)
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={bar.warn}
          oninput={(event) => setConfig({ ...bar, warn: Number((event.currentTarget as HTMLInputElement).value) || 0.7 })}
        />
      </label>
      <label>
        Crit (0-1)
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={bar.crit}
          oninput={(event) => setConfig({ ...bar, crit: Number((event.currentTarget as HTMLInputElement).value) || 0.9 })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'dial'}
  {@const dial = readDialConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Dial</h4>
    <div class="row-grid">
      <label>
        Min
        <input
          type="number"
          value={dial.min}
          oninput={(event) => setConfig({ ...dial, min: Number((event.currentTarget as HTMLInputElement).value) || 0 })}
        />
      </label>
      <label>
        Max
        <input
          type="number"
          value={dial.max}
          oninput={(event) => setConfig({ ...dial, max: Number((event.currentTarget as HTMLInputElement).value) || 100 })}
        />
      </label>
      <label>
        Unit
        <input
          value={dial.unit}
          oninput={(event) => setConfig({ ...dial, unit: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'imu_3d' && primarySignal}
  {@const imu3d = readImu3dConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>3D IMU</h4>
    <div class="row-grid">
      <label>
        Roll signal
        <select
          value={imu3d.rollSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, rollSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Pitch signal
        <select
          value={imu3d.pitchSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, pitchSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Yaw signal
        <select
          value={imu3d.yawSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, yawSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Units
        <select
          value={imu3d.units}
          onchange={(event) =>
            setConfig({
              ...imu3d,
              units: (event.currentTarget as HTMLSelectElement).value === 'rad' ? 'rad' : 'deg'
            })}
        >
          <option value="deg">Degrees</option>
          <option value="rad">Radians</option>
        </select>
      </label>
    </div>
    <p class="hint">Uses [roll, pitch, yaw] when signal value is a numeric array.</p>
  </section>
{:else if selectedWidget.kind === 'timer'}
  {@const timer = readTimerConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Timer</h4>
    <div class="row-grid">
      <label>
        Mode
        <select
          value={timer.mode}
          onchange={(event) =>
            setConfig({ ...timer, mode: (event.currentTarget as HTMLSelectElement).value as 'elapsed' | 'countdown' })}
        >
          <option value="countdown">Countdown</option>
          <option value="elapsed">Elapsed</option>
        </select>
      </label>
      <label>
        Duration (sec)
        <input
          type="number"
          min="1"
          value={timer.durationSec}
          oninput={(event) =>
            setConfig({ ...timer, durationSec: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 150) })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'status_matrix' && primarySignal}
  {@const matrix = readStatusMatrixConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Status Board</h4>
    <div class="row-grid">
      <label>
        Columns
        <input
          type="number"
          min="1"
          max="8"
          value={matrix.columns}
          oninput={(event) =>
            setConfig({
              ...matrix,
              columns: Math.max(1, Math.min(8, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 4)))
            })}
        />
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={matrix.showSummary}
        onchange={(event) => setConfig({ ...matrix, showSummary: (event.currentTarget as HTMLInputElement).checked })}
      />
      Show summary chips
    </label>

    <button class="btn" disabled={boolSignals.length === 0} onclick={requestStatusMatrixSignalFromExplorer}>
      Add bool signal from explorer
    </button>

    <div class="series-list">
      {#each matrix.items as item, index (`${item.signalId}-${index}`)}
        {@const bound = signalById.get(item.signalId)}
        <div class="series-row">
          <strong>{bound?.path ?? `Signal ${item.signalId}`}</strong>
          <select
            value={item.healthyWhen}
            onchange={(event) =>
              setConfig({
                ...matrix,
                items: matrix.items.map((entry, itemIndex) =>
                  itemIndex === index
                    ? { ...entry, healthyWhen: (event.currentTarget as HTMLSelectElement).value as 'true' | 'false' }
                    : entry
                )
              })}
          >
            <option value="true">Healthy when true</option>
            <option value="false">Healthy when false</option>
          </select>
          <input
            value={item.label}
            oninput={(event) =>
              setConfig({
                ...matrix,
                items: matrix.items.map((entry, itemIndex) =>
                  itemIndex === index
                    ? { ...entry, label: (event.currentTarget as HTMLInputElement).value }
                    : entry
                )
              })}
          />
          <button
            class="btn btn-danger"
            onclick={() =>
              setConfig({
                ...matrix,
                items: matrix.items.filter((_, itemIndex) => itemIndex !== index)
              })}
          >
            Remove
          </button>
        </div>
      {/each}
    </div>
  </section>
{:else if selectedWidget.kind === 'camera_overlay' && primarySignal}
  {@const camera = readCameraOverlayConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Camera + Overlay</h4>
    <div class="row-grid">
      <label>
        Stream signal
        <select
          value={camera.streamSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, streamSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Stream URL override
        <input
          value={camera.streamUrl}
          oninput={(event) => setConfig({ ...camera, streamUrl: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Pose X signal
        <select
          value={camera.poseXSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, poseXSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Pose Y signal
        <select
          value={camera.poseYSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, poseYSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Heading signal
        <select
          value={camera.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Targets signal
        <select
          value={camera.targetsSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, targetsSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Detections signal
        <select
          value={camera.detectionsSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, detectionsSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Source width
        <input
          type="number"
          min="1"
          value={camera.sourceWidth}
          oninput={(event) =>
            setConfig({ ...camera, sourceWidth: Math.max(1, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 1280)) })}
        />
      </label>

      <label>
        Source height
        <input
          type="number"
          min="1"
          value={camera.sourceHeight}
          oninput={(event) =>
            setConfig({ ...camera, sourceHeight: Math.max(1, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 720)) })}
        />
      </label>
    </div>

    <div class="toggle-grid">
      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showPose}
          onchange={(event) => setConfig({ ...camera, showPose: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show pose
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showTargets}
          onchange={(event) => setConfig({ ...camera, showTargets: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show targets
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showDetections}
          onchange={(event) => setConfig({ ...camera, showDetections: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show detections
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.mirrorX}
          onchange={(event) => setConfig({ ...camera, mirrorX: (event.currentTarget as HTMLInputElement).checked })}
        />
        Mirror X
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'dropdown' && primarySignal}
  {@const dropdown = readDropdownConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Dropdown</h4>
    <label>
      Mode
      <select
        value={dropdown.commit}
        onchange={(event) =>
          setConfig({ ...dropdown, commit: (event.currentTarget as HTMLSelectElement).value as 'auto' | 'button' })}
      >
        <option value="button">Set button</option>
        <option value="auto">Auto-commit</option>
      </select>
    </label>

    <label>
      Options (label:value, ...)
      <input
        value={dropdownInput}
        oninput={(event) => (dropdownInput = (event.currentTarget as HTMLInputElement).value)}
        onblur={commitDropdownOptions}
      />
    </label>
    <button class="btn" onclick={commitDropdownOptions}>Apply options</button>
  </section>
{:else if selectedWidget.kind === 'toggle'}
  {@const toggle = readToggleConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Toggle</h4>
    <div class="row-grid">
      <label>
        Style
        <select
          value={toggle.style}
          onchange={(event) => {
            const nextStyle: ToggleWidgetStyle =
              (event.currentTarget as HTMLSelectElement).value === 'button' ? 'button' : 'switch';
            setConfig({
              ...toggle,
              style: nextStyle
            });
          }}
        >
          <option value="switch">Switch</option>
          <option value="button">Button</option>
        </select>
      </label>
      <label>
        True label
        <input
          value={toggle.trueLabel}
          oninput={(event) => setConfig({ ...toggle, trueLabel: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        False label
        <input
          value={toggle.falseLabel}
          oninput={(event) => setConfig({ ...toggle, falseLabel: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>
  </section>
{:else if (selectedWidget.kind === 'button_group' || selectedWidget.kind === 'radio') && primarySignal}
  {@const choice = readChoiceConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>{selectedWidget.kind === 'radio' ? 'Radio' : 'Buttons'}</h4>
    <div class="row-grid">
      <label>
        Direction
        <select
          value={choice.direction}
          onchange={(event) => {
            const nextDirection: ChoiceWidgetDirection =
              (event.currentTarget as HTMLSelectElement).value === 'horizontal'
                ? 'horizontal'
                : 'vertical';
            setConfig({
              ...choice,
              direction: nextDirection
            });
          }}
        >
          <option value="vertical">Vertical</option>
          <option value="horizontal">Horizontal</option>
        </select>
      </label>
      <label>
        Commit mode
        <select
          value={choice.commit}
          onchange={(event) => {
            const nextCommit: ChoiceWidgetCommit =
              (event.currentTarget as HTMLSelectElement).value === 'button' ? 'button' : 'auto';
            setConfig({
              ...choice,
              commit: nextCommit
            });
          }}
        >
          <option value="auto">Auto</option>
          <option value="button">Apply button</option>
        </select>
      </label>
      {#if choice.commit === 'button'}
        <label>
          Apply button label
          <input
            value={choice.buttonLabel}
            oninput={(event) => setConfig({ ...choice, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
    </div>

    <label>
      Options (label:value, ...)
      <input
        value={choiceInput}
        oninput={(event) => (choiceInput = (event.currentTarget as HTMLInputElement).value)}
        onblur={commitChoiceOptions}
      />
    </label>
    <button class="btn" onclick={commitChoiceOptions}>Apply options</button>
  </section>
{:else if selectedWidget.kind === 'input' && primarySignal}
  {@const inputCfg = readInputConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Input</h4>
    <div class="row-grid">
      <label>
        Input type
        <select
          value={inputCfg.inputType}
          onchange={(event) => {
            const nextType = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...inputCfg, inputType: nextType as HtmlInputType });
          }}
        >
          <option value="text">text</option>
          <option value="search">search</option>
          <option value="password">password</option>
          <option value="email">email</option>
          <option value="url">url</option>
          <option value="tel">tel</option>
          <option value="number">number</option>
          <option value="range">range</option>
          <option value="date">date</option>
          <option value="time">time</option>
          <option value="datetime-local">datetime-local</option>
          <option value="month">month</option>
          <option value="week">week</option>
          <option value="color">color</option>
        </select>
      </label>
      <label>
        Commit mode
        <select
          value={inputCfg.commit}
          onchange={(event) =>
            setConfig({
              ...inputCfg,
              commit: (event.currentTarget as HTMLSelectElement).value as InputWidgetCommit
            })}
        >
          <option value="auto">Auto</option>
          <option value="enter">Enter</option>
          <option value="blur">Blur</option>
          <option value="button">Set button</option>
        </select>
      </label>
      <label>
        Placeholder
        <input
          value={inputCfg.placeholder}
          oninput={(event) => setConfig({ ...inputCfg, placeholder: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      {#if inputCfg.commit === 'button'}
        <label>
          Button label
          <input
            value={inputCfg.buttonLabel}
            oninput={(event) => setConfig({ ...inputCfg, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
      <label>
        Min
        <input
          value={inputCfg.min}
          oninput={(event) => setConfig({ ...inputCfg, min: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Max
        <input
          value={inputCfg.max}
          oninput={(event) => setConfig({ ...inputCfg, max: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Step
        <input
          value={inputCfg.step}
          oninput={(event) => setConfig({ ...inputCfg, step: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Pattern
        <input
          value={inputCfg.pattern}
          oninput={(event) => setConfig({ ...inputCfg, pattern: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Autocomplete
        <input
          value={inputCfg.autocomplete}
          oninput={(event) => setConfig({ ...inputCfg, autocomplete: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={inputCfg.spellcheck}
        onchange={(event) => setConfig({ ...inputCfg, spellcheck: (event.currentTarget as HTMLInputElement).checked })}
      />
      Spellcheck
    </label>
  </section>
{:else if selectedWidget.kind === 'textarea'}
  {@const textArea = readTextAreaConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Text Area</h4>
    <div class="row-grid">
      <label>
        Placeholder
        <input
          value={textArea.placeholder}
          oninput={(event) => setConfig({ ...textArea, placeholder: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Rows
        <input
          type="number"
          min="2"
          max="16"
          value={textArea.rows}
          oninput={(event) =>
            setConfig({
              ...textArea,
              rows: Math.max(2, Math.min(16, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 4)))
            })}
        />
      </label>
      <label>
        Max length (blank = none)
        <input
          type="number"
          min="1"
          value={textArea.maxLength ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value.trim();
            const value = raw === '' ? null : Math.max(1, Math.floor(Number(raw) || 0));
            setConfig({ ...textArea, maxLength: value });
          }}
        />
      </label>
      <label>
        Commit mode
        <select
          value={textArea.commit}
          onchange={(event) =>
            setConfig({
              ...textArea,
              commit: (event.currentTarget as HTMLSelectElement).value as TextAreaWidgetCommit
            })}
        >
          <option value="auto">Auto</option>
          <option value="ctrl-enter">Ctrl+Enter</option>
          <option value="blur">Blur</option>
          <option value="button">Set button</option>
        </select>
      </label>
      {#if textArea.commit === 'button' || textArea.commit === 'ctrl-enter'}
        <label>
          Button label
          <input
            value={textArea.buttonLabel}
            oninput={(event) => setConfig({ ...textArea, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={textArea.spellcheck}
        onchange={(event) => setConfig({ ...textArea, spellcheck: (event.currentTarget as HTMLInputElement).checked })}
      />
      Spellcheck
    </label>
  </section>
{:else if selectedWidget.kind === 'field' && primarySignal}
  {@const field = readFieldConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Field Viewer</h4>
    <div class="row-grid">
      <label>
        X signal
        <select
          value={field.xSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, xSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Y signal
        <select
          value={field.ySignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, ySignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Heading signal
        <select
          value={field.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Trajectory signal
        <select
          value={field.trajectorySignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, trajectorySignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Field image signal
        <select
          value={field.imageSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, imageSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Field image URL override
        <input
          value={field.imageUrl}
          oninput={(event) => setConfig({ ...field, imageUrl: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Image opacity
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={field.imageOpacity}
          oninput={(event) =>
            setConfig({ ...field, imageOpacity: Math.max(0, Math.min(1, Number((event.currentTarget as HTMLInputElement).value) || 0)) })}
        />
      </label>

      <label>
        Field length (m)
        <input
          type="number"
          min="1"
          step="0.01"
          value={field.fieldLength}
          oninput={(event) =>
            setConfig({ ...field, fieldLength: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 16.54) })}
        />
      </label>

      <label>
        Field width (m)
        <input
          type="number"
          min="1"
          step="0.01"
          value={field.fieldWidth}
          oninput={(event) =>
            setConfig({ ...field, fieldWidth: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 8.02) })}
        />
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={field.allowPoseSet}
        onchange={(event) => setConfig({ ...field, allowPoseSet: (event.currentTarget as HTMLInputElement).checked })}
      />
      Allow click-to-set pose
    </label>
  </section>
{:else if selectedWidget.kind === 'mech2d'}
  {@const mech2d = readMech2dConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Mechanism2d</h4>
    <div class="row-grid">
      <label>
        Line width
        <input
          type="number"
          min="0.002"
          max="0.25"
          step="0.002"
          value={mech2d.lineWidth}
          oninput={(event) =>
            setConfig({
              ...mech2d,
              lineWidth: Math.max(0.002, Math.min(0.25, Number((event.currentTarget as HTMLInputElement).value) || mech2d.lineWidth))
            })}
        />
      </label>
    </div>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={mech2d.showGrid}
        onchange={(event) => setConfig({ ...mech2d, showGrid: (event.currentTarget as HTMLInputElement).checked })}
      />
      Show grid
    </label>
  </section>
{:else if selectedWidget.kind === 'swerve_module' && primarySignal}
  {@const swerveModule = readSwerveModuleConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Swerve Module</h4>
    <div class="row-grid">
      <label>
        Label
        <input
          value={swerveModule.label}
          oninput={(event) => setConfig({ ...swerveModule, label: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Angle signal
        <select
          value={swerveModule.angleSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, angleSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Auto / none</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Speed signal
        <select
          value={swerveModule.speedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, speedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Auto / none</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={swerveModule.maxSpeed}
          oninput={(event) =>
            setConfig({ ...swerveModule, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'swerve_drive' && primarySignal}
  {@const swerveDrive = readSwerveDriveConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Swerve Drive</h4>
    <div class="row-grid">
      <label>
        Heading signal
        <select
          value={swerveDrive.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveDrive, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={swerveDrive.maxSpeed}
          oninput={(event) =>
            setConfig({ ...swerveDrive, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>
    </div>

    <div class="series-list">
      {#each swerveDrive.modules as module, index (module.key)}
        <div class="series-row">
          <strong>{module.key.toUpperCase()}</strong>
          <input
            value={module.label}
            oninput={(event) =>
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index
                    ? { ...entry, label: (event.currentTarget as HTMLInputElement).value }
                    : entry
                )
              })}
          />
          <select
            value={module.angleSignalId ?? ''}
            onchange={(event) => {
              const raw = (event.currentTarget as HTMLSelectElement).value;
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index ? { ...entry, angleSignalId: raw ? Number(raw) : null } : entry
                )
              });
            }}
          >
            <option value="">Angle</option>
            {#each numericSignals as option (option.signal_id)}
              <option value={option.signal_id}>{option.path}</option>
            {/each}
          </select>
          <select
            value={module.speedSignalId ?? ''}
            onchange={(event) => {
              const raw = (event.currentTarget as HTMLSelectElement).value;
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index ? { ...entry, speedSignalId: raw ? Number(raw) : null } : entry
                )
              });
            }}
          >
            <option value="">Speed</option>
            {#each numericSignals as option (option.signal_id)}
              <option value={option.signal_id}>{option.path}</option>
            {/each}
          </select>
        </div>
      {/each}
    </div>
  </section>
{:else if selectedWidget.kind === 'differential_drive' && primarySignal}
  {@const diff = readDifferentialDriveConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Differential Drive</h4>
    <div class="row-grid">
      <label>
        Left speed signal
        <select
          value={diff.leftSpeedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, leftSpeedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Right speed signal
        <select
          value={diff.rightSpeedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, rightSpeedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Heading signal
        <select
          value={diff.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={diff.maxSpeed}
          oninput={(event) =>
            setConfig({ ...diff, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'layout_title'}
  {@const titleCfg = readLayoutTitleConfig(selectedWidget.config, selectedWidget.title || 'Title')}
  <section class="config-block">
    <h4>Title</h4>
    <div class="row-grid">
      <label>
        Text
        <input
          value={titleCfg.text}
          oninput={(event) =>
            setConfig({
              ...titleCfg,
              text: (event.currentTarget as HTMLInputElement).value
            })}
        />
      </label>
      <label>
        Text color
        <input
          type="color"
          value={titleCfg.color.startsWith('#') ? titleCfg.color : '#f5f7fb'}
          oninput={(event) =>
            setConfig({
              ...titleCfg,
              color: (event.currentTarget as HTMLInputElement).value
            })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'layout_grid'}
  {@const layoutColumns = Math.max(1, Math.floor(selectedWidget.layout.w))}
  {@const layoutRows = Math.max(1, Math.floor(selectedWidget.layout.h))}
  {@const gridCfg = readLayoutGridConfig(selectedWidget.config, layoutColumns, layoutRows)}
  {@const effectiveColumns = gridCfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(selectedWidget.layout.w)))
    : gridCfg.columns}
  {@const effectiveRows = gridCfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(selectedWidget.layout.h)))
    : gridCfg.rows}
  <section class="config-block">
    <h4>Grid Layout</h4>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={gridCfg.autoSize}
        oninput={(event) =>
          setConfig({
            ...gridCfg,
            autoSize: (event.currentTarget as HTMLInputElement).checked
          })}
      />
      Auto size to canvas cells
    </label>
    <div class="row-grid">
      <label>
        Columns
        <input
          type="number"
          min="1"
          max="96"
          value={gridCfg.columns}
          disabled={gridCfg.autoSize}
          oninput={(event) =>
            setConfig({
              ...gridCfg,
              columns: Math.max(
                1,
                Math.min(96, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || layoutColumns))
              )
            })}
        />
      </label>
      <label>
        Rows
        <input
          type="number"
          min="1"
          max="96"
          value={gridCfg.rows}
          disabled={gridCfg.autoSize}
          oninput={(event) =>
            setConfig({
              ...gridCfg,
              rows: Math.max(
                1,
                Math.min(96, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || layoutRows))
              )
            })}
        />
      </label>
    </div>
    {#if gridCfg.autoSize}
      <p class="hint">Effective size: {effectiveColumns} columns x {effectiveRows} rows (tracks canvas cell size).</p>
    {:else}
      <p class="hint">Children snap to this grid's rows/columns.</p>
    {/if}
  </section>
{:else}
  <p class="empty">This widget has no extra configuration.</p>
{/if}

<style>
  .config-block {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.5rem;
    display: grid;
    gap: 0.38rem;
  }

  .config-block h4 {
    margin: 0;
    font-size: 0.76rem;
    color: var(--text-strong);
  }

  .hint {
    margin: 0;
    color: var(--text-soft);
    font-size: 0.7rem;
  }

  .row-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.34rem;
  }

  label {
    display: grid;
    gap: 0.18rem;
    color: var(--text-soft);
    font-size: 0.68rem;
  }

  label input,
  label select {
    width: 100%;
    min-width: 0;
    padding: 0.24rem 0.34rem;
    font-size: 0.7rem;
  }

  .row-inline {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
    width: fit-content;
  }

  .row-inline input[type='checkbox'] {
    width: auto;
    margin: 0;
  }

  .check-row {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
    width: fit-content;
    font-size: 0.68rem;
    color: var(--text-soft);
  }

  .check-row input[type='checkbox'] {
    width: auto;
    margin: 0;
  }

  .toggle-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.28rem 0.42rem;
  }

  .series-list {
    display: grid;
    gap: 0.28rem;
  }

  .series-row {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    padding: 0.24rem;
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto auto auto;
    gap: 0.24rem;
    align-items: center;
  }

  .series-row-map {
    grid-template-columns: minmax(0, 1fr) minmax(0, 0.85fr) minmax(0, 1.25fr) auto;
  }

  .series-row strong {
    min-width: 0;
    font-size: 0.68rem;
    color: var(--text);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .series-row select,
  .series-row input {
    font-size: 0.66rem;
    padding: 0.18rem 0.26rem;
  }

  .series-row input[type='color'] {
    width: 2rem;
    height: 1.4rem;
    padding: 0.08rem;
  }

  .series-row .btn {
    padding: 0.2rem 0.34rem;
    font-size: 0.62rem;
  }

  .chips {
    display: flex;
    flex-wrap: wrap;
    gap: 0.24rem;
  }

  .chips span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.12rem 0.34rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    background: var(--surface-3);
  }

  .config-collapsible {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-3);
    padding: 0.34rem;
    display: grid;
    gap: 0.34rem;
  }

  .config-collapsible > summary {
    cursor: pointer;
    color: var(--text-soft);
    font-size: 0.7rem;
    font-weight: 600;
    user-select: none;
  }

  .config-collapsible[open] > summary {
    color: var(--text-strong);
  }

  .empty {
    margin: 0;
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  @media (max-width: 700px) {
    .row-grid {
      grid-template-columns: 1fr;
    }

    .toggle-grid {
      grid-template-columns: 1fr;
    }

    .series-row {
      grid-template-columns: 1fr;
    }
  }
</style>
