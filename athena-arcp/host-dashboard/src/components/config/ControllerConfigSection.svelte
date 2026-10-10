<script lang="ts">
  import { getContext } from 'svelte';
  import type { SignalRow } from '../../lib/arcp';
  import SignalMapField from '../SignalMapField.svelte';
  import { readControllerConfig } from '../../lib/widget-config';
  import type { ControllerParam, ControllerWidgetConfig } from '../../lib/widget-config';
  import { isWritableNumeric, normalizeControllerKey } from '../../lib/widget-config/editor-helpers';
  import type { SignalSectionProps, SignalMapRequest } from './types';
  import {
    appendControllerParam,
    patchControllerParam,
    removeControllerParamAt
  } from '../../lib/widget-config/editor-helpers';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const writableNumericSignals = $derived(pools.writableNumericSignals);
  const actionSignals = $derived(pools.actionSignals);
  const modeSignals = $derived(pools.modeSignals);
  const requestSignalMap =
    getContext<((request: SignalMapRequest) => void) | undefined>('arcp.signal-map-request');

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
  function updateControllerParam(index: number, patch: Partial<ControllerParam>) {
    if (!primarySignal) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    setConfig(patchControllerParam(controller, index, patch) satisfies ControllerWidgetConfig);
  }
  function removeControllerParam(index: number) {
    if (!primarySignal) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    const next = removeControllerParamAt(controller, index);
    if (next) setConfig(next satisfies ControllerWidgetConfig);
  }
  function addControllerParam(signal: SignalRow) {
    if (!primarySignal || !isWritableNumeric(signal)) return;
    const controller = readControllerConfig(selectedWidget.config, primarySignal, signals);
    const next = appendControllerParam(controller, signal);
    if (next) setConfig(next satisfies ControllerWidgetConfig);
  }
  function addSelectedControllerParam() {
    if (!isWritableNumeric(selectedSignal)) return;
    addControllerParam(selectedSignal);
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
</script>

{#if selectedWidget.kind === 'controller'}
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
{/if}
