<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import SignalMapField from '../SignalMapField.svelte';
  import { readDioConfig } from '../../lib/widget-config';
  import type { DioWidgetConfig } from '../../lib/widget-config';
  import type { SignalSectionProps } from './types';

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
  const boolSignals = $derived(pools.boolSignals);
  const stringSignals = $derived(pools.stringSignals);
  const writableBoolSignals = $derived(pools.writableBoolSignals);
  const modeSignals = $derived(pools.modeSignals);

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
</script>

{#if selectedWidget.kind === 'dio'}
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
{/if}
