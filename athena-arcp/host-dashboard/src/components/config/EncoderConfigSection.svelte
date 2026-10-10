<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import SignalMapField from '../SignalMapField.svelte';
  import { readEncoderConfig } from '../../lib/widget-config';
  import type { EncoderWidgetConfig } from '../../lib/widget-config';
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

{#if selectedWidget.kind === 'encoder'}
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
        label="Rate signal"
        selectedSignalId={encoder.rateSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, rateSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Connected signal"
        selectedSignalId={encoder.connectedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, connectedSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Conversion signal"
        selectedSignalId={encoder.conversionSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...encoder, conversionSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Conversion offset signal"
        selectedSignalId={encoder.conversionOffsetSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) =>
          setConfig({ ...encoder, conversionOffsetSignalId: signalId } satisfies EncoderWidgetConfig)}
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
      <SignalMapField
        label="Discontinuity point signal"
        selectedSignalId={encoder.discontinuityPointSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) =>
          setConfig({ ...encoder, discontinuityPointSignalId: signalId } satisfies EncoderWidgetConfig)}
      />
      <SignalMapField
        label="Discontinuity range signal"
        selectedSignalId={encoder.discontinuityRangeSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) =>
          setConfig({ ...encoder, discontinuityRangeSignalId: signalId } satisfies EncoderWidgetConfig)}
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
{/if}
