<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import SignalMapField from '../SignalMapField.svelte';
  import { readMotorConfig } from '../../lib/widget-config';
  import type { MotorWidgetConfig } from '../../lib/widget-config';
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
  const writableNumericSignals = $derived(pools.writableNumericSignals);
  const modeSignals = $derived(pools.modeSignals);
  const writableMotorCommandSignals = $derived(pools.writableMotorCommandSignals);
  const preferredMotorCommandSignals = $derived(pools.preferredMotorCommandSignals);

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

{#if selectedWidget.kind === 'motor'}
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
{/if}
